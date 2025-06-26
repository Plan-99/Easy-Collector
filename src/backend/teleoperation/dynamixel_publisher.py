# #!/usr/bin/env python

import rospy
import math
# from std_msgs.msg import Float64
import dynamixel_sdk as dxl  # 다이나믹셀 SDK 사용
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from robotiq_85_msgs.msg import GripperCmd
# from sensor_msgs.msg import JointState
# from gello_controller.srv import MoveRobot, MoveRobotRequest
from collections import deque
# from std_srvs.srv import Trigger, TriggerResponse
import argparse

# Import database and data models
import os
from ..database.db_init import get_db_connection
from ..database.models.robot_model import RobotModel
from ..database.models.policy_model import PolicyModel
from ..database.models.task_model import TaskModel
from ..database.models.gripper_model import GripperModel
from ..database.models.sensor_model import SensorModel
from ..database.models.checkpoint_model import CheckpointModel

class Leader():
    def __init__(self, robot_config) -> None:
        # ROS 노드 초기화ur5e/ur5e_scaled_pos_joint_traj_controller/command
        rospy.init_node('dynamixel_publisher', anonymous=False)
        
        self.joint_names = robot_config['joint_names']
        self.current_robot_js = {
            robot_joint_name: deque(maxlen=10) for robot_joint_name in robot_joint_names
        }
        # self.current_robot_js = {
        #     "ur5e_elbow_joint": deque(maxlen=10),
        #     "ur5e_shoulder_lift_joint": deque(maxlen=10),
        #     "ur5e_shoulder_pan_joint": deque(maxlen=10),
        #     "ur5e_wrist_1_joint": deque(maxlen=10),
        #     "ur5e_wrist_2_joint": deque(maxlen=10),
        #     "ur5e_wrist_3_joint": deque(maxlen=10),
        # }

        self.joint_names = ["ur5e_elbow_joint", "ur5e_shoulder_lift_joint", "ur5e_shoulder_pan_joint", "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"]
        self.origin = [968, 3055, 64461, 1030, 2007, 61410, 233]
        self.max_pos = 65536

        self.dxl_ids = [2, 1, 0, 3, 4, 5, 6]  # 다이나믹셀 ID
        self.dxl_pos = [0, 0, 0, 0, 0, 0, 0]
        self.target_pos = [0, 0, 0, 0, 0, 0, 0]
        self.rad_pos = [0, 0, 0, 0, 0, 0, 0]
        self.address = 132  # 다이나믹셀의 현재 위치 주소 (주소 132번은 현재 위치)

        self.rate = rospy.Rate(10)  # 10Hz로 퍼블리시

        self.p_gain = 1

        self.pos_diff_limit = math.pi / 180 * 45
        self.is_synced = False

        self.pub = rospy.Publisher('dynamixel_position', Float64, queue_size=10)

        self.trajectory_sub = rospy.Subscriber('ur5e/joint_states', JointState, self.sub_js)
        self.trajectory_pub = rospy.Publisher('ur5e/ur5e_scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=1)
        self.gripper_publisher = rospy.Publisher('gripper/cmd', GripperCmd, queue_size=1)


        # 다이나믹셀 포트 및 패킷 핸들러 설정
        self.portHandler = dxl.PortHandler('/dev/ttyUSB0')  # 다이나믹셀 포트
        self.packetHandler = dxl.PacketHandler(2.0)         # 프로토콜 2.0 사용

        # 포트 열기 및 Baud rate 설정
        if not self.portHandler.openPort():
            rospy.logerr("Failed to open the port")
            return
        if not self.portHandler.setBaudRate(57600):
            rospy.logerr("Failed to set baudrate")
            return
    
        self.move_robot_client()
        # self.gripper_pub()
        # self.position_pub()


    def sub_js(self, msg):  # subscriber 콜백 함수

        # subscribe하는 msg의 각 joint 별 position을 current_robot_js의 deque에 저장
        for index, joint_name in enumerate(msg.name):
            self.current_robot_js[joint_name].append(msg.position[index])

    def get_rad_pos(self, position, dxl_id):
        pos = (position - self.origin[dxl_id]) % 4096
        pos = pos / 4096 * 360
        if pos > 180:
            pos -= 360
        elif pos < -180:
            pos += 360

        return pos / 360 * 2 * math.pi


    def move_robot_client(self):
        rospy.wait_for_service('mover/move_robot_planning')

        try:
            move_robot = rospy.ServiceProxy('mover/move_robot_planning', MoveRobot)


            joint_trajectory = JointTrajectory()
            joint_trajectory.joint_names = self.joint_names

            rad_pos = [0,0,0,0,0,0]


            for index, dxl_id in enumerate(self.dxl_ids[:6]):
                position, comm_result, error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, self.address)

                rad_pos[index] = self.get_rad_pos(position, dxl_id)

                if index == 0:
                    rad_pos[index] = -rad_pos[index]
                

                if comm_result != dxl.COMM_SUCCESS:
                    rospy.logerr("Failed to read position: %s" % self.packetHandler.getTxRxResult(comm_result))
                elif error != 0:
                    rospy.logerr("Error: %s" % self.packetHandler.getRxPacketError(error))
                

            rospy.loginfo(f"[0] {rad_pos[0]} [1] {rad_pos[1]} [2] {rad_pos[2]} [3] {rad_pos[3]} [4] {rad_pos[4]} [5] {rad_pos[5]}")

            request = MoveRobotRequest()
            request.joint_trajectory = rad_pos[:6]
            # print(joint_trajectory)

            response = move_robot(request)

            if response.success:
                rospy.loginfo("Success: %s", response.message)
                self.is_synced = True
                self.position_pub()
            else:
                rospy.logwarn("Failed: %s", response.message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def position_pub(self):
        while not rospy.is_shutdown() and self.is_synced:
            publish = False
            for index, dxl_id in enumerate(self.dxl_ids[:6]):

                # 다이나믹셀 값 읽어오기
                position, comm_result, error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, self.address)

                self.rad_pos[index] = self.get_rad_pos(position, dxl_id)
                if index == 0:
                    self.rad_pos[index] = -self.rad_pos[index]  # 0번째 모터 값 반대

                latest_follower_pos = self.current_robot_js[self.joint_names[index]][-1]  # ur5e의 최신 위치
                pos_error = self.rad_pos[index] - latest_follower_pos  # 컨트롤러-ur5e 차이
                if pos_error > math.pi:
                    pos_error -= 2*math.pi
                elif pos_error < -math.pi:
                    pos_error += 2*math.pi
                

                # 목표 위치
                self.target_pos[index] = latest_follower_pos + pos_error*self.p_gain

                if comm_result != dxl.COMM_SUCCESS:
                    rospy.logerr("Failed to read position: %s" % self.packetHandler.getTxRxResult(comm_result))
                elif error != 0:
                    rospy.logerr("Error: %s" % self.packetHandler.getRxPacketError(error))

                # 컨트롤러와 리더의 차이가 너무 클 때
                if abs(self.target_pos[index] - latest_follower_pos) > self.pos_diff_limit:
                    publish = False
                    print("Action is too big ", dxl_id)
                    print(pos_error)
                    print(position)
                    rospy.loginfo(f"[0] {self.target_pos[0]} [1] {self.target_pos[1]} [2] {self.target_pos[2]} [3] {self.target_pos[3]} [4] {self.target_pos[4]} [5] {self.target_pos[5]}")
                    rospy.loginfo(self.current_robot_js)
                    break
                else:
                    publish = True

            if not publish:
                continue

            # rospy.loginfo(f"[0] {self.rad_pos[0]} [1] {self.rad_pos[1]} [2] {self.rad_pos[2]} [3] {self.rad_pos[3]} [4] {self.rad_pos[4]} [5] {self.rad_pos[5]}")

            traj = JointTrajectory()
            traj.joint_names = self.joint_names 
            # traj.header.stamp = rospy.Time.now()

            point = JointTrajectoryPoint()
            point.positions = self.target_pos[:6]
            point.velocities = []
            point.time_from_start.secs = 1

            traj.points.append(point)

            self.trajectory_pub.publish(traj)

            position, comm_result, error = self.packetHandler.read2ByteTxRx(self.portHandler, 6, self.address)  # 6: gripper id

            pos = self.get_rad_pos(position, 6)
            
            self.target_pos[6]=[pos]
            gripper_msg = GripperCmd()
            scaled_value = ((pos +0.95) / (0.023 +0.95)) * (0.085 - 0.0)
            gripper_msg.position = scaled_value
            self.gripper_publisher.publish(gripper_msg)

            self.rate.sleep()
        
        self.portHandler.closePort()
        
        
def main(args):
    
    robot_config = vars(RobotModel.find_one({'id': args['robot_id']}))['args']
    try:
        leader = Leader(robot_config)
        # leader.gripper_pub()
        # leader.position_pub()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot_id', default=None, required=True)
    
    main(vars(parser.parse_args()))
    sys.exit(0)
