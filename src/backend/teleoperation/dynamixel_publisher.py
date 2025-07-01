# #!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
import dynamixel_sdk as dxl  # 다이나믹셀 SDK 사용
import sys
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from robotiq_85_msgs.msg import GripperCmd
# from sensor_msgs.msg import JointState
# from gello_controller.srv import MoveRobot, MoveRobotRequest
# from collections import deque
# from std_srvs.srv import Trigger, TriggerResponse
import argparse
import threading
# import numpy as np
import time
import math

# Import database and data models
import os
from ..database.db_init import get_db_connection
from ..database.models.robot_model import RobotModel
from ..database.models.policy_model import PolicyModel
from ..database.models.task_model import TaskModel
from ..database.models.gripper_model import GripperModel
from ..database.models.sensor_model import SensorModel
from ..database.models.checkpoint_model import CheckpointModel
from ..database.models.leader_robot_preset_model import LeaderRobotPresetModel

from ..env.agent import Agent

class Leader():
    def __init__(self, robot_config, leader_robot_presets_config) -> None:
        # ROS 노드 초기화ur5e/ur5e_scaled_pos_joint_traj_controller/command
        rospy.init_node('dynamixel_publisher', anonymous=False)
        
        self.origin = leader_robot_presets_config['origin']  # 다이나믹셀의 원점 위치
        self.sign_corrector = leader_robot_presets_config['sign_corrector']
        self.max_pos = 65536
        self.dxl_ids = leader_robot_presets_config['dxl_ids']  # 다이나믹셀 ID
        self.target_pos = [0, 0, 0, 0, 0, 0, 0]
        self.rad_pos = [0, 0, 0, 0, 0, 0, 0]
        
        self.address = 132  # 다이나믹셀의 현재 위치 주소 (주소 132번은 현재 위치)
        self.rate = rospy.Rate(10)  # 10Hz로 퍼블리시
        self.p_gain = 1
        self.pos_diff_limit = math.pi / 180 * 45
        self.is_synced = False
        self.pub = rospy.Publisher('dynamixel_position', Float64, queue_size=10)
        # self.gripper_publisher = rospy.Publisher('gripper/cmd', GripperCmd, queue_size=1)
        
        # self.trajectory_sub = rospy.Subscriber(robot_config['read_topic'], JointState, self.sub_js)
        # self.trajectory_pub = rospy.Publisher(robot_config['write_topic'], JointTrajectory, queue_size=1)
        self.agent = Agent(robot_config)
        
        
        time.sleep(0.1)


        # 다이나믹셀 포트 및 패킷 핸들러 설정
        self.portHandler = dxl.PortHandler('/dev/ttyUSB0')  # 다이나믹셀 포트
        self.packetHandler = dxl.PacketHandler(2.0)         # 프로토콜 2.0 사용
        self.port_lock = threading.Lock()

        # 포트 열기 및 Baud rate 설정
        if not self.portHandler.openPort():
            rospy.logerr("Failed to open the port")
            return
        if not self.portHandler.setBaudRate(57600):
            rospy.logerr("Failed to set baudrate")
            return
    
        # self.move_robot_client()
        # self.gripper_pub()
        self.sync_leader_robot()
        self.position_pub()
        

    def get_rad_pos(self, position, dxl_id):
        pos = (position - self.origin[dxl_id]) % 4096
        pos = pos / 4096 * 360
        if pos > 180:
            pos -= 360
        elif pos < -180:
            pos += 360
        return pos / 360 * 2 * math.pi
    
    
    def rad_to_tick(self, rad):
        return int((rad + math.pi / 2) * (4096 / math.pi)) + 2048
        


    # def move_robot_client(self):

    #     try:
    #         move_robot = rospy.ServiceProxy('mover/move_robot_planning', MoveRobot)


    #         joint_trajectory = JointTrajectory()
    #         joint_trajectory.joint_names = self.joint_names

    #         rad_pos = [0,0,0,0,0,0]


    #         for index, dxl_id in enumerate(self.dxl_ids[:6]):
    #             position, comm_result, error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, self.address)

    #             rad_pos[index] = self.get_rad_pos(position, dxl_id)

    #             if index == 0:
    #                 rad_pos[index] = -rad_pos[index]
                

    #             if comm_result != dxl.COMM_SUCCESS:
    #                 rospy.logerr("Failed to read position: %s" % self.packetHandler.getTxRxResult(comm_result))
    #             elif error != 0:
    #                 rospy.logerr("Error: %s" % self.packetHandler.getRxPacketError(error))
                

    #         rospy.loginfo(f"[0] {rad_pos[0]} [1] {rad_pos[1]} [2] {rad_pos[2]} [3] {rad_pos[3]} [4] {rad_pos[4]} [5] {rad_pos[5]}")

    #         request = MoveRobotRequest()
    #         request.joint_trajectory = rad_pos[:6]
    #         # print(joint_trajectory)

    #         response = move_robot(request)

    #         if response.success:
    #             rospy.loginfo("Success: %s", response.message)
    #             self.is_synced = True
    #             self.position_pub()
    #         else:
    #             rospy.logwarn("Failed: %s", response.message)

    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service call failed: %s", e)

    
    def enable_torque(self):
        torque_enable_address = 64  # MX, X 시리즈 기준
        for index, dxl_id in enumerate(self.dxl_ids):
            with self.port_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, torque_enable_address, 1)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Torque Enable Comm Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Torque Enable Error: {self.packetHandler.getRxPacketError(dxl_error)}")
                return
            
    def remove_torque(self):
        torque_enable_address = 64  # MX, X 시리즈 기준
        for index, dxl_id in enumerate(self.dxl_ids):
            with self.port_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, torque_enable_address, 0)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Torque Remove Comm Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Torque Remove Error: {self.packetHandler.getRxPacketError(dxl_error)}")
                return
            
            
    def read_dynamixel(self, dxl_id):
        present_position_address = 132
        with self.port_lock:
            position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, present_position_address)
            
        if dxl_comm_result != dxl.COMM_SUCCESS:
            rospy.logerr("Failed to read position: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr("Error: %s" % self.packetHandler.getRxPacketError(dxl_error))
            
        return position
    
    
    def move_controller(self, goal_position):
        goal_position_address = 116  # Goal Position address (X 시리즈)
        velocity_address = 112       # Profile Velocity address
        success = True

        self.enable_torque()

        # 1. 속도 설정 (각 모터 동일한 속도)
        for dxl_id in self.dxl_ids[:6]:
            with self.port_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, velocity_address, 10)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"[ID:{dxl_id}] Velocity Comm Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                success = False
            elif dxl_error != 0:
                print(f"[ID:{dxl_id}] Velocity Error: {self.packetHandler.getRxPacketError(dxl_error)}")
                success = False

        # 2. GroupSyncWrite 객체 생성
        groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, goal_position_address, 4)

        # 3. 목표 위치 패킷 구성
        for index, dxl_id in enumerate(self.dxl_ids[:6]):
            pos = goal_position[index]
            param_goal_position = [
                pos & 0xFF,
                (pos >> 8) & 0xFF,
                (pos >> 16) & 0xFF,
                (pos >> 24) & 0xFF
            ]
            groupSyncWrite.addParam(dxl_id, bytearray(param_goal_position))

        # 4. 한번에 전송
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("GroupSyncWrite failed:", self.packetHandler.getTxRxResult(dxl_comm_result))
            return False

        groupSyncWrite.clearParam()

        # 5. 모든 모터가 도달할 때까지 대기
        present_position_address = 132  # Present Position address
        reached = [False] * len(self.dxl_ids[:6])

        while not all(reached):
            for i, dxl_id in enumerate(self.dxl_ids[:6]):
                if reached[i]:
                    continue

                with self.port_lock:
                    position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, present_position_address)
                if dxl_comm_result != dxl.COMM_SUCCESS:
                    print(f"[ID:{dxl_id}] Read Position Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    success = False
                    reached[i] = True  # 에러 났으니 더 이상 기다리지 않음
                    continue
                elif dxl_error != 0:
                    print(f"[ID:{dxl_id}] Read Packet Error: {self.packetHandler.getRxPacketError(dxl_error)}")
                    success = False
                    reached[i] = True
                    continue

                print(f"[ID:{dxl_id}] GoalPos:{goal_position[i]}  PresPos:{position}")
                if abs(goal_position[i] - position) < 40:
                    print(f"[ID:{dxl_id}] Reached goal position.")
                    reached[i] = True

            time.sleep(0.1)

        return success

    
    
    def sync_leader_robot(self):
        follower_pos = self.agent.joint_states
        follower_tick = []
        pos = []
        
        for index, dxl_id in enumerate(self.dxl_ids[:6]):
            follower_tick.append(self.rad_to_tick(follower_pos[dxl_id] * self.sign_corrector[dxl_id]))
            pos.append(int((self.origin[dxl_id] + follower_tick[index]) % 4096))
            
        moved = self.move_controller(pos)
        
        if moved:
            input('Will you start teleoperation? (Press Enter)')
            self.is_synced = True
            self.remove_torque()
        else:
            raise Exception


    def position_pub(self):
        while not rospy.is_shutdown():
            if not self.is_synced:
                continue
            for index, dxl_id in enumerate(self.dxl_ids[:6]):
                # 다이나믹셀 값 읽어오기
                position = self.read_dynamixel(dxl_id)
                    
                self.rad_pos[index] = self.get_rad_pos(position, dxl_id) 
                self.target_pos[index] = self.rad_pos[index] * self.sign_corrector[dxl_id]


                # # 컨트롤러와 리더의 차이가 너무 클 때
                # if abs(self.target_pos[index] - latest_follower_pos) > self.pos_diff_limit:
                #     publish = False
                #     # print("Action is too big ", dxl_id)
                #     # print(pos_error)
                #     # print(position)
                #     # rospy.loginfo(f"[0] {self.target_pos[0]} [1] {self.target_pos[1]} [2] {self.target_pos[2]} [3] {self.target_pos[3]} [4] {self.target_pos[4]} [5] {self.target_pos[5]}")
                #     break
                # else:
                #     publish = True
                
            self.agent.move_step(self.target_pos)
            self.rate.sleep()

            # rospy.loginfo(f"[0] {self.rad_pos[0]} [1] {self.rad_pos[1]} [2] {self.rad_pos[2]} [3] {self.rad_pos[3]} [4] {self.rad_pos[4]} [5] {self.rad_pos[5]}")

        
        self.portHandler.closePort()
        
        
def main(args):
    
    leader_robot_presets_config = vars(LeaderRobotPresetModel.find_one({'id': args['leader_robot_preset_id']}))
    robot_config = vars(RobotModel.find_one({'id': leader_robot_presets_config['robot_id']}))
    print(robot_config)
    try:
        leader = Leader(robot_config, leader_robot_presets_config)
        # leader.gripper_pub()
        # leader.position_pub()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--leader_robot_preset_id', default=None, required=True)
    
    main(vars(parser.parse_args()))
    sys.exit(0)
