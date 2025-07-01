from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rospy
import threading
from collections import deque

class Agent():
    def __init__(self, robot_config, gripper_config=None):
        
        self.js_mutex = threading.Lock()
        self.joint_states = None
        self.robot_type = robot_config['type']
        self.joint_names = robot_config['joint_names']
        self.current_robot_js = {
            robot_joint_name: deque(maxlen=10) for robot_joint_name in self.joint_names
        }
        
        rospy.Subscriber(robot_config['read_topic'], JointState, self.joint_state_cb)
        self.move_robot_pub = rospy.Publisher(robot_config['write_topic'], JointState, queue_size=10)
            
        
    def move_step(self, action):
        if self.robot_type == 'ur5e':
            pass
            # self.move_step_ur5(action)
        elif self.robot_type == 'piper':
            self.move_step_piper(action)
            
            
    def move_step_piper(self, action):
        js = JointState()
        js.position = action[:6]
        self.move_robot_pub.publish(js)
        
        
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg.position
        
        
        
    # def move_step_ur5(self, action):

    #     try:
    #         joint_trajectory = JointTrajectory()
    #         joint_trajectory.joint_names = ["ur5e_elbow_joint", "ur5e_shoulder_lift_joint", "ur5e_shoulder_pan_joint", "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"]

    #         rad_pos = action[:6]

    #         # rospy.loginfo(f"[0] {rad_pos[0]} [1] {rad_pos[1]} [2] {rad_pos[2]} [3] {rad_pos[3]} [4] {rad_pos[4]} [5] {rad_pos[5]}")

    #         joint_point = JointTrajectoryPoint()
    #         joint_point.positions = rad_pos

    #         joint_point.time_from_start = rospy.Duration(0.5)

    #         joint_trajectory.points.append(joint_point)

    #         joint_goal = FollowJointTrajectoryGoal()
    #         joint_goal.trajectory = joint_trajectory

    #         self.move_robot.send_goal(joint_goal)