from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
from rclpy.node import Node
import threading
from collections import deque
import time

class Agent:
    def __init__(self, node: Node, robot, gripper=None):
        self.id = robot['id']
        self.leader_robot_preset = robot.get('leader_robot_preset', None)    
        self.js_mutex = threading.Lock()
        self.joint_states = None
        self.joint_actions = None
        self.robot_type = robot['type']
        self.joint_len = len(robot['joint_names'])
        self.joint_names = robot['joint_names']
        
        if gripper is None:
            self.gripper_range = robot['gripper_range']

        node.create_subscription(JointState, robot['read_topic'], self.joint_state_cb, 10)
        node.create_subscription(JointState, robot['write_topic'], self.joint_action_cb, 10)
        self.move_robot_pub = node.create_publisher(JointState, robot['write_topic'], 10)
        time.sleep(0.1)  # Wait for subscriber to be ready
            
        
    def move_step(self, action):
        action = [float(a) for a in action]
        if self.robot_type == 'ur5e':
            pass
            # self.move_step_ur5(action)
        else:
            js = JointState()
            js.name = self.joint_names
            js.position = action
            js.velocity = [0.0] * self.joint_len
            js.velocity[-1] = 100
            self.move_robot_pub.publish(js)
            
        
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg.position


    def joint_action_cb(self, msg):
        with self.js_mutex:
            self.joint_actions = msg.position


    def move_to(self, target_pos, step_size=0.1):
        if self.robot_type == 'ur5e':
            pass
            # self.move_to_ur5(target_pos)
        if self.robot_type == 'piper':
            self.move_step(target_pos)
            time.sleep(3)
        else:
            while True:
                with self.js_mutex:
                    current_pos = self.joint_states
                
                # 현재 위치와 목표 위치의 차이를 계산
                pos_diff = [target - current for target, current in zip(target_pos, current_pos)]
                
                # 목표 위치에 도달했는지 확인
                if all(abs(diff) < 0.03 for diff in pos_diff):
                    print("Reached target position.")
                    break

                # 각 관절의 위치를 step_size만큼 이동
                next_pos = [current + step_size * diff for current, diff in zip(current_pos, pos_diff)]
                
                # 이동 명령을 발행
                self.move_step(next_pos)
                time.sleep(0.1)  # 잠시 대기하여 이동이 완료될 시간을 줌