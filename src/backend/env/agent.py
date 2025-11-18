from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

import rclpy
from rclpy.node import Node
import threading
from collections import deque
import time
from ..ik_solver.pinocchio_solver.piper_arm_ik import Piper_ArmIK
import numpy as np

class Agent:
    def __init__(self, node: Node, robot):
        self.id = robot['id']
        self.tools = robot['tools'] if 'tools' in robot else []
        self.tool_agents = [Agent(node, tool) for tool in self.tools]
        self.leader_robot_preset = robot.get('leader_robot_preset', None)    
        self.js_mutex = threading.Lock()
        self.joint_states = None
        self.joint_actions = None
        self.ee_pos = None
        self.ee_target = None
        self.robot_type = robot['type']
        self.joint_len = len(robot['joint_names'])
        self.joint_names = robot['joint_names']
        self.joint_upper_bounds = robot['joint_upper_bounds']
        self.joint_lower_bounds = robot['joint_lower_bounds']
        
        if len(self.tools) == 0:
            self.gripper_range = [self.joint_lower_bounds[-1], self.joint_upper_bounds[-1]]
        # else:
        #     self.gripper_range = [tool['joint_lower_bounds'][0], tool['joint_upper_bounds'][0]]
             
        self.ik_solver = None
        if robot['type'] == 'piper':
            self.ik_solver = Piper_ArmIK()

        node.create_subscription(JointState, robot['read_topic'], self.joint_state_cb, 10)
        node.create_subscription(JointState, robot['write_topic'], self.joint_action_cb, 10)

        self.move_robot_pub = node.create_publisher(JointState, robot['write_topic'], 10)
            
        time.sleep(0.1)  # Wait for subscriber to be ready


    def move_step(self, joint_action, tool_actions=[]):
        self.move_step_joints(joint_action)
        for tool, tool_action in zip(self.tool_agents, tool_actions):
            tool.move_step(tool_action)

    def move_step_joints(self, action):
        action = [float(a) for a in action]
        js = JointState()
        js.name = self.joint_names
        js.position = action
        js.velocity = [0.0] * self.joint_len
        js.velocity[-1] = 100
        self.move_robot_pub.publish(js)

    def move_ee_step(self, target_ee_pos):
        if self.ik_solver is not None:
            with self.js_mutex:
                current_joint_positions = self.joint_states
            sol_q, sol_tauff = self.ik_solver.solve_ik(target_ee_pos, current_joint_positions[:-1])
            if sol_q is not None:
                print(f"IK solution found: {sol_q}")
                sol_q = np.append(sol_q, current_joint_positions[-1])  # Keep gripper joint unchanged
                self.move_step_joints(sol_q)
            else:
                print("IK solution not found for the given target EE position.")
        else:
            print("IK solver not initialized for this robot type.")

        
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg.position

    def joint_action_cb(self, msg):
        with self.js_mutex:
            self.joint_actions = msg.position

    def tool_state_cb(self, msg):
        with self.js_mutex:
            self.tool_states = msg.position

    def tool_action_cb(self, msg):
        with self.js_mutex:
            self.tool_actions = msg.position
            
    def get_joint_states(self):
        with self.js_mutex:
            joint_positions = self.joint_states
        return joint_positions
    
    def get_joint_actions(self):
        with self.js_mutex:
            joint_actions = self.joint_actions
        return joint_actions

    def get_ee_position(self):
        with self.js_mutex:
            joint_positions = self.joint_states
        if joint_positions is None:
            return None
        if self.ik_solver is not None:
            ee_pos_dict = self.ik_solver.get_ee_position(joint_positions[:-1])
            return ee_pos_dict
        else:
            return None
        
    def get_ee_target(self):
        with self.js_mutex:
            joint_actions = self.joint_actions
        if joint_actions is None:
            return None
        if self.ik_solver is not None:
            ee_target = self.ik_solver.get_ee_position(joint_actions[:-1])
            return ee_target
        else:
            return None

    def move_to(self, target_pos, step_size=0.1):
        if self.robot_type == 'ur5e':
            pass
            # self.move_to_ur5(target_pos)
        if self.robot_type == 'piper':
            self.move_step(target_pos)
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