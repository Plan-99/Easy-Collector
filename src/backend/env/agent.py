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

        self.ee_pos_cmd = None
            
        time.sleep(0.1)  # Wait for subscriber to be ready


    def fetch_joint_map_to_action(self, joint_map):
        action = [0] * self.joint_len
        for joint in joint_map:
            action_index = self.joint_names.index(joint['joint_name'])
            action[action_index] = joint['target_agent_position']
        return action


    def move_joint_step(self, action, from_ee=False):
        if not from_ee:
            self.ee_pos_cmd = None

        action = [float(a) for a in action]
        js = JointState()
        js.name = self.joint_names
        js.position = action
        js.velocity = [0.0] * self.joint_len
        js.velocity[-1] = 100
        self.move_robot_pub.publish(js)


    # def move_step(self, action):
    #     action = [float(a) for a in action]
    #     js = JointState()
    #     js.name = self.joint_names
    #     js.position = action
    #     js.velocity = [0.0] * self.joint_len
    #     js.velocity[-1] = 100
    #     self.move_robot_pub.publish(js)

    def move_ee_step(self, target_ee_pos):
        self.ee_pos_cmd = target_ee_pos
        if self.ik_solver is not None:
            current_joint_positions = self.get_joint_states()

            if len(self.tools) == 0:
                sol_q, sol_tauff = self.ik_solver.solve_ik(target_ee_pos, current_joint_positions[:-1])
                if sol_q is not None:
                    sol_q = np.append(sol_q, current_joint_positions[-1])  # Keep gripper joint unchanged
            else:
                sol_q, sol_tauff = self.ik_solver.solve_ik(target_ee_pos, current_joint_positions)
                
            self.move_joint_step(sol_q, from_ee=True)

        else:
            print("IK solver not initialized for this robot type.")

        
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg

    def joint_action_cb(self, msg):
        with self.js_mutex:
            self.joint_actions = msg

    def tool_state_cb(self, msg):
        with self.js_mutex:
            self.tool_states = msg

    def tool_action_cb(self, msg):
        with self.js_mutex:
            self.tool_actions = msg
            
    def get_joint_states(self):
        with self.js_mutex:
            if self.joint_states is None:
                return None
            joint_positions = []
            for i, joint_name in enumerate(self.joint_names):
                topic_index = self.joint_states.name.index(joint_name)
                joint_positions.append(self.joint_states.position[topic_index])
        return joint_positions
    
    def get_joint_actions(self):
        if self.joint_actions is None:
            return None
        joint_actions = []
        for i, joint_name in enumerate(self.joint_names):
            topic_index = self.joint_actions.name.index(joint_name)
            joint_actions.append(self.joint_actions.position[topic_index])
        return joint_actions

    def get_ee_position(self):
        joint_positions = self.get_joint_states()
        if joint_positions is None:
            return None
        if self.ik_solver is not None:
            ee_pos_dict = self.ik_solver.get_ee_position(joint_positions[:-1])
            return ee_pos_dict
        else:
            return None
        
    def get_ee_target(self):
        if self.ee_pos_cmd is not None:
            return self.ee_pos_cmd
        else:
            joint_actions = self.get_joint_actions()
            if joint_actions is None:
                return None
            if self.ik_solver is not None:
                ee_pos_dict = self.ik_solver.get_ee_position(joint_actions[:-1])
                return ee_pos_dict
            else:
                return None

    def move_to(self, target_pos, step_size=0.1):
        self.move_joint_step(target_pos)
        # else:
        #     while True:
        #         with self.js_mutex:
        #             current_pos = self.joint_states
                
        #         # 현재 위치와 목표 위치의 차이를 계산
        #         pos_diff = [target - current for target, current in zip(target_pos, current_pos)]
                
        #         # 목표 위치에 도달했는지 확인
        #         if all(abs(diff) < 0.03 for diff in pos_diff):
        #             print("Reached target position.")
        #             break

        #         # 각 관절의 위치를 step_size만큼 이동
        #         next_pos = [current + step_size * diff for current, diff in zip(current_pos, pos_diff)]
                
        #         # 이동 명령을 발행
        #         self.move_joint_step(next_pos)
        #         time.sleep(0.1)  # 잠시 대기하여 이동이 완료될 시간을 줌