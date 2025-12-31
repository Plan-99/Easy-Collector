from geometry_msgs.msg import Pose

from rosidl_runtime_py.utilities import get_message, get_service

import rclpy
from rclpy.node import Node
import threading
from collections import deque
import time
from ..ik_solver.pinocchio_solver.common_arm_ik import Common_ArmIK
import numpy as np

from ..configs.global_configs import get_robot_by_name

class Agent:
    def __init__(self, node: Node, robot):
        self.id = robot['id']
        self.leader_robot_preset = robot.get('leader_robot_preset', None)    
        self.js_mutex = threading.Lock()
        self.joint_states = None
        self.joint_actions = None
        self.ee_pos = None
        self.ee_target = None
        self.last_joint_update = None
        self.robot_type = robot['type']
        self.robot_company = robot['company']
        self.joint_len = len(robot['joint_names'])
        self.joint_names = robot['joint_names']
        self.joint_upper_bounds = robot['joint_upper_bounds']
        self.joint_lower_bounds = robot['joint_lower_bounds']

        self.read_topic_msg = robot['read_topic_msg']
        self.write_topic_msg = robot['write_topic_msg']
        
        self.tool_inner = robot.get('tool_inner', False)

        self.ik_solver = None
        robot_info = get_robot_by_name(self.robot_type)
        if robot_info is not None and 'ik_setting' in robot_info:
            urdf_path = robot_info['urdf_path']
            urdf_package_dir = robot_info['urdf_package_dir']
            ik_setting = robot_info['ik_setting']
            self.ik_solver = Common_ArmIK(urdf_path=urdf_path, urdf_package_dir=urdf_package_dir, **ik_setting)

        read_topic_msg_cls = get_message(robot['read_topic_msg'])
        node.create_subscription(read_topic_msg_cls, robot['read_topic'], self.joint_state_cb, 10)


        self.write_type = robot.get('write_type', 'topic')  
        if self.write_type == 'topic':
            self.write_topic_msg_cls = get_message(robot['write_topic_msg'])
            node.create_subscription(self.write_topic_msg_cls, robot['write_topic'], self.joint_action_cb, 10)
            self.move_robot_pub = node.create_publisher(self.write_topic_msg_cls, robot['write_topic'], 10)

        elif self.write_type == 'service':
            self.write_service_srv_cls = get_service(robot['write_topic_msg'])
            self.move_robot_client = node.create_client(self.write_service_srv_cls, robot['write_topic'])
            if not self.move_robot_client.wait_for_service(timeout_sec=5.0):
                print(f'Service {robot["write_topic"]} not available. Please check the connection.')

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

        try:
            if action is None:
                raise ValueError("action is None")
            # Replace None entries with 0.0 to avoid crashes from partial UI payloads
            action = [0.0 if a is None else float(a) for a in action]
            if len(action) != self.joint_len:
                print(f"[WARN] action length {len(action)} does not match joint_len {self.joint_len}; padding/truncating")
                if len(action) < self.joint_len:
                    action = action + [0.0] * (self.joint_len - len(action))
                else:
                    action = action[:self.joint_len]
        except Exception as exc:
            print(f"[ERROR] move_joint_step received invalid action {action}: {exc}")
            return

        if self.write_type == 'topic':
            self.move_joint_step_by_topic(action)
        elif self.write_type == 'service':
            self.move_joint_step_by_service(action)
        
    def move_joint_step_by_topic(self, action):
        msg = self.write_topic_msg_cls()
        if self.write_topic_msg == 'std_msgs/Float64MultiArray':
            msg.data = action
            self.move_robot_pub.publish(msg)
        elif self.write_topic_msg == 'sensor_msgs/JointState':
            msg.name = self.joint_names
            msg.position = action
            msg.velocity = [0.0] * self.joint_len
            msg.velocity[-1] = 100
            self.move_robot_pub.publish(msg)
        else:
            print("Unsupported write topic message type for move_joint_step_by_topic.")
            return
        
    def move_joint_step_by_service(self, action):
        req = self.write_service_srv_cls.Request()
        if self.write_topic_msg == 'onrobot_rg_msgs/SetCommand':
            # 서비스가 준비되었는지 확인 (Blocking 하지 않음)
            if not self.move_robot_client.service_is_ready():
                print("Service is not ready, skipping command.")
                return

            req.command = int(action[0])
            
            self.move_robot_client.call_async(req)
            
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
            current_positions = self.get_joint_states()
            current_joint_positions, tool_position = self.get_joint_and_tool_pos(current_positions)
            
            sol_q, sol_tauff = self.ik_solver.solve_ik(target_ee_pos, current_joint_positions)
            
            if sol_q is not None:
                if tool_position is not None:
                    sol_q = np.append(sol_q, tool_position)  # Keep gripper joint unchanged
                self.move_joint_step(sol_q, from_ee=True)

        else:
            print("IK solver not initialized for this robot type.")

        
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg
            try:
                import time as _t
                self.last_joint_update = _t.time()
            except Exception:
                self.last_joint_update = None

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
            if self.read_topic_msg == 'sensor_msgs/JointState':
                for i, joint_name in enumerate(self.joint_names):
                    topic_index = self.joint_states.name.index(joint_name)
                    joint_positions.append(self.joint_states.position[topic_index])
            else:
                for i in range(self.joint_len):
                    joint_positions.append(self.joint_states.data[i])
        return joint_positions
    
    def get_joint_actions(self):
        if self.joint_actions is None:
            return None
        joint_actions = []
        if self.write_topic_msg == 'sensor_msgs/JointState':
            for i, joint_name in enumerate(self.joint_names):
                topic_index = self.joint_actions.name.index(joint_name)
                joint_actions.append(self.joint_actions.position[topic_index])
        elif self.write_topic_msg == 'std_msgs/Float64MultiArray':
            joint_actions = list(self.joint_actions.data)
        return joint_actions

    def get_ee_position(self):
        positions = self.get_joint_states()
        joint_positions, _ = self.get_joint_and_tool_pos(positions)
        if joint_positions is None:
            return None
        if self.ik_solver is not None:
            ee_pos_dict = self.ik_solver.get_ee_position(joint_positions)
            return ee_pos_dict
        else:
            return None
        
    def get_ee_target(self):
        actions = self.get_joint_actions()
        joint_actions, _ = self.get_joint_and_tool_pos(actions)
        if self.ee_pos_cmd is not None:
            return self.ee_pos_cmd
        else:
            if joint_actions is None:
                return None
            if self.ik_solver is not None:
                ee_pos_dict = self.ik_solver.get_ee_position(joint_actions)
                return ee_pos_dict
            else:
                return None
            
    def get_joint_and_tool_pos(self, joint_positions):
        if joint_positions is None:
            return None, None
        if self.tool_inner:
            return joint_positions[:-1], joint_positions[-1]
        else:
            return joint_positions, None

    def move_to(self, target_pos, step_size=0.1):
        if self.robot_company == 'Piper':
            self.move_joint_step(target_pos)
        else:
            while True:
                current_pos = self.get_joint_states()
                # 현재 위치와 목표 위치의 차이를 계산
                pos_diff = [target - current for target, current in zip(target_pos, current_pos)]
                
                # 목표 위치에 도달했는지 확인
                if all(abs(diff) < 0.01 for diff in pos_diff):
                    print("Reached target position.")
                    break

                # 각 관절의 위치를 step_size만큼 이동
                next_pos = [current + step_size * diff for current, diff in zip(current_pos, pos_diff)]
                
                # 이동 명령을 발행
                self.move_joint_step(next_pos)
                time.sleep(0.01)  # 짧은 대기 시간 추가
