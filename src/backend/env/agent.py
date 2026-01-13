from geometry_msgs.msg import Pose

from rosidl_runtime_py.utilities import get_message, get_service, get_action

import rclpy
from rclpy.node import Node
import rclpy.action
import threading
from collections import deque
import time
from ..ik_solver.pinocchio_solver.common_arm_ik import Common_ArmIK
import numpy as np

from ..configs.global_configs import get_robot_by_name

from trajectory_msgs.msg import JointTrajectoryPoint

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Agent:
    def __init__(self, node: Node, robot):
        self.node = node
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


        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Hz 향상을 위해 권장
            history=HistoryPolicy.KEEP_LAST,
            depth=30 # 여기서 큐 사이즈를 결정합니다.
        )

        self.ik_solver = None
        robot_info = get_robot_by_name(self.robot_type)
        if robot_info is not None and 'ik_setting' in robot_info:
            urdf_path = robot_info['urdf_path']
            urdf_package_dir = robot_info['urdf_package_dir']
            ik_setting = robot_info['ik_setting']
            self.ik_solver = Common_ArmIK(urdf_path=urdf_path, urdf_package_dir=urdf_package_dir, **ik_setting)

        self.read_topic_msg_cls = get_message(robot['read_topic_msg'])
        self.read_topic_sub = node.create_subscription(self.read_topic_msg_cls, robot['read_topic'], self.joint_state_cb, self.qos_profile)
        self.read_topic_msg_data = self.read_topic_msg_cls()


        self.write_type = robot.get('write_type', 'topic')  
        if self.write_type == 'topic':
            self.write_topic_msg_cls = get_message(robot['write_topic_msg'])
            self.write_topic_msg_data = self.write_topic_msg_cls()
            self.write_topic_sub = node.create_subscription(self.write_topic_msg_cls, robot['write_topic'], self.joint_action_cb, self.qos_profile)
            self.move_robot_pub = node.create_publisher(self.write_topic_msg_cls, robot['write_topic'], self.qos_profile)
            
        elif self.write_type == 'service':
            self.write_service_srv_cls = get_service(robot['write_topic_msg'])
            self.write_service_srv_data = self.write_service_srv_cls()
            self.move_robot_client = node.create_client(self.write_service_srv_cls, robot['write_topic'])
            if not self.move_robot_client.wait_for_service(timeout_sec=5.0):
                print(f'Service {robot["write_topic"]} not available. Please check the connection.')

        elif self.write_type == 'action':
            self.write_action_goal_cls = get_action(robot['write_topic_msg']).Goal
            self.write_action_goal_data = self.write_action_goal_cls()
            self.move_robot_client = rclpy.action.ActionClient(node, get_action(robot['write_topic_msg']), robot['write_topic'])
            if not self.move_robot_client.wait_for_server(timeout_sec=5.0):
                print(f'Action server {robot["write_topic"]} not available. Please check the connection.')

        self.ee_pos_cmd = None

        self.joint_trajectory_point = JointTrajectoryPoint()
            
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
        elif self.write_type == 'action':
            self.move_joint_step_by_action(action)
        
    def move_joint_step_by_topic(self, action):
        if self.write_topic_msg == 'std_msgs/Float64MultiArray':
            self.write_topic_msg_data.data = action
            self.move_robot_pub.publish(self.write_topic_msg_data)
        elif self.write_topic_msg == 'sensor_msgs/JointState':
            self.write_topic_msg_data.name = self.joint_names
            self.write_topic_msg_data.position = action
            self.write_topic_msg_data.velocity = [0.0] * self.joint_len
            self.write_topic_msg_data.velocity[-1] = 100
            self.move_robot_pub.publish(self.write_topic_msg_data)
        elif self.write_topic_msg == 'trajectory_msgs/JointTrajectory':
            self.write_topic_msg_data.joint_names = self.joint_names
            self.joint_trajectory_point.positions = action
             # velocities를 목적지 - 현재 위치 차이의 절반으로 설정
            current_pos = self.get_joint_states()
            if current_pos is None:
                print("Current joint states are None, cannot move.")
                return
            self.joint_trajectory_point.velocities = [(abs(t - c) / 2) for t, c in zip(action, current_pos)]
            self.joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=0.7).to_msg()
            self.write_topic_msg_data.points = [self.joint_trajectory_point]
            self.move_robot_pub.publish(self.write_topic_msg_data)
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

    def move_joint_step_by_action(self, action):
        action = [float(a) for a in action]
        if self.write_topic_msg == 'control_msgs/action/GripperCommand':
            self.write_action_goal_data.command.position = action[0]
            self.write_action_goal_data.command.max_effort = 50.0  # Set a default max effort; adjust as needed
        send_goal_future = self.move_robot_client.send_goal_async(self.write_action_goal_data)
            
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
            elif self.read_topic_msg == 'control_msgs/JointTrajectoryControllerState':
                for i, joint_name in enumerate(self.joint_names):
                    topic_index = self.joint_states.joint_names.index(joint_name)
                    joint_positions.append(self.joint_states.actual.positions[topic_index])
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
        elif self.write_topic_msg == 'control_msgs/JointTrajectoryControllerState':
            for i, joint_name in enumerate(self.joint_names):
                topic_index = self.joint_actions.joint_names.index(joint_name)
                joint_actions.append(self.joint_actions.actual.positions[topic_index])
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
        elif self.robot_company == 'Kinova':
            self.write_topic_msg_data.joint_names = self.joint_names
            print("Moving to target position:", target_pos)
            self.joint_trajectory_point.positions = [float(x) for x in target_pos]
            # velocities를  0으로 설정
            self.joint_trajectory_point.velocities = [0.0] * self.joint_len
            self.joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=5.0).to_msg()
            self.write_topic_msg_data.points = [self.joint_trajectory_point]
            self.move_robot_pub.publish(self.write_topic_msg_data)
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

    def cleanup(self):
        """ROS 2 리소스를 안전하게 파괴합니다. 중복 호출 방지 로직 포함."""
        print(f"[CLEANUP] Agent {self.id} 리소스 해제 중...")
        
        # 1. Subscription 해제
        if hasattr(self, 'read_topic_sub') and self.read_topic_sub:
            try:
                self.node.destroy_subscription(self.read_topic_sub)
            except Exception: pass
            self.read_topic_sub = None # 중복 실행 방지
        if hasattr(self, 'write_topic_sub') and self.write_topic_sub:
            try:
                self.node.destroy_subscription(self.write_topic_sub)
            except Exception: pass
            self.write_topic_sub = None
        # 2. Publisher 해제
        if hasattr(self, 'move_robot_pub') and self.move_robot_pub:
            try:
                self.node.destroy_publisher(self.move_robot_pub)
            except Exception: pass
            self.move_robot_pub = None

        # 3. Service Client 해제
        if self.write_type == 'service' and self.move_robot_client:
            try:
                self.node.destroy_client(self.move_robot_client)
            except Exception: pass
            self.move_robot_client = None

        # 4. Action Client 해제 (가장 빈번한 에러 발생 지점)
        if self.write_type == 'action' and self.move_robot_client:
            try:
                # 로컬 변수에 저장 후 전역 변수 초기화로 레이스 컨디션 방지
                client = self.move_robot_client
                self.move_robot_client = None 
                client.destroy()
            except ValueError as e:
                # "list.remove(x): x not in list" 에러 무시
                print(f"[DEBUG] ActionClient 이미 제거됨: {e}")
            except Exception as e:
                print(f"[DEBUG] ActionClient 제거 중 오류: {e}")

        print(f"[CLEANUP] Agent {self.id} 리소스 정리 완료.")

