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

        self.role = robot.get('role', 'single_arm')
        self.tool_inner = robot.get('tool_inner', False)

        # self.qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT, # Hz 향상을 위해 권장
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=30 # 여기서 큐 사이즈를 결정합니다.
        # )

        self.ik_solver = None
        robot_info = get_robot_by_name(self.robot_type)
        if robot_info is not None and 'ik_setting' in robot_info:
            urdf_path = robot_info['urdf_path']
            urdf_package_dir = robot_info['urdf_package_dir']
            ik_setting = robot_info['ik_setting']
            self.ik_solver = Common_ArmIK(urdf_path=urdf_path, urdf_package_dir=urdf_package_dir, **ik_setting)
            self.ee_names = self.ik_solver.ee_names

        self.read_topic_msg_cls = get_message(robot['read_topic_msg'])
        self.read_topic_sub = node.create_subscription(self.read_topic_msg_cls, robot['read_topic'], self.joint_state_cb, 10)
        self.read_topic_msg_data = self.read_topic_msg_cls()


        self.write_type = robot.get('write_type', 'topic')  
        if self.write_type == 'topic':
            self.write_topic_msg_cls = get_message(robot['write_topic_msg'])
            self.write_topic_msg_data = self.write_topic_msg_cls()
            self.write_topic_sub = node.create_subscription(self.write_topic_msg_cls, robot['write_topic'], self.joint_action_cb, 10)
            self.move_robot_pub = node.create_publisher(self.write_topic_msg_cls, robot['write_topic'], 10)
            
        elif self.write_type == 'service':
            self.write_service_srv_cls = get_service(robot['write_topic_msg'])
            self.write_service_srv_data = None
            self.move_robot_client = node.create_client(self.write_service_srv_cls, robot['write_topic'])
            if not self.move_robot_client.wait_for_service(timeout_sec=5.0):
                print(f'Service {robot["write_topic"]} not available. Please check the connection.')

        elif self.write_type == 'action':
            self.write_action_goal_cls = get_action(robot['write_topic_msg']).Goal
            self.write_action_goal_data = self.write_action_goal_cls()
            self.move_robot_client = rclpy.action.ActionClient(node, get_action(robot['write_topic_msg']), robot['write_topic'])
            if not self.move_robot_client.wait_for_server(timeout_sec=5.0):
                print(f'Action server {robot["write_topic"]} not available. Please check the connection.')
            self.is_waiting_for_goal = False

        self.ee_pos_cmd = None

        self.joint_trajectory_point = JointTrajectoryPoint()
            
        self.moved_by_ui = False
        self.move_lock = False
        self.is_waiting_for_service = False
        time.sleep(0.1)  # Wait for subscriber to be ready


    def fetch_joint_map_to_action(self, joint_map):
        action = [0] * self.joint_len
        for joint in joint_map:
            action_index = self.joint_names.index(joint['joint_name'])
            action[action_index] = joint['target_agent_position']
        return action


    def move_joint_step(self, action, from_ee=False, velocity_arg=None):
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
        
        # clipping with joint bound
        if self.joint_upper_bounds is not None and self.joint_lower_bounds is not None:
            # numpy를 사용하여 각 관절의 인덱스에 맞는 범위를 한 번에 적용합니다.
            action = np.clip(action, self.joint_lower_bounds, self.joint_upper_bounds).tolist()

        if self.write_type == 'topic':
            self.move_joint_step_by_topic(action, velocity_arg)
        elif self.write_type == 'service':
            self.move_joint_step_by_service(action, velocity_arg)
        elif self.write_type == 'action':
            self.move_joint_step_by_action(action, velocity_arg)
        
    def move_joint_step_by_topic(self, action, vel_arg=None):
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
            # self.joint_trajectory_point.velocities = [0.0] * self.joint_len
            self.joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=0.2).to_msg()
            self.write_topic_msg_data.points = [self.joint_trajectory_point]
            self.move_robot_pub.publish(self.write_topic_msg_data)
        else:
            print("Unsupported write topic message type for move_joint_step_by_topic.")
            return
        
    def move_joint_step_by_service(self, action, vel_arg=None):
        req = self.write_service_srv_cls.Request()
        self.joint_actions = action

        if self.write_topic_msg == 'onrobot_rg_msgs/SetCommand':
            # 서비스가 준비되었는지 확인 (Blocking 하지 않음)
            if not self.move_robot_client.service_is_ready():
                print("Service is not ready, skipping command.")
                return

            req.command = int(action[0])
            
            self.move_robot_client.call_async(req)

        elif self.write_topic_msg == 'tm_msgs/srv/SendScript':
            if not self.move_robot_client.service_is_ready():
                return

            # TM Script는 도(degree) 단위를 사용하므로 라디안에서 변환
            # If there are 7 joints (6 arm + 1 gripper), use only first 6 for PTP
            arm_action = action[:6]
            angles_deg = [float(np.degrees(a)) for a in arm_action]
            
            # PTP("JPP", j1, j2, j3, j4, j5, j6, 속도%, 가속ms, 블렌딩%, 가상디지털출력)
            script = 'PTP("JPP",{},{},{},{},{},{},{},25,100,false)'.format(*[f"{a:.4f}" for a in angles_deg] + [vel_arg if vel_arg is not None else 62])  # 속도 인자 추가, 기본값은 50%
            
            # Gripper control: Append SET command to the same script string
            # Module 1 (EndEffector), Type 1 (Digital Out), Pin 0
            if len(action) > 6 and self.tool_inner:
                gripper_state = 1 if action[6] > 0.4 else 0
                script += '\r\nSET(1,1,0,{})'.format(gripper_state)
            
            req.id = '1'
            req.script = script
            self.is_waiting_for_service = True
            self.move_robot_client.call_async(req)

    def service_response_callback(self, future):
        """서비스 응답이 도착했을 때 호출되는 콜백"""
        try:
            response = future.result()
            # 성공적으로 응답을 받았으므로 다음 명령 전송 가능
            self.is_waiting_for_service = False
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            # 에러가 발생해도 플래그를 풀어줘야 다음 시도가 가능합니다.
            self.is_waiting_for_service = False

    def move_joint_step_by_action(self, action, vel_arg=None):
        # 1. 이전 명령이 아직 '수락' 대기 중이면 바로 리턴 (병목 방지)
        if self.is_waiting_for_goal:
            return

        # 2. 값의 변화가 거의 없다면 통신하지 않음 (Deadband 필터)
        # 10Hz에서 미세한 떨림으로 계속 Goal을 쏘는 것을 방지합니다.
        current_states = self.get_joint_states()
        if current_states is not None and all(abs(a - b) < 0.1 for a, b in zip(action, current_states)):
            return

        self.joint_actions = action

        # 3. 중요: 멤버 변수 대신 '로컬 변수'로 Goal 객체 생성
        # 여러 스레드나 루프에서 공유 변수를 수정하는 위험을 방지합니다.
        goal_msg = get_action(self.write_topic_msg).Goal()
        
        if self.write_topic_msg == 'control_msgs/action/GripperCommand':
            goal_msg.command.position = float(action[0])
            goal_msg.command.max_effort = 10.0 

        self.is_waiting_for_goal = True
        
        # send_goal_async 자체가 Non-blocking이므로 그대로 사용
        send_goal_future = self.move_robot_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # 서버가 Goal 수락 여부를 결정하면 즉시 플래그 해제
        self.is_waiting_for_goal = False
        
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                # Rejection 사유 확인을 위해 로그 출력
                pass 
        except Exception as e:
            print(f"Goal call failed: {e}")

        
    # def move_step(self, action):
    #     action = [float(a) for a in action]
    #     js = JointState()
    #     js.name = self.joint_names
    #     js.position = action
    #     js.velocity = [0.0] * self.joint_len
    #     js.velocity[-1] = 100
    #     self.move_robot_pub.publish(js)

    def move_joint_delta_step(self, delta_action):
        current_js = self.get_joint_states()
        if current_js is None:
            return
        target_js = [curr + delta for curr, delta in zip(current_js, delta_action)]
        self.move_joint_step(target_js)
        

    def move_ee_step(self, target_ee_dict):
        """
        입력 규격: target_ee_dict = {'L_ee': [x, y, z, r, p, y, tool], 'R_ee': [x, y, z, r, p, y, tool]}
        """
        if self.role == 'tool' or self.ik_solver is None:
            return

        self.ee_pos_cmd = target_ee_dict
        full_js = self.get_joint_states()
        arm_js, tool_js = self.get_joint_and_tool_pos(full_js)

        # 1. IK Solver 입력용 타겟 정제 및 Tool 값 별도 추출
        ik_targets = {}
        target_tool_values = {} # 추출된 tool 값 저장용

        for name in self.ee_names:
            if name in target_ee_dict:
                val_list = target_ee_dict[name]
                
                # 주석 규격에 따라 마지막 요소가 tool이라고 가정
                if len(val_list) >= 7:
                    ik_targets[name] = val_list[:6]      # [x, y, z, r, p, y]
                    target_tool_values[name] = val_list[6] # tool
                else:
                    # tool 값이 포함되지 않은 경우 기존 로직 유지
                    ik_targets[name] = val_list
                    target_tool_values[name] = None
        # 2. IK 풀기 (모든 팔을 한 번에 계산)
        # current_lr_arm_motor_q에 현재 조인트 상태를 전달하여 연속성 확보
        sol_q, _ = self.ik_solver.solve_ik(ik_targets, current_lr_arm_motor_q=np.array(arm_js))

        sol_q_fk = self.ik_solver.get_ee_position(sol_q)


        if sol_q is not None:
            # 3. 조인트 합치기 (IK 결과 + 툴 포즈)
            final_action = []
            sol_q_list = sol_q.tolist()

            if self.role == 'single_arm':
                if self.tool_inner:
                    # 추출한 tool 값이 있으면 쓰고, 없으면 현재 상태(tool_js) 유지
                    ee_name = self.ee_names[0]
                    t_val = target_tool_values.get(ee_name)
                    if t_val is None: t_val = tool_js[0]
                    
                    final_action = sol_q_list + [t_val]
                else:
                    final_action = sol_q_list

            elif self.role == 'dual_arm':
                # IK Solver의 sol_q가 [Left_Arm_Joints, Right_Arm_Joints] 순서라고 가정
                half = len(sol_q_list) // 2
                left_sol = sol_q_list[:half]
                right_sol = sol_q_list[half:]

                if self.tool_inner:
                    # 왼쪽 툴 처리
                    l_ee_name = self.ee_names[0]
                    l_tool = target_tool_values.get(l_ee_name)
                    if l_tool is None: l_tool = tool_js[0]
                    
                    # 오른쪽 툴 처리
                    r_ee_name = self.ee_names[1]
                    r_tool = target_tool_values.get(r_ee_name)
                    if r_tool is None: r_tool = tool_js[1]
                    
                    # 최종 배열 조립: [L_arm, L_tool, R_arm, R_tool]
                    final_action = left_sol + [l_tool] + right_sol + [r_tool]
                else:
                    final_action = sol_q_list

            # 4. 로봇에 명령 발행
            if final_action:
                print(final_action)
                self.move_joint_step(final_action, from_ee=True)

    def move_ee_delta_step(self, delta_ee_dict):
        if self.role == 'tool' or self.ik_solver is None:
            return

        full_js = self.get_joint_states()
        arm_js, _ = self.get_joint_and_tool_pos(full_js)

        target_ee_dict = {}
        for name in self.ee_names:
            if name in delta_ee_dict:
                # 1. Solver에게 "다음 목표 계산해줘"라고 요청
                # 여기서 frame='global' 또는 'local'을 선택할 수 있습니다.
                target_pose = self.ik_solver.compute_delta_target(
                    name, 
                    np.array(arm_js), 
                    delta_ee_dict[name][:6],
                    frame='global' 
                )
                
                # 2. 툴(그리퍼) 값 처리
                if len(delta_ee_dict[name]) >= 7:
                    # 현재 툴 값 + 델타 툴 (툴은 단순 덧셈 가능)
                    _, tool_js = self.get_joint_and_tool_pos(full_js)
                    target_pose.append(tool_js[0] + delta_ee_dict[name][6])
                
                target_ee_dict[name] = target_pose

        # 3. 계산된 절대 좌표 타겟으로 이동 명령
        self.move_ee_step(target_ee_dict)
        
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
                    try:
                        topic_index = self.joint_states.name.index(joint_name)
                        joint_positions.append(self.joint_states.position[topic_index])
                    except (ValueError, AttributeError):
                        joint_positions.append(0.0)
            elif self.read_topic_msg == 'control_msgs/JointTrajectoryControllerState':
                for i, joint_name in enumerate(self.joint_names):
                    topic_index = self.joint_states.joint_names.index(joint_name)
                    joint_positions.append(self.joint_states.actual.positions[topic_index])
            elif self.read_topic_msg == 'tm_msgs/msg/FeedbackState':
                # TM FeedbackState: joint_pos (6 arm joints) + optional gripper from DI
                joint_positions = list(self.joint_states.joint_pos)
                if len(self.joint_names) > len(joint_positions) and self.tool_inner:
                    if len(self.joint_states.ee_digital_input) > 0:
                        # Map DI 0 to 0.0 (Open) or 0.85 (Closed)
                        gripper_val = 0.85 if self.joint_states.ee_digital_input[0] == 1 else 0.0
                        joint_positions.append(gripper_val)
                    else:
                        joint_positions.append(0.0)
            else:
                for i in range(self.joint_len):
                    if hasattr(self.joint_states, 'data'):
                        joint_positions.append(self.joint_states.data[i])
                    else:
                        joint_positions.append(0.0)
        return joint_positions
    
    def get_joint_actions(self):
        if self.joint_actions is None:
            return None
        if self.write_type == 'topic':
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
            elif self.write_topic_msg == 'trajectory_msgs/JointTrajectory':
                if self.joint_actions.points:
                    point = self.joint_actions.points[-1]  # 가장 최근 포인트 사용
                    for i, joint_name in enumerate(self.joint_names):
                        joint_actions.append(point.positions[i])
        elif self.write_type == 'action' or self.write_type == 'service':
            joint_actions = self.joint_actions if self.joint_actions is not None else self.get_joint_states()
        return joint_actions

    def get_ee_position(self):
        """현재 상태의 EE 포즈 + 그리퍼 상태 반환"""
        if self.role == 'tool' or self.ik_solver is None:
            return None

        full_js = self.get_joint_states()
        arm_js, tool_js = self.get_joint_and_tool_pos(full_js)
        
        # IK Solver에서 FK 계산 (결과: {'L_ee': [x,y,z,r,p,y], ...})
        if arm_js is None:
            return None
        
        ee_poses = self.ik_solver.get_ee_position(arm_js)
        
        # 툴 상태를 딕셔너리에 병합
        if tool_js is not None:
            for i, name in enumerate(self.ee_names):
                if i < len(tool_js):
                    ee_poses[name].append(tool_js[i])
        
        return ee_poses
        
    def get_ee_target(self):
        """
        목표 End-Effector 포즈를 반환 (규격: {'L_ee': [x, y, z, r, p, y, tool]})
        """
        if self.role == 'tool':
            return self.get_joint_actions()
        
        if self.ik_solver is None:
            return None

        # 1. 이미 저장된 EE 명령이 있다면 그대로 반환 (이미 [x,y,z,r,p,y,tool] 형태임)
        if self.ee_pos_cmd is not None:
            return self.ee_pos_cmd
        
        # 2. 저장된 명령이 없다면 현재 발행 중인 Joint Action으로부터 역계산(FK)
        actions = self.get_joint_actions()
        if actions is None:
            return None

        arm_joints, tools = self.get_joint_and_tool_pos(actions)
        
        # IK Solver를 통해 각 팔의 EE 포즈 계산 (결과: {'L_ee': [x,y,z,r,p,y], ...})
        ee_poses_dict = self.ik_solver.get_ee_position(arm_joints)
        
        if ee_poses_dict is None:
            return None

        # 3. 규격에 맞게 딕셔너리 재구성 (각 리스트 끝에 tool 값 추가)
        final_targets = {}
        for i, name in enumerate(self.ee_names):
            if name in ee_poses_dict:
                pose_list = ee_poses_dict[name] # [x, y, z, r, p, y]
                
                if tools is not None and i < len(tools):
                    t_val = tools[i]
                    # 최종 리스트 생성: [x, y, z, r, p, y, tool]
                    final_targets[name] = pose_list + [t_val]

        return final_targets
    
            
    def get_joint_and_tool_pos(self, full_joint_positions):
        """
        로봇의 전체 조인트에서 IK용 조인트와 툴(그리퍼) 조인트를 분리합니다.
        IK Solver는 reduced_model을 쓰므로, 여기서 분리된 joints가 IK 입력값과 일치해야 합니다.
        """
        if full_joint_positions is None:
            return None, None

        if self.role == 'tool':
            return None, full_joint_positions

        if self.role == 'single_arm':
            if self.tool_inner:
                return full_joint_positions[:-1], [full_joint_positions[-1]]
            return full_joint_positions, None

        elif self.role == 'dual_arm':
            mid = len(full_joint_positions) // 2
            left = full_joint_positions[:mid]
            right = full_joint_positions[mid:]
            
            if self.tool_inner:
                # 각 팔의 마지막 조인트가 툴임: [arm_joints...], [tool_joints...]
                arm_joints = left[:-1] + right[:-1]
                tool_joints = [left[-1], right[-1]]
                return arm_joints, tool_joints
            return full_joint_positions, None

        return full_joint_positions, None

    def move_to(self, target_pos, step_size=0.1, duration=5.0):
        if self.robot_company == 'Piper':
            print("Moving to target position:", target_pos)
            self.move_joint_step(target_pos)
            print("Moving to target position:", target_pos)
            time.sleep(0.5)
        elif self.robot_company == 'Kinova':
            self.write_topic_msg_data.joint_names = self.joint_names
            print("Moving to target position:", target_pos)
            self.joint_trajectory_point.positions = [float(x) for x in target_pos]
            # velocities를  0으로 설정
            self.joint_trajectory_point.velocities = [0.0] * self.joint_len
            self.joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
            self.write_topic_msg_data.points = [self.joint_trajectory_point]
            self.move_robot_pub.publish(self.write_topic_msg_data)
        elif self.robot_company == 'OMRON':
            self.move_joint_step(target_pos, velocity_arg=10)
        elif self.role == 'tool':
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


    def reset_ik_solver(self, q):
        if self.ik_solver is not None:
            # 1. home_pose에서 IK에 사용되는 조인트만 추출 (예: 7개 중 앞의 6개)
            arm_home, _ = self.get_joint_and_tool_pos(q)
            
            # dual_arm인 경우 arm_home은 [[left], [right]] 형태이므로 평탄화(flatten) 필요
            if self.role == 'dual_arm':
                # arm_home[0] + arm_home[1] 등을 통해 하나의 리스트로 만듦
                flat_arm_home = []
                for joints in arm_home:
                    flat_arm_home.extend(joints)
                self.ik_solver.reset_state(flat_arm_home)
            else:
                # single_arm인 경우 arm_home[0]이 [j1, j2, j3, j4, j5, j6] 형태
                self.ik_solver.reset_state(arm_home)
