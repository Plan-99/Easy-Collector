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


from trajectory_msgs.msg import JointTrajectoryPoint

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def _apply_ee_offset_override(ee_definitions, user_ee_offset):
    """ee_definitions의 offset(3번째 항목)을 user override로 교체.

    ee_definitions: [(name, parent, offset), ...] — offset은 None / np.ndarray / list
    user_ee_offset: {ee_name: [x, y, z]}
    같은 ee_name에 override가 있으면 np.array로 변환해 적용.
    """
    if not user_ee_offset:
        return ee_definitions
    result = []
    for item in ee_definitions:
        name = item[0]
        parent = item[1]
        offset = item[2] if len(item) > 2 else None
        if name in user_ee_offset:
            override = user_ee_offset[name]
            if override is None:
                offset = None
            else:
                offset = np.array(override).T
        result.append((name, parent, offset))
    return result


class Agent:
    def __init__(self, node: Node, robot):
        self.node = node
        self.id = robot['id']
        self.leader_robot_preset = robot.get('leader_robot_preset', None)    
        self.js_mutex = threading.Lock()
        self.joint_states = None
        self.joint_actions = None
        self.joint_vel = None
        self.joint_effort = None
        self.ee_pos = None
        self.ee_target = None
        self.last_joint_update = None
        self.robot_type = robot['type']
        self.robot_company = robot['company']
        self.joint_len = len(robot['joint_names'])
        self.joint_names = robot['joint_names']
        self.joint_upper_bounds = robot['joint_upper_bounds']
        self.joint_lower_bounds = robot['joint_lower_bounds']
        self.is_sim = robot['is_sim']

        self.read_topic_msg = robot.get('read_topic_msg', '')
        self.write_topic_msg = robot.get('write_topic_msg', '')
        self.sdk_control = robot.get('sdk_control', False)

        self.role = robot.get('role') or 'single_arm'
        self.tool_inner = robot.get('tool_inner', False)

        self.ik_solver = None
        self.ik_lock = threading.RLock()
        # 사용자가 frontend에서 편집한 ee_offset 오버라이드. 형식: {ee_name: [x, y, z]}
        user_ee_offset = (robot.get('settings', {}) or {}).get('ee_offset') or {}
        # ros2 전용 robot_configs에서 IK 설정 조회
        from ..configs.robot_configs import get_robot_config
        robot_config = get_robot_config(self.robot_type)
        if robot_config and 'ik_setting' in robot_config:
            urdf_path = robot_config['urdf_path']
            urdf_package_dir = robot_config.get('urdf_package_dir', '')
            ik_setting = dict(robot_config['ik_setting'])
            ik_setting['ee_definitions'] = _apply_ee_offset_override(
                ik_setting.get('ee_definitions', []), user_ee_offset
            )
            self.ik_solver = Common_ArmIK(urdf_path=urdf_path, urdf_package_dir=urdf_package_dir, **ik_setting)
            self.ee_names = self.ik_solver.ee_names
        elif self.robot_type == 'custom':
            settings = robot.get('settings', {}) or {}
            if 'ik_setting' in settings:
                urdf_path = settings['urdf_path']
                urdf_package_dir = settings.get('urdf_package_dir', '')
                ik_setting = dict(settings['ik_setting'])
                # JSON에서 list로 들어온 ee_definitions를 tuple로 변환하고, offset을 np.array로 변환
                if 'ee_definitions' in ik_setting:
                    converted = []
                    for item in ik_setting['ee_definitions']:
                        name, parent, offset = item[0], item[1], item[2] if len(item) > 2 else None
                        if isinstance(offset, list):
                            offset = np.array(offset).T
                        converted.append((name, parent, offset))
                    ik_setting['ee_definitions'] = converted
                ik_setting['ee_definitions'] = _apply_ee_offset_override(
                    ik_setting['ee_definitions'], user_ee_offset
                )
                self.ik_solver = Common_ArmIK(urdf_path=urdf_path, urdf_package_dir=urdf_package_dir, **ik_setting)
                self.ee_names = self.ik_solver.ee_names

        # 보간 노드용 publisher: interpolation=True이면 ec_joint_cmd로 보냄
        self._interp_pub = None
        self._direct_pub = None  # 보간 우회 직접 명령 (move_to 등)
        ns = f'/ec_robot_{robot["id"]}'

        if robot.get('interpolation'):
            from sensor_msgs.msg import JointState as JointStateMsg
            interp_topic = f'{ns}/ec_joint_cmd'
            direct_topic = f'{ns}/ec_joint_cmd_direct'
            self._interp_pub = node.create_publisher(JointStateMsg, interp_topic, 10)
            self._direct_pub = node.create_publisher(JointStateMsg, direct_topic, 10)

        if self.sdk_control:
            # ── SDK 모드 ──
            # 보간 노드가 SDK로 제어+상태읽기. Agent는 ec_joint_cmd로 명령, state 토픽 구독.
            from sensor_msgs.msg import JointState as JointStateMsg
            state_topic = f'{ns}/interpolated_joint_cmd'
            self.read_topic_msg = 'sensor_msgs/JointState'
            self.read_topic_msg_cls = JointStateMsg
            self.read_topic_sub = node.create_subscription(
                JointStateMsg, state_topic, self.joint_state_cb, 10)
            self.read_topic_msg_data = JointStateMsg()
            self.write_type = 'sdk'
            self._sdk_write_msg = JointStateMsg()
        else:
            # ── 기존 ROS2 토픽/서비스/액션 모드 ──
            self.read_topic_msg_cls = get_message(robot['read_topic_msg'])
            self.read_topic_sub = node.create_subscription(
                self.read_topic_msg_cls, robot['read_topic'], self.joint_state_cb, 10)
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
        self.last_ee_delta = None  # keyboard 모드에서 raw EE delta 저장

        self.joint_trajectory_point = JointTrajectoryPoint()

        self.moved_by_ui = False
        self.move_lock = False
        self.is_moving = False
        self._move_target = None
        self._move_threshold = 0.01
        self.is_waiting_for_service = False
        time.sleep(0.1)  # Wait for subscriber to be ready

    def destroy(self):
        """ROS2 구독/퍼블리셔/클라이언트를 해제하여 리소스 누수를 방지한다."""
        if hasattr(self, 'read_topic_sub') and self.read_topic_sub is not None:
            self.node.destroy_subscription(self.read_topic_sub)
            self.read_topic_sub = None

        if hasattr(self, '_interp_pub') and self._interp_pub is not None:
            self.node.destroy_publisher(self._interp_pub)
            self._interp_pub = None

        if hasattr(self, '_direct_pub') and self._direct_pub is not None:
            self.node.destroy_publisher(self._direct_pub)
            self._direct_pub = None

        if self.write_type == 'sdk':
            pass  # SDK 모드: 위에서 이미 정리됨
        elif self.write_type == 'topic':
            if hasattr(self, 'write_topic_sub') and self.write_topic_sub is not None:
                self.node.destroy_subscription(self.write_topic_sub)
                self.write_topic_sub = None
            if hasattr(self, 'move_robot_pub') and self.move_robot_pub is not None:
                self.node.destroy_publisher(self.move_robot_pub)
                self.move_robot_pub = None
        elif self.write_type == 'service':
            if hasattr(self, 'move_robot_client') and self.move_robot_client is not None:
                self.node.destroy_client(self.move_robot_client)
                self.move_robot_client = None
        elif self.write_type == 'action':
            if hasattr(self, 'move_robot_client') and self.move_robot_client is not None:
                self.move_robot_client.destroy()
                self.move_robot_client = None

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

        if self.write_type == 'sdk':
            # SDK 모드: ec_joint_cmd → 보간 노드 → SDK
            self._sdk_write_msg.name = self.joint_names
            self._sdk_write_msg.position = action
            self._sdk_write_msg.velocity = [0.0] * self.joint_len
            if self.tool_inner:
                self._sdk_write_msg.velocity[-1] = 100.0
            self.joint_actions = action
            self._interp_pub.publish(self._sdk_write_msg)
            return
        elif self.write_type == 'topic':
            self.move_joint_step_by_topic(action, velocity_arg)
        elif self.write_type == 'service':
            self.move_joint_step_by_service(action, velocity_arg)
        elif self.write_type == 'action':
            self.move_joint_step_by_action(action, velocity_arg)
        
    def move_joint_step_by_topic(self, action, vel_arg=None):
        # 보간 ON 일 때는 출력 msg_type 과 무관하게 ec_joint_cmd(JointState) 로
        # 한 번에 통일해 흘려보낸다 — interp_node 가 200Hz 보간 후 사용자
        # write_topic_msg 형식에 맞춰 변환/발행한다. write_topic 으로 직접 쏘는
        # 경로는 보간 OFF 일 때만 사용.
        if self._interp_pub is not None:
            from sensor_msgs.msg import JointState as JointStateMsg
            interp_msg = JointStateMsg()
            interp_msg.name = self.joint_names
            interp_msg.position = list(action)
            interp_msg.velocity = [0.0] * self.joint_len
            self._interp_pub.publish(interp_msg)
            return

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
            second = 0 if vel_arg is None else vel_arg
            self.joint_trajectory_point.velocities = []
            self.joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=second).to_msg()
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
            if self.robot_type == 'tm_12':
                if vel_arg is None:
                    curr_joints = self.get_joint_states()
                    if curr_joints is None:
                        vel_arg = 10
                    else:
                        scale_factor = 3
                        max_speeds_deg = np.array([180, 180, 180, 225, 225, 225])
                        # 3. 이동할 거리 계산 (도 단위)
                        arm_action = action[:6]
                        target_deg = np.degrees(arm_action)
                        curr_deg = np.degrees(curr_joints[:6])
                        diff_deg = np.abs(target_deg - curr_deg)

                        # 4. 0.1초 내에 도달하기 위해 필요한 속도 비율(%) 계산
                        # 공식: (거리 / 시간) / 최대속도 * 100
                        # 25ms의 가속 시간(acc_ms)을 고려하면 실제 가용 시간은 더 짧아질 수 있습니다.
                        required_speed_pct = (diff_deg / 0.1) / max_speeds_deg * scale_factor * 100

                        # 5. 모든 관절 중 가장 큰 비율을 선택하고 1~100 사이로 제한
                        vel_arg = int(np.max(required_speed_pct))
                        vel_arg = max(1, min(100, vel_arg))

                script = 'PTP("JPP",{},{},{},{},{},{},{},25,100,false)'.format(*[f"{a:.4f}" for a in angles_deg] + [vel_arg])  # 속도 인자 추가, 기본값은 50%
                
                # Gripper control: Append SET command to the same script string
                # Module 1 (EndEffector), Type 1 (Digital Out), Pin 0
                if len(action) > 6 and self.tool_inner:
                    gripper_state = 1 if action[6] > 0.4 else 0
                    script += '\r\nSET(1,1,0,{})'.format(gripper_state)
                
                req.id = '1'
                req.script = script
                self.is_waiting_for_service = True
                self.move_robot_client.call_async(req)

            elif self.robot_type == 'tm_12s':
                script = 'Position({},{},{},{},{},{})'.format(*[f"{a:.4f}" for a in angles_deg])
                req.id = '1'
                req.script = script
                self.move_robot_client.call_async(req)


        elif self.write_topic_msg == 'fairino_msgs/srv/RemoteCmdInterface':
            if not self.move_robot_client.service_is_ready():
                print("Fairino command service is not ready, skipping command.")
                return

            # ServoJ 모드 시작 (최초 1회)
            if not getattr(self, '_fairino_servo_started', False):
                req_start = self.write_service_srv_cls.Request()
                req_start.cmd_str = 'ServoMoveStart()'
                future = self.move_robot_client.call_async(req_start)
                # spin_until_future_complete 대신 future 직접 대기 (executor 충돌 방지)
                timeout = time.time() + 1.0
                while not future.done() and time.time() < timeout:
                    time.sleep(0.01)
                self._fairino_servo_started = True

            # Fairino SDK는 도(degree) 단위를 사용하므로 라디안에서 변환
            angles_deg = [float(np.degrees(a)) for a in action]
            jnt_str = ','.join([f"{a:.4f}" for a in angles_deg])

            req = self.write_service_srv_cls.Request()
            req.cmd_str = f'ServoJ({jnt_str},0,0,0.008,0,0)'
            self.move_robot_client.call_async(req)

        elif self.write_topic_msg == 'jaka_msgs/srv/ServoMove':
            if not self.move_robot_client.service_is_ready():
                print("JAKA servo_j service is not ready, skipping command.")
                return
            
            current_pos = self.get_joint_states()
            if current_pos is None:
                print("JAKA agent cannot get current joint states, skipping servo command.")
                return
            
            # servo_j는 증분(increment) 값을 받으므로 delta를 계산
            delta = np.array(action) - np.array(current_pos)
            
            req.pose = delta.tolist()
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

        if self.write_topic_msg == 'control_msgs/action/GripperCommand':
            move_threshold = vel_arg if vel_arg is not None else 0.08
            if  current_states[0] < 0.7 and current_states is not None and all(abs(a - b) < move_threshold for a, b in zip(action, current_states)):
                return
            
            if current_states[0] >= 0.7 and current_states is not None and all(abs(a - b) < move_threshold / 3 for a, b in zip(action, current_states)):
                return
            
            if current_states[0] > 0.78 and action[0] > 0.78:
                return
        
        self.joint_actions = action

        # 3. 중요: 멤버 변수 대신 '로컬 변수'로 Goal 객체 생성
        # 여러 스레드나 루프에서 공유 변수를 수정하는 위험을 방지합니다.
        goal_msg = get_action(self.write_topic_msg).Goal()

        if self.write_topic_msg == 'control_msgs/action/GripperCommand':
            goal_msg.command.position = float(action[0])
            goal_msg.command.max_effort = 1.0

        elif self.write_topic_msg == 'control_msgs/action/FollowJointTrajectory':
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            seconds = 0 if vel_arg is None else vel_arg

            point = JointTrajectoryPoint()
            point.positions = [float(a) for a in action]
            point.velocities = []
            point.time_from_start = rclpy.duration.Duration(seconds=seconds).to_msg()

            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            traj.points = [point]

            goal_msg.trajectory = traj

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
        

    def move_ee_step(self, target_ee_dict, vel_arg=None):
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
        with self.ik_lock:
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
                self.move_joint_step(final_action, from_ee=True, velocity_arg=vel_arg)

    def compute_fk_delta(self, qaction, qpos):
        """FK(commanded) - FK(actual): 로봇이 이번 스텝에서 이동해야 할 EE 변위."""
        if self.ik_solver is None or qaction is None or qpos is None:
            return None
        arm_action, _ = self.get_joint_and_tool_pos(qaction)
        arm_state, _ = self.get_joint_and_tool_pos(qpos)
        if arm_action is None or arm_state is None:
            return None
        with self.ik_lock:
            return self.ik_solver.compute_fk_delta(arm_action, arm_state)

    def move_ee_delta_step(self, delta_ee_dict, vel_arg=None, tool_positions=None):
        """EE delta를 적용하여 로봇을 이동.

        tool_positions: 제공 시 tool joint를 해당 절대 위치로 이동 (inference용).
                        None이면 delta_ee_dict의 7번째 이후 값을 delta로 사용 (keyboard teleop용).
        """
        if self.role == 'tool' or self.ik_solver is None:
            return

        self.last_ee_delta = {
            name: list(delta_ee_dict[name][:6])
            for name in self.ee_names
            if name in delta_ee_dict
        }

        full_js = self.get_joint_states()
        arm_js, _ = self.get_joint_and_tool_pos(full_js)

        target_ee_dict = {}
        with self.ik_lock:
            for name in self.ee_names:
                if name in delta_ee_dict:
                    target_pose = self.ik_solver.compute_delta_target(
                        name,
                        np.array(arm_js),
                        delta_ee_dict[name][:6],
                        frame='global'
                    )

                    # 툴(그리퍼) 값 처리
                    if tool_positions is not None:
                        target_pose.extend(tool_positions)
                    elif len(delta_ee_dict[name]) >= 7:
                        _, tool_js = self.get_joint_and_tool_pos(full_js)
                        target_pose.append(tool_js[0] + delta_ee_dict[name][6])

                    target_ee_dict[name] = target_pose

        # 계산된 절대 좌표 타겟으로 이동 명령
        print(f"Moving EE with delta step. Target EE dict: {target_ee_dict}")
        self.move_ee_step(target_ee_dict, vel_arg=vel_arg)

    def move_ee_from_origin(self, origin, offset_ee_dict):
        """
        origin으로부터 offset만큼 떨어진 절대 EE 포즈를 계산하여 이동한다.

        origin:          [x, y, z, ax, ay, az] - 기준 EE 포즈 (월드 프레임, axis-angle)
        offset_ee_dict:  {ee_name: [dx, dy, dz, dax, day, daz]} - origin 기준 offset
        """
        if self.role == 'tool' or self.ik_solver is None:
            return

        target_ee_dict = {}
        with self.ik_lock:
            for name, offset in offset_ee_dict.items():
                target = self.ik_solver.compute_target_from_origin(origin, offset)
                if len(offset) >= 7:
                    target.append(offset[6])
                target_ee_dict[name] = target

        self.move_ee_step(target_ee_dict)

    def joint_state_cb(self, msg):
        try:
            with self.js_mutex:
                self.joint_states = msg
                self.last_joint_update = time.time()
                if not getattr(self, '_cb_logged', False):
                    print(f"[Fairino] joint_state_cb first call, msg type: {type(msg).__name__}")
                    self._cb_logged = True
        except Exception as e:
            print(f"[ERROR] joint_state_cb: {e}")

        if self.is_moving and self._move_target is not None:
            current = self.get_joint_states()
            if current is not None:
                diff = np.linalg.norm(np.array(self._move_target) - np.array(current))
                if diff < self._move_threshold:
                    self.is_moving = False
                    self._move_target = None

    def joint_action_cb(self, msg):
        try:
            with self.js_mutex:
                self.joint_actions = msg
        except Exception as e:
            print(f"[ERROR] joint_action_cb: {e}")

    def tool_state_cb(self, msg):
        try:
            with self.js_mutex:
                self.tool_states = msg
        except Exception as e:
            print(f"[ERROR] tool_state_cb: {e}")

    def tool_action_cb(self, msg):
        try:
            with self.js_mutex:
                self.tool_actions = msg
        except Exception as e:
            print(f"[ERROR] tool_action_cb: {e}")
            
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
            elif self.read_topic_msg in (
                'control_msgs/JointTrajectoryControllerState',
                'control_msgs/msg/JointTrajectoryControllerState',
            ):
                for i, joint_name in enumerate(self.joint_names):
                    try:
                        topic_index = self.joint_states.joint_names.index(joint_name)
                        joint_positions.append(self.joint_states.actual.positions[topic_index])
                    except (ValueError, IndexError, AttributeError):
                        joint_positions.append(0.0)
            elif self.read_topic_msg == 'fairino_msgs/msg/RobotNonrtState':
                # Fairino nonrt_state_data: 도(degree) 단위 → 라디안 변환
                joint_positions = [
                    np.radians(self.joint_states.j1_cur_pos),
                    np.radians(self.joint_states.j2_cur_pos),
                    np.radians(self.joint_states.j3_cur_pos),
                    np.radians(self.joint_states.j4_cur_pos),
                    np.radians(self.joint_states.j5_cur_pos),
                    np.radians(self.joint_states.j6_cur_pos),
                ]
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

    def get_joint_vel(self):
        with self.js_mutex:
            if self.joint_states is None:
                return [0.0] * self.joint_len
            if self.read_topic_msg == 'sensor_msgs/JointState' and len(self.joint_states.velocity) > 0:
                vel = []
                for joint_name in self.joint_names:
                    try:
                        idx = self.joint_states.name.index(joint_name)
                        vel.append(self.joint_states.velocity[idx] if idx < len(self.joint_states.velocity) else 0.0)
                    except (ValueError, IndexError):
                        vel.append(0.0)
                return vel
            if self.read_topic_msg in (
                'control_msgs/JointTrajectoryControllerState',
                'control_msgs/msg/JointTrajectoryControllerState',
            ) and len(self.joint_states.actual.velocities) > 0:
                vel = []
                for joint_name in self.joint_names:
                    try:
                        idx = self.joint_states.joint_names.index(joint_name)
                        vel.append(self.joint_states.actual.velocities[idx] if idx < len(self.joint_states.actual.velocities) else 0.0)
                    except (ValueError, IndexError):
                        vel.append(0.0)
                return vel
            return [0.0] * self.joint_len

    def get_joint_effort(self):
        with self.js_mutex:
            if self.joint_states is None:
                return [0.0] * self.joint_len
            if self.read_topic_msg == 'sensor_msgs/JointState' and len(self.joint_states.effort) > 0:
                effort = []
                for joint_name in self.joint_names:
                    try:
                        idx = self.joint_states.name.index(joint_name)
                        effort.append(self.joint_states.effort[idx] if idx < len(self.joint_states.effort) else 0.0)
                    except (ValueError, IndexError):
                        effort.append(0.0)
                return effort
            if self.read_topic_msg in (
                'control_msgs/JointTrajectoryControllerState',
                'control_msgs/msg/JointTrajectoryControllerState',
            ) and len(self.joint_states.actual.effort) > 0:
                effort = []
                for joint_name in self.joint_names:
                    try:
                        idx = self.joint_states.joint_names.index(joint_name)
                        effort.append(self.joint_states.actual.effort[idx] if idx < len(self.joint_states.actual.effort) else 0.0)
                    except (ValueError, IndexError):
                        effort.append(0.0)
                return effort
            return [0.0] * self.joint_len

    def get_joint_actions(self):
        if self.write_type == 'sdk':
            # SDK: joint_actions는 list (move_joint_step에서 직접 할당)
            return self.joint_actions if self.joint_actions is not None else self.get_joint_states()

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
            elif self.write_topic_msg in (
                'control_msgs/JointTrajectoryControllerState',
                'control_msgs/msg/JointTrajectoryControllerState',
            ):
                for i, joint_name in enumerate(self.joint_names):
                    topic_index = self.joint_actions.joint_names.index(joint_name)
                    joint_actions.append(self.joint_actions.actual.positions[topic_index])
            elif self.write_topic_msg == 'trajectory_msgs/JointTrajectory':
                if self.joint_actions.points:
                    point = self.joint_actions.points[-1]
                    for i, joint_name in enumerate(self.joint_names):
                        joint_actions.append(point.positions[i])
        elif self.write_type == 'action' or self.write_type == 'service':
            joint_actions = self.joint_actions if self.joint_actions is not None else self.get_joint_states()
        else:
            return None
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
        
        with self.ik_lock:
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
        with self.ik_lock:
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

    def move_to(self, target_pos, step_size=0.0005, duration=5.0):
        self._move_target = list(target_pos)
        self.is_moving = True

        if self.is_sim:
            print("Moving to target position:", target_pos)
            self.move_joint_step(target_pos)
            time.sleep(0.5)
        elif self.sdk_control:
            # SDK 모드: 보간 우회 직접 명령
            print("Moving to target position (SDK direct):", target_pos)
            self._sdk_write_msg.name = self.joint_names
            self._sdk_write_msg.position = [float(x) for x in target_pos]
            self._sdk_write_msg.velocity = [0.0] * self.joint_len
            if self.tool_inner:
                self._sdk_write_msg.velocity[-1] = 100.0
            self._direct_pub.publish(self._sdk_write_msg)
            time.sleep(0.5)
        elif self.robot_company == 'Piper':
            print("Moving to target position:", target_pos)
            # 보간 우회 직접 토픽으로 보냄
            self.write_topic_msg_data.name = self.joint_names
            self.write_topic_msg_data.position = [float(x) for x in target_pos]
            self.write_topic_msg_data.velocity = [0.0] * self.joint_len
            self.write_topic_msg_data.velocity[-1] = 100.0
            pub = self._direct_pub if self._direct_pub is not None else self.move_robot_pub
            pub.publish(self.write_topic_msg_data)
            time.sleep(0.5)
        elif self.robot_company == 'Kinova' or self.write_topic_msg == 'trajectory_msgs/JointTrajectory':
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
            self.move_joint_step(target_pos, velocity_arg=0)
        else:
            while True:
                current_pos = self.get_joint_states()
                # 현재 위치와 목표 위치의 차이를 계산
                pos_diff = [target - current for target, current in zip(target_pos, current_pos)]
                
                # 목표 위치에 도달했는지 확인
                if all(abs(diff) < 0.001 for diff in pos_diff):
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

            with self.ik_lock:
                # dual_arm인 경우 arm_home은 [[left], [right]] 형태이므로 평탄화(flatten) 필요
                if self.role == 'dual_arm':
                    flat_arm_home = []
                    for joints in arm_home:
                        flat_arm_home.extend(joints)
                    self.ik_solver.reset_state(flat_arm_home)
                else:
                    self.ik_solver.reset_state(arm_home)
