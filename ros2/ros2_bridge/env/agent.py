from geometry_msgs.msg import Pose

from rosidl_runtime_py.utilities import get_message

import rclpy
from rclpy.node import Node
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

        # ── Write 경로: 두 갈래만 존재 ──
        #   (a) interpolation ON  → JointState 를 ec_joint_cmd 에 publish.
        #                          그 다음은 interp_node 가 책임 (변환/200Hz 보간).
        #   (b) interpolation OFF → 사용자 정의 driver (custom robot 한정).
        #                          driver.py 가 action → 실제 transport (publish/
        #                          service/action 등) 로직을 본인이 들고 있음.
        # 빌트인 robot 은 모두 (a) 로 가는 게 권장 정책. 기존 service/action 빌트인
        # (TM/Jaka/Robotiq) 의 in-agent 분기는 SDK / driver-plugin 으로 흡수 예정
        # 이라 더 이상 agent 가 들고 있지 않는다.
        self._interp_pub = None
        self._direct_pub = None
        self._driver = None
        ns = f'/ec_robot_{robot["id"]}'

        from sensor_msgs.msg import JointState as JointStateMsg

        # custom robot 은 폼에서 사용자가 interpolation 사용 여부를 직접 고른다.
        # ON  → 우리 쪽 interpolation_node 가 200Hz 보간해서 write_topic 으로
        #       publish (write_topic_msg 가 trajectory_msgs/JointTrajectory 면
        #       JTC 호환 단일-point trajectory 로 직렬화).
        # OFF → 외부 ROS2 노드(JTC, MoveIt, 직접 작성한 컨트롤러 등)가 보간/명령
        #       처리를 책임진다는 전제 — agent 는 fallback driver 로 한 번씩만 publish.
        interpolation_on = bool(robot.get('interpolation'))

        if interpolation_on:
            interp_topic = f'{ns}/ec_joint_cmd'
            direct_topic = f'{ns}/ec_joint_cmd_direct'
            self._interp_pub = node.create_publisher(JointStateMsg, interp_topic, 10)
            self._direct_pub = node.create_publisher(JointStateMsg, direct_topic, 10)
            self._sdk_write_msg = JointStateMsg()  # move_to direct path 가 재사용
        else:
            # custom robot 의 interp OFF 모드 — module.json 에서 driver entrypoint 를
            # 찾아 dynamic load. (없으면 None 으로 남겨 — 모든 write_joints 호출이
            # NotImplemented 로 떨어지므로 호출자가 명시적으로 인지 가능.)
            try:
                from ..drivers import load_driver
                self._driver = load_driver(robot, node=node)
            except Exception as e:
                print(f"[Agent] driver load failed for {robot.get('name')}: {e}", flush=True)

        # ── Read 경로: SDK 모드 / 일반 모드 두 갈래 ──
        if self.sdk_control:
            # SDK 모드에서는 interp_node 가 SDK 로부터 상태를 읽어
            # ec_robot_<id>/interpolated_joint_cmd 로 publish 해준다.
            state_topic = f'{ns}/interpolated_joint_cmd'
            self.read_topic_msg = 'sensor_msgs/JointState'
            self.read_topic_msg_cls = JointStateMsg
            self.read_topic_sub = node.create_subscription(
                JointStateMsg, state_topic, self.joint_state_cb, 10)
            self.read_topic_msg_data = JointStateMsg()
        else:
            # 일반 모드: robot 의 실제 read_topic 을 직접 구독.
            self.read_topic_msg_cls = get_message(robot['read_topic_msg'])
            self.read_topic_sub = node.create_subscription(
                self.read_topic_msg_cls, robot['read_topic'], self.joint_state_cb, 10)
            self.read_topic_msg_data = self.read_topic_msg_cls()

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
        # move_to async 실행 + 취소 지원. is_moving 만으로 동시성 다 못 다루므로
        # 실제 작업 thread + 취소 플래그를 별도 보관.
        self._move_to_thread = None
        self._move_to_aborted = False
        time.sleep(0.1)  # Wait for subscriber to be ready

    def destroy(self):
        """ROS2 구독/퍼블리셔/driver 를 해제. write 경로가 (a) interp ON / (b)
        custom driver 두 갈래뿐이므로 정리도 단순.
        """
        if hasattr(self, 'read_topic_sub') and self.read_topic_sub is not None:
            self.node.destroy_subscription(self.read_topic_sub)
            self.read_topic_sub = None

        if hasattr(self, '_interp_pub') and self._interp_pub is not None:
            self.node.destroy_publisher(self._interp_pub)
            self._interp_pub = None

        if hasattr(self, '_direct_pub') and self._direct_pub is not None:
            self.node.destroy_publisher(self._direct_pub)
            self._direct_pub = None

        if self._driver is not None:
            try:
                self._driver.destroy()
            except Exception as e:
                print(f"[Agent] driver.destroy() raised: {e}")
            self._driver = None

    def fetch_joint_map_to_action(self, joint_map):
        action = [0] * self.joint_len
        for joint in joint_map:
            action_index = self.joint_names.index(joint['joint_name'])
            action[action_index] = joint['target_agent_position']
        return action


    def move_joint_step(self, action, from_ee=False, velocity_arg=None):
        """Joint position 한 발 송신.

        두 갈래만 존재:
          (a) interpolation ON  → JointState 를 ec_joint_cmd 에 publish.
                                  변환/200Hz smoothing 은 interp_node 가 책임.
          (b) interpolation OFF → custom robot 의 사용자 정의 driver 호출.
                                  driver 가 publish/service/action 등 transport
                                  로직을 본인이 보유.
        """
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

        # Joint bound clipping
        if self.joint_upper_bounds is not None and self.joint_lower_bounds is not None:
            action = np.clip(
                action, self.joint_lower_bounds, self.joint_upper_bounds
            ).tolist()

        self.joint_actions = action

        if self._interp_pub is not None:
            # (a) interpolation ON — 모든 빌트인 + custom(interp ON) 의 경로.
            from sensor_msgs.msg import JointState as JointStateMsg
            msg = JointStateMsg()
            msg.name = self.joint_names
            msg.position = action
            msg.velocity = [0.0] * self.joint_len
            if self.tool_inner:
                # tool 채널에 100 표식 (SDK controller / interp_node 가 인식)
                msg.velocity[-1] = 100.0
            self._interp_pub.publish(msg)
            return

        if self._driver is not None:
            # (b) interpolation OFF — custom robot 의 driver plugin.
            try:
                self._driver.write_joints(action, vel_arg=velocity_arg)
            except Exception as e:
                print(f"[ERROR] driver.write_joints failed for {self.id}: {e}")
            return

        # 둘 다 없으면 transport 미설정.
        print(f"[ERROR] agent {self.id} has no transport "
              f"(interp OFF + no driver). module.json driver 설정 확인 필요.")

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
        """move_joint_step 이 마지막으로 보낸 joint position. (refactor 후 항상
        list[float]). 명령이 한 번도 발행 안 됐으면 현재 joint_states 로 대체.

        write_type / write_topic_msg 별 분기는 더 이상 필요 없음 — agent 가
        보유하는 transport 가 (a) interp_pub publish 또는 (b) custom driver
        호출 두 가지로 단일화됐고, 둘 다 list[float] 만 다룬다.
        """
        if self.joint_actions is not None:
            return self.joint_actions
        return self.get_joint_states()

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

    def move_to(self, target_pos, duration=5.0, hz=100):
        """비동기 — 별도 thread 에서 현재 자세 → target_pos 까지 hz 주기로
        선형 보간된 명령을 보낸다. duration 초 안에 도달.

        호출 즉시 return. 호출자는 (a) `agent.is_moving` 폴링으로 완료 감지,
        또는 (b) `agent.cancel_move_to()` 로 중단.

        모든 로봇 종류에 대해 동일 경로 — 보간 ON 일 때는 `move_joint_step` 이
        `_interp_pub` 으로 라우팅 → interp_node 가 200Hz 로 추가 smoothing.
        보간 OFF 일 때는 `move_joint_step` 이 직접 write_topic 으로 발행.
        어느 쪽이든 hz 의 small step 명령이 흘러간다.

        Args:
            target_pos: 목표 joint position (rad). 길이 = joint_len.
            duration: 도달 목표 시간 (초). 기본 5s.
            hz: 명령 송신 주기. 기본 100Hz (interp_node 의 200Hz publish_rate
                보다 낮게 — interp_node 가 그 사이를 다시 메우게).
        """
        target_pos = [float(x) for x in target_pos]
        duration = float(duration) if duration and duration > 0 else 5.0
        hz = float(hz) if hz and hz > 0 else 100.0

        # 이미 다른 move_to 가 도는 중이면 그걸 먼저 취소하고 새 motion 시작.
        if self.is_moving:
            self.cancel_move_to()
            # join 잠깐 기다렸다 새로 시작 (전 thread 가 cleanup 할 시간).
            if self._move_to_thread is not None:
                self._move_to_thread.join(timeout=0.5)

        # role=tool (그리퍼 등) 은 SDK/하드웨어가 자체 velocity profile 로 동작 →
        # 소프트웨어 보간이 오히려 Modbus 트래픽만 낭비한다. target 을 한 번에 송신.
        if self.role == 'tool':
            self._move_target = list(target_pos)
            self.move_joint_step(target_pos)
            return

        self._move_target = list(target_pos)
        self._move_to_aborted = False
        self.is_moving = True

        def _runner():
            try:
                current_pos = self.get_joint_states()
                if current_pos is None:
                    print("[move_to] joint_states 없음 — 중단")
                    return
                current_pos = [float(x) for x in current_pos]
                # 차원이 안 맞으면 가장 안전한 선택은 중단.
                if len(current_pos) != len(target_pos):
                    print(f"[move_to] dim mismatch current={len(current_pos)} "
                          f"target={len(target_pos)} — 중단")
                    return

                period = 1.0 / hz
                n_steps = max(1, int(round(duration * hz)))
                deltas = [t - c for t, c in zip(target_pos, current_pos)]

                for step in range(1, n_steps + 1):
                    if self._move_to_aborted:
                        print(f"[move_to] aborted at step {step}/{n_steps}")
                        return
                    t = step / n_steps
                    # smoothstep ease-in-out: 양 끝점 속도 0, 중간에서 최대 속도
                    s = t * t * (3.0 - 2.0 * t)
                    next_pos = [c + s * d for c, d in zip(current_pos, deltas)]
                    self.move_joint_step(next_pos)
                    time.sleep(period)
            finally:
                self.is_moving = False
                self._move_to_aborted = False

        self._move_to_thread = threading.Thread(
            target=_runner, name=f"move_to_{self.id}", daemon=True
        )
        self._move_to_thread.start()

    def cancel_move_to(self):
        """진행 중인 move_to thread 에 abort 신호. is_moving 플래그가 곧 False
        로 떨어진다. thread 가 없거나 이미 끝났으면 no-op."""
        self._move_to_aborted = True


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
