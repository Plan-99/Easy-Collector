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
        self._waypoint_pub = None
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
            waypoint_topic = f'{ns}/ec_joint_waypoint'
            self._interp_pub = node.create_publisher(JointStateMsg, interp_topic, 10)
            self._direct_pub = node.create_publisher(JointStateMsg, direct_topic, 10)
            # 스케줄 waypoint publisher (DexUMI 스타일 absolute-time scheduling).
            # header.stamp 에 목표 도달 시각(epoch seconds) 을 담아 보낸다.
            self._waypoint_pub = node.create_publisher(JointStateMsg, waypoint_topic, 20)
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

        # Subscribe to write_topic so external publishers (e.g. a motion
        # planner in tutorial mode) populate self.joint_actions for
        # record_episode. Built-in robots with interpolation/driver also
        # publish here and the subscription just echoes their own command,
        # which is harmless (the value is identical to what move_joint_step
        # set).
        self.write_topic_sub = None
        write_topic = robot.get('write_topic')
        if write_topic and self.write_topic_msg == 'sensor_msgs/JointState':
            self.write_topic_sub = node.create_subscription(
                JointStateMsg, write_topic,
                self._external_action_cb, 10)

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

        if hasattr(self, 'write_topic_sub') and self.write_topic_sub is not None:
            self.node.destroy_subscription(self.write_topic_sub)
            self.write_topic_sub = None

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

        # role=tool (그리퍼) 은 항상 direct 경로. 하드웨어가 자체 velocity profile
        # 을 가지므로 소프트웨어 linear interp 를 거치면 Modbus write 지연으로
        # 200Hz 보간 타이머가 늘어져 interp 가 0.3s stale timeout 에 걸려 target
        # 직전에서 끊긴다. _direct_pub 이 없으면 (interp OFF) 아래 일반 경로로.
        if self.role == 'tool' and self._direct_pub is not None:
            self.move_joint_direct(action, velocity_arg=velocity_arg)
            return

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

    def move_joint_direct(self, action, velocity_arg=None):
        """보간/큐 우회 — target 을 interpolation_node 의 direct 경로로 즉시 송신.

        role=tool (그리퍼) 처럼 하드웨어가 자체 velocity profile 을 갖는 경우
        소프트웨어 linear interp 는 (a) Modbus 트래픽만 낭비하고, (b) SDK write
        지연으로 200Hz 보간 타이머가 늘어져 interp 가 0.3s stale timeout 에
        걸려 target 직전에서 끊긴다. direct 경로는 _direct_cmd_callback 이 받아
        SDK 로 target 을 한 번에 write — interp/LPF/timeout 모두 우회.

        _direct_pub 이 없으면 (custom robot interp OFF) move_joint_step 폴백.
        """
        if self._direct_pub is None:
            self.move_joint_step(action, velocity_arg=velocity_arg)
            return
        try:
            if action is None:
                raise ValueError("action is None")
            action = [0.0 if a is None else float(a) for a in action]
            if len(action) != self.joint_len:
                if len(action) < self.joint_len:
                    action = action + [0.0] * (self.joint_len - len(action))
                else:
                    action = action[:self.joint_len]
        except Exception as exc:
            print(f"[ERROR] move_joint_direct invalid action {action}: {exc}")
            return

        if self.joint_upper_bounds is not None and self.joint_lower_bounds is not None:
            action = np.clip(
                action, self.joint_lower_bounds, self.joint_upper_bounds
            ).tolist()

        self.joint_actions = action

        from sensor_msgs.msg import JointState as JointStateMsg
        msg = JointStateMsg()
        msg.name = self.joint_names
        msg.position = action
        msg.velocity = [0.0] * self.joint_len
        if self.tool_inner:
            msg.velocity[-1] = 100.0
        self._direct_pub.publish(msg)

    def move_to_joints_at(self, action, t_absolute, velocity_arg=None):
        """스케줄 waypoint 송신 — DexUMI 스타일 absolute-time scheduling.

        ``action`` joint target 을 ``t_absolute`` (epoch seconds, time.time() 기준)
        시각에 도달하도록 큐에 추가한다. interpolation_node 가 200Hz 로 큐를 보간
        해 robot 에 출력. 호출 자체는 즉시 반환 (non-blocking).

        chunk inference 결과를 N 개 한 번에 push 해두면, inference 가 100~300ms
        걸려도 robot 은 큐 안의 미래 waypoint 따라 끊김 없이 진행.

        interpolation ON 일 때만 동작 — 그 외 경로는 move_joint_step 으로 폴백.
        """
        # role=tool (그리퍼) 은 scheduled waypoint 도 쓰지 않고 항상 direct —
        # 하드웨어 자체 velocity profile. interp/큐/timeout 전부 우회.
        if self.role == 'tool':
            self.move_joint_direct(action, velocity_arg=velocity_arg)
            return

        if self._waypoint_pub is None:
            # interp OFF 인 custom robot — scheduled waypoint 미지원, 즉시 명령으로.
            self.move_joint_step(action, velocity_arg=velocity_arg)
            return

        try:
            if action is None:
                raise ValueError("action is None")
            action = [0.0 if a is None else float(a) for a in action]
            if len(action) != self.joint_len:
                if len(action) < self.joint_len:
                    action = action + [0.0] * (self.joint_len - len(action))
                else:
                    action = action[:self.joint_len]
        except Exception as exc:
            print(f"[ERROR] move_to_joints_at invalid action {action}: {exc}")
            return

        # Joint bound clipping (move_joint_step 과 동일)
        if self.joint_upper_bounds is not None and self.joint_lower_bounds is not None:
            action = np.clip(
                action, self.joint_lower_bounds, self.joint_upper_bounds
            ).tolist()

        # 마지막 명령 추적 — joint_actions 는 dataset eepos 계산에도 쓰임.
        self.joint_actions = action

        from sensor_msgs.msg import JointState as JointStateMsg
        msg = JointStateMsg()
        # header.stamp = t_absolute (epoch seconds → sec + nanosec)
        t_sec = int(t_absolute)
        t_nsec = int((t_absolute - t_sec) * 1e9)
        msg.header.stamp.sec = t_sec
        msg.header.stamp.nanosec = max(0, min(t_nsec, 999_999_999))
        msg.name = self.joint_names
        msg.position = action
        msg.velocity = [0.0] * self.joint_len
        if self.tool_inner:
            msg.velocity[-1] = 100.0
        self._waypoint_pub.publish(msg)

    def _solve_ee_to_joints(self, target_ee_dict, orientation_cost=None):
        """
        target_ee_dict 를 IK 로 풀어서 로봇에 보낼 final_action(조인트 벡터)을 반환한다.
        풀 수 없으면 None.
        입력 규격: target_ee_dict = {'L_ee': [x, y, z, r, p, y, tool], 'R_ee': [x, y, z, r, p, y, tool]}
        orientation_cost: 설정 시 이번 풀이에서만 방향 가중치를 일시 override(낮을수록
        위치 우선). 풀이 후 원래 값으로 복원해 teleop 등 다른 경로에 영향 없음.
        """
        if self.role == 'tool' or self.ik_solver is None:
            return None

        self.ee_pos_cmd = target_ee_dict
        full_js = self.get_joint_states()
        arm_js, tool_js = self.get_joint_and_tool_pos(full_js)

        # 1. IK Solver 입력용 타겟 정제 및 Tool 값 별도 추출
        ik_targets = {}
        target_tool_values = {}  # 추출된 tool 값 저장용

        for name in self.ee_names:
            if name in target_ee_dict:
                val_list = target_ee_dict[name]

                # 주석 규격에 따라 마지막 요소가 tool이라고 가정
                if len(val_list) >= 7:
                    ik_targets[name] = val_list[:6]      # [x, y, z, r, p, y]
                    target_tool_values[name] = val_list[6]  # tool
                else:
                    # tool 값이 포함되지 않은 경우 기존 로직 유지
                    ik_targets[name] = val_list
                    target_tool_values[name] = None

        # 2. IK 풀기 — pink 의 solve_ik 는 task-space velocity 를 한 step 만
        # 적분하는 incremental solver 라 한 번 호출로는 target 의 ~30-40% 만
        # 진행한다 (teleop 처럼 매 step 호출하는 경로는 누적으로 도달).
        # move_ee_to / move_ee_step 같은 one-shot 호출에선 수렴할 때까지
        # 반복 호출해야 final_action 이 진짜 target 에 해당하는 joint config 가
        # 된다. 안 그러면 robot 이 commanded delta 의 일부 거리만 이동.
        # 수렴 판정은 position only (xyz 0.1mm) — rotation 은 axis-angle 의
        # Euler wrap 으로 산술적 비교가 불안정해서 IK 가 orientation 도 함께
        # 풀고 있다는 가정 하에 위치만으로 수렴 체크. orientation 비활성 task
        # 라면 어차피 의미 없음.
        max_iters = 200
        tol_xyz = 1e-4   # 0.1 mm
        q_iter = np.array(arm_js)
        sol_q = None
        with self.ik_lock:
            # orientation_cost override (이번 풀이 한정) — 끝나면 finally 에서 복원.
            _restore_oc = None
            if orientation_cost is not None and hasattr(self.ik_solver, 'set_orientation_cost'):
                try:
                    _restore_oc = self.ik_solver.get_orientation_cost()
                    self.ik_solver.set_orientation_cost(float(orientation_cost))
                except Exception:
                    _restore_oc = None
            try:
                # Dual-arm 안전장치: 이번 호출에서 명령받지 않은 EE 는 현재 포즈로
                # target 을 고정한다. pink 의 FrameTask 는 set_target 을 호출한 EE 만
                # 갱신하므로, 한쪽 팔만 움직이는 teleop(키보드/RobotPendant) 에서
                # 반대팔 task 의 target 이 미설정(첫 호출 → solve 예외 → 전체 no-op)
                # 이거나 직전 값으로 남아 반대팔을 엉뚱하게 끌어당기는 문제가 있다.
                # 명령 안 된 EE 를 현재 FK 포즈로 잡아두면 그 팔은 제자리에 머문다.
                if len(self.ee_names) > 1:
                    held_fk = self.ik_solver.get_ee_position(list(arm_js))
                    for name in self.ee_names:
                        if name not in ik_targets and name in held_fk:
                            ik_targets[name] = held_fk[name]
                            target_tool_values.setdefault(name, None)
                for _ in range(max_iters):
                    sol_q, _ = self.ik_solver.solve_ik(
                        ik_targets, current_lr_arm_motor_q=q_iter
                    )
                    if sol_q is None:
                        break
                    q_iter = np.asarray(sol_q)
                    fk = self.ik_solver.get_ee_position(q_iter.tolist())
                    err = 0.0
                    for name, target_vec in ik_targets.items():
                        cur_vec = fk.get(name)
                        if cur_vec is None:
                            err = float('inf')
                            break
                        dx = max(abs(cur_vec[i] - target_vec[i]) for i in range(3))
                        err = max(err, dx)
                    if err < tol_xyz:
                        break
            finally:
                if _restore_oc is not None:
                    try:
                        self.ik_solver.set_orientation_cost(_restore_oc)
                    except Exception:
                        pass

        if sol_q is None:
            return None

        # 3. 조인트 합치기 (IK 결과 + 툴 포즈)
        final_action = []
        sol_q_list = sol_q.tolist()

        if self.role == 'single_arm':
            if self.tool_inner:
                ee_name = self.ee_names[0]
                t_val = target_tool_values.get(ee_name)
                if t_val is None:
                    t_val = tool_js[0]
                final_action = sol_q_list + [t_val]
            else:
                final_action = sol_q_list

        elif self.role == 'dual_arm':
            # IK Solver의 sol_q가 [Left_Arm_Joints, Right_Arm_Joints] 순서라고 가정
            half = len(sol_q_list) // 2
            left_sol = sol_q_list[:half]
            right_sol = sol_q_list[half:]

            if self.tool_inner:
                l_ee_name = self.ee_names[0]
                l_tool = target_tool_values.get(l_ee_name)
                if l_tool is None:
                    l_tool = tool_js[0]

                r_ee_name = self.ee_names[1]
                r_tool = target_tool_values.get(r_ee_name)
                if r_tool is None:
                    r_tool = tool_js[1]

                final_action = left_sol + [l_tool] + right_sol + [r_tool]
            else:
                final_action = sol_q_list

        return final_action or None

    def move_ee_step(self, target_ee_dict, vel_arg=None):
        """
        입력 규격: target_ee_dict = {'L_ee': [x, y, z, r, p, y, tool], 'R_ee': [x, y, z, r, p, y, tool]}
        """
        final_action = self._solve_ee_to_joints(target_ee_dict)
        if final_action:
            self.move_joint_step(final_action, from_ee=True, velocity_arg=vel_arg)

    def move_ee_to(self, target_ee_dict, duration=5.0, hz=100):
        """
        target_ee_dict 를 IK 로 풀어서, move_to 로 duration 초에 걸쳐 보간 이동한다.
        move_ee_step 과 달리 단발 명령이 아니라 시간 기반 보간 모션이다.
        입력 규격: target_ee_dict = {'L_ee': [x, y, z, r, p, y, tool], ...}
        예약 키 ``__orientation_cost__`` 가 있으면 이번 풀이의 방향 가중치로 쓴다
        (proto 변경 없이 backend 가 실어 보냄 — visual_reach 의 느슨한 회전 도달).
        """
        target = dict(target_ee_dict)
        orientation_cost = target.pop('__orientation_cost__', None)
        final_action = self._solve_ee_to_joints(target, orientation_cost=orientation_cost)
        if final_action:
            self.move_to(final_action, duration=duration, hz=hz)

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

    def move_ee_delta_step_at(self, delta_ee_dict, t_absolute, tool_positions=None):
        """move_ee_delta_step 의 scheduled 버전.

        IK 로 joint target 계산 → move_to_joints_at 으로 큐에 push. inference
        측에서 chunk 의 각 step waypoint 를 (delta_ee, t_absolute) 로 한 번에
        예약하는 데 사용.

        Note: 현재 robot state(arm_js) 기준으로 IK 를 풀어 절대 joint target 을
        만든다. chunk 내 후속 waypoint 들도 같은 호출 시점의 arm_js 기준이라
        모델이 학습 시 가정한 "현재 state 기준 relative trajectory" 와 부합.
        """
        if self.role == 'tool' or self.ik_solver is None:
            return

        full_js = self.get_joint_states()
        arm_js, _ = self.get_joint_and_tool_pos(full_js)

        ik_targets = {}
        with self.ik_lock:
            for name in self.ee_names:
                if name not in delta_ee_dict:
                    continue
                target_pose = self.ik_solver.compute_delta_target(
                    name,
                    np.array(arm_js),
                    delta_ee_dict[name][:6],
                    frame='global'
                )
                ik_targets[name] = target_pose[:6]  # IK 입력은 xyz+rpy 만

            if not ik_targets:
                return
            sol_q, _ = self.ik_solver.solve_ik(
                ik_targets, current_lr_arm_motor_q=np.array(arm_js)
            )

        if sol_q is None:
            print(f"[move_ee_delta_step_at] IK failed (no solution)")
            return
        target_joint_list = list(sol_q)

        # tool joint 추가 (tool_inner 면 같은 agent 가 그리퍼 joint 도 들고 있음)
        if tool_positions is not None:
            target_joint_list = target_joint_list + list(tool_positions)
        elif self.tool_inner:
            _, tool_js = self.get_joint_and_tool_pos(full_js)
            if tool_js is not None:
                target_joint_list = target_joint_list + list(tool_js)

        self.move_to_joints_at(target_joint_list, t_absolute)

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

    def _external_action_cb(self, msg):
        # JointState 가 robot.joint_names 순서가 아닐 수 있으므로 매핑.
        try:
            names = list(getattr(msg, 'name', []))
            positions = list(getattr(msg, 'position', []))
            ordered = []
            for jn in self.joint_names:
                try:
                    idx = names.index(jn)
                    ordered.append(float(positions[idx]))
                except (ValueError, IndexError):
                    ordered.append(0.0)
            with self.js_mutex:
                self.joint_actions = ordered
        except Exception as e:
            print(f"[ERROR] _external_action_cb: {e}")

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
        # 소프트웨어 보간이 오히려 Modbus 트래픽만 낭비한다. direct 경로로 target
        # 을 한 번에 송신 — interp/LPF/0.3s stale timeout 모두 우회.
        if self.role == 'tool':
            self._move_target = list(target_pos)
            self.move_joint_direct(target_pos)
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
                    # smootherstep (5차 polynomial) ease-in-out: 양 끝점에서 속도/
                    # 가속도/jerk 모두 0. 일반 smoothstep(t²(3-2t)) 보다 한 단계 더
                    # 부드럽고, ServoJ 처럼 짧은 cmdT 에 민감한 컨트롤러에서 양 끝
                    # 의 "덜컹"을 추가로 흡수. 피크 속도는 평균의 1.875배 (smoothstep
                    # 의 1.5배보다 약간 높음).
                    s = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
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
            # IK reduced-model 에 들어갈 arm joint 만 추출. get_joint_and_tool_pos
            # 는 dual_arm 에서도 [L1..L6, R1..R6] 평탄(flat) 리스트를 돌려준다
            # (left[:-1] + right[:-1]). 과거 주석은 [[left],[right]] 중첩 형태를
            # 가정해 float 를 다시 순회 → "'float' object is not iterable" 에러를
            # 냈다. 이미 평탄하므로 그대로 reset_state 에 넘긴다.
            arm_home, _ = self.get_joint_and_tool_pos(q)
            with self.ik_lock:
                self.ik_solver.reset_state(arm_home)
