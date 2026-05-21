#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Joint Command Interpolation Node.

저주파 관절 명령을 받아 보간하여 고주파로 출력함으로써
로봇의 진동/떨림을 방지한다.

출력 모드:
  - topic (기본): 보간된 명령을 ROS2 토픽으로 퍼블리시
  - sdk: 보간된 명령을 로봇 SDK로 직접 전송 + 관절 상태를 토픽으로 퍼블리시

Usage (standalone):
    ros2 run --ros-args -r __ns:=/ec_robot_1 -- python3 -m ros2_bridge.interpolation_node

Usage (as subprocess):
    driver_service.py에서 드라이버와 함께 자동 실행됨.
"""
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ros2_bridge.joint_trajectory_interpolator import JointTrajectoryInterpolator

# legacy interp 완료 후 LPF 가 target 에 이만큼 가까워지면 정착으로 보고
# publish 중단. rad / m 단위 — 충분히 작아 그리퍼/관절 모두 무해.
_LPF_SETTLE_EPS = 1e-4


class JointInterpolationNode(Node):
    """
    /ec_joint_cmd 토픽을 구독하여 선형 보간 후 출력.

    control_mode='topic': output_topic으로 퍼블리시 (기본)
    control_mode='sdk': SDK 컨트롤러로 직접 전송 + read_topic에 관절 상태 퍼블리시

    명령 간격을 측정하여, 다음 명령이 도착할 때까지 일정 속도로 목표에 도달.
    """

    def __init__(self):
        super().__init__('joint_interpolation')

        # Parameters
        self.declare_parameter('publish_rate', 200.0)
        self.declare_parameter('output_topic', 'joint_states')
        # 'sensor_msgs/JointState' (default) or 'std_msgs/Float64MultiArray'.
        # Float64MultiArray 는 position 값만 .data 배열에 그대로 publish.
        self.declare_parameter('output_msg_type', 'sensor_msgs/JointState')
        self.declare_parameter('control_mode', 'topic')     # 'topic' or 'sdk'
        self.declare_parameter('sdk_type', '')               # 'piper', 'fairino', etc.
        self.declare_parameter('sdk_can_port', 'can0')       # SDK CAN 포트 (Piper)
        self.declare_parameter('sdk_has_gripper', True)      # SDK 그리퍼 유무 (Piper)
        self.declare_parameter('sdk_ip_address', '')         # SDK IP (Fairino 등 ethernet 로봇)
        self.declare_parameter('sdk_serial_port', '')        # SDK serial 포트 (Robotiq 등 USB/RS485)
        self.declare_parameter('read_topic', 'interpolated_joint_cmd')  # SDK 모드: 상태 퍼블리시 토픽
        # Output LPF (option 3 — chunk boundary 안전망).
        # 매 publish 직전 1차 IIR low-pass: out = α * target + (1-α) * prev_out.
        # α=1.0 → 무필터 (기존 동작). α=0.5 → 강한 smoothing.
        # 200Hz timer 기준 α≈0.6 이면 ~30Hz cutoff. jerk 가 부드러워지지만 tracking
        # lag 10-15ms 추가. Direct cmd / 신호 정리 시 자동 reset 으로 lag 누적 방지.
        self.declare_parameter('output_lpf_alpha', 0.5)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_msg_type = self.get_parameter('output_msg_type').value
        self.control_mode = self.get_parameter('control_mode').value
        self._dt = 1.0 / self.publish_rate
        self._lpf_alpha = float(self.get_parameter('output_lpf_alpha').value)
        if not (0.0 < self._lpf_alpha <= 1.0):
            self._lpf_alpha = 1.0  # 안전: 잘못된 값이면 무필터
        self._lpf_state = None  # filtered position (np.ndarray). None 이면 첫 cmd 로 초기화.

        # State
        self._current_pos = None
        self._start_pos = None
        self._target_pos = None
        self._target_vel = None
        self._target_names = None
        self._has_target = False

        # 타이밍
        self._cmd_time = None
        self._cmd_interval = 0.1
        self._interp_progress = 1.0

        # SDK 컨트롤러
        self._sdk_controller = None

        # 스케줄 waypoint 큐 (DexUMI 스타일).
        # ec_joint_waypoint 토픽으로 (target_joints, header.stamp=t_absolute) 수신.
        # 큐가 비어있지 않으면 timer 가 큐 보간을 우선 사용하고, 비면 legacy linear
        # interp (ec_joint_cmd 경로) fallback.
        self._traj_interp = JointTrajectoryInterpolator()
        self._traj_lock = threading.Lock()
        # 큐가 마지막으로 비워진 시각. 비어있으면 timer 는 legacy path 사용.
        self._traj_active = False

        # ROS2 sub (공통: ec_joint_cmd 수신 — legacy 즉시 명령 경로)
        self._sub = self.create_subscription(
            JointState, 'ec_joint_cmd', self._cmd_callback, 10
        )
        # 보간 우회 직접 명령 (move_to 등)
        self._direct_sub = self.create_subscription(
            JointState, 'ec_joint_cmd_direct', self._direct_cmd_callback, 10
        )
        # 스케줄 waypoint 수신 (header.stamp = target time in epoch seconds).
        self._waypoint_sub = self.create_subscription(
            JointState, 'ec_joint_waypoint', self._waypoint_callback, 20
        )

        if self.control_mode == 'sdk':
            self._init_sdk_mode()
        else:
            self._init_topic_mode()

        # 보간 타이머 (공통)
        self._timer = self.create_timer(self._dt, self._timer_callback)

        self.get_logger().info(
            f'Interpolation node started: rate={self.publish_rate}Hz, '
            f'mode={self.control_mode} (linear)'
        )

    def _init_topic_mode(self):
        """토픽 출력 모드 초기화. msg_type 에 따라 publisher 와 직렬화 함수가 달라진다.

        지원 타입:
          - sensor_msgs/JointState           — name + position(+velocity)
          - std_msgs/Float64MultiArray       — position 만 .data 배열
          - trajectory_msgs/JointTrajectory  — joint_names + 단일 point(positions, dt=0)
        """
        if self.output_msg_type == 'std_msgs/Float64MultiArray':
            from std_msgs.msg import Float64MultiArray
            self._pub_msg_cls = Float64MultiArray
        elif self.output_msg_type == 'trajectory_msgs/JointTrajectory':
            from trajectory_msgs.msg import JointTrajectory
            self._pub_msg_cls = JointTrajectory
        else:
            self._pub_msg_cls = JointState
        self._pub = self.create_publisher(self._pub_msg_cls, self.output_topic, 10)
        self._state_pub = None

    def _init_sdk_mode(self):
        """SDK 출력 모드 초기화. SDK로 제어+상태읽기."""
        from ros2_bridge.sdk_controllers import create_sdk_controller

        sdk_type = self.get_parameter('sdk_type').value
        sdk_config = {
            'can_port': self.get_parameter('sdk_can_port').value,
            'has_gripper': self.get_parameter('sdk_has_gripper').value,
            'ip_address': self.get_parameter('sdk_ip_address').value,
            'serial_port': self.get_parameter('sdk_serial_port').value,
        }
        read_topic = self.get_parameter('read_topic').value

        self.get_logger().info(f'SDK mode: type={sdk_type}, config={sdk_config}')

        self._sdk_controller = create_sdk_controller(sdk_type, sdk_config)
        if not self._sdk_controller.connect():
            self.get_logger().error('SDK connect failed! Node will exit.')
            self._sdk_controller = None
            raise RuntimeError('SDK connect failed')
        if not self._sdk_controller.enable():
            self.get_logger().error('SDK enable failed! Node will exit.')
            self._sdk_controller.disconnect()
            self._sdk_controller = None
            raise RuntimeError('SDK enable failed')

        # 토픽 출력 비활성화
        self._pub = None

        # 관절 상태 퍼블리시 (Agent가 이 토픽을 구독)
        self._state_pub = self.create_publisher(JointState, read_topic, 10)

        # 관절 상태 읽기 타이머 (50Hz)
        self._state_timer = self.create_timer(0.02, self._read_state_callback)

    def _cmd_callback(self, msg: JointState):
        """새 관절 명령 수신 시 보간 시작."""
        target = np.array(msg.position, dtype=np.float64)
        now = time.monotonic()

        # 스케줄 큐가 활성 상태였다면 — legacy 명령 도착 = 모드 전환 의도.
        # 큐를 정리하지 않으면 timer 가 path 1 (큐 hold) 에서 RETURN 해버려서
        # 여기 저장된 _target_pos 가 무시되고, move_to/외부 즉시 명령이 1초간
        # 먹통 → 그 뒤 점프. 따라서 큐 즉시 reset.
        if self._traj_active:
            with self._traj_lock:
                self._traj_interp.reset()
                self._traj_active = False

        if self._cmd_time is not None:
            interval = now - self._cmd_time
            if interval > 0.001:
                # 단발 명령 시 너무 느리게 보간되지 않도록 상한 설정
                self._cmd_interval = min(interval, 0.2)
        self._cmd_time = now

        if self._current_pos is None:
            self._current_pos = target.copy()

        self._start_pos = self._current_pos.copy()
        self._target_pos = target
        self._interp_progress = 0.0

        self._target_names = list(msg.name) if msg.name else None
        self._target_vel = list(msg.velocity) if msg.velocity else None
        self._has_target = True

    def _waypoint_callback(self, msg: JointState):
        """스케줄 waypoint 수신.

        msg.header.stamp 는 목표 도달 절대 시각 (epoch seconds + nanoseconds,
        Python 의 time.time() 과 동일 기준). msg.position 은 해당 시각의 joint
        target. names 도 함께 사용 (write_joints 호환).

        큐는 시간순 단조 증가 유지 — 과거 시각은 무시. 큐가 활성화되면 legacy
        linear interp 분기를 끄고 큐 기반 출력으로 전환.
        """
        target = np.array(msg.position, dtype=np.float64)
        # header.stamp → epoch seconds (float)
        t_target = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if t_target <= 0:
            # stamp 미설정 — 현재 시각으로 대체 (= 즉시)
            t_target = time.time()
        names = list(msg.name) if msg.name else None

        with self._traj_lock:
            # Cold start anchor: 큐가 비어 있고 (cold start 또는 idle 후 새 chunk
            # 도착) 현재 pose 가 있으면, t=now 에 현재 pose 를 anchor 로 넣은 뒤
            # target 을 schedule. 이게 없으면 interpolator 가 큐에 점이 1 개라
            # t_now < t_target 이어도 즉시 target 을 반환 (joint_trajectory_interpolator
            # __call__ 의 ``len==1`` 분기). 결과: homepose 직후 첫 publish 에서
            # robot 이 target 으로 점프하면서 덜컹.
            if self._traj_interp.empty and self._current_pos is not None:
                t_anchor = time.time()
                if t_anchor < t_target:
                    self._traj_interp.schedule_waypoint(
                        self._current_pos.copy(), t_anchor, names=names
                    )
            ok = self._traj_interp.schedule_waypoint(target, t_target, names=names)
            if ok:
                self._traj_active = True
                # legacy interp 상태 정리 — 두 path 동시 사용 금지
                self._has_target = False

    def _direct_cmd_callback(self, msg: JointState):
        """보간 없이 즉시 출력 (move_to 등)."""
        target = np.array(msg.position, dtype=np.float64)

        # 스케줄 큐도 함께 정리 — direct 명령은 "지금 즉시 여기" 의도라 기존
        # 큐의 미래 waypoint 가 남아 있으면 충돌.
        if self._traj_active:
            with self._traj_lock:
                self._traj_interp.reset()
                self._traj_active = False

        # 보간 상태 초기화 (현재 보간 중이면 중단)
        self._has_target = False
        self._current_pos = target.copy()
        self._target_pos = target.copy()
        self._interp_progress = 1.0
        # Direct cmd 는 LPF bypass — 즉시 target 으로 점프해야 하는 의도
        # (move_to 의 첫 cmd, 안전정지 등). 필터 state 를 target 으로 reset 해
        # 다음 timer cycle 에서 lag 없이 시작.
        self._lpf_state = target.copy()

        names = list(msg.name) if msg.name else None
        vel = list(msg.velocity) if msg.velocity else None

        if self.control_mode == 'sdk':
            if self._sdk_controller is not None:
                self._sdk_controller.write_joints(target.tolist(), names)
        else:
            self._publish_topic(target.tolist(), names, vel)

    def _timer_callback(self):
        """고주파 타이머. 우선순위:
        1) 스케줄 waypoint 큐가 활성: 현재 시각 t_now (epoch) 의 보간값을 출력
        2) legacy linear interp (ec_joint_cmd path)
        """
        # ── Path 1: scheduled waypoint queue ────────────────────────────────
        if self._traj_active:
            t_now = time.time()
            cmd_pose = None
            cmd_names = None
            queue_last = None
            with self._traj_lock:
                if not self._traj_interp.empty:
                    cmd_pose = self._traj_interp(t_now)
                    cmd_names = self._traj_interp.names
                    queue_last = self._traj_interp.last_time
                    # 너무 옛날 waypoint 들 trim (간단한 메모리 관리)
                    self._traj_interp.trim_before(t_now - 0.5)
                else:
                    self._traj_active = False

            if cmd_pose is not None:
                # LPF: chunk boundary anchor 만으로 잡히지 않는 미세 진동 흡수.
                cmd_pose = self._apply_lpf(cmd_pose)
                self._current_pos = cmd_pose
                self._target_names = cmd_names if cmd_names else self._target_names
                if self.control_mode == 'sdk':
                    self._output_sdk()
                else:
                    self._publish_topic(
                        cmd_pose.tolist(), self._target_names, None
                    )
                # 큐 마지막 waypoint 시각이 지나도록 일정 시간 더 hold 한 뒤 비활성
                # (마지막 값 유지 → 갑작스런 정지 방지). 1초 이상 새 waypoint 없으면
                # 큐 자체를 비워서 stale lock 방지.
                if queue_last is not None and t_now > queue_last + 1.0:
                    with self._traj_lock:
                        self._traj_interp.reset()
                        self._traj_active = False
                        self.get_logger().info('Waypoint queue idle >1s, cleared')
            return

        # ── Path 2: legacy linear interp ────────────────────────────────────
        if not self._has_target:
            return

        # 0.3초 이상 명령이 없으면 이전 상태 폐기 (텔레옵 중지 시 잔여 동작 방지)
        if self._cmd_time is not None and (time.monotonic() - self._cmd_time) > 0.3:
            self._has_target = False
            self._cmd_time = None
            self.get_logger().info('Stale command discarded (>0.3s idle)')
            return

        if self._interp_progress < 1.0:
            self._interp_progress += self._dt / self._cmd_interval
            self._interp_progress = min(self._interp_progress, 1.0)
        t = self._interp_progress

        _raw_pos = self._start_pos + t * (self._target_pos - self._start_pos)
        # LPF 적용 — legacy interp 도 200Hz publish 시 미세 진동 흡수.
        self._current_pos = self._apply_lpf(_raw_pos)

        if self.control_mode == 'sdk':
            self._output_sdk()
        else:
            self._output_topic()

        # interp 완료(progress>=1.0) 후에도 LPF lag 때문에 마지막 publish 값이
        # target 보다 ~1 step 짧다. 여기서 바로 publish 를 멈추면 (구버전 동작)
        # robot/gripper 가 목표 pose 직전에서 영구히 멈춤. → progress 1.0 도달
        # 후에도 _raw_pos(=target) 를 LPF 에 계속 통과시켜 완전히 수렴할 때까지
        # publish 를 이어간다. _lpf_state 가 None 이면 (α>=1.0 무필터) lag 자체가
        # 없으므로 즉시 종료.
        if self._interp_progress >= 1.0:
            if (self._lpf_state is None
                    or np.max(np.abs(self._lpf_state - self._target_pos))
                    < _LPF_SETTLE_EPS):
                self._has_target = False

    def _apply_lpf(self, target_np):
        """1차 IIR low-pass filter: out = α * target + (1-α) * prev_out.

        α=1.0 이면 무필터 (target 그대로 통과). α<1.0 이면 진동/jerk smoothing.
        첫 호출 시 prev_out 이 없으면 target 으로 초기화 (lag 0).
        target_np shape 가 기존 state 와 다르면 (joint 수 변경 등) state 재초기화.
        """
        if self._lpf_alpha >= 1.0:
            return target_np
        if self._lpf_state is None or self._lpf_state.shape != target_np.shape:
            self._lpf_state = target_np.copy()
            return self._lpf_state
        self._lpf_state = (
            self._lpf_alpha * target_np
            + (1.0 - self._lpf_alpha) * self._lpf_state
        )
        return self._lpf_state

    def _output_topic(self):
        """보간 결과를 ROS2 토픽으로 퍼블리시."""
        self._publish_topic(
            self._current_pos.tolist(),
            self._target_names,
            self._target_vel,
        )

    def _publish_topic(self, positions, names, vel):
        """msg_type 에 맞춰 직렬화하고 publish.

        지원 타입별:
          - JointState                       : name + position(+velocity)
          - Float64MultiArray                : .data = positions
          - JointTrajectory                  : joint_names + 단일 point(time_from_start=0)
        """
        cls_name = self._pub_msg_cls.__name__
        if cls_name == 'JointState':
            msg = JointState()
            if names:
                msg.name = names
            msg.position = positions
            if vel:
                msg.velocity = vel
            self._pub.publish(msg)
        elif cls_name == 'JointTrajectory':
            # ros2_control / MoveIt 류 컨트롤러용. trajectory 컨트롤러가 자체
            # 보간을 하므로, 보간된 1-point trajectory 를 200Hz 로 흘려보내면
            # 사실상 매번 마지막 point 로 점프 — 우리의 outer 보간이 그대로
            # 실로봇으로 전달된다.
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            msg = JointTrajectory()
            if names:
                msg.joint_names = names
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            if vel:
                point.velocities = list(vel)
            point.time_from_start = Duration(sec=0, nanosec=0)
            msg.points = [point]
            self._pub.publish(msg)
        else:
            # Float64MultiArray: position 만 .data 배열로 (rbpodo 등에서 사용)
            msg = self._pub_msg_cls()
            msg.data = list(positions)
            self._pub.publish(msg)

    def _output_sdk(self):
        """보간 결과를 SDK로 직접 전송."""
        if self._sdk_controller is None:
            return
        self._sdk_controller.write_joints(
            self._current_pos.tolist(), self._target_names)

    def _read_state_callback(self):
        """SDK에서 관절 상태를 읽어 ROS2 토픽으로 퍼블리시. 지원하는 SDK 는
        velocity/effort 도 같이 채워서 dataset 수집/추론 시 qvel, qeffort 가
        실제 값으로 들어오게 한다. 미지원 SDK 는 빈 배열 → JointState 호환."""
        if self._sdk_controller is None or self._state_pub is None:
            return
        try:
            names, positions, velocities, efforts = (
                self._sdk_controller.read_joints_extended()
            )
            msg = JointState()
            msg.name = names
            msg.position = positions
            if velocities:
                msg.velocity = velocities
            if efforts:
                msg.effort = efforts
            self._state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'SDK read error: {e}', throttle_duration_sec=5.0)

    def destroy_node(self):
        """노드 종료 시 SDK 정리."""
        if self._sdk_controller is not None:
            self._sdk_controller.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointInterpolationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        print(f"[interpolation_node] Fatal: {e}", flush=True)
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
