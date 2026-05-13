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
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


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
        self.declare_parameter('read_topic', 'interpolated_joint_cmd')  # SDK 모드: 상태 퍼블리시 토픽

        self.publish_rate = self.get_parameter('publish_rate').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_msg_type = self.get_parameter('output_msg_type').value
        self.control_mode = self.get_parameter('control_mode').value
        self._dt = 1.0 / self.publish_rate

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

        # idle gap 후 첫 segment 한정 cubic ease-in. linear 는 정지→이동 전이에서
        # velocity step 을 만들어 Fairino ServoJ 가 cmdT=5ms 안에 큰 가속도로
        # 따라가려다 덜컹. s(t)=t²(2-t) 는 s'(0)=0, s'(1)=1 이라 정지에서 부드럽게
        # 가속하고 끝점 속도는 linear 와 동일 → 다음 segment(linear) 와 매끄럽게 연결.
        self._smooth_first_segment = False

        # Software acceleration limiter (SDK 모드 한정).
        # interp_node 의 output position 변화율을 강제로 제한해서 ServoJ stream 에
        # 큰 velocity step 이 실리지 않게 한다. Fairino ServoJ 의 acc 파라미터가
        # 비공개라 SDK 단에서 가속도 제어 불가 → 우리가 인위적으로 step 을 매끈하게.
        # max_accel: 관절당 최대 가속도 (rad/s²). 5~15 사이에서 튜닝.
        self._max_accel = 8.0
        self._prev_step = None  # 직전 tick 의 (current_pos - prev_current_pos)

        # SDK 컨트롤러
        self._sdk_controller = None

        # SDK 모드에서 _read_state_callback 이 50Hz 로 채우는 '실제 로봇 위치' 캐시.
        # idle gap 후 첫 cmd 의 보간 시작점을 이걸로 잡아 drift gap 을 흡수한다.
        self._latest_actual_pos = None
        self._latest_actual_names = None

        # ROS2 sub (공통: ec_joint_cmd 수신)
        self._sub = self.create_subscription(
            JointState, 'ec_joint_cmd', self._cmd_callback, 10
        )
        # 보간 우회 직접 명령 (move_to 등)
        self._direct_sub = self.create_subscription(
            JointState, 'ec_joint_cmd_direct', self._direct_cmd_callback, 10
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

    # 큰 jump (예: inference 첫 step) 감지 임계값과 그 때 보장할 최소 보간 시간.
    # teleop 의 작은 step (0.003~0.05) 보다 충분히 위라 일반 텔레옵에는 영향 없음.
    _JUMP_THRESHOLD_RAD = 0.05  # 한 cmd 에 0.05 rad(≈2.9°) 이상 변화 = jump (이전 0.1)
    _JUMP_MIN_INTERVAL = 0.5    # jump 인 경우 cmd_interval 의 floor (s) — 0.2 는 짧아서 여전히 jerky

    def _cmd_callback(self, msg: JointState):
        """새 관절 명령 수신 시 보간 시작."""
        target = np.array(msg.position, dtype=np.float64)
        now = time.monotonic()

        if self._cmd_time is not None:
            interval = now - self._cmd_time
            if interval > 0.001:
                # 단발 명령 시 너무 느리게 보간되지 않도록 상한 설정
                self._cmd_interval = min(interval, 0.2)
        else:
            interval = 0.0
        self._cmd_time = now

        if self._current_pos is None:
            self._current_pos = target.copy()

        # 매 cmd 마다 ease-in 모드를 새로 결정 (mid-segment 새 cmd 가 와도 잘못된
        # ease-in 이 carry over 되지 않도록). 아래 활성화 조건에서만 True 로 켠다.
        self._smooth_first_segment = False

        # SDK 모드 + idle gap 후 첫 cmd: 보간 시작점을 '마지막 commanded 위치'(_current_pos)
        # 대신 '실제 로봇 위치'(read_joints 캐시)로 잡고, 정지→이동 velocity step 을
        # cubic ease-in 으로 흡수한다. 평상시 cadence(10~100ms)는 무시하기 위해 0.3s 임계.
        # 이름 순서가 cmd 와 다르면 fallback (안전).
        if (self.control_mode == 'sdk'
                and interval > 0.3
                and self._latest_actual_pos is not None
                and self._latest_actual_pos.shape == target.shape
                and self._latest_actual_names is not None
                and list(msg.name) == self._latest_actual_names):
            _drift = float(np.max(np.abs(self._latest_actual_pos - self._current_pos)))
            _delta_to_target = target - self._latest_actual_pos
            _max_delta = float(np.max(np.abs(_delta_to_target)))
            _argmax = int(np.argmax(np.abs(_delta_to_target)))
            self._current_pos = self._latest_actual_pos.copy()
            self._smooth_first_segment = True
            # idle gap = 정지 상태 → 직전 step (velocity) 도 0 으로 리셋해야
            # acceleration limiter 가 0 부터 부드럽게 ramp up 함. 안 하면 이전
            # 세션의 step 잔여가 carry over 됨.
            self._prev_step = None
            self.get_logger().info(
                f'[interp] idle gap {interval:.2f}s | drift={_drift:.4f} rad | '
                f'max_delta_to_target={_max_delta:.4f} rad (joint {_argmax}) | '
                f'home={[f"{v:.4f}" for v in self._latest_actual_pos.tolist()]} | '
                f'target={[f"{v:.4f}" for v in target.tolist()]}'
            )

        # 큰 jump 감지 — home pose 직후 inference 첫 cmd 처럼 _cmd_interval 이 짧게
        # 잡힌 상태에서 큰 차이의 target 이 들어오면 수십 ms 안에 점프해서 모터가
        # "덜컹". 이 경우만 _cmd_interval 을 _JUMP_MIN_INTERVAL 로 floor 해서
        # 첫 보간을 부드럽게.
        if self._current_pos.shape == target.shape:
            try:
                max_delta = float(np.max(np.abs(target - self._current_pos)))
                _orig_interval = self._cmd_interval
                if (max_delta > self._JUMP_THRESHOLD_RAD
                        and self._cmd_interval < self._JUMP_MIN_INTERVAL):
                    self._cmd_interval = self._JUMP_MIN_INTERVAL
                # debug: 어느 정도 변화에서 floor 가 발화하는지 추적
                if max_delta > 0.02:
                    self.get_logger().info(
                        f'[interp] cmd Δmax={max_delta:.4f} rad cmd_interval '
                        f'{_orig_interval:.3f}→{self._cmd_interval:.3f}s'
                    )
            except Exception:
                pass

        self._start_pos = self._current_pos.copy()
        self._target_pos = target
        self._interp_progress = 0.0

        self._target_names = list(msg.name) if msg.name else None
        self._target_vel = list(msg.velocity) if msg.velocity else None
        self._has_target = True

    def _direct_cmd_callback(self, msg: JointState):
        """보간 없이 즉시 출력 (move_to 등)."""
        target = np.array(msg.position, dtype=np.float64)

        # 보간 상태 초기화 (현재 보간 중이면 중단)
        self._has_target = False
        self._current_pos = target.copy()
        self._target_pos = target.copy()
        self._interp_progress = 1.0

        names = list(msg.name) if msg.name else None
        vel = list(msg.velocity) if msg.velocity else None

        if self.control_mode == 'sdk':
            if self._sdk_controller is not None:
                self._sdk_controller.write_joints(target.tolist(), names)
        else:
            self._publish_topic(target.tolist(), names, vel)

    def _timer_callback(self):
        """고주파 타이머: 선형 보간으로 목표 추종 + 유휴 시 SDK hold."""
        # 1) 보간 완료/대기 상태:
        #    - SDK 모드면 _current_pos 를 계속 ServoJ 로 흘려보낸다 (continuous hold).
        #      ServoJ stream 이 끊기면 Fairino 등은 internal trajectory state 가
        #      "done" 으로 떨어졌다가 다음 cmd 에서 fresh start 로 처리해 큰 가속도
        #      적용 → 덜컹. 같은 위치 반복 ServoJ 는 정지 명령과 동일하므로 안전.
        #    - topic 모드는 외부 컨트롤러가 hold 를 책임지므로 publish 안함.
        if not self._has_target or self._interp_progress >= 1.0:
            if self.control_mode == 'sdk' and self._current_pos is not None:
                self._output_sdk()
            return

        # 0.3초 이상 명령이 없으면 이전 상태 폐기 (텔레옵 중지 시 잔여 동작 방지).
        # 이 분기는 보간 진행 중인 경우에만 도달 (위 early-return 통과).
        if self._cmd_time is not None and (time.monotonic() - self._cmd_time) > 0.3:
            self._has_target = False
            self._cmd_time = None
            self.get_logger().info('Stale command discarded (>0.3s idle)')
            # SDK 모드: stale 폐기 후에도 _current_pos hold 는 다음 tick 의 위쪽
            # 분기에서 계속 ServoJ 로 흘려보낸다.
            return

        self._interp_progress += self._dt / self._cmd_interval
        self._interp_progress = min(self._interp_progress, 1.0)
        t = self._interp_progress

        # idle 후 첫 segment 만 cubic ease-in. s(t)=t²(2-t) 는 s'(0)=0, s'(1)=1.
        # 정지에서 부드럽게 가속, 끝점은 linear 와 같은 속도로 마무리해서 다음
        # segment(linear) 와 속도 연속성 유지.
        if self._smooth_first_segment:
            s = t * t * (2.0 - t)
        else:
            s = t
        desired_pos = self._start_pos + s * (self._target_pos - self._start_pos)

        # Software acceleration limit (SDK 모드만). 직전 step 대비 step 변화량을
        # 가속도 한계로 클램프해서 output velocity 가 매끈하게 변하도록 강제한다.
        # max_step_change = max_accel * dt²  (= 한 tick 사이에 step 이 변할 수 있는 양).
        if self.control_mode == 'sdk':
            desired_step = desired_pos - self._current_pos
            if self._prev_step is None or self._prev_step.shape != desired_step.shape:
                self._prev_step = np.zeros_like(desired_step)
            step_change = desired_step - self._prev_step
            max_step_change = self._max_accel * self._dt * self._dt
            step_change = np.clip(step_change, -max_step_change, max_step_change)
            actual_step = self._prev_step + step_change
            self._current_pos = self._current_pos + actual_step
            self._prev_step = actual_step
        else:
            self._current_pos = desired_pos

        # 첫 segment 끝나면 ease-in 모드 해제 (이후 cmd 들은 평소대로 linear).
        if self._interp_progress >= 1.0 and self._smooth_first_segment:
            self._smooth_first_segment = False

        if self.control_mode == 'sdk':
            self._output_sdk()
        else:
            self._output_topic()

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
        """SDK에서 관절 상태를 읽어 ROS2 토픽으로 퍼블리시."""
        if self._sdk_controller is None or self._state_pub is None:
            return
        try:
            names, positions = self._sdk_controller.read_joints()
            msg = JointState()
            msg.name = names
            msg.position = positions
            self._state_pub.publish(msg)
            # _cmd_callback 에서 idle gap 후 보간 시작점으로 쓰기 위한 캐시.
            self._latest_actual_names = list(names)
            self._latest_actual_pos = np.array(positions, dtype=np.float64)
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
