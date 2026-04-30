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
        self.declare_parameter('control_mode', 'topic')     # 'topic' or 'sdk'
        self.declare_parameter('sdk_type', '')               # 'piper', etc.
        self.declare_parameter('sdk_can_port', 'can0')       # SDK CAN 포트
        self.declare_parameter('sdk_has_gripper', True)      # SDK 그리퍼 유무
        self.declare_parameter('read_topic', 'interpolated_joint_cmd')  # SDK 모드: 상태 퍼블리시 토픽

        self.publish_rate = self.get_parameter('publish_rate').value
        self.output_topic = self.get_parameter('output_topic').value
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

        # SDK 컨트롤러
        self._sdk_controller = None

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
        """토픽 출력 모드 초기화."""
        self._pub = self.create_publisher(JointState, self.output_topic, 10)
        self._state_pub = None

    def _init_sdk_mode(self):
        """SDK 출력 모드 초기화. SDK로 제어+상태읽기."""
        from ros2_bridge.sdk_controllers import create_sdk_controller

        sdk_type = self.get_parameter('sdk_type').value
        sdk_config = {
            'can_port': self.get_parameter('sdk_can_port').value,
            'has_gripper': self.get_parameter('sdk_has_gripper').value,
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
            out = JointState()
            if names:
                out.name = names
            out.position = target.tolist()
            if vel:
                out.velocity = vel
            self._pub.publish(out)

    def _timer_callback(self):
        """고주파 타이머: 선형 보간으로 목표 추종."""
        if not self._has_target or self._interp_progress >= 1.0:
            return

        # 2초 이상 명령이 없으면 이전 상태 폐기
        if self._cmd_time is not None and (time.monotonic() - self._cmd_time) > 2.0:
            self._has_target = False
            self._cmd_time = None
            self.get_logger().info('Stale command discarded (>2s idle)')
            return

        self._interp_progress += self._dt / self._cmd_interval
        self._interp_progress = min(self._interp_progress, 1.0)
        t = self._interp_progress

        self._current_pos = self._start_pos + t * (self._target_pos - self._start_pos)

        if self.control_mode == 'sdk':
            self._output_sdk()
        else:
            self._output_topic()

    def _output_topic(self):
        """보간 결과를 ROS2 토픽으로 퍼블리시."""
        msg = JointState()
        if self._target_names:
            msg.name = self._target_names
        msg.position = self._current_pos.tolist()
        if self._target_vel:
            msg.velocity = self._target_vel
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
