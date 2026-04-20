#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Joint Command Interpolation Node.

저주파 관절 명령을 받아 보간하여 고주파로 퍼블리시함으로써
로봇의 진동/떨림을 방지한다.

Usage (standalone):
    ros2 run --ros-args -r __ns:=/ec_robot_1 -- python3 -m ros2_bridge.interpolation_node

Usage (as subprocess):
    driver_service.py에서 Piper 드라이버와 함께 자동 실행됨.
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointInterpolationNode(Node):
    """
    /ec_joint_cmd 토픽을 구독하여 선형 보간 후 /joint_states 토픽으로 퍼블리시.

    명령 간격을 측정하여, 다음 명령이 도착할 때까지 일정 속도로 목표에 도달.
    속도 불연속 없이 부드러운 동작을 보장한다.

    Parameters (ROS2 params):
        publish_rate: 퍼블리시 주파수 (Hz, default: 200)
    """

    def __init__(self):
        super().__init__('joint_interpolation')

        # Parameters
        self.declare_parameter('publish_rate', 200.0)
        self.declare_parameter('output_topic', 'joint_states')

        self.publish_rate = self.get_parameter('publish_rate').value
        self.output_topic = self.get_parameter('output_topic').value
        self._dt = 1.0 / self.publish_rate

        # State
        self._current_pos = None
        self._start_pos = None     # 보간 시작 위치
        self._target_pos = None    # 목표 위치
        self._target_vel = None
        self._target_names = None
        self._has_target = False

        # 타이밍
        self._cmd_time = None       # 현재 명령 도착 시각
        self._cmd_interval = 0.1    # 명령 간격 (초), 초기값
        self._interp_progress = 1.0 # 0~1, 1이면 완료

        # 디버그
        self._log_cnt = 0

        # ROS2 pub/sub
        self._sub = self.create_subscription(
            JointState, 'ec_joint_cmd', self._cmd_callback, 10
        )
        self._pub = self.create_publisher(JointState, self.output_topic, 10)
        self._timer = self.create_timer(self._dt, self._timer_callback)

        self.get_logger().info(
            f'Interpolation node started: rate={self.publish_rate}Hz (linear)'
        )

    def _cmd_callback(self, msg: JointState):
        """새 관절 명령 수신 시 보간 시작."""
        target = np.array(msg.position, dtype=np.float64)
        now = time.monotonic()

        # 명령 간격 측정
        if self._cmd_time is not None:
            interval = now - self._cmd_time
            if interval > 0.001:  # 1ms 이상만
                self._cmd_interval = interval
        self._cmd_time = now

        if self._current_pos is None:
            self._current_pos = target.copy()

        # 현재 위치에서 새 목표로 선형 보간 시작
        self._start_pos = self._current_pos.copy()
        self._target_pos = target
        self._interp_progress = 0.0

        self._target_names = list(msg.name) if msg.name else None
        self._target_vel = list(msg.velocity) if msg.velocity else None
        self._has_target = True

        self.get_logger().info(
            f'CMD: joint2={target[1]:.4f} | interval={self._cmd_interval*1000:.0f}ms'
        )

    def _timer_callback(self):
        """고주파 타이머: 선형 보간으로 목표 추종."""
        if not self._has_target or self._interp_progress >= 1.0:
            return

        # 진행도 업데이트
        self._interp_progress += self._dt / self._cmd_interval
        self._interp_progress = min(self._interp_progress, 1.0)

        t = self._interp_progress

        # 선형 보간: start + t * (target - start)
        self._current_pos = self._start_pos + t * (self._target_pos - self._start_pos)

        # 디버그: 10번에 1번 출력
        self._log_cnt += 1
        if self._log_cnt % 10 == 0:
            self.get_logger().info(
                f'PUB: joint2={self._current_pos[1]:.4f} | '
                f'TGT: {self._target_pos[1]:.4f} | '
                f't={t:.2f}'
            )

        # Publish
        msg = JointState()
        if self._target_names:
            msg.name = self._target_names
        msg.position = self._current_pos.tolist()
        if self._target_vel:
            msg.velocity = self._target_vel
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointInterpolationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
