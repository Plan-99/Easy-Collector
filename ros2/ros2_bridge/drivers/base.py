# -*- coding: utf-8 -*-
"""Robot driver ABC.

Custom robot 가 보간 노드를 사용하지 않을 때 agent 가 호출할 transport
어댑터. agent.move_joint_step 이 driver.write_joints 를 호출하며, 이 메서드
안에서 사용자는 ROS2 topic publish, service call, action send_goal, 또는 직접
SDK 호출 등 임의의 transport 를 구현할 수 있다.

설계 원칙:
  - rclpy node 는 agent 가 보유 — driver __init__ 에 주입 받음 (publisher
    /service client 등을 만들고 싶을 때 사용)
  - read 경로는 agent 가 자체 ROS2 read_topic 구독으로 처리 — driver 가
    state 를 push 할 필요 없음
  - destroy 는 agent.destroy 가 호출 — publisher/client 정리는 driver 책임
"""
from abc import ABC, abstractmethod
from typing import Optional


class RobotDriver(ABC):
    """Custom robot 의 write 경로 전용 어댑터.

    구현 예 (간단한 JointState publisher):

        from sensor_msgs.msg import JointState
        from ros2_bridge.drivers import RobotDriver

        class MyRobotDriver(RobotDriver):
            def __init__(self, node, robot, config):
                super().__init__(node, robot, config)
                self._pub = node.create_publisher(
                    JointState, robot['settings']['write_topic'], 10
                )
                self._msg = JointState()
                self._msg.name = robot['joint_names']

            def write_joints(self, action, vel_arg=None):
                self._msg.position = list(action)
                self._pub.publish(self._msg)

            def destroy(self):
                self._node.destroy_publisher(self._pub)
    """

    def __init__(self, node, robot: dict, config: Optional[dict] = None):
        """
        Args:
            node:   rclpy.node.Node — bridge 의 메인 노드. publisher/client 생성용.
            robot:  RobotConfig dict (joint_names, joint_*_bounds, settings, ...).
            config: module.json::driver.config 의 임의 dict (driver 가 자유롭게
                    사용). None 일 수 있음.
        """
        self._node = node
        self._robot = robot
        self._config = config or {}

    @abstractmethod
    def write_joints(self, action: list, vel_arg=None) -> None:
        """Joint position 명령을 실 transport 로 송신. agent.move_joint_step 이
        clipping/검증을 마친 action 만 넘긴다 (None 미포함, joint_len 보장).

        Args:
            action:  joint position list (rad). 길이 = joint_len.
            vel_arg: 선택적 속도/duration 힌트. driver 가 무시해도 됨.
        """
        ...

    def destroy(self) -> None:
        """publisher/service client 등 driver 가 만든 ROS2 리소스 해제.
        기본 구현은 no-op — 자원을 만든 driver 만 override.
        """
        return None
