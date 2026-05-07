# -*- coding: utf-8 -*-
"""Built-in fallback driver: 단순 ROS2 topic publish.

custom robot 이 interpolation=False 인데 module.json driver 가 명시되지 않은
경우 (e.g. 튜토리얼 robot, write_topic 만 있는 단순 외부 sim 노드) 사용된다.
robot.settings.write_topic / write_topic_msg 를 그대로 사용해 JointState 류
메시지를 publish 한다.

지원 메시지: position 필드를 가진 메시지 (sensor_msgs/JointState 등). 기본값은
sensor_msgs/JointState — `name` 도 함께 채운다.
"""
from typing import Optional

from rosidl_runtime_py.utilities import get_message

from .base import RobotDriver


class TopicJointStateDriver(RobotDriver):
    """write_topic 으로 JointState 류 메시지를 publish 하는 단순 driver."""

    def __init__(self, node, robot: dict, config: Optional[dict] = None):
        super().__init__(node, robot, config)
        settings = robot.get('settings', {}) or {}
        self._topic = settings.get('write_topic') or robot.get('write_topic')
        msg_type = (settings.get('write_topic_msg')
                    or robot.get('write_topic_msg')
                    or 'sensor_msgs/JointState')
        if not self._topic:
            raise ValueError(
                f"TopicJointStateDriver requires write_topic in robot settings"
            )
        self._msg_cls = get_message(msg_type)
        self._pub = node.create_publisher(self._msg_cls, self._topic, 10)
        self._joint_names = robot.get('joint_names') or []
        self._tool_inner = bool(robot.get('tool_inner') or settings.get('tool_inner'))

    def write_joints(self, action, vel_arg=None):
        msg = self._msg_cls()
        if hasattr(msg, 'name'):
            msg.name = list(self._joint_names)
        if hasattr(msg, 'position'):
            msg.position = [float(v) for v in action]
        if hasattr(msg, 'velocity'):
            vel = [0.0] * len(action)
            if self._tool_inner and vel:
                vel[-1] = 100.0
            msg.velocity = vel
        self._pub.publish(msg)

    def destroy(self):
        if self._pub is not None:
            try:
                self._node.destroy_publisher(self._pub)
            except Exception:
                pass
            self._pub = None
