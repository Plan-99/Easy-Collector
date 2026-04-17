"""Minimal ROS 2 single-arm agent.

Subscribes to a joint-state topic, parses ``qpos`` from it, and publishes joint
commands back via the configured write topic. Designed for ``write_type='topic'``
robots whose state and command messages can be parsed/built generically.

Supported message types
-----------------------

Read (joint state subscription):
- ``sensor_msgs/JointState``         (uses ``msg.position``)
- ``trajectory_msgs/JointTrajectoryPoint`` (uses ``msg.positions``)
- ``std_msgs/Float64MultiArray``     (uses ``msg.data``)

Write (joint command publish):
- ``sensor_msgs/JointState``         (fills ``msg.position``)
- ``trajectory_msgs/JointTrajectoryPoint`` (fills ``msg.positions``)
- ``std_msgs/Float64MultiArray``     (fills ``msg.data``)

If your robot uses a different message type or relies on
``write_type='service'`` / ``write_type='action'``, this simple agent will not
work and you should run inference inside EasyTrainer instead.
"""
from __future__ import annotations

import threading
import time
from typing import List, Optional

import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.node import Node


class SimpleAgent:
    """Single-arm ROS agent for closed-loop inference outside EasyTrainer."""

    def __init__(self, node: Node, robot: dict):
        self.node = node
        self.id = robot['id']
        self.name = robot.get('name', f"robot_{self.id}")
        self.joint_names: List[str] = robot.get('joint_names', []) or []
        self.joint_len = len(self.joint_names) or robot.get('joint_dim') or 7
        self.role = robot.get('role', 'single_arm')

        self.read_topic: str = robot['read_topic']
        self.read_topic_msg: str = robot['read_topic_msg']
        self.write_topic: str = robot['write_topic']
        self.write_topic_msg: str = robot['write_topic_msg']
        self.write_type: str = robot.get('write_type', 'topic')

        if self.write_type != 'topic':
            raise NotImplementedError(
                f"SimpleAgent only supports write_type='topic'. "
                f"Robot {self.name} uses write_type='{self.write_type}'. "
                f"Run inference inside EasyTrainer instead."
            )

        self._js_lock = threading.Lock()
        self._joint_states: Optional[List[float]] = None
        self._last_joint_update: Optional[float] = None

        # Resolve message classes
        try:
            self._read_cls = get_message(self.read_topic_msg)
        except Exception as e:
            raise RuntimeError(
                f"Could not resolve read_topic_msg='{self.read_topic_msg}' for "
                f"robot {self.name}. Is the corresponding ROS package installed? "
                f"Original error: {e}"
            )
        try:
            self._write_cls = get_message(self.write_topic_msg)
        except Exception as e:
            raise RuntimeError(
                f"Could not resolve write_topic_msg='{self.write_topic_msg}' for "
                f"robot {self.name}. Is the corresponding ROS package installed? "
                f"Original error: {e}"
            )

        self._read_sub = node.create_subscription(
            self._read_cls, self.read_topic, self._joint_state_cb, 10
        )
        self._write_pub = node.create_publisher(self._write_cls, self.write_topic, 10)

        print(f"[SimpleAgent] {self.name}: read {self.read_topic} ({self.read_topic_msg}) "
              f"→ write {self.write_topic} ({self.write_topic_msg}) "
              f"joint_names={self.joint_names}")

    # ────────────────────────────────────────────────────────────────────
    def _joint_state_cb(self, msg) -> None:
        """Extract joint positions from a generic JointState-like message."""
        positions = self._extract_positions(msg)
        if positions is None:
            return

        with self._js_lock:
            # If joint_names are present in both msg and config, reorder to
            # match the training-time canonical order. Otherwise trust the
            # message order and slice to joint_len.
            if (
                self.joint_names
                and getattr(msg, 'name', None)
                and len(getattr(msg, 'name', [])) == len(positions)
            ):
                name_to_idx = {n: i for i, n in enumerate(msg.name)}
                ordered = []
                for jn in self.joint_names:
                    if jn not in name_to_idx:
                        # missing joint — bail out and use raw order
                        ordered = list(positions[: self.joint_len])
                        break
                    ordered.append(float(positions[name_to_idx[jn]]))
                self._joint_states = ordered
            else:
                self._joint_states = list(map(float, positions[: self.joint_len]))
            self._last_joint_update = time.time()

    @staticmethod
    def _extract_positions(msg):
        """Pull a list of joint positions from common ROS message types."""
        # sensor_msgs/JointState
        if hasattr(msg, 'position') and msg.position is not None:
            try:
                return list(msg.position)
            except TypeError:
                pass
        # trajectory_msgs/JointTrajectoryPoint
        if hasattr(msg, 'positions') and msg.positions is not None:
            try:
                return list(msg.positions)
            except TypeError:
                pass
        # std_msgs/Float64MultiArray (data is a sequence<double>)
        if hasattr(msg, 'data') and msg.data is not None:
            try:
                return list(msg.data)
            except TypeError:
                pass
        return None

    # ────────────────────────────────────────────────────────────────────
    def get_joint_states(self) -> List[float]:
        """Return the latest qpos. Blocks up to 5s waiting for the first message."""
        with self._js_lock:
            qpos = self._joint_states
        if qpos is not None:
            return qpos
        # First call: wait for the subscription to deliver something.
        waited = 0.0
        while waited < 5.0:
            time.sleep(0.05)
            waited += 0.05
            with self._js_lock:
                if self._joint_states is not None:
                    return self._joint_states
        raise RuntimeError(
            f"[SimpleAgent {self.name}] no joint state received on "
            f"{self.read_topic} after 5 seconds. Is the robot driver running?"
        )

    def move_joint_step(self, target_qpos) -> None:
        """Publish a single joint command. Mirrors EasyTrainer Agent.move_joint_step
        but with no smoothing / no internal interpolation — that lives in the
        robot driver."""
        target = list(map(float, target_qpos[: self.joint_len]))
        msg = self._build_command_message(target)
        if msg is None:
            return
        self._write_pub.publish(msg)

    def _build_command_message(self, positions: List[float]):
        """Construct a write message of the configured type with positions filled in."""
        msg = self._write_cls()
        # sensor_msgs/JointState
        if hasattr(msg, 'position'):
            try:
                msg.position = positions
            except Exception:
                msg.position = list(positions)
            if hasattr(msg, 'name') and self.joint_names:
                msg.name = list(self.joint_names)
            return msg
        # trajectory_msgs/JointTrajectoryPoint
        if hasattr(msg, 'positions'):
            msg.positions = positions
            return msg
        # std_msgs/Float64MultiArray
        if hasattr(msg, 'data'):
            try:
                msg.data = positions
            except Exception:
                msg.data = list(positions)
            return msg
        print(f"[SimpleAgent {self.name}] WARN: don't know how to fill positions on "
              f"{self.write_topic_msg}; not publishing.")
        return None
