"""High-rate joint-command interpolation node.

Mirrors EasyTrainer's in-container ``ros2_bridge.interpolation_node`` but
slimmed down to **topic-output mode only** — the SDK output mode is not
bundled (it needs vendor SDKs like piper / fairino that aren't pip-installable
and aren't part of the export).

What it does
------------
- Subscribes to ``<ns>/ec_joint_cmd`` (low-rate goals, e.g. 10Hz from a
  checkpoint inference loop).
- Linearly interpolates between successive goals and publishes the smoothed
  stream to ``<output_topic>`` at ``publish_rate`` Hz (default 200Hz).
- Also subscribes to ``<ns>/ec_joint_cmd_direct`` for commands that should
  bypass smoothing (typically ``SimpleAgent.move_to``, which already does its
  own smoothstep interpolation).

This file is instantiated *in-process* by ``planner_engine.build_context`` —
one node per robot whose meta has ``interpolation: true``. There is no
``main()``: the runners (``run_planner.py`` / ``ros_planner_service.py``)
attach the node to their existing rclpy executor.
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosidl_runtime_py.utilities import get_message


class JointInterpolationNode(Node):
    """Single-robot joint-command smoothing node (topic mode only)."""

    def __init__(
        self,
        robot_id: int,
        *,
        write_topic: str,
        write_topic_msg: str,
        joint_names: list,
        publish_rate: float = 200.0,
        node_name: Optional[str] = None,
    ):
        super().__init__(node_name or f"interp_node_{robot_id}")

        self.robot_id = robot_id
        self.write_topic = write_topic
        self.write_topic_msg = write_topic_msg
        self.joint_names = list(joint_names or [])
        self.publish_rate = float(publish_rate)
        self._dt = 1.0 / self.publish_rate

        # Resolve the output message class up front.
        try:
            self._pub_cls = get_message(self.write_topic_msg)
        except Exception as e:
            raise RuntimeError(
                f"[interp_node_{robot_id}] cannot resolve write_topic_msg="
                f"'{self.write_topic_msg}'. Is the ROS package installed? "
                f"({e})"
            )

        # Interpolation state.
        self._current_pos: Optional[np.ndarray] = None
        self._start_pos: Optional[np.ndarray] = None
        self._target_pos: Optional[np.ndarray] = None
        self._has_target = False
        self._cmd_time: Optional[float] = None
        self._cmd_interval = 0.1
        self._interp_progress = 1.0

        # The agent publishes goals to /ec_robot_<id>/ec_joint_cmd.
        ns = f"/ec_robot_{robot_id}"
        self._sub = self.create_subscription(
            JointState, f"{ns}/ec_joint_cmd", self._cmd_cb, 10,
        )
        # And bypass goals (move_to) to /ec_robot_<id>/ec_joint_cmd_direct.
        self._direct_sub = self.create_subscription(
            JointState, f"{ns}/ec_joint_cmd_direct", self._direct_cb, 10,
        )

        # Publisher to the real robot driver topic.
        self._pub = self.create_publisher(self._pub_cls, self.write_topic, 10)

        # Smoothing timer.
        self._timer = self.create_timer(self._dt, self._timer_cb)

        self.get_logger().info(
            f"[interp_node_{robot_id}] subscribing {ns}/ec_joint_cmd → "
            f"publishing {self.write_topic} ({self.write_topic_msg}) "
            f"at {self.publish_rate}Hz"
        )

    # ── callbacks ────────────────────────────────────────────────────────
    def _cmd_cb(self, msg: JointState):
        """Receive a new low-rate goal — kick off smoothing."""
        target = np.array(msg.position, dtype=np.float64)
        now = time.monotonic()
        if self._cmd_time is not None:
            interval = now - self._cmd_time
            if interval > 0.001:
                # Cap the smoothing window so a single straggling command
                # doesn't make the robot drift slowly.
                self._cmd_interval = min(interval, 0.2)
        self._cmd_time = now

        if self._current_pos is None:
            self._current_pos = target.copy()

        self._start_pos = self._current_pos.copy()
        self._target_pos = target
        self._interp_progress = 0.0
        self._has_target = True

    def _direct_cb(self, msg: JointState):
        """Receive a bypass goal — publish immediately, no smoothing."""
        target = np.array(msg.position, dtype=np.float64)
        self._has_target = False
        self._current_pos = target.copy()
        self._target_pos = target.copy()
        self._interp_progress = 1.0
        self._publish(target.tolist())

    def _timer_cb(self):
        """High-rate tick — advance the linear interpolation and publish."""
        if not self._has_target or self._interp_progress >= 1.0:
            return

        # Drop stale goals: if the goal stream has stopped for >300ms, freeze
        # the robot at the current position rather than overshoot.
        if self._cmd_time is not None and (time.monotonic() - self._cmd_time) > 0.3:
            self._has_target = False
            self._cmd_time = None
            return

        self._interp_progress += self._dt / max(self._cmd_interval, 1e-6)
        self._interp_progress = min(self._interp_progress, 1.0)
        t = self._interp_progress
        self._current_pos = self._start_pos + t * (self._target_pos - self._start_pos)
        self._publish(self._current_pos.tolist())

    # ── publish helpers ──────────────────────────────────────────────────
    def _publish(self, positions: list):
        """Build the configured msg type and publish it."""
        cls_name = self._pub_cls.__name__
        if cls_name == "JointState":
            msg = JointState()
            if self.joint_names:
                msg.name = list(self.joint_names)
            msg.position = positions
            self._pub.publish(msg)
            return
        if cls_name == "JointTrajectory":
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            msg = JointTrajectory()
            if self.joint_names:
                msg.joint_names = list(self.joint_names)
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = Duration(sec=0, nanosec=0)
            msg.points = [point]
            self._pub.publish(msg)
            return
        # Float64MultiArray and the rest: just dump positions into .data.
        msg = self._pub_cls()
        try:
            msg.data = positions
        except Exception:
            msg.data = list(positions)
        self._pub.publish(msg)
