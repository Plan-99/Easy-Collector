"""High-rate joint-command interpolation node.

Mirrors EasyTrainer's in-container ``ros2_bridge.interpolation_node`` but
slimmed down to **topic-output mode only** — the SDK output mode is not
bundled (it needs vendor SDKs like piper / fairino that aren't pip-installable
and aren't part of the export).

What it does
------------
- Two input paths, in priority order:
  * ``<ns>/ec_joint_waypoint`` (preferred, used by checkpoint inference) —
    each message carries a target joint vector + ``header.stamp`` set to the
    absolute wall-clock time the robot should be at that pose. The node keeps
    a queue and at every 200Hz tick interpolates between waypoints based on
    ``time.time()``. Decouples inference jitter from robot motion: the robot
    follows the schedule even if the next chunk arrives late.
  * ``<ns>/ec_joint_cmd`` (legacy, used by ``move_joint_step`` when no time
    target is known) — linearly interpolates between successive goals over
    the observed command interval.
- ``<ns>/ec_joint_cmd_direct`` always bypasses smoothing (typically
  ``SimpleAgent.move_to``, which already does its own smoothstep).
- Optional 1st-order IIR low-pass filter on the publish stream (default
  ``α=0.5``) — absorbs the jerk at chunk boundaries / discrete waypoint
  transitions. ``α=1.0`` disables it.

This file is instantiated *in-process* by ``planner_engine.build_context`` —
one node per robot whose meta has ``interpolation: true``. There is no
``main()``: the runners (``run_planner.py`` / ``ros_planner_service.py``)
attach the node to their existing rclpy executor.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosidl_runtime_py.utilities import get_message

from .joint_trajectory_interpolator import JointTrajectoryInterpolator


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
        output_lpf_alpha: float = 0.5,
        node_name: Optional[str] = None,
    ):
        super().__init__(node_name or f"interp_node_{robot_id}")

        self.robot_id = robot_id
        self.write_topic = write_topic
        self.write_topic_msg = write_topic_msg
        self.joint_names = list(joint_names or [])
        self.publish_rate = float(publish_rate)
        self._dt = 1.0 / self.publish_rate

        # Output LPF state.
        self._lpf_alpha = float(output_lpf_alpha)
        if not (0.0 < self._lpf_alpha <= 1.0):
            self._lpf_alpha = 1.0  # invalid → no filter
        self._lpf_state: Optional[np.ndarray] = None

        # Resolve the output message class up front.
        try:
            self._pub_cls = get_message(self.write_topic_msg)
        except Exception as e:
            raise RuntimeError(
                f"[interp_node_{robot_id}] cannot resolve write_topic_msg="
                f"'{self.write_topic_msg}'. Is the ROS package installed? "
                f"({e})"
            )

        # Legacy linear-interp state (ec_joint_cmd path).
        self._current_pos: Optional[np.ndarray] = None
        self._start_pos: Optional[np.ndarray] = None
        self._target_pos: Optional[np.ndarray] = None
        self._target_names: Optional[list] = None
        self._has_target = False
        self._cmd_time: Optional[float] = None
        self._cmd_interval = 0.1
        self._interp_progress = 1.0

        # Scheduled-waypoint queue (DexUMI-style absolute-time scheduling).
        self._traj_interp = JointTrajectoryInterpolator()
        self._traj_lock = threading.Lock()
        self._traj_active = False

        ns = f"/ec_robot_{robot_id}"
        # ec_joint_cmd: legacy "send a target now" path.
        self._sub = self.create_subscription(
            JointState, f"{ns}/ec_joint_cmd", self._cmd_cb, 10,
        )
        # ec_joint_cmd_direct: skip smoothing.
        self._direct_sub = self.create_subscription(
            JointState, f"{ns}/ec_joint_cmd_direct", self._direct_cb, 10,
        )
        # ec_joint_waypoint: scheduled waypoint with header.stamp = target time.
        self._waypoint_sub = self.create_subscription(
            JointState, f"{ns}/ec_joint_waypoint", self._waypoint_cb, 20,
        )

        # Publisher to the real robot driver topic.
        self._pub = self.create_publisher(self._pub_cls, self.write_topic, 10)

        # Smoothing timer.
        self._timer = self.create_timer(self._dt, self._timer_cb)

        self.get_logger().info(
            f"[interp_node_{robot_id}] subscribing {ns}/ec_joint_cmd "
            f"+ {ns}/ec_joint_waypoint → publishing {self.write_topic} "
            f"({self.write_topic_msg}) at {self.publish_rate}Hz "
            f"(lpf_alpha={self._lpf_alpha})"
        )

    # ── callbacks ────────────────────────────────────────────────────────
    def _cmd_cb(self, msg: JointState):
        """Receive a new low-rate goal — kick off smoothing."""
        target = np.array(msg.position, dtype=np.float64)
        now = time.monotonic()

        # If the scheduled queue was active, switching back to legacy commands
        # means "ignore the schedule, go here now". Reset the queue so the
        # timer doesn't return early from the scheduled-path branch.
        if self._traj_active:
            with self._traj_lock:
                self._traj_interp.reset()
                self._traj_active = False

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
        self._target_names = list(msg.name) if msg.name else self._target_names
        self._interp_progress = 0.0
        self._has_target = True

    def _waypoint_cb(self, msg: JointState):
        """Receive a scheduled waypoint (header.stamp = absolute target time
        in ``time.time()`` epoch seconds)."""
        target = np.array(msg.position, dtype=np.float64)
        t_target = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if t_target <= 0:
            t_target = time.time()
        names = list(msg.name) if msg.name else None

        with self._traj_lock:
            ok = self._traj_interp.schedule_waypoint(target, t_target, names=names)
            if ok:
                self._traj_active = True
                # Disable legacy interp path — only one path drives output.
                self._has_target = False

    def _direct_cb(self, msg: JointState):
        """Receive a bypass goal — publish immediately, no smoothing."""
        target = np.array(msg.position, dtype=np.float64)

        # Direct commands win — clear both schedules.
        if self._traj_active:
            with self._traj_lock:
                self._traj_interp.reset()
                self._traj_active = False

        self._has_target = False
        self._current_pos = target.copy()
        self._target_pos = target.copy()
        self._interp_progress = 1.0
        # Reset LPF state to target so the next tick doesn't lag through the filter.
        self._lpf_state = target.copy()
        names = list(msg.name) if msg.name else self._target_names
        self._publish(target.tolist(), names=names)

    def _timer_cb(self):
        """High-rate tick.

        Priority:
          1) scheduled waypoint queue (ec_joint_waypoint) — interpolate at
             time.time()
          2) legacy linear interp (ec_joint_cmd) — interpolate over the
             observed inter-command interval
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
                    # Trim stale waypoints for memory.
                    self._traj_interp.trim_before(t_now - 0.5)
                else:
                    self._traj_active = False

            if cmd_pose is not None:
                cmd_pose = self._apply_lpf(cmd_pose)
                self._current_pos = cmd_pose
                if cmd_names:
                    self._target_names = cmd_names
                self._publish(cmd_pose.tolist(), names=self._target_names)
                # If no new waypoint arrives for >1s, idle the queue (the last
                # value is already held by `holding the last point` logic
                # inside the interpolator).
                if queue_last is not None and t_now > queue_last + 1.0:
                    with self._traj_lock:
                        self._traj_interp.reset()
                        self._traj_active = False
                        self.get_logger().info("Waypoint queue idle >1s, cleared")
            return

        # ── Path 2: legacy linear interp ────────────────────────────────────
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
        raw = self._start_pos + t * (self._target_pos - self._start_pos)
        self._current_pos = self._apply_lpf(raw)
        self._publish(self._current_pos.tolist(), names=self._target_names)

    # ── helpers ──────────────────────────────────────────────────────────
    def _apply_lpf(self, target_np: np.ndarray) -> np.ndarray:
        """1st-order IIR low-pass: ``out = α·target + (1-α)·prev_out``.

        ``α=1.0`` passes target through unchanged; smaller values smooth
        harder. State is reset on shape change (e.g. joint count differs)."""
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

    # ── publish ──────────────────────────────────────────────────────────
    def _publish(self, positions: list, names: Optional[list] = None):
        """Build the configured msg type and publish it."""
        cls_name = self._pub_cls.__name__
        joint_names = list(names) if names else list(self.joint_names)
        if cls_name == "JointState":
            msg = JointState()
            if joint_names:
                msg.name = joint_names
            msg.position = positions
            self._pub.publish(msg)
            return
        if cls_name == "JointTrajectory":
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            msg = JointTrajectory()
            if joint_names:
                msg.joint_names = joint_names
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
