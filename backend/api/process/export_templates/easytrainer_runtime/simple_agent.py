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
from sensor_msgs.msg import JointState as _JointStateMsg


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

        # When ``interpolation`` is true the agent does NOT publish to the
        # robot's write_topic directly — instead it publishes JointState goals
        # to /ec_robot_<id>/ec_joint_cmd (low rate) and the bundled
        # JointInterpolationNode smooths them to ``write_topic`` at 200Hz.
        # ``move_to`` bypasses smoothing via /ec_robot_<id>/ec_joint_cmd_direct
        # (it already does its own smoothstep interpolation).
        self.interpolation: bool = bool(robot.get('interpolation', False))

        if self.write_type != 'topic':
            raise NotImplementedError(
                f"SimpleAgent only supports write_type='topic'. "
                f"Robot {self.name} uses write_type='{self.write_type}'. "
                f"Run inference inside EasyTrainer instead."
            )

        self._js_lock = threading.Lock()
        self._joint_states: Optional[List[float]] = None
        self._last_joint_update: Optional[float] = None

        # ── move_to interpolation state ──────────────────────────────────
        # SimpleAgent.move_joint_step publishes a single command with no
        # smoothing. The planner's joint_position / query_pose / homepose
        # blocks need a duration-based interpolated move, so move_to() spins a
        # background thread that smoothsteps from the current qpos to the
        # target over `duration` seconds. is_moving / cancel_move_to mirror the
        # EasyTrainer RemoteAgent API the planner engine expects.
        self._move_lock = threading.Lock()
        self._move_thread: Optional[threading.Thread] = None
        self._cancel_move = False
        self._is_moving = False

        # Resolve message classes
        try:
            self._read_cls = get_message(self.read_topic_msg)
        except Exception as e:
            raise RuntimeError(
                f"Could not resolve read_topic_msg='{self.read_topic_msg}' for "
                f"robot {self.name}. Is the corresponding ROS package installed? "
                f"Original error: {e}"
            )
        # Resolve / open the write path. With interpolation on we ALWAYS use
        # JointState on the in-process bridge topics — the interpolation node
        # handles serialization to the robot's actual write_topic_msg.
        if self.interpolation:
            self._write_cls = _JointStateMsg
            ns = f"/ec_robot_{self.id}"
            self._interp_pub = node.create_publisher(_JointStateMsg, f"{ns}/ec_joint_cmd", 10)
            self._direct_pub = node.create_publisher(_JointStateMsg, f"{ns}/ec_joint_cmd_direct", 10)
            self._write_pub = None
        else:
            try:
                self._write_cls = get_message(self.write_topic_msg)
            except Exception as e:
                raise RuntimeError(
                    f"Could not resolve write_topic_msg='{self.write_topic_msg}' for "
                    f"robot {self.name}. Is the corresponding ROS package installed? "
                    f"Original error: {e}"
                )
            self._interp_pub = None
            self._direct_pub = None
            self._write_pub = node.create_publisher(self._write_cls, self.write_topic, 10)

        self._read_sub = node.create_subscription(
            self._read_cls, self.read_topic, self._joint_state_cb, 10
        )

        if self.interpolation:
            print(f"[SimpleAgent] {self.name}: read {self.read_topic} ({self.read_topic_msg}) "
                  f"→ interpolation_node → {self.write_topic} ({self.write_topic_msg}) "
                  f"joint_names={self.joint_names}")
        else:
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
            # If joint name list is present in the message (as `name` for
            # sensor_msgs/JointState or `joint_names` for control_msgs/
            # JointTrajectoryControllerState), reorder positions to match the
            # training-time canonical order. Otherwise trust msg order.
            msg_names = getattr(msg, 'name', None) or getattr(msg, 'joint_names', None)
            if (
                self.joint_names
                and msg_names
                and len(msg_names) == len(positions)
            ):
                name_to_idx = {n: i for i, n in enumerate(msg_names)}
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
        # control_msgs/JointTrajectoryControllerState — check before sensor_msgs/JointState
        # because this msg also has no top-level `position`, but nested `actual.positions`.
        if hasattr(msg, 'actual') and hasattr(msg.actual, 'positions') and msg.actual.positions is not None:
            try:
                return list(msg.actual.positions)
            except TypeError:
                pass
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
        """Publish a single joint command. Mirrors EasyTrainer Agent.move_joint_step.
        With interpolation on, the goal is sent to the bridge topic and the
        JointInterpolationNode smooths it to ``write_topic`` at 200Hz; with
        interpolation off, it goes straight to ``write_topic``."""
        target = list(map(float, target_qpos[: self.joint_len]))
        if self.interpolation:
            msg = _JointStateMsg()
            if self.joint_names:
                msg.name = list(self.joint_names)
            msg.position = target
            self._interp_pub.publish(msg)
            return
        msg = self._build_command_message(target)
        if msg is None:
            return
        self._write_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────────
    # Duration-based interpolated motion (move_to / cancel / is_moving)
    # ────────────────────────────────────────────────────────────────────
    @property
    def is_moving(self) -> bool:
        return self._is_moving

    def move_to(self, target_qpos, duration: float = 5.0, hz: float = 100.0) -> None:
        """Smoothly interpolate from the current qpos to ``target_qpos`` over
        ``duration`` seconds, publishing joint commands at ``hz``.

        Non-blocking — the interpolation runs in a background thread. Poll
        ``is_moving`` to know when it finishes, or call ``cancel_move_to`` to
        abort. Mirrors EasyTrainer's RemoteAgent.move_to so the planner engine
        can treat SimpleAgent the same way.
        """
        target = list(map(float, list(target_qpos)[: self.joint_len]))
        with self._move_lock:
            # Pre-empt any in-flight move.
            if self._move_thread is not None and self._move_thread.is_alive():
                self._cancel_move = True
                self._move_thread.join(timeout=1.0)
            self._cancel_move = False
            self._is_moving = True
            self._move_thread = threading.Thread(
                target=self._move_to_worker,
                args=(target, float(duration), float(hz)),
                daemon=True,
            )
            self._move_thread.start()

    def _move_to_worker(self, target: List[float], duration: float, hz: float) -> None:
        try:
            start_qpos = list(map(float, self.get_joint_states()[: self.joint_len]))
            n = min(len(start_qpos), len(target))
            if n == 0:
                return
            start_qpos = start_qpos[:n]
            target = target[:n]
            hz = max(hz, 1.0)
            steps = max(1, int(duration * hz))
            period = 1.0 / hz
            for i in range(1, steps + 1):
                if self._cancel_move:
                    break
                t = i / steps
                # smoothstep easing — soft accel/decel, no jerk at the ends.
                s = t * t * (3.0 - 2.0 * t)
                interp = [sp + (tp - sp) * s for sp, tp in zip(start_qpos, target)]
                # With interpolation on, bypass the smoothing node — the
                # smoothstep above is already a smooth trajectory; routing it
                # through the 200Hz linear smoother would just add lag.
                if self.interpolation:
                    msg = _JointStateMsg()
                    if self.joint_names:
                        msg.name = list(self.joint_names)
                    msg.position = interp
                    self._direct_pub.publish(msg)
                else:
                    self.move_joint_step(interp)
                time.sleep(period)
        except Exception as e:  # noqa: BLE001
            print(f"[SimpleAgent {self.name}] move_to worker error: {e}")
        finally:
            self._is_moving = False

    def cancel_move_to(self) -> None:
        """Signal the in-flight move_to thread to stop publishing."""
        self._cancel_move = True

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
