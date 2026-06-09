"""Standalone peg-watchdog node — runs SEPARATELY from EasyTrainer.

Watches the tutorial sim's peg pose and, when the peg has tipped over (its long
axis is far from vertical for a sustained period), re-places it: x,y randomized
within ±0.05 m of its home position, hole left fixed. This lets a curriculum /
DAgger run recover automatically after a grasp attempt knocks the peg down,
without any human intervention.

It only TALKS to the sim's existing services/topics — it does not touch the
EasyTrainer backend. Launch it on its own::

    ros2 run mujoco_world peg_watchdog_node
    # or: python3 peg_watchdog_node.py

Topics / services used (default prefix ``/tutorial``):
  Sub  <prefix>/object_poses_json   std_msgs/String  (JSON {peg:[x,y,z,qw,qx,qy,qz], ...})
  Cli  <prefix>/randomize_peg       std_srvs/Trigger
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _peg_tilt_cos(quat) -> float:
    """World-z component of the peg's local z-axis (its long axis) for MuJoCo
    quat [w,x,y,z]. 1.0 = perfectly upright, ~0 = lying on its side."""
    w, x, y, z = quat
    return 1.0 - 2.0 * (x * x + y * y)


class PegWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("peg_watchdog_node")

        self.declare_parameter("topic_prefix", "/tutorial")
        # Tipped when the upright-cosine drops below this (≈ >60° from vertical).
        self.declare_parameter("tip_cos_threshold", 0.5)
        # Must stay tipped this long before resetting (ignore transient knocks
        # the grasp might recover from, and mid-motion wobble).
        self.declare_parameter("tip_debounce_sec", 1.0)
        # After a reset, ignore the peg this long so the new drop can settle.
        self.declare_parameter("cooldown_sec", 3.0)
        # Never reset a peg above this height — it's being carried, not down.
        self.declare_parameter("max_reset_z", 0.05)

        self._prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        self._tip_cos = float(self.get_parameter("tip_cos_threshold").value)
        self._debounce = float(self.get_parameter("tip_debounce_sec").value)
        self._cooldown = float(self.get_parameter("cooldown_sec").value)
        self._max_reset_z = float(self.get_parameter("max_reset_z").value)

        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            String, f"{self._prefix}/object_poses_json", self._on_poses, reliable_qos)
        self._reset_cli = self.create_client(Trigger, f"{self._prefix}/randomize_peg")

        self._tipped_since: float | None = None   # monotonic time peg first seen tipped
        self._cooldown_until: float = 0.0
        self._reset_inflight = False

        self.get_logger().info(
            f"peg-watchdog up on '{self._prefix}': reset when tilt_cos < {self._tip_cos} "
            f"for {self._debounce}s (cooldown {self._cooldown}s)")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_poses(self, msg: String) -> None:
        now = self._now()
        if self._reset_inflight or now < self._cooldown_until:
            return
        try:
            data = json.loads(msg.data)
            peg = data.get("peg")
        except Exception:
            return
        if not peg or len(peg) < 7:
            return

        # Only ever reset a peg that is DOWN ON THE GROUND. A peg being carried
        # by the gripper is high (z well above the plate) and may momentarily
        # look tilted during the reach — never reset that, or we'd knock the peg
        # out of a good grasp mid-task.
        if float(peg[2]) > self._max_reset_z:
            self._tipped_since = None
            return

        tilt_cos = _peg_tilt_cos(peg[3:7])
        if tilt_cos >= self._tip_cos:
            self._tipped_since = None          # upright enough → clear timer
            return

        # peg is tipped — start / continue the debounce timer.
        if self._tipped_since is None:
            self._tipped_since = now
            self.get_logger().info(f"peg tipped (tilt_cos={tilt_cos:.2f}) — watching...")
            return
        if (now - self._tipped_since) < self._debounce:
            return

        # sustained tip → reset the peg.
        self._tipped_since = None
        self._trigger_reset(tilt_cos)

    def _trigger_reset(self, tilt_cos: float) -> None:
        if not self._reset_cli.service_is_ready():
            # service not up yet (sim down?) — try again on the next pose msg.
            self.get_logger().warn("randomize_peg service not available; will retry")
            return
        self._reset_inflight = True
        self.get_logger().info(f"peg down (tilt_cos={tilt_cos:.2f}) → randomize_peg (±0.05, hole fixed)")
        fut = self._reset_cli.call_async(Trigger.Request())

        def _done(f):
            self._reset_inflight = False
            self._cooldown_until = self._now() + self._cooldown
            try:
                resp = f.result()
                self.get_logger().info(f"reset {'ok' if resp and resp.success else 'failed'}")
            except Exception as e:
                self.get_logger().error(f"reset call error: {e}")

        fut.add_done_callback(_done)


def main(args=None):
    rclpy.init(args=args)
    node = PegWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
