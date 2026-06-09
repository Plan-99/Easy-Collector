"""Tutorial scene motion planner — ROS2 service node.

Provides ``/tutorial/run_episode`` (std_srvs/Trigger) that drives the tutorial
arm through a peg-in-hole trajectory using Pinocchio IK + linear joint-space
interpolation. Designed to be invoked by the Model Tester data collector
while the backend records joint_states/images in parallel.

Run with::

    ros2 run mujoco_world tutorial_planner_node

Topics:
  Sub  /tutorial/joint_states         sensor_msgs/JointState
  Sub  /tutorial/object_poses_json    std_msgs/String  (JSON)
  Pub  /tutorial/joint_command        sensor_msgs/JointState

Services:
  /tutorial/run_episode  std_srvs/Trigger  — execute one full peg-in-hole
                                                episode and return when done.

Behavior:
  * Scene-aware: branches on ``scene_id`` (inferred from which freejoint
    bodies are present in /tutorial/object_poses_json).
  * Defensive: if peg/hole pose unknown after a few seconds, fail fast
    (Trigger.response.success=False) so the collector can move on.
  * Idempotent on parallel invocations: a re-entry while one episode is
    already running just returns success=False with a clear message.
"""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Optional

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TOOL_JOINT = "gripper"


def _quat_to_rot(quat) -> np.ndarray:
    """quat = [w, x, y, z] → 3x3 rotation matrix."""
    w, x, y, z = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


class IK:
    """Damped least-squares 6DOF IK around the EE frame ``ee``.

    EE frame is defined as a local SE3 offset on top of link6 — same convention
    as `TUTORIAL_ROBOT['settings']['ik_setting']` in `tutorial_defaults.py`.
    """

    EE_PARENT_FRAME = "joint6"     # joint name in URDF (= frame after that joint)
    # link6-local offset of the gripper-opening centerline (where the fingers
    # close around an object). NOT the same as the "ee_site" used by IK in
    # tutorial_defaults.py (0.095) — that's beyond the fingertips and gives
    # a grasp that misses the object. 0.075 = gripper_base(0.045) + finger
    # forward offset(0.030).
    EE_OFFSET = np.array([0.075, 0.0, 0.0])

    def __init__(self, urdf_path: str) -> None:
        # Lock gripper joints — we only want the 6-DOF arm for IK.
        self.model = pin.buildModelFromUrdf(urdf_path)
        gripper_ids = []
        for name in ("gripper", "gripper_mirror"):
            if self.model.existJointName(name):
                gripper_ids.append(self.model.getJointId(name))
        if gripper_ids:
            reduced = pin.buildReducedModel(
                self.model,
                list_of_joints_to_lock=gripper_ids,
                reference_configuration=pin.neutral(self.model),
            )
            self.model = reduced
        self.data = self.model.createData()

        # Attach an EE frame as an offset of link6.
        parent_frame = self.model.getFrameId("link6")
        if parent_frame >= self.model.nframes:
            raise RuntimeError("URDF has no 'link6' frame")
        parent_joint = self.model.frames[parent_frame].parentJoint
        placement = pin.SE3(np.eye(3), self.EE_OFFSET)
        self.ee_frame_id = self.model.addFrame(
            pin.Frame("ee", parent_joint, parent_frame, placement, pin.FrameType.OP_FRAME)
        )
        # NOTE: addFrame requires rebuilding data so it sees the new frame.
        self.data = self.model.createData()

        # Map joint name → URDF q-index. For revolute joints with 1-DOF this is
        # `joint_id - 1` (skipping the universe joint at index 0).
        self.q_index = {}
        for j_id in range(1, self.model.njoints):
            j_name = self.model.names[j_id]
            self.q_index[j_name] = self.model.joints[j_id].idx_q

    def solve(self, target_xyz: np.ndarray, target_R: np.ndarray,
              q_init: np.ndarray, max_iters: int = 200, tol: float = 1e-4,
              damping: float = 1e-3) -> tuple[np.ndarray, bool]:
        """Damped Newton-Raphson IK in SE3. Returns (q, converged)."""
        q = q_init.copy()
        target_SE3 = pin.SE3(target_R, target_xyz)
        for _ in range(max_iters):
            pin.framesForwardKinematics(self.model, self.data, q)
            current = self.data.oMf[self.ee_frame_id]
            err = pin.log6(current.inverse() * target_SE3).vector  # 6-vec twist
            if np.linalg.norm(err) < tol:
                return q, True
            J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id)
            JJt = J @ J.T + (damping ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, err)
            q = pin.integrate(self.model, q, dq * 0.5)  # 0.5 = step size
        return q, False


# Stock "EE points world -Z" orientation. Local X axis of link6 is the gripper
# forward direction in the URDF; rotating it onto world -Z requires a -90° pitch
# about world Y.
def _down_orientation(yaw_rad: float) -> np.ndarray:
    """Rotation matrix with EE x-axis pointing world -z and y-axis rotated
    around world z by `yaw_rad` (so the peg gripper opening lines up with peg
    yaw)."""
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    # Yaw about world z applied to a base "down" frame.
    base = np.array([
        [0.0,  0.0, 1.0],   # column 0: where local X maps to in world (down → +x_world * 0, y*0, z*1? we want -z)
        [0.0,  1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ])
    # base maps local x → world -z, local y → world y, local z → world x.
    # Then rotate by yaw about world z.
    Rz = np.array([
        [cy, -sy, 0.0],
        [sy,  cy, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return Rz @ base


class TutorialPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("tutorial_planner_node")

        self.declare_parameter("topic_prefix", "/tutorial")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("plan_hz", 20.0)
        self.declare_parameter("approach_height", 0.10)
        self.declare_parameter("insert_depth", 0.012)
        # gripper_open: 0.020 keeps the finger center within the gripper_base
        # footprint (which extends ±0.030 in y) and is plenty wide to clear a
        # 14 mm peg.
        # gripper_closed=0.000: setting target *below* the contact equilibrium
        # is what creates grip force. The finger's actual position will settle
        # at ~0.005 (where the inner face contacts the peg surface), and with
        # target=0 the position actuator continuously generates F = kp*(0 -
        # 0.005) = -2 N — a 2 N normal pinch on each side. Setting target=0.005
        # leaves zero steady-state error and zero grip force (peg slips out).
        self.declare_parameter("gripper_open", 0.020)
        self.declare_parameter("gripper_closed", 0.000)
        # Total joint travel per waypoint hop is capped at this many radians/m
        # per step so the sim doesn't see step-function commands.
        self.declare_parameter("max_step_rad", 0.05)

        self._prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        urdf_path = str(self.get_parameter("urdf_path").value)
        if not urdf_path:
            urdf_path = os.path.join(
                get_package_share_directory("mujoco_world"), "urdf", "tutorial_arm.urdf"
            )
        if not os.path.isfile(urdf_path):
            # Fall back to the in-tree path the bind-mount exposes.
            alt = "/root/ros2_ws/src/mujoco_world/urdf/tutorial_arm.urdf"
            if os.path.isfile(alt):
                urdf_path = alt
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"tutorial_arm.urdf not found: {urdf_path}")

        self.get_logger().info(f"Loading URDF for IK: {urdf_path}")
        self.ik = IK(urdf_path)

        self._plan_hz = float(self.get_parameter("plan_hz").value)
        self._approach_h = float(self.get_parameter("approach_height").value)
        self._insert_depth = float(self.get_parameter("insert_depth").value)
        self._grip_open = float(self.get_parameter("gripper_open").value)
        self._grip_closed = float(self.get_parameter("gripper_closed").value)
        self._max_step_rad = float(self.get_parameter("max_step_rad").value)

        # --- State caches ----------------------------------------------
        self._lock = threading.Lock()
        self._latest_joints: Optional[dict] = None  # name → position
        self._latest_poses: Optional[dict] = None
        self._busy = False

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Without an explicit reentrant callback group the default mutually
        # exclusive group serializes ALL callbacks on this node. That means
        # the long-running run_episode service blocks the joint_states /
        # object_poses subscriptions for the entire trajectory — `latest_joints`
        # never updates and the planner can't see the sim's progress.
        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            JointState, f"{self._prefix}/joint_states",
            self._on_joint_state, reliable_qos, callback_group=cb_group)
        self.create_subscription(
            String, f"{self._prefix}/object_poses_json",
            self._on_object_poses, reliable_qos, callback_group=cb_group)
        self._cmd_pub = self.create_publisher(
            JointState, f"{self._prefix}/joint_command", reliable_qos,
            callback_group=cb_group)

        self.create_service(
            Trigger, f"{self._prefix}/run_episode", self._on_run_episode,
            callback_group=cb_group)

        # Ground-truth success client. After the trajectory finishes the
        # planner asks the world node whether the peg actually landed in the
        # hole and reports that back as the service's `success`. Record_episode
        # already discards on service-fail, so this single hook gives the
        # UI-driven flow the same ground-truth filtering the CLI collector did.
        self._check_success_client = self.create_client(
            Trigger, f"{self._prefix}/check_success", callback_group=cb_group)
        # Scene randomizer. /tutorial/randomize snaps the arm to home and
        # re-samples peg / hole_base poses per randomize_ranges. Called once at
        # the start of every run_episode so each collected episode sees a
        # different scene without the UI flow needing a second service call.
        self._randomize_client = self.create_client(
            Trigger, f"{self._prefix}/randomize", callback_group=cb_group)

        self.get_logger().info(
            f"Tutorial planner ready on prefix='{self._prefix}'")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        names = list(msg.name) or ARM_JOINTS + [TOOL_JOINT]
        positions = list(msg.position)
        with self._lock:
            self._latest_joints = {n: float(p) for n, p in zip(names, positions)}

    def _on_object_poses(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                with self._lock:
                    self._latest_poses = data
        except Exception:
            pass  # ignore malformed payloads

    # ------------------------------------------------------------------
    # Service
    # ------------------------------------------------------------------

    def _on_run_episode(self, request: Trigger.Request,
                         response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            if self._busy:
                response.success = False
                response.message = "planner already running"
                return response
            self._busy = True
        try:
            ok, msg = self._run_one_episode()
            response.success = bool(ok)
            response.message = msg
        except Exception as e:
            response.success = False
            response.message = f"planner exception: {e!r}"
            self.get_logger().error(response.message)
        finally:
            with self._lock:
                self._busy = False
        return response

    # ------------------------------------------------------------------
    # Planner core
    # ------------------------------------------------------------------

    def _wait_for_state(self, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._latest_joints is not None and self._latest_poses is not None:
                    return True
            time.sleep(0.05)
        return False

    def _current_q(self) -> Optional[np.ndarray]:
        with self._lock:
            j = self._latest_joints
        if j is None:
            return None
        # Order according to the IK model — arm joints only.
        q = np.zeros(len(self.ik.q_index))
        for name, qidx in self.ik.q_index.items():
            q[qidx] = j.get(name, 0.0)
        return q

    def _fk_ee_xyz(self, q: np.ndarray) -> np.ndarray:
        """Forward-kinematics: world xyz of the EE frame for joint config q."""
        pin.framesForwardKinematics(self.ik.model, self.ik.data, q)
        return np.asarray(self.ik.data.oMf[self.ik.ee_frame_id].translation, dtype=float)

    def _current_gripper(self) -> float:
        with self._lock:
            j = self._latest_joints
        if j is None:
            return self._grip_open
        return float(j.get(TOOL_JOINT, self._grip_open))

    def _publish_joints(self, q_arm: np.ndarray, gripper: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + [TOOL_JOINT]
        positions = [0.0] * 7
        for name in ARM_JOINTS:
            positions[ARM_JOINTS.index(name)] = float(q_arm[self.ik.q_index[name]])
        positions[6] = float(gripper)
        msg.position = positions
        self._cmd_pub.publish(msg)

    def _interp_to(self, q_target: np.ndarray, grip_target: float,
                   q_start: Optional[np.ndarray] = None,
                   grip_start: Optional[float] = None,
                   hold_steps: int = 25) -> np.ndarray:
        """Linearly interpolate from current (or supplied start) to target,
        publishing at plan_hz. Returns the final joint configuration."""
        if q_start is None:
            q_start = self._current_q()
            if q_start is None:
                q_start = q_target.copy()
        if grip_start is None:
            grip_start = self._current_gripper()

        delta = q_target - q_start
        max_change = float(np.max(np.abs(delta))) if delta.size else 0.0
        steps_arm = int(np.ceil(max_change / max(self._max_step_rad, 1e-3)))
        grip_change = abs(grip_target - grip_start)
        steps_grip = int(np.ceil(grip_change / max(self._max_step_rad, 1e-3)))
        n_steps = max(1, steps_arm, steps_grip)

        period = 1.0 / max(1.0, self._plan_hz)
        for k in range(1, n_steps + 1):
            alpha = k / n_steps
            q_now = q_start + alpha * delta
            g_now = grip_start + alpha * (grip_target - grip_start)
            self._publish_joints(q_now, g_now)
            time.sleep(period)
        # Hold a few cycles so the sim PD has time to settle.
        for _ in range(hold_steps):
            self._publish_joints(q_target, grip_target)
            time.sleep(period)
        return q_target

    def _ik_to_pose(self, xyz: np.ndarray, yaw: float,
                     q_seed: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        if q_seed is None:
            q_seed = self._current_q()
            if q_seed is None:
                q_seed = np.zeros(len(self.ik.q_index))
        R = _down_orientation(yaw)
        q_sol, ok = self.ik.solve(np.asarray(xyz, dtype=float), R, q_seed)
        if not ok:
            self.get_logger().warn(
                f"IK did not converge for xyz={xyz.tolist()} yaw={yaw:.3f}")
            return None
        # Clamp arm joints to URDF limits.
        for i, j_id in enumerate(range(1, self.ik.model.njoints)):
            j = self.ik.model.joints[j_id]
            lo = self.ik.model.lowerPositionLimit[j.idx_q]
            hi = self.ik.model.upperPositionLimit[j.idx_q]
            if not np.isfinite(lo) or not np.isfinite(hi):
                continue
            q_sol[j.idx_q] = float(np.clip(q_sol[j.idx_q], lo, hi))
        return q_sol

    def _run_one_episode(self) -> tuple[bool, str]:
        if not self._wait_for_state(timeout=5.0):
            return False, "no joint_state or object_poses cached"

        # Pre-snapshot _latest_poses so we can detect when the post-randomize
        # publish lands (poses changed) instead of trusting a fixed sleep.
        with self._lock:
            poses_pre = dict(self._latest_poses or {})

        sampled_poses: dict | None = None
        if self._randomize_client.wait_for_service(timeout_sec=2.0):
            fut = self._randomize_client.call_async(Trigger.Request())
            t_deadline = time.time() + 5.0
            while not fut.done() and time.time() < t_deadline:
                time.sleep(0.02)
            if not fut.done():
                self.get_logger().warn("randomize call timed out; proceeding with current scene")
            else:
                rresp = fut.result()
                if not (rresp and rresp.success):
                    self.get_logger().warn(
                        f"randomize failed: {getattr(rresp,'message','?')}; using current scene")
                else:
                    self.get_logger().info(f"randomized scene: {rresp.message}")
                    try:
                        parsed = json.loads(rresp.message or "{}")
                        sampled_poses = parsed.get("sampled") or None
                    except Exception:
                        sampled_poses = None
        else:
            self.get_logger().warn("/tutorial/randomize unavailable; using current scene")

        # Wait for the publish thread to catch up — up to 2s, polling for an
        # _latest_poses snapshot whose peg differs from the pre-randomize one.
        deadline = time.time() + 2.0
        while time.time() < deadline:
            with self._lock:
                cur = dict(self._latest_poses or {})
            if cur and (cur.get("peg") != poses_pre.get("peg")):
                break
            time.sleep(0.05)

        with self._lock:
            poses = dict(self._latest_poses or {})

        # Prefer the sampled poses we got back from /tutorial/randomize — that
        # is the authoritative state immediately after the write, before any
        # mj_step settling makes the topic-side poses look slightly different.
        # Falling back to _latest_poses only when the randomize response was
        # missing or unparseable.
        if sampled_poses:
            for name, qpos in sampled_poses.items():
                poses[name] = list(qpos)

        peg = poses.get("peg")
        hole = poses.get("hole_base")
        if peg is None or hole is None:
            return False, f"missing peg/hole in poses: keys={list(poses)}"

        peg_xyz = np.array(peg[:3], dtype=float)
        hole_xyz = np.array(hole[:3], dtype=float)
        # Pegs and hole are symmetric so yaw 0 is fine; use hole's yaw for the
        # alignment if non-zero.
        from math import atan2
        hw, _, _, hz = hole[3:7]  # quat = [w, x, y, z]
        hole_yaw = atan2(2.0 * hw * hz, 1.0 - 2.0 * hz * hz)

        hole_top = hole_xyz + np.array([0.0, 0.0, 0.025])  # marker site offset
        plate_top_z = hole_xyz[2] + 0.005
        # When the gripper is grasping the peg with its centerline at
        # (peg_x, peg_y, peg_z + grasp_offset) — i.e. above peg center by
        # grasp_offset — the peg origin sits grasp_offset below the gripper.
        # For "peg bottom on plate top" the gripper must end up at:
        #   gripper_z = plate_top_z + peg_half_height(0.030) + grasp_offset
        # so when the gripper releases the peg falls to its rest pose with
        # bottom exactly on the plate.

        approach = self._approach_h
        grasp_offset = 0.005     # grasp 5 mm above peg origin so fingers wrap mid-peg
        insert_z = plate_top_z + 0.030 + grasp_offset

        waypoints = [
            ("open_above_peg",   peg_xyz + np.array([0, 0, approach]), 0.0,       self._grip_open),
            ("descend_to_peg",   peg_xyz + np.array([0, 0, grasp_offset]), 0.0,    self._grip_open),
            ("close_gripper",    peg_xyz + np.array([0, 0, grasp_offset]), 0.0,    self._grip_closed),
            ("lift_peg",         peg_xyz + np.array([0, 0, approach]),   0.0,      self._grip_closed),
            ("over_hole",        np.array([hole_top[0], hole_top[1], hole_top[2] + approach]),
                                                                          hole_yaw, self._grip_closed),
            ("insert",           np.array([hole_top[0], hole_top[1], insert_z]),
                                                                          hole_yaw, self._grip_closed),
            ("release",          np.array([hole_top[0], hole_top[1], insert_z]),
                                                                          hole_yaw, self._grip_open),
            ("retreat",          np.array([hole_top[0], hole_top[1], hole_top[2] + approach]),
                                                                          hole_yaw, self._grip_open),
        ]

        q_seed = self._current_q()
        gripper_now = self._current_gripper()
        for name, xyz, yaw, grip in waypoints:
            q_target = self._ik_to_pose(xyz, yaw, q_seed=q_seed)
            if q_target is None:
                return False, f"IK failed at waypoint '{name}'"
            target_xyz = np.asarray(xyz, dtype=float)
            ik_xyz = self._fk_ee_xyz(q_target)
            ik_err = float(np.linalg.norm(ik_xyz - target_xyz))
            self.get_logger().info(
                f"WP {name} → tgt=({target_xyz[0]:.3f},{target_xyz[1]:.3f},{target_xyz[2]:.3f}) "
                f"grip={grip} ik_err={ik_err*1000:.2f}mm")
            q_seed = self._interp_to(q_target, grip, q_start=q_seed, grip_start=gripper_now)
            gripper_now = grip
            with self._lock:
                peg_now = (self._latest_poses or {}).get("peg")
                jnt_now = self._latest_joints
            # FK on the *actual* joint state the sim achieved → compare to target
            # so we can localize whether issues are IK convergence or PD lag.
            if jnt_now is not None:
                q_actual = np.zeros(len(self.ik.q_index))
                for jn, qidx in self.ik.q_index.items():
                    q_actual[qidx] = jnt_now.get(jn, 0.0)
                actual_xyz = self._fk_ee_xyz(q_actual)
                track_err = float(np.linalg.norm(actual_xyz - target_xyz))
                achieved_grip = jnt_now.get(TOOL_JOINT, -1.0)
                peg_str = (
                    f"peg=({peg_now[0]:.3f},{peg_now[1]:.3f},{peg_now[2]:.3f}) "
                    if peg_now is not None else "peg=?"
                )
                self.get_logger().info(
                    f"  → ee_act=({actual_xyz[0]:.3f},{actual_xyz[1]:.3f},{actual_xyz[2]:.3f}) "
                    f"track_err={track_err*1000:.2f}mm grip_pos={achieved_grip:.4f} {peg_str}"
                )

        # Settle a moment so the peg's free-joint dynamics resolve before we
        # query the world for ground-truth success.
        time.sleep(0.4)
        gt_ok, gt_msg = self._check_ground_truth_success(timeout=2.0)
        if not gt_ok:
            return False, f"ground_truth_fail: {gt_msg}"
        return True, f"trajectory complete; gt_ok ({gt_msg})"

    def _check_ground_truth_success(self, timeout: float = 2.0) -> tuple[bool, str]:
        cli = self._check_success_client
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, "check_success unavailable"
        future = cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not future.done() and (time.monotonic() - t0) < timeout:
            time.sleep(0.02)
        if not future.done():
            return False, "check_success timeout"
        resp = future.result()
        if resp is None:
            return False, "check_success returned None"
        return bool(resp.success), (resp.message or "")


def main(args=None):
    rclpy.init(args=args)
    node = TutorialPlannerNode()
    # Use a multi-threaded executor so the service callback can run while the
    # joint_state / object_poses subscriptions keep firing.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
