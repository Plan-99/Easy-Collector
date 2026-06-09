"""Tutorial scene auto data collector.

Runs inside the ros2 container. Drives the tutorial scene through N
randomized episodes by calling:

    /tutorial/randomize         (Trigger)
    /tutorial/run_episode       (Trigger)   — long-running planner episode
    /tutorial/check_success     (Trigger)   — ground-truth success check

While each episode runs, a background thread snapshots the latest
joint_states + joint_command + compressed camera frames at a fixed rate
into in-memory buffers. **Only successful episodes are written to disk**,
matching the user's rule that the dataset must never contain "almost
worked" trajectories.

Disk layout — one directory per successful episode::

    /opt/easytrainer/training_data/raw_episodes/<uuid>/
        meta.json              # scene_id, success, metrics, fps, sizes,
                               # cameras (slugs in capture order)
        state.npy              # (T, 7) float32  — observation.state
        action.npy             # (T, 7) float32  — joint_command at same tick
        ee_pos.npy             # (T, 6) float32  — EE pose [x,y,z,rx,ry,rz]
        timestamps.npy         # (T,)   float32  — seconds since episode start
        <cam_slug>/frame_NNNN.jpg   (one dir per --cameras entry)

The backend-side assembler walks these directories and packs them into a
LeRobot v2.1 dataset (videos + parquet) — see ``backend/tools/model_tester``.

Usage (inside ros2 container)::

    ros2 run mujoco_world tutorial_collector \\
        --num-episodes 50 --output-dir /opt/easytrainer/training_data/raw_episodes
"""
from __future__ import annotations

import argparse
import json
import os
import threading
import time
import uuid
from pathlib import Path
from typing import Optional

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TOOL_JOINT = "gripper"
ALL_JOINTS = ARM_JOINTS + [TOOL_JOINT]

DEFAULT_OUTPUT = "/opt/easytrainer/training_data/raw_episodes"

# URDF used for FK. Must match the MJCF kinematic chain — same offsets/axes.
DEFAULT_URDF = "/root/ros2_ws/src/mujoco_world/urdf/tutorial_arm.urdf"

# EE point in link6 frame: gripper_base(0.045) + ee_site(0.05) = 0.095. Same
# point the MJCF declares as ``ee_site`` and that fk_raw_episodes.py reads.
# Use this so any retroactive FK pass matches the on-line collector.
EE_OFFSET = np.array([0.095, 0.0, 0.0])


class _EEFKSolver:
    """Pinocchio FK from arm qpos (6) to EE 6-vec [x,y,z,rx,ry,rz].

    Builds a *reduced* model with gripper joints locked (matches the planner).
    Locked-joint configuration uses pin.neutral so the gripper position
    doesn't shift the EE — fine because EE_OFFSET is anchored at link6.
    """

    def __init__(self, urdf_path: str = DEFAULT_URDF) -> None:
        model = pin.buildModelFromUrdf(urdf_path)
        locked = [model.getJointId(n) for n in ("gripper", "gripper_mirror")
                  if model.existJointName(n)]
        if locked:
            model = pin.buildReducedModel(
                model, list_of_joints_to_lock=locked,
                reference_configuration=pin.neutral(model),
            )
        link6 = model.getFrameId("link6")
        parent_joint = model.frames[link6].parentJoint
        self.ee_frame_id = model.addFrame(
            pin.Frame("ee", parent_joint, link6,
                       pin.SE3(np.eye(3), EE_OFFSET),
                       pin.FrameType.OP_FRAME)
        )
        self.model = model
        self.data = model.createData()

    def fk(self, arm_qpos6: np.ndarray) -> np.ndarray:
        """arm_qpos6: (6,) — j1..j6. Returns (6,) = [x,y,z,rx,ry,rz]."""
        q = np.asarray(arm_qpos6, dtype=np.float64).reshape(self.model.nq)
        pin.framesForwardKinematics(self.model, self.data, q)
        T = self.data.oMf[self.ee_frame_id]
        rvec = pin.log3(T.rotation)
        out = np.empty(6, dtype=np.float32)
        out[:3] = T.translation
        out[3:] = rvec
        return out


class CollectorNode(Node):
    def __init__(self, prefix: str = "/tutorial", record_hz: float = 30.0,
                 urdf_path: str = DEFAULT_URDF,
                 cameras: Optional[list[str]] = None):
        super().__init__("tutorial_collector_node")
        self._prefix = prefix.rstrip("/")
        self._record_hz = float(record_hz)
        # Build FK solver once — the recorder thread calls .fk() per tick.
        # Pinocchio's framesForwardKinematics is ~30 µs per call on a 6-DoF
        # model, so this is well below our 30 Hz budget.
        self._fk = _EEFKSolver(urdf_path)

        # Default = round_1 2-wrist setup. Order here is the dataset capture
        # order — assembler maps it to sensor_<id> columns in same order.
        self._camera_names = list(cameras) if cameras else ["wrist_cam", "wrist_cam_down"]

        self._lock = threading.Lock()
        self._latest_state: Optional[JointState] = None
        self._latest_command: Optional[JointState] = None
        self._latest_images: dict[str, Optional[CompressedImage]] = {
            name: None for name in self._camera_names
        }

        cb = ReentrantCallbackGroup()
        # Control topics (joint_states, joint_command): publisher uses
        # RELIABLE, so match that.
        ctrl_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Cameras: mujoco_world_node publishes images with BEST_EFFORT QoS
        # (sensor-data convention). Subscribing as RELIABLE causes a QoS
        # mismatch and the subscription silently drops every frame.
        img_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            JointState, f"{self._prefix}/joint_states",
            self._on_state, ctrl_qos, callback_group=cb)
        self.create_subscription(
            JointState, f"{self._prefix}/joint_command",
            self._on_cmd, ctrl_qos, callback_group=cb)
        for cam in self._camera_names:
            self.create_subscription(
                CompressedImage,
                f"{self._prefix}/{cam}/image_raw/compressed",
                self._make_on_image(cam),
                img_qos,
                callback_group=cb,
            )

        self._cli_random = self.create_client(
            Trigger, f"{self._prefix}/randomize", callback_group=cb)
        self._cli_run = self.create_client(
            Trigger, f"{self._prefix}/run_episode", callback_group=cb)
        self._cli_check = self.create_client(
            Trigger, f"{self._prefix}/check_success", callback_group=cb)

    # ------------------------------------------------------------------
    # Subscriptions just stash the latest message; recording thread snaps
    # ------------------------------------------------------------------
    def _on_state(self, msg: JointState) -> None:
        with self._lock:
            self._latest_state = msg

    def _on_cmd(self, msg: JointState) -> None:
        with self._lock:
            self._latest_command = msg

    def _make_on_image(self, cam_name: str):
        # Bind cam_name into a separate callback closure per camera so the
        # subscriber writes to the right slot in _latest_images.
        def _cb(msg: CompressedImage) -> None:
            with self._lock:
                self._latest_images[cam_name] = msg
        return _cb

    # ------------------------------------------------------------------
    # Service helpers
    # ------------------------------------------------------------------
    def call_trigger(self, client, timeout: float = 60.0) -> Trigger.Response:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service {client.srv_name} not available")
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError(f"timeout on {client.srv_name}")
            time.sleep(0.05)
        return future.result()

    def wait_for_subs(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                state_ready = self._latest_state is not None
                cams_ready = all(
                    self._latest_images.get(c) is not None
                    for c in self._camera_names
                )
                if state_ready and cams_ready:
                    return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------------
    # Episode loop
    # ------------------------------------------------------------------
    def collect_episode(self, output_dir: Path) -> tuple[bool, dict]:
        # Tell the planner where to put the objects this run.
        rnd = self.call_trigger(self._cli_random)
        if not rnd.success:
            return False, {"reason": f"randomize_failed: {rnd.message}"}
        # Give the sim a beat to stabilize after the freejoints jump.
        time.sleep(0.5)

        # Reset latest_command so we don't capture stale commands from the
        # previous episode's retreat motion.
        with self._lock:
            self._latest_command = None

        # Start recorder thread, then run the (blocking) episode service.
        buf = _EpisodeBuffer(self._camera_names)
        stop = threading.Event()

        def _record():
            period = 1.0 / max(1.0, self._record_hz)
            t0 = time.time()
            while not stop.is_set():
                tick_t0 = time.time()
                with self._lock:
                    s = self._latest_state
                    c = self._latest_command
                    cam_msgs = {n: self._latest_images.get(n) for n in self._camera_names}
                # We only commit a frame when we have BOTH a state and a
                # command — otherwise observation/action would misalign.
                # Cameras are allowed to be `None` briefly at startup; in
                # that case we drop the tick so all streams stay synchronised.
                cams_ready = all(m is not None for m in cam_msgs.values())
                if s is not None and c is not None and cams_ready:
                    state_arr = _extract_positions(s)
                    # FK from the 6 arm joints — gripper (idx 6) is locked in
                    # the reduced model. Computed inline so ee_pos.npy stays
                    # frame-aligned with state.npy / action.npy without a
                    # separate post-processing pass.
                    eepos = self._fk.fk(state_arr[:6])
                    buf.add(
                        t=time.time() - t0,
                        state=state_arr,
                        action=_extract_positions(c),
                        eepos=eepos,
                        cam_jpegs={n: bytes(m.data) for n, m in cam_msgs.items()},
                    )
                # Aim for steady cadence even if recording took some time.
                slept = time.time() - tick_t0
                if slept < period:
                    time.sleep(period - slept)

        recorder = threading.Thread(target=_record, daemon=True)
        recorder.start()
        try:
            run = self.call_trigger(self._cli_run, timeout=120.0)
        finally:
            stop.set()
            recorder.join(timeout=2.0)

        if not run.success:
            return False, {"reason": f"run_episode_failed: {run.message}"}

        # Settle so the success check sees the resting pose, not the
        # mid-release transient.
        time.sleep(0.3)
        chk = self.call_trigger(self._cli_check)
        try:
            payload = json.loads(chk.message)
        except Exception:
            payload = {"raw": chk.message}
        success = bool(payload.get("success", chk.success))

        if not success:
            return False, {"reason": "ground_truth_fail", "payload": payload}

        # Write to disk — successful episodes only.
        ep_id = uuid.uuid4().hex[:12]
        ep_dir = output_dir / ep_id
        for cam in self._camera_names:
            (ep_dir / cam).mkdir(parents=True, exist_ok=True)
        buf.write(ep_dir, payload=payload, record_hz=self._record_hz)
        return True, {"episode_id": ep_id, "frames": len(buf.t), "payload": payload}


class _EpisodeBuffer:
    """In-memory buffer for one episode. Holds frames as raw JPEG bytes so
    we don't pay decode cost during the hot loop."""

    def __init__(self, camera_names: list[str]) -> None:
        self.t: list[float] = []
        self.state: list[np.ndarray] = []
        self.action: list[np.ndarray] = []
        # End-effector pose at the same tick — [x,y,z,rx,ry,rz] in world frame.
        # Required for any policy trained with action_key='relative_ee_pos'.
        self.eepos: list[np.ndarray] = []
        self.camera_names = list(camera_names)
        self.cam: dict[str, list[bytes]] = {n: [] for n in self.camera_names}

    def add(self, t: float, state: np.ndarray, action: np.ndarray,
             eepos: np.ndarray, cam_jpegs: dict[str, bytes]) -> None:
        self.t.append(t)
        self.state.append(state)
        self.action.append(action)
        self.eepos.append(eepos)
        for n in self.camera_names:
            self.cam[n].append(cam_jpegs[n])

    def write(self, ep_dir: Path, payload: dict, record_hz: float) -> None:
        np.save(ep_dir / "state.npy", np.asarray(self.state, dtype=np.float32))
        np.save(ep_dir / "action.npy", np.asarray(self.action, dtype=np.float32))
        np.save(ep_dir / "ee_pos.npy", np.asarray(self.eepos, dtype=np.float32))
        np.save(ep_dir / "timestamps.npy", np.asarray(self.t, dtype=np.float32))
        for cam, frames in self.cam.items():
            for i, jpg in enumerate(frames):
                (ep_dir / cam / f"frame_{i:05d}.jpg").write_bytes(jpg)
        meta = {
            "scene_id": payload.get("scene_id"),
            "success": payload.get("success"),
            "metrics": payload.get("metrics"),
            "object_poses": payload.get("poses"),
            "fps": float(record_hz),
            "n_frames": len(self.t),
            "joint_order": ALL_JOINTS,
            "cameras": self.camera_names,
        }
        (ep_dir / "meta.json").write_text(json.dumps(meta, indent=2))


def _extract_positions(msg: JointState) -> np.ndarray:
    """Reorder a JointState into our canonical 7-vector ordering."""
    name_to_pos = {n: float(p) for n, p in zip(msg.name, msg.position)}
    out = np.zeros(len(ALL_JOINTS), dtype=np.float32)
    for i, name in enumerate(ALL_JOINTS):
        out[i] = name_to_pos.get(name, 0.0)
    return out


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Collect successful peg-in-hole episodes for training.")
    parser.add_argument("--num-episodes", type=int, default=20,
                        help="Total *successful* episodes to collect.")
    parser.add_argument("--max-attempts", type=int, default=0,
                        help="Hard cap on attempts (0 = 3x num-episodes).")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT)
    parser.add_argument("--record-hz", type=float, default=30.0)
    parser.add_argument("--topic-prefix", default="/tutorial")
    parser.add_argument("--cameras", default="wrist_cam,wrist_cam_down",
                        help="Comma-separated camera names matching MJCF + launch. "
                             "Default = round_1 2-wrist setup.")
    cli_args, ros_args = parser.parse_known_args(args)
    cameras = [c.strip() for c in cli_args.cameras.split(",") if c.strip()]
    if not cameras:
        raise SystemExit("--cameras parsed to empty list")
    rclpy.init(args=ros_args)

    node = CollectorNode(prefix=cli_args.topic_prefix,
                         record_hz=cli_args.record_hz,
                         cameras=cameras)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    out = Path(cli_args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    if not node.wait_for_subs(timeout=10.0):
        node.get_logger().error("topics not ready (joint_states / cameras)")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 2

    max_attempts = cli_args.max_attempts or (cli_args.num_episodes * 3)
    succ = 0
    attempts = 0
    while succ < cli_args.num_episodes and attempts < max_attempts:
        attempts += 1
        node.get_logger().info(
            f"=== attempt {attempts} (got {succ}/{cli_args.num_episodes}) ===")
        try:
            ok, info = node.collect_episode(out)
        except Exception as e:
            node.get_logger().error(f"attempt failed: {e!r}")
            ok, info = False, {"reason": f"exception: {e!r}"}
        if ok:
            succ += 1
            node.get_logger().info(
                f"  ✓ saved {info['episode_id']} ({info['frames']} frames)")
        else:
            node.get_logger().warn(f"  ✗ dropped: {info.get('reason')}")

    node.get_logger().info(
        f"Done. {succ}/{cli_args.num_episodes} successful in {attempts} attempts.")

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if succ == cli_args.num_episodes else 1


if __name__ == "__main__":
    raise SystemExit(main())
