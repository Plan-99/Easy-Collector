"""Service-based ROS 2 inference node for an exported EasyTrainer checkpoint.

Unlike ``ros_inference.py`` which runs an autonomous closed-loop at a fixed
hz, this script keeps the model warm and only runs inference when an external
client calls a ROS 2 service. Useful when an upstream planner / state machine
wants to step the policy on demand.

Lifecycle
---------
At startup the node:

1. Loads the policy and the assembly's robots / task sensors (same code paths
   as ``ros_inference.py``).
2. Subscribes to every robot's joint-state topic and every sensor's image
   topic — observations are kept hot in the background via the executor.
3. Advertises two services:
       <node_name>/run_inference  (std_srvs/srv/Trigger)
       <node_name>/reset          (std_srvs/srv/Trigger)
4. Spins until Ctrl-C.

When ``run_inference`` is called the node grabs a fresh observation snapshot,
runs one inference step, publishes the resulting joint commands to each agent
(unless ``--dry-run``), and returns success/message in the service response.

When ``reset`` is called the policy's temporal ensembler / action queue is
cleared so the next call is treated as a fresh episode.

Limitations match ``ros_inference.py`` (qaction, resnet18, write_type=topic,
JointState/JointTrajectoryPoint/Float64MultiArray robots only). See README.

Usage
-----
::

    # Bring the node up (run in one terminal)
    python3 ros_inference_service.py

    # From another terminal:
    ros2 service call /easytrainer_inference_service/run_inference std_srvs/srv/Trigger
    ros2 service call /easytrainer_inference_service/reset std_srvs/srv/Trigger

    # Custom node name (so you can run multiple side-by-side):
    python3 ros_inference_service.py --node-name my_policy

    # Dry-run: compute & log actions but don't publish to the robot
    python3 ros_inference_service.py --dry-run
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
from pathlib import Path

# ── Bundled lerobot isolation (mirrors inference.py / ros_inference.py) ──────
_HERE = Path(__file__).resolve().parent
_BUNDLED_LEROBOT_PARENT = _HERE / "lerobot"
_BUNDLED_LEROBOT_PKG = _BUNDLED_LEROBOT_PARENT / "lerobot"

if _BUNDLED_LEROBOT_PKG.is_dir():
    for _name in list(sys.modules.keys()):
        if _name == "lerobot" or _name.startswith("lerobot."):
            del sys.modules[_name]

    _bundled_parent_resolved = _BUNDLED_LEROBOT_PARENT.resolve()
    _filtered = []
    for _entry in sys.path:
        if not _entry:
            _filtered.append(_entry)
            continue
        try:
            _entry_resolved = Path(_entry).resolve()
        except (OSError, RuntimeError):
            _filtered.append(_entry)
            continue
        if _entry_resolved == _bundled_parent_resolved:
            continue
        if (_entry_resolved / "lerobot" / "__init__.py").is_file():
            continue
        if _entry_resolved.name == "lerobot" and (_entry_resolved / "__init__.py").is_file():
            continue
        _filtered.append(_entry)
    sys.path[:] = [str(_bundled_parent_resolved)] + _filtered

    import importlib as _importlib
    _importlib.invalidate_caches()

if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from inference import CheckpointInference  # the single-step API
from easytrainer_runtime.simple_agent import SimpleAgent
from easytrainer_runtime.simple_env import SimpleEnv
from easytrainer_runtime.image_parser import fetch_image_with_config


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────
def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--meta",
        default=str(_HERE / "export_meta.json"),
        help="Path to export_meta.json (default: ./export_meta.json)",
    )
    p.add_argument(
        "--model-dir",
        default=str(_HERE / "model"),
        help="Path to the model/ subfolder (default: ./model)",
    )
    p.add_argument(
        "--node-name",
        default="easytrainer_inference_service",
        help="ROS 2 node name (default: easytrainer_inference_service)",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute actions but don't publish to the robot.",
    )
    p.add_argument(
        "--device",
        default="cuda",
        help="cuda (default) or cpu",
    )
    return p.parse_args()


def _resolve_sensor_config(meta: dict, sensor_id) -> dict:
    """Per-sensor crop / rotate / resize from the task block."""
    sid_str = str(sensor_id)
    task = meta.get("task", {}) or {}
    return {
        "cropped_area": (task.get("sensor_cropped_area") or {}).get(sid_str),
        "rotate": (task.get("sensor_rotate") or {}).get(sid_str, 0),
        "resize": (task.get("sensor_img_size") or {}).get(sid_str),
    }


# ──────────────────────────────────────────────────────────────────────────────
# Service node
# ──────────────────────────────────────────────────────────────────────────────
class InferenceServiceNode(Node):
    def __init__(self, args: argparse.Namespace, meta: dict, inf: CheckpointInference):
        super().__init__(args.node_name)
        self.args = args
        self.meta = meta
        self.inf = inf
        self.has_succeed = bool(meta.get("has_succeed", False))

        # Inference is not thread-safe (CUDA + temporal ensembler state) so we
        # serialise concurrent service calls behind this lock. Subscriptions
        # still update freely on other threads.
        self._infer_lock = threading.Lock()
        self._step_count = 0

        # Build agents + env. Putting subscriptions in a ReentrantCallbackGroup
        # lets the executor process image / joint state callbacks while a
        # service callback is running, so the next inference call has fresh
        # data instead of stale frames.
        self._cb_group = ReentrantCallbackGroup()

        robots = meta.get("robots") or []
        sensors = meta.get("sensors") or []
        if not robots:
            raise RuntimeError("export_meta.json has no 'robots' entries")
        if not sensors:
            raise RuntimeError("export_meta.json has no 'sensors' entries")

        self.agents = [
            SimpleAgent(self, r) for r in sorted(robots, key=lambda r: r["id"])
        ]
        self.env = SimpleEnv(self, self.agents, sensors)
        self.sensors = sensors

        # Services. Same callback group so they don't block subscriptions.
        self._run_srv = self.create_service(
            Trigger,
            f"~/run_inference",
            self._handle_run_inference,
            callback_group=self._cb_group,
        )
        self._reset_srv = self.create_service(
            Trigger,
            f"~/reset",
            self._handle_reset,
            callback_group=self._cb_group,
        )

        ns = self.get_name()
        self.get_logger().info(
            f"Inference service ready. Call:\n"
            f"  ros2 service call /{ns}/run_inference std_srvs/srv/Trigger\n"
            f"  ros2 service call /{ns}/reset std_srvs/srv/Trigger"
        )

    # ──────────────────────────────────────────────────────────────────────
    def _handle_reset(self, request, response):
        with self._infer_lock:
            self.inf.reset()
            self._step_count = 0
        response.success = True
        response.message = "policy reset"
        self.get_logger().info("[reset] policy state cleared")
        return response

    # ──────────────────────────────────────────────────────────────────────
    def _handle_run_inference(self, request, response):
        try:
            with self._infer_lock:
                action = self._run_one_step()
        except Exception as e:
            self.get_logger().error(f"[run_inference] failed: {e}")
            response.success = False
            response.message = f"{type(e).__name__}: {e}"
            return response

        msg = f"step {self._step_count - 1}: action={np.round(action, 4).tolist()}"
        if self.args.dry_run:
            msg = "[DRY] " + msg
        response.success = True
        response.message = msg
        self.get_logger().info(f"[run_inference] {msg}")
        return response

    # ──────────────────────────────────────────────────────────────────────
    def _run_one_step(self) -> np.ndarray:
        """Single inference step. Same logic as ros_inference.py main loop body
        minus the pacing / episode limit."""
        obs = self.env.get_observation()

        # State: concat of every agent's qpos in id order
        qpos_concat = np.concatenate(
            [
                np.asarray(obs["robot_states"][a.id]["qpos"], dtype=np.float32)
                for a in self.agents
            ]
        )

        # Per-sensor crop/rotate/resize, then BGR→RGB for the model
        images = {}
        for sensor in self.sensors:
            sid = sensor["id"]
            raw = obs["images"][f"sensor_{sid}"]
            cfg = _resolve_sensor_config(self.meta, sid)
            processed = fetch_image_with_config(raw, cfg)
            if processed.ndim == 3 and processed.shape[2] == 3:
                processed = processed[:, :, ::-1]
            images[f"sensor_{sid}"] = np.ascontiguousarray(processed)

        action = self.inf.infer(qpos_concat, images)

        # Strip succeed bit before publishing (the policy was trained to predict
        # it as the trailing dim, but it isn't a joint command).
        if self.has_succeed and len(action) > 0:
            action_to_send = action[:-1]
        else:
            action_to_send = action

        # Split across agents. Same offset arithmetic as ros_inference.py.
        start = 0
        for a in self.agents:
            target = action_to_send[start : start + a.joint_len]
            start += a.joint_len
            if not self.args.dry_run:
                a.move_joint_step(target)

        self._step_count += 1
        return action  # full vector incl. succeed (caller decides what to do)


# ──────────────────────────────────────────────────────────────────────────────
# main
# ──────────────────────────────────────────────────────────────────────────────
def main() -> int:
    args = _parse_args()

    meta_path = Path(args.meta)
    if not meta_path.exists():
        raise SystemExit(f"export_meta.json not found at {meta_path}")
    with open(meta_path) as f:
        meta = json.load(f)

    # Same compatibility checks as ros_inference.py.
    action_key = meta.get("action_key", "qaction")
    if action_key != "qaction":
        raise SystemExit(
            f"This bundle only supports action_key='qaction'. Checkpoint uses "
            f"'{action_key}'."
        )
    backbone = meta.get("vision_backbone", "resnet18")
    if backbone != "resnet18":
        raise SystemExit(
            f"This bundle only supports vision_backbone='resnet18'. Checkpoint "
            f"uses '{backbone}'."
        )

    print(f"[ros_inference_service] policy_type={meta.get('policy_type')} "
          f"action_key={action_key} has_succeed={meta.get('has_succeed', False)}")
    print(f"[ros_inference_service] assembly robots: "
          f"{[(r['id'], r.get('name')) for r in meta.get('robots', [])]}")
    print(f"[ros_inference_service] task sensors:    "
          f"{[(s['id'], s.get('name')) for s in meta.get('sensors', [])]}")
    print(f"[ros_inference_service] node_name={args.node_name} dry_run={args.dry_run}")

    print(f"[ros_inference_service] loading checkpoint from {args.model_dir} ...")
    inf = CheckpointInference(args.model_dir, args.meta, device=args.device)
    inf.reset()
    print("[ros_inference_service] checkpoint loaded.")

    rclpy.init()
    node = None
    executor = None
    try:
        node = InferenceServiceNode(args, meta, inf)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\n[ros_inference_service] interrupted, shutting down.")
    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
