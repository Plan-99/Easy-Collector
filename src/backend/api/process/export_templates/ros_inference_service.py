"""Service-based ROS 2 inference node for an exported EasyTrainer checkpoint.

Unlike ``ros_inference.py`` which runs an autonomous closed-loop immediately,
this script keeps the model warm and only starts/stops the inference loop when
an external client calls a ROS 2 service. Useful when an upstream planner or
state machine wants to trigger a full task execution on demand.

Lifecycle
---------
At startup the node:

1. Loads the policy and the assembly's robots / task sensors (same code paths
   as ``ros_inference.py``).
2. Subscribes to every robot's joint-state topic and every sensor's image
   topic — observations are kept hot in the background via the executor.
3. Advertises three services:
       <node_name>/start  (std_srvs/srv/Trigger)  — begin inference loop
       <node_name>/stop   (std_srvs/srv/Trigger)  — stop inference loop
       <node_name>/reset  (std_srvs/srv/Trigger)  — clear temporal ensembler
4. Spins until Ctrl-C.

When ``start`` is called the node spawns a background worker thread that runs
the full hz-paced inference loop (identical to ``ros_inference.py``). The
service returns immediately with ``success=True``. If the node is already
running it returns ``success=False``.

When ``stop`` is called the worker thread is signalled to stop and joined. The
service returns once the loop has cleanly exited.

The loop also auto-stops when ``--episode-len`` steps are reached (if non-zero).
After that the node returns to idle and can be started again.

When ``reset`` is called the policy's temporal ensembler / action queue is
cleared so the next run is treated as a fresh episode.

Limitations match ``ros_inference.py`` (qaction, resnet18, write_type=topic,
JointState/JointTrajectoryPoint/Float64MultiArray robots only). See README.

Usage
-----
::

    # Bring the node up (run in one terminal)
    python3 ros_inference_service.py

    # From another terminal:
    ros2 service call /easytrainer_inference_service/start std_srvs/srv/Trigger
    ros2 service call /easytrainer_inference_service/stop std_srvs/srv/Trigger
    ros2 service call /easytrainer_inference_service/reset std_srvs/srv/Trigger

    # Custom node name / loop options:
    python3 ros_inference_service.py --node-name my_policy --hz 15 --episode-len 300

    # Dry-run: compute & log actions but don't publish to the robot
    python3 ros_inference_service.py --dry-run
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
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
        "--hz",
        type=float,
        default=10.0,
        help="Inference loop frequency (default: 10 Hz)",
    )
    p.add_argument(
        "--episode-len",
        type=int,
        default=0,
        help="Auto-stop after N steps (0 = run until /stop is called, default: 0)",
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

        # Worker thread state
        self._worker: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()  # guards start/stop transitions
        self._step_count = 0

        # Subscriptions in a ReentrantCallbackGroup so observations keep
        # updating while the worker thread is running inference.
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

        # Services
        self._start_srv = self.create_service(
            Trigger, "~/start", self._handle_start,
            callback_group=self._cb_group,
        )
        self._stop_srv = self.create_service(
            Trigger, "~/stop", self._handle_stop,
            callback_group=self._cb_group,
        )
        self._reset_srv = self.create_service(
            Trigger, "~/reset", self._handle_reset,
            callback_group=self._cb_group,
        )

        ns = self.get_name()
        self.get_logger().info(
            f"Inference service ready. Call:\n"
            f"  ros2 service call /{ns}/start std_srvs/srv/Trigger\n"
            f"  ros2 service call /{ns}/stop  std_srvs/srv/Trigger\n"
            f"  ros2 service call /{ns}/reset std_srvs/srv/Trigger"
        )

    # ── property ──────────────────────────────────────────────────────────
    @property
    def is_running(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    # ── /start ────────────────────────────────────────────────────────────
    def _handle_start(self, request, response):
        with self._lock:
            if self.is_running:
                response.success = False
                response.message = "already running"
                self.get_logger().warn("[start] rejected — loop already running")
                return response

            self._stop_event.clear()
            self._step_count = 0
            self.inf.reset()

            self._worker = threading.Thread(
                target=self._inference_loop, daemon=True,
            )
            self._worker.start()

        response.success = True
        response.message = "inference loop started"
        self.get_logger().info("[start] inference loop started")
        return response

    # ── /stop ─────────────────────────────────────────────────────────────
    def _handle_stop(self, request, response):
        with self._lock:
            if not self.is_running:
                response.success = False
                response.message = "not running"
                self.get_logger().warn("[stop] rejected — loop is not running")
                return response

            self._stop_event.set()

        # Join outside the lock so the worker can finish cleanly.
        self._worker.join(timeout=5.0)
        if self._worker.is_alive():
            self.get_logger().warn("[stop] worker did not exit within 5s")

        response.success = True
        response.message = f"stopped after {self._step_count} steps"
        self.get_logger().info(f"[stop] stopped after {self._step_count} steps")
        return response

    # ── /reset ────────────────────────────────────────────────────────────
    def _handle_reset(self, request, response):
        self.inf.reset()
        response.success = True
        response.message = "policy reset"
        self.get_logger().info("[reset] policy state cleared")
        return response

    # ── Worker loop (runs in background thread) ──────────────────────────
    def _inference_loop(self):
        """Full hz-paced inference loop — mirrors ros_inference.py main loop."""
        period = 1.0 / max(self.args.hz, 0.1)
        episode_len = self.args.episode_len

        self.get_logger().info(
            f"[loop] running at {self.args.hz} Hz, "
            f"episode_len={'infinite' if episode_len == 0 else episode_len}"
        )

        try:
            while not self._stop_event.is_set():
                if episode_len > 0 and self._step_count >= episode_len:
                    self.get_logger().info(
                        f"[loop] episode_len={episode_len} reached, auto-stopping"
                    )
                    break

                loop_start = time.time()

                # 1) Observation snapshot
                obs = self.env.get_observation()

                # 2) State: concat of every agent's qpos in id order
                qpos_concat = np.concatenate(
                    [
                        np.asarray(obs["robot_states"][a.id]["qpos"], dtype=np.float32)
                        for a in self.agents
                    ]
                )

                # 3) Per-sensor crop/rotate/resize, BGR→RGB
                images = {}
                for sensor in self.sensors:
                    sid = sensor["id"]
                    raw = obs["images"][f"sensor_{sid}"]
                    cfg = _resolve_sensor_config(self.meta, sid)
                    processed = fetch_image_with_config(raw, cfg)
                    if processed.ndim == 3 and processed.shape[2] == 3:
                        processed = processed[:, :, ::-1]
                    images[f"sensor_{sid}"] = np.ascontiguousarray(processed)

                # 4) Inference
                if self._step_count % 10 == 0:
                    self.get_logger().info(
                        f"[step {self._step_count}] qpos={np.round(qpos_concat, 4).tolist()}"
                    )
                action = self.inf.infer(qpos_concat, images)

                # 5) Strip succeed bit
                if self.has_succeed and len(action) > 0:
                    succeed_val = float(action[-1])
                    action_to_send = action[:-1]
                else:
                    succeed_val = None
                    action_to_send = action

                # 6) Split across agents and publish
                start = 0
                for a in self.agents:
                    target = action_to_send[start : start + a.joint_len]
                    start += a.joint_len
                    self.get_logger().info(
                        f"[step {self._step_count}] {a.name}: "
                        f"{np.round(target, 4).tolist()}"
                    )
                    if not self.args.dry_run:
                        a.move_joint_step(target)

                if succeed_val is not None:
                    self.get_logger().info(
                        f"[step {self._step_count}] succeed_score={succeed_val:.3f}"
                    )

                self._step_count += 1

                # Pace the loop
                elapsed = time.time() - loop_start
                remaining = period - elapsed
                if remaining > 0:
                    # Use stop_event.wait() so /stop can interrupt the sleep
                    self._stop_event.wait(timeout=remaining)

        except Exception as e:
            self.get_logger().error(f"[loop] exception: {e}")
        finally:
            self.get_logger().info(
                f"[loop] exited after {self._step_count} steps"
            )


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
    print(f"[ros_inference_service] node_name={args.node_name} "
          f"hz={args.hz} episode_len={args.episode_len} dry_run={args.dry_run}")

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
