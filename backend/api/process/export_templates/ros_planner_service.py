"""Service-triggered ROS 2 node for an exported EasyTrainer planner.

Unlike ``run_planner.py`` which executes the planner immediately and exits,
this node stays up, keeps every checkpoint warm, and only runs the planner when
an external client calls a ROS 2 service. Useful when an upstream state machine
or operator wants to trigger a full planner execution on demand.

Lifecycle
---------
At startup the node:

1. Loads ``planner_meta.json`` and every bundled checkpoint.
2. Builds a ``SimpleAgent`` per robot and a ``SimpleEnv`` per workspace —
   joint states and camera frames are kept hot in the background.
3. Advertises two services:
       <node_name>/start  (std_srvs/srv/Trigger)  — run the planner once
       <node_name>/stop   (std_srvs/srv/Trigger)  — abort a running planner
4. Spins until Ctrl-C.

When ``start`` is called the node spawns a background worker thread that runs
the full planner (``--repeat`` controls how many iterations per call). The
service returns immediately with ``success=True``. If a run is already in
progress it returns ``success=False``.

When ``stop`` is called the run's ``task_control['stop']`` flag is set; every
group worker aborts and the node returns to idle, ready for the next ``start``.

Usage
-----
::

    # Bring the node up (run in one terminal)
    python3 ros_planner_service.py

    # From another terminal:
    ros2 service call /easytrainer_planner_service/start std_srvs/srv/Trigger
    ros2 service call /easytrainer_planner_service/stop  std_srvs/srv/Trigger

    # Custom node name / repeat / device:
    python3 ros_planner_service.py --node-name my_planner --repeat 3 --device cpu

Limitations match ``run_planner.py``. See README.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
from pathlib import Path

# ── Bundled lerobot isolation (mirrors inference.py / run_planner.py) ────────
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

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from easytrainer_runtime.planner_engine import build_context, run_planner


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--meta",
        default=str(_HERE / "planner_meta.json"),
        help="Path to planner_meta.json (default: ./planner_meta.json)",
    )
    p.add_argument(
        "--node-name",
        default="easytrainer_planner_service",
        help="ROS 2 node name (default: easytrainer_planner_service)",
    )
    p.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Iterations per /start call. 0 (or negative) = loop forever until "
             "/stop is called (default: 1).",
    )
    p.add_argument(
        "--device",
        default="cuda",
        help="cuda (default) or cpu — used for checkpoint inference.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute checkpoint actions but don't publish joint commands.",
    )
    return p.parse_args()


class PlannerServiceNode(Node):
    def __init__(self, args: argparse.Namespace, meta: dict, executor=None):
        super().__init__(args.node_name)
        self.args = args
        self.meta = meta

        # Worker state.
        self._worker: threading.Thread | None = None
        self._task_control = {"stop": False}
        self._lock = threading.Lock()

        # ReentrantCallbackGroup so /stop can be served while /start's worker
        # thread is running inference (subscriptions also stay live).
        self._cb_group = ReentrantCallbackGroup()

        # Build the planner context once — agents / envs / checkpoints stay hot
        # across repeated /start calls. The executor (if provided) is what the
        # per-robot interpolation nodes get attached to.
        self.get_logger().info("building planner context (agents / envs / checkpoints) ...")
        self.ctx = build_context(self, _HERE, meta, device=args.device,
                                 dry_run=args.dry_run, executor=executor)
        self.get_logger().info("planner context ready.")

        self._start_srv = self.create_service(
            Trigger, "~/start", self._handle_start, callback_group=self._cb_group,
        )
        self._stop_srv = self.create_service(
            Trigger, "~/stop", self._handle_stop, callback_group=self._cb_group,
        )

        ns = self.get_name()
        self.get_logger().info(
            f"Planner service ready. Call:\n"
            f"  ros2 service call /{ns}/start std_srvs/srv/Trigger\n"
            f"  ros2 service call /{ns}/stop  std_srvs/srv/Trigger"
        )

    @property
    def is_running(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    # ── /start ────────────────────────────────────────────────────────────
    def _handle_start(self, request, response):
        with self._lock:
            if self.is_running:
                response.success = False
                response.message = "planner already running"
                self.get_logger().warn("[start] rejected — planner already running")
                return response

            self._task_control = {"stop": False}
            self._worker = threading.Thread(target=self._run, daemon=True)
            self._worker.start()

        response.success = True
        response.message = "planner run started"
        self.get_logger().info("[start] planner run started")
        return response

    # ── /stop ─────────────────────────────────────────────────────────────
    def _handle_stop(self, request, response):
        with self._lock:
            if not self.is_running:
                response.success = False
                response.message = "planner is not running"
                self.get_logger().warn("[stop] rejected — planner not running")
                return response
            self._task_control["stop"] = True

        self._worker.join(timeout=20.0)
        if self._worker.is_alive():
            self.get_logger().warn("[stop] worker did not exit within 20s")
            response.success = False
            response.message = "stop signalled but worker still running"
            return response

        response.success = True
        response.message = "planner stopped"
        self.get_logger().info("[stop] planner stopped")
        return response

    # ── worker ────────────────────────────────────────────────────────────
    def _run(self):
        try:
            status, error = run_planner(
                self.ctx, self._task_control, repeat_count=self.args.repeat,
            )
            if error:
                self.get_logger().error(f"[run] planner finished with error: {error}")
            else:
                self.get_logger().info(f"[run] planner run {status}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"[run] planner crashed: {e}")


def main() -> int:
    args = _parse_args()

    meta_path = Path(args.meta)
    if not meta_path.exists():
        raise SystemExit(f"planner_meta.json not found at {meta_path}")
    with open(meta_path) as f:
        meta = json.load(f)

    print(f"[ros_planner_service] planner '{meta.get('planner_name')}' "
          f"(id={meta.get('planner_id')})")
    print(f"[ros_planner_service] node_name={args.node_name} repeat={args.repeat} "
          f"device={args.device} dry_run={args.dry_run}")

    rclpy.init()
    node = None
    executor = None
    try:
        # Executor must exist before PlannerServiceNode is built so that the
        # per-robot interpolation nodes (if any) can be attached to it.
        executor = MultiThreadedExecutor()
        node = PlannerServiceNode(args, meta, executor=executor)
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\n[ros_planner_service] interrupted, shutting down.")
    finally:
        if node is not None:
            node._task_control["stop"] = True
            # Cleanly tear down any interpolation nodes we attached.
            for inode in getattr(node.ctx, 'interpolation_nodes', []):
                try:
                    executor.remove_node(inode)
                    inode.destroy_node()
                except Exception:
                    pass
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
