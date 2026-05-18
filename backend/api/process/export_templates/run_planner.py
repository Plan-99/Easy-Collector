"""Standalone runner for an exported EasyTrainer planner.

Run this one file to execute the whole planner outside the EasyTrainer
container — no Flask, no database, no gRPC bridge. Robots are driven directly
over ROS 2 topics and the planner's checkpoints run through the bundled
inference code.

Usage::

    # Run the planner once (default):
    python3 run_planner.py

    # Repeat the planner 5 times:
    python3 run_planner.py --repeat 5

    # Loop forever until Ctrl-C:
    python3 run_planner.py --repeat 0

    # Compute checkpoint actions but don't publish to the robots:
    python3 run_planner.py --dry-run

    # Run on CPU instead of CUDA:
    python3 run_planner.py --device cpu

What it does
------------
1. Initialises rclpy and creates a node ``easytrainer_exported_planner``.
2. Loads ``planner_meta.json`` (groups, workspaces, robot/sensor topology) and
   every bundled checkpoint via ``CheckpointInference``.
3. Builds a ``SimpleAgent`` per robot and a ``SimpleEnv`` per workspace.
4. Executes the planner: groups run in parallel threads, blocks run
   sequentially within a group. Ctrl-C stops cleanly.

Limitations vs in-container planner execution
----------------------------------------------
- Robot ``write_type`` must be ``topic`` (service/action robots are skipped).
- ``query_pose`` blocks support joint-position mode only — end-effector mode
  needs the Pinocchio IK solver, which is not bundled.
- Checkpoint blocks: ``action_key`` must be ``qaction`` and ``vision_backbone``
  must be ``resnet18`` (same constraints as the checkpoint exporter).
- No socketio progress broadcasts, no OOD/Grad-CAM, no OTI-RL.

If a block hits one of these limits the run fails with a clear error so you can
run that planner inside EasyTrainer instead.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

# ── Bundled lerobot isolation (mirrors inference.py) ─────────────────────────
# Force the bundled lerobot copy to win over any pip-installed / PYTHONPATH one.
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
from rclpy.executors import SingleThreadedExecutor

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
        "--repeat",
        type=int,
        default=1,
        help="How many times to run the planner. 0 (or negative) = loop "
             "forever until Ctrl-C (default: 1).",
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


def main() -> int:
    args = _parse_args()

    meta_path = Path(args.meta)
    if not meta_path.exists():
        raise SystemExit(f"planner_meta.json not found at {meta_path}")
    with open(meta_path) as f:
        meta = json.load(f)

    groups = meta.get("groups") or []
    workspaces = meta.get("workspaces") or {}
    checkpoints = meta.get("checkpoints") or {}
    print(f"[run_planner] planner '{meta.get('planner_name')}' "
          f"(id={meta.get('planner_id')})")
    print(f"[run_planner] {len(groups)} group(s), {len(workspaces)} workspace(s), "
          f"{len(checkpoints)} checkpoint(s)")
    print(f"[run_planner] repeat={args.repeat} device={args.device} "
          f"dry_run={args.dry_run}")

    rclpy.init()
    node = Node("easytrainer_exported_planner")

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    task_control = {"stop": False}
    status = "error"
    try:
        print("[run_planner] building context (agents / envs / checkpoints) ...")
        ctx = build_context(node, _HERE, meta, device=args.device,
                            dry_run=args.dry_run, executor=executor)
        print("[run_planner] context ready — starting planner. Ctrl-C to stop.")
        status, error = run_planner(ctx, task_control, repeat_count=args.repeat)
        if error:
            print(f"[run_planner] planner finished with error: {error}")
    except KeyboardInterrupt:
        print("\n[run_planner] interrupted — signalling stop ...")
        task_control["stop"] = True
        status = "stopped"
    finally:
        task_control["stop"] = True
        # Tear down any interpolation nodes we attached so rclpy.shutdown
        # doesn't complain about orphan publishers.
        try:
            for inode in getattr(ctx, 'interpolation_nodes', []) if 'ctx' in locals() else []:
                try:
                    executor.remove_node(inode)
                    inode.destroy_node()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    print(f"[run_planner] done ({status})")
    return 0 if status in ("finished", "stopped") else 1


if __name__ == "__main__":
    raise SystemExit(main())
