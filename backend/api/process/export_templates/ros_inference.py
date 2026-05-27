"""Closed-loop ROS 2 inference for an exported EasyTrainer checkpoint.

Mirrors the qaction inference path of EasyTrainer's
``src/backend/api/process/checkpoint_test.py`` but without the EasyTrainer
container, database, Flask backend, or robot-specific Agent classes. The bundle
talks to the robot directly via ROS 2 topics resolved from
``export_meta.json``.

The checkpoint was trained for a specific task, which is bound to a specific
assembly (left/right arm + tools + optional mobile base). This script drives
exactly that assembly — the set of robots is not configurable because the
model's output layer is sized for it. Sensors are also fixed to the ones the
task used for training.

Usage::

    # Run the checkpoint's assembly at 10 Hz, until Ctrl-C:
    python3 ros_inference.py

    # Override rate / set a step limit:
    python3 ros_inference.py --hz 15 --episode-len 300

    # Send the raw model output to the robot but don't actually publish
    # (useful for sanity checking before powering the arms on):
    python3 ros_inference.py --dry-run

What it does
------------
1. Initialises rclpy and creates a node ``easytrainer_exported_inference``.
2. Loads the policy from ``./model`` via the existing
   ``CheckpointInference`` class (single-step API in ``inference.py``).
3. Subscribes to every sensor's image topic via ``SimpleEnv``.
4. Wraps each robot in a ``SimpleAgent`` (joint state in / joint command out).
5. Runs an inference loop at the requested hz: read observation → preprocess
   images per task config → call policy → publish joint commands.

Limitations vs full EasyTrainer inference
------------------------------------------
- ``action_key`` must be ``qaction``. ``ee_delta_action`` and
  ``relative_ee_pos`` need an IK solver and are not supported here.
- Robot ``write_type`` must be ``topic``. Service / action goal robots are
  not supported.
- ``has_succeed`` checkpoints automatically strip the trailing succeed bit
  before publishing the joint command (matches checkpoint_test.py).
- No homepose movement, no failure detection, no OOD scoring, no Grad-CAM,
  no OTI-RL, no socketio broadcasts. Run the robot from its current pose.
- Robots use ``sensor_msgs/JointState`` (or trajectory_msgs/JointTrajectoryPoint)
  for both state read and command write. See ``easytrainer_runtime/simple_agent.py``
  for the message format reference.

If your checkpoint hits one of these limits the script raises a clear error
at startup so you can run it inside EasyTrainer instead.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

# ── Bundled lerobot isolation (mirrors inference.py) ─────────────────────────
# Force the bundled lerobot to win over any user-installed / PYTHONPATH copy.
# See inference.py for the rationale and detailed comments.
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

# Make ``from inference import ...`` and ``from easytrainer_runtime import ...``
# work regardless of cwd.
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import numpy as np

import rclpy
from rclpy.node import Node

from inference import CheckpointInference  # the single-step API
from easytrainer_runtime.simple_agent import SimpleAgent
from easytrainer_runtime.simple_env import SimpleEnv
from easytrainer_runtime.image_parser import fetch_image_with_config


# ──────────────────────────────────────────────────────────────────────────────
# Argument parsing
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
        "--hz",
        type=float,
        default=10.0,
        help="Inference loop frequency (default: 10 Hz)",
    )
    p.add_argument(
        "--episode-len",
        type=int,
        default=0,
        help="Stop after N steps (0 = run until Ctrl-C, default: 0)",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute actions but don't publish to robot. Logs actions to stdout.",
    )
    p.add_argument(
        "--device",
        default="cuda",
        help="cuda (default) or cpu",
    )
    return p.parse_args()


# ──────────────────────────────────────────────────────────────────────────────
# Setup helpers
# ──────────────────────────────────────────────────────────────────────────────
def _resolve_sensor_config(meta: dict, view_key: str, sensor_id) -> dict:
    """Pull the per-view crop / rotate / resize / sam3 config from the task block.

    Multi-view: 같은 물리 sensor 의 view 들은 (sensor_5, sensor_5_2 …) 별도
    설정을 가질 수 있어 view_key 로 lookup. view_key 가 없으면 sensor_id 로
    fallback (single-view 환경 호환).
    """
    sid_str = str(sensor_id)
    vkey = str(view_key)
    task = meta.get("task", {}) or {}
    def _pick(field, default=None):
        d = task.get(field) or {}
        if vkey in d:
            return d.get(vkey)
        return d.get(sid_str, default)
    return {
        "sensor_id": vkey,
        "sam3": _pick("sensor_sam3"),
        "cropped_area": _pick("sensor_cropped_area"),
        "rotate": _pick("sensor_rotate", 0),
        "resize": _pick("sensor_img_size"),
    }


def _enumerate_view_keys_for_sensors(sensors):
    """``sensors`` 는 sensor_ids 순서 (중복 포함). 결과는 (sensor_id, occurrence)
    오름차순 정렬 — features 순서와 일치. backend ``enumerate_views`` /
    ``task_model._enumerate_view_keys`` 와 동일 규칙."""
    seen = {}
    items = []
    for s in sensors:
        sid = int(s["id"])
        idx = seen.get(sid, 0)
        seen[sid] = idx + 1
        vkey = str(sid) if idx == 0 else f"{sid}_{idx + 1}"
        items.append((s, sid, idx, vkey))
    items.sort(key=lambda t: (t[1], t[2]))
    return [(sensor, vkey) for sensor, _sid, _occ, vkey in items]


# ──────────────────────────────────────────────────────────────────────────────
# Main loop
# ──────────────────────────────────────────────────────────────────────────────
def main() -> int:
    args = _parse_args()

    meta_path = Path(args.meta)
    if not meta_path.exists():
        raise SystemExit(f"export_meta.json not found at {meta_path}")
    with open(meta_path) as f:
        meta = json.load(f)

    # Sanity-check the configuration we know how to handle.
    action_key = meta.get("action_key", "qaction")
    if action_key != "qaction":
        raise SystemExit(
            f"This bundle only supports action_key='qaction'. Checkpoint uses "
            f"'{action_key}'. Run inside EasyTrainer instead."
        )
    backbone = meta.get("vision_backbone", "resnet18")
    if backbone != "resnet18":
        raise SystemExit(
            f"This bundle only supports vision_backbone='resnet18'. Checkpoint "
            f"uses '{backbone}'."
        )

    has_succeed = bool(meta.get("has_succeed", False))

    # ── Assembly & sensors (fixed by training time — no subset flags) ─────
    robots = meta.get("robots") or []
    sensors = meta.get("sensors") or []
    if not robots:
        raise SystemExit(
            "export_meta.json has no 'robots' entries. The checkpoint's task "
            "must point to an assembly with at least one robot. Re-export after "
            "fixing the assembly in EasyTrainer."
        )
    if not sensors:
        raise SystemExit(
            "export_meta.json has no 'sensors' entries. The task must reference "
            "at least one sensor."
        )

    print(f"[ros_inference] policy_type={meta.get('policy_type')} "
          f"action_key={action_key} has_succeed={has_succeed}")
    print(f"[ros_inference] assembly robots: {[(r['id'], r.get('name')) for r in robots]}")
    print(f"[ros_inference] task sensors:    {[(s['id'], s.get('name')) for s in sensors]}")
    print(f"[ros_inference] hz={args.hz} episode_len={args.episode_len} dry_run={args.dry_run}")

    # ── Load model ─────────────────────────────────────────────────────────
    print(f"[ros_inference] loading checkpoint from {args.model_dir} ...")
    inf = CheckpointInference(args.model_dir, args.meta, device=args.device)
    inf.reset()
    print("[ros_inference] checkpoint loaded.")

    # ── ROS init ───────────────────────────────────────────────────────────
    rclpy.init()
    node = Node("easytrainer_exported_inference")

    try:
        agents = [SimpleAgent(node, r) for r in sorted(robots, key=lambda r: r["id"])]
        env = SimpleEnv(node, agents, sensors)

        # Spin executor in a background thread so subscriptions actually fire.
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # ── Main loop ───────────────────────────────────────────────────────
        period = 1.0 / max(args.hz, 0.1)
        step_num = 0
        print("[ros_inference] entering inference loop. Ctrl-C to stop.")
        try:
            while args.episode_len == 0 or step_num < args.episode_len:
                loop_start = time.time()

                # 1) Observation snapshot
                obs = env.get_observation()

                # 2) Build the model input. State is the concatenation of all
                #    agent qpos in the same order as checkpoint_test.py
                #    (sorted by agent.id, which is what `sorted(... key=lambda a: a.id)`
                #    does in the original loop).
                qpos_concat = np.concatenate(
                    [np.asarray(obs["robot_states"][a.id]["qpos"], dtype=np.float32)
                     for a in agents]
                )

                # 3) Per-view crop/rotate/resize using the same task config
                #    that EasyTrainer used when collecting and training.
                #    Multi-view: 같은 물리 sensor 의 여러 view 마다 독립 crop.
                #    raw frame 은 sensor_{id} 단위로 한 번만 읽고 (image_dict 캐시)
                #    view 별로 fetch_image_with_config 으로 가공.
                images = {}
                raw_cache = {}
                for sensor, vkey in _enumerate_view_keys_for_sensors(sensors):
                    sid = sensor["id"]
                    if sid not in raw_cache:
                        raw_cache[sid] = obs["images"][f"sensor_{sid}"]
                    raw = raw_cache[sid]
                    cfg = _resolve_sensor_config(meta, vkey, sid)
                    processed = fetch_image_with_config(raw, cfg)
                    # process_image inside CheckpointInference expects RGB. Our
                    # ros_image_to_numpy returns BGR (OpenCV convention). Flip.
                    if processed.ndim == 3 and processed.shape[2] == 3:
                        processed = processed[:, :, ::-1]
                    images[f"sensor_{vkey}"] = np.ascontiguousarray(processed)

                # 4) Inference
                action = inf.infer(qpos_concat, images)

                # 5) Strip the succeed bit (last dim) — checkpoint_test.py
                #    publishes only the joint commands, never the succeed flag.
                if has_succeed and len(action) > 0:
                    succeed_val = float(action[-1])
                    action = action[:-1]
                else:
                    succeed_val = None

                # 6) Split action across agents and send the joint commands.
                start = 0
                for a in agents:
                    target_qpos = action[start : start + a.joint_len]
                    start += a.joint_len
                    if args.dry_run:
                        print(f"[step {step_num}] {a.name}: {np.round(target_qpos, 4).tolist()}")
                    else:
                        a.move_joint_step(target_qpos)

                if args.dry_run and succeed_val is not None:
                    print(f"[step {step_num}] succeed_score={succeed_val:.3f}")

                step_num += 1

                # Pace the loop.
                elapsed = time.time() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)
        except KeyboardInterrupt:
            print("\n[ros_inference] interrupted, shutting down.")

    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
