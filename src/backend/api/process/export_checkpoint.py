"""Bundle a trained checkpoint + standalone inference code into a zip.

Used by the ``/api/checkpoint/<id>/:export`` route. The bundle is meant to run
outside the EasyTrainer container with no DB / no Flask / no ROS — just
``pip install -r requirements.txt`` and ``python inference.py``.

Bundle layout (see export_templates/README.md for the user-facing doc)::

    checkpoint_<id>_<name>.zip
    ├── inference.py            (verbatim copy of export_templates/inference.py)
    ├── README.md               (verbatim)
    ├── requirements.txt        (verbatim)
    ├── export_meta.json        (built from DB row)
    ├── model/                  (every file in /root/src/backend/checkpoints/<id>/)
    └── lerobot/                (vendored src/backend/lerobot/src/lerobot/)
"""
from __future__ import annotations

import io
import json
import os
import zipfile
from pathlib import Path

from ...database.models.assembly_model import Assembly as AssemblyModel
from ...database.models.sensor_model import Sensor as SensorModel

# Where the trained checkpoints live inside the container.
CHECKPOINT_DIR = "/root/src/backend/checkpoints"

# Vendored lerobot package — copied wholesale into the bundle so the exported
# inference script doesn't depend on the user pip-installing a matching version.
# (We've patched lerobot in this repo for Py3.10 compat etc., so PyPI lerobot
# wouldn't work.)
_BACKEND_DIR = Path(__file__).resolve().parent.parent.parent  # src/backend
LEROBOT_SRC_DIR = _BACKEND_DIR / "lerobot" / "src" / "lerobot"

# Static templates that get bundled verbatim.
_TEMPLATE_DIR = Path(__file__).resolve().parent / "export_templates"
INFERENCE_TEMPLATE = _TEMPLATE_DIR / "inference.py"
README_TEMPLATE = _TEMPLATE_DIR / "README.md"
REQUIREMENTS_TEMPLATE = _TEMPLATE_DIR / "requirements.txt"
ROS_INFERENCE_TEMPLATE = _TEMPLATE_DIR / "ros_inference.py"
ROS_INFERENCE_SERVICE_TEMPLATE = _TEMPLATE_DIR / "ros_inference_service.py"
RUNTIME_TEMPLATE_DIR = _TEMPLATE_DIR / "easytrainer_runtime"


# ──────────────────────────────────────────────────────────────────────────────
# Meta extraction
# ──────────────────────────────────────────────────────────────────────────────
def _query_assembly_robots(task: dict) -> list:
    """Return the robots that belong to the task's assembly.

    A checkpoint is trained for a specific task, which in turn is bound to an
    assembly (left/right arm + tools + optional mobile base). Exporting every
    robot in the DB is wrong — we only want the ones that matter for *this*
    checkpoint's physical setup. The assembly model already dedupes the
    left_arm/right_arm/left_tool/right_tool/mobile_base fields for us.
    """
    assembly_id = task.get("assembly_id")
    if assembly_id is None:
        return []
    assembly = AssemblyModel.find(assembly_id)
    if assembly is None:
        return []
    # The model's `robots` accessor returns each unique robot as a plain dict
    # (joints, topics, etc.). That's exactly the shape ros_inference.py wants.
    return list(assembly.robots or [])


def _query_task_sensors(task: dict) -> list:
    """Return the sensors referenced by this task, preserving the task's order."""
    sensor_ids = list(task.get("sensor_ids") or [])
    if not sensor_ids:
        return []
    rows = {
        s.id: s.to_dict()
        for s in SensorModel.where_in("id", sensor_ids).get()
    }
    # Preserve task.sensor_ids ordering; drop any ids that can't be resolved.
    return [rows[sid] for sid in sensor_ids if sid in rows]


def build_export_meta(checkpoint, task, policy) -> dict:
    """Extract every inference-relevant field from the DB rows.

    The returned dict is what the standalone ``inference.py`` and
    ``ros_inference.py`` scripts read at runtime — sensor topics, robot
    settings, vision backbone, action shape, etc. Self-describing on purpose
    so the bundle is portable.
    """
    train_settings = checkpoint.get("train_settings") or {}
    policy_settings = policy.get("settings") or {}

    # vision_backbone lives in policy.settings; default to resnet18 for ACT.
    vision_backbone = policy_settings.get("vision_backbone", "resnet18")

    # action_key / obs_state_keys / has_succeed live in train_settings.
    action_key = train_settings.get("action_key", "qaction")
    obs_state_keys = train_settings.get("obs_state_keys", ["qpos"])
    use_relative_trajectory = bool(train_settings.get("use_relative_trajectory", False))
    has_succeed = bool(train_settings.get("has_succeed", False))

    # Sensor topology comes from the task row.
    sensor_ids = list(task.get("sensor_ids") or [])

    # state_dim / action_dim live in the saved config.json. We read them here
    # so the bundle has a single self-describing meta file (the user doesn't
    # need to parse the policy config to know shapes).
    ckpt_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint["id"]))
    state_dim = None
    action_dim = None
    config_path = os.path.join(ckpt_dir, "config.json")
    if os.path.exists(config_path):
        with open(config_path) as f:
            cfg = json.load(f)
        try:
            state_dim = cfg["input_features"]["observation.state"]["shape"][0]
        except (KeyError, IndexError, TypeError):
            pass
        try:
            action_dim = cfg["output_features"]["action"]["shape"][0]
        except (KeyError, IndexError, TypeError):
            pass

    # Only bundle the robots that belong to this task's assembly and the
    # sensors referenced by the task — the checkpoint is trained for this
    # specific physical setup and nothing else makes sense.
    task_sensors = _query_task_sensors(task)
    assembly_robots = _query_assembly_robots(task)

    # Slim down the task block — we only need crop/rotate/resize info plus
    # sensor_ids. Keep everything else as a passthrough for completeness.
    task_block = {
        "id": task.get("id"),
        "name": task.get("name"),
        "assembly_id": task.get("assembly_id"),
        "episode_len": task.get("episode_len"),
        "home_pose": task.get("home_pose"),
        "end_pose": task.get("end_pose"),
        "sensor_ids": sensor_ids,
        "sensor_img_size": task.get("sensor_img_size") or {},
        "sensor_cropped_area": task.get("sensor_cropped_area") or {},
        "sensor_rotate": task.get("sensor_rotate") or {},
    }

    return {
        # ── core ───────────────────────────────────────────────────────
        "checkpoint_id": checkpoint["id"],
        "checkpoint_name": checkpoint.get("name"),
        "policy_type": policy["type"],
        "policy_name": policy.get("name"),
        "vision_backbone": vision_backbone,
        "action_key": action_key,
        "obs_state_keys": obs_state_keys,
        "use_relative_trajectory": use_relative_trajectory,
        "has_succeed": has_succeed,
        "state_dim": state_dim,
        "action_dim": action_dim,
        # ── task / sensor / robot for closed-loop ROS inference ────────
        # Only the robots from this task's assembly and the sensors this task
        # actually uses. ros_inference.py will use these directly.
        "task": task_block,
        "sensors": task_sensors,
        "robots": assembly_robots,
        # ── back-compat alias for the single-step inference.py CLI ──────
        # (older bundles read these top-level fields directly)
        "sensor_ids": sensor_ids,
        "sensor_img_size": task_block["sensor_img_size"],
        "sensor_cropped_area": task_block["sensor_cropped_area"],
        "sensor_rotate": task_block["sensor_rotate"],
        "task_name": task.get("name"),
    }


# ──────────────────────────────────────────────────────────────────────────────
# Zip bundling
# ──────────────────────────────────────────────────────────────────────────────
def _add_dir_to_zip(
    zf: zipfile.ZipFile,
    src_dir: Path,
    arc_root: str,
    skip_names: tuple = ("__pycache__", ".pytest_cache", ".git"),
    skip_suffixes: tuple = (".pyc", ".pyo"),
) -> None:
    """Recursively add a directory's files to the zip under ``arc_root/``.

    Skips bytecode caches and similar dev junk so the bundle isn't bloated.
    """
    src_dir = src_dir.resolve()
    if not src_dir.is_dir():
        raise FileNotFoundError(f"source dir not found: {src_dir}")

    for root, dirs, files in os.walk(src_dir):
        # Mutate dirs in-place to skip noise dirs.
        dirs[:] = [d for d in dirs if d not in skip_names]
        for fname in files:
            if any(fname.endswith(s) for s in skip_suffixes):
                continue
            abs_path = Path(root) / fname
            rel = abs_path.relative_to(src_dir)
            zf.write(abs_path, arcname=str(Path(arc_root) / rel))


def bundle_checkpoint_zip(checkpoint, task, policy) -> tuple[io.BytesIO, str]:
    """Build the zip in-memory.

    Returns
    -------
    (buffer, filename)
        ``buffer`` is a BytesIO with cursor at 0, ready to ``send_file``.
        ``filename`` is the suggested download name (sanitized).
    """
    ckpt_id = checkpoint["id"]
    ckpt_dir = Path(CHECKPOINT_DIR) / str(ckpt_id)
    if not ckpt_dir.is_dir():
        raise FileNotFoundError(
            f"Checkpoint directory not found: {ckpt_dir}. "
            f"Did training finish successfully?"
        )

    # Sanity: required files.
    required = ["config.json", "model.safetensors"]
    missing = [f for f in required if not (ckpt_dir / f).exists()]
    if missing:
        raise FileNotFoundError(
            f"Checkpoint {ckpt_id} is incomplete — missing: {missing}"
        )

    # Lerobot src must exist for the bundle to be runnable.
    if not LEROBOT_SRC_DIR.is_dir():
        raise FileNotFoundError(
            f"Vendored lerobot package not found at {LEROBOT_SRC_DIR}. "
            f"Cannot build a self-contained bundle."
        )
    if not RUNTIME_TEMPLATE_DIR.is_dir():
        raise FileNotFoundError(
            f"Runtime template dir missing: {RUNTIME_TEMPLATE_DIR}"
        )

    meta = build_export_meta(checkpoint, task, policy)

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, mode="w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        # 1. model/ — every file in the checkpoint dir
        for entry in sorted(ckpt_dir.iterdir()):
            if entry.is_file():
                zf.write(entry, arcname=f"model/{entry.name}")

        # 2. lerobot/ — vendored src tree
        _add_dir_to_zip(zf, LEROBOT_SRC_DIR, arc_root="lerobot/lerobot")

        # 3. easytrainer_runtime/ — SimpleAgent + SimpleEnv + image_parser
        _add_dir_to_zip(zf, RUNTIME_TEMPLATE_DIR, arc_root="easytrainer_runtime")

        # 4. inference.py / ros_inference.py / ros_inference_service.py / README.md / requirements.txt
        for tpl, arcname in (
            (INFERENCE_TEMPLATE, "inference.py"),
            (ROS_INFERENCE_TEMPLATE, "ros_inference.py"),
            (ROS_INFERENCE_SERVICE_TEMPLATE, "ros_inference_service.py"),
            (README_TEMPLATE, "README.md"),
            (REQUIREMENTS_TEMPLATE, "requirements.txt"),
        ):
            if not tpl.exists():
                raise FileNotFoundError(f"Export template missing: {tpl}")
            zf.write(tpl, arcname=arcname)

        # 5. export_meta.json — DB-derived metadata
        zf.writestr("export_meta.json", json.dumps(meta, indent=2, ensure_ascii=False))

    buf.seek(0)

    # Sanitize filename — strip path separators and weird characters.
    raw_name = (checkpoint.get("name") or "checkpoint").strip()
    safe_name = "".join(c if (c.isalnum() or c in "-_") else "_" for c in raw_name)[:60]
    filename = f"checkpoint_{ckpt_id}_{safe_name}.zip"

    return buf, filename
