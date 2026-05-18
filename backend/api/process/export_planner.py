"""Bundle a planner + every checkpoint it references + standalone execution
code into a single downloadable zip.

Used by the ``/api/planner/<id>/:export`` route. The bundle runs outside the
EasyTrainer container with no DB / no Flask / no gRPC bridge — just
``pip install -r requirements.txt`` and either:

    python3 run_planner.py [--repeat N]      # run the planner directly
    python3 ros_planner_service.py           # run it on a ROS 2 service call

Bundle layout::

    planner_<id>_<name>.zip
    ├── run_planner.py            (verbatim — autonomous runner)
    ├── ros_planner_service.py    (verbatim — service-triggered node)
    ├── inference.py              (verbatim — CheckpointInference single-step API)
    ├── planner_meta.json         (built here — groups + workspaces + checkpoints)
    ├── README.md                 (verbatim)
    ├── requirements.txt          (verbatim)
    ├── easytrainer_runtime/      (SimpleAgent + SimpleEnv + image_parser + planner_engine)
    ├── lerobot/lerobot/          (vendored backend/lerobot/src/lerobot/)
    └── checkpoints/
        └── <checkpoint_id>/
            ├── config.json, model.safetensors, policy_*processor.json, ...
            └── export_meta.json  (per-checkpoint meta for CheckpointInference)
"""
from __future__ import annotations

import io
import json
import zipfile
from pathlib import Path

from ...configs.global_configs import resolve_checkpoint_dir
from ...database.models.task_model import Task as TaskModel
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
from ._export_ik import collect_ik_info, rebase_for_bundle
from .export_checkpoint import (
    build_export_meta,
    _add_dir_to_zip,
    LEROBOT_SRC_DIR,
    RUNTIME_TEMPLATE_DIR,
    INFERENCE_TEMPLATE,
    REQUIREMENTS_TEMPLATE,
)

_TEMPLATE_DIR = Path(__file__).resolve().parent / "export_templates"
RUN_PLANNER_TEMPLATE = _TEMPLATE_DIR / "run_planner.py"
ROS_PLANNER_SERVICE_TEMPLATE = _TEMPLATE_DIR / "ros_planner_service.py"
README_PLANNER_TEMPLATE = _TEMPLATE_DIR / "README_planner.md"
DOCKERFILE_TEMPLATE = _TEMPLATE_DIR / "Dockerfile"
DOCKER_COMPOSE_TEMPLATE = _TEMPLATE_DIR / "docker-compose.yml"

# Assembly slots a workspace can fill — kept in sync with planner_run.ASSEMBLY_SLOTS.
ASSEMBLY_SLOTS = ("left_arm", "right_arm", "left_tool", "right_tool", "mobile_base")


# ──────────────────────────────────────────────────────────────────────────────
# Collection helpers
# ──────────────────────────────────────────────────────────────────────────────
def _slim_workspace(task_dict: dict, robot_ik_map: dict | None = None) -> dict:
    """Keep only what the standalone planner engine needs from a task row:
    assembly topology (so SimpleAgent can be built + query_pose slots resolved),
    sensors (topics), home pose, and per-sensor crop/rotate/resize config.

    If ``robot_ik_map`` is provided (``{robot_id: ik_meta_for_bundle}``) each
    robot entry gets an ``ik`` key with the in-zip URDF paths + ee definitions
    so SimpleAgent can build a Common_ArmIK at runtime.
    """
    assembly = task_dict.get("assembly") or {}
    raw_robots = assembly.get("robots") or []
    if robot_ik_map:
        annotated_robots = []
        for robot in raw_robots:
            rid = int(robot["id"])
            meta = robot_ik_map.get(rid)
            if meta is not None:
                robot = dict(robot)
                robot["ik"] = {
                    "urdf_path": meta["urdf_path"],
                    "urdf_package_dir": meta["urdf_package_dir"],
                    "ik_setting": meta["ik_setting"],
                }
            annotated_robots.append(robot)
        raw_robots = annotated_robots
    slim_assembly = {"robots": raw_robots}
    for slot in ASSEMBLY_SLOTS:
        slim_assembly[slot] = assembly.get(slot)

    return {
        "id": task_dict.get("id"),
        "name": task_dict.get("name"),
        "assembly": slim_assembly,
        "sensors": task_dict.get("sensors") or [],
        "home_pose": task_dict.get("home_pose") or {},
        "sensor_img_size": task_dict.get("sensor_img_size") or {},
        "sensor_cropped_area": task_dict.get("sensor_cropped_area") or {},
        "sensor_rotate": task_dict.get("sensor_rotate") or {},
        "sensor_sam3": task_dict.get("sensor_sam3") or {},
    }


def _collect_referenced_ids(groups: list) -> tuple[set, set]:
    """Return (workspace_ids, checkpoint_ids) referenced by any block in any
    group. Sync / timesleep blocks contribute nothing."""
    workspace_ids: set = set()
    checkpoint_ids: set = set()
    for grp in groups:
        for block in grp.get("blocks") or []:
            ws_id = block.get("workspace_id")
            if ws_id is not None:
                workspace_ids.add(ws_id)
            if block.get("type") == "checkpoint" and block.get("checkpoint_id") is not None:
                checkpoint_ids.add(block.get("checkpoint_id"))
    return workspace_ids, checkpoint_ids


# ──────────────────────────────────────────────────────────────────────────────
# Zip bundling
# ──────────────────────────────────────────────────────────────────────────────
def bundle_planner_zip(planner: dict, groups: list) -> tuple[io.BytesIO, str]:
    """Build the planner export zip in-memory.

    Parameters
    ----------
    planner:
        The planner DB row as a dict (``id``, ``name``).
    groups:
        The planner's ``plans`` — ``[{id, workspace_ids, blocks}, ...]``. Only
        groups with at least one block are exported.

    Returns
    -------
    (buffer, filename)
        ``buffer`` is a BytesIO at cursor 0, ready for ``send_file``.
    """
    groups = [g for g in (groups or []) if g.get("blocks")]
    if not groups:
        raise ValueError("Planner has no groups with blocks — nothing to export.")

    if not LEROBOT_SRC_DIR.is_dir():
        raise FileNotFoundError(
            f"Vendored lerobot package not found at {LEROBOT_SRC_DIR}. "
            f"Cannot build a self-contained bundle."
        )
    if not RUNTIME_TEMPLATE_DIR.is_dir():
        raise FileNotFoundError(f"Runtime template dir missing: {RUNTIME_TEMPLATE_DIR}")

    workspace_ids, checkpoint_ids = _collect_referenced_ids(groups)

    # ── workspaces ──────────────────────────────────────────────────────
    # First pass: load task dicts, collect IK info per unique robot.
    task_dicts: dict = {}
    robot_ik_bundle: dict = {}  # robot_id -> rebase_for_bundle output
    for ws_id in workspace_ids:
        task = TaskModel.find(ws_id)
        if task is None:
            raise ValueError(
                f"Workspace (task) {ws_id} referenced by a planner block no "
                f"longer exists. Fix the planner before exporting."
            )
        td = task.to_dict()
        task_dicts[ws_id] = td
        for robot in (td.get("assembly") or {}).get("robots") or []:
            rid = int(robot["id"])
            if rid in robot_ik_bundle:
                continue
            try:
                ik_info = collect_ik_info(robot)
            except Exception as e:
                print(f"[export_planner] WARN: IK lookup failed for robot "
                      f"{rid} ({robot.get('name')}): {e}")
                ik_info = None
            if ik_info is None:
                continue
            robot_ik_bundle[rid] = rebase_for_bundle(rid, ik_info)

    workspaces: dict = {
        str(ws_id): _slim_workspace(td, robot_ik_bundle)
        for ws_id, td in task_dicts.items()
    }

    # ── checkpoints — meta + which model dirs to bundle ─────────────────
    checkpoints_meta: dict = {}
    checkpoint_dirs: dict = {}  # checkpoint_id -> resolved model dir Path
    for cid in checkpoint_ids:
        ckpt = CheckpointModel.find(cid)
        if ckpt is None:
            raise ValueError(
                f"Checkpoint {cid} referenced by a planner block no longer "
                f"exists. Fix the planner before exporting."
            )
        if not ckpt.policy:
            raise ValueError(f"Checkpoint {cid} has no associated policy.")
        if not ckpt.task:
            raise ValueError(f"Checkpoint {cid} has no associated task.")

        ckpt_dir = Path(resolve_checkpoint_dir(cid))
        if not ckpt_dir.is_dir():
            raise FileNotFoundError(
                f"Checkpoint {cid} directory not found: {ckpt_dir}. "
                f"Did training finish successfully?"
            )
        missing = [f for f in ("config.json", "model.safetensors")
                   if not (ckpt_dir / f).exists()]
        if missing:
            raise FileNotFoundError(
                f"Checkpoint {cid} is incomplete — missing: {missing}"
            )

        meta = build_export_meta(
            checkpoint=ckpt.to_dict(),
            task=ckpt.task.to_dict(),
            policy=ckpt.policy.to_dict(),
        )

        # The standalone engine only knows how to run qaction / resnet18
        # checkpoints — fail at export time with a clear message rather than
        # letting the bundle blow up on the user's machine.
        action_key = meta.get("action_key", "qaction")
        if action_key != "qaction":
            raise ValueError(
                f"Checkpoint {cid} ('{meta.get('checkpoint_name')}') uses "
                f"action_key='{action_key}'. The exported planner only supports "
                f"'qaction' checkpoints — run this planner inside EasyTrainer."
            )
        backbone = meta.get("vision_backbone", "resnet18")
        if backbone != "resnet18":
            raise ValueError(
                f"Checkpoint {cid} ('{meta.get('checkpoint_name')}') uses "
                f"vision_backbone='{backbone}'. The exported planner only "
                f"supports 'resnet18' — run this planner inside EasyTrainer."
            )

        checkpoints_meta[str(cid)] = meta
        checkpoint_dirs[str(cid)] = ckpt_dir

    # ── planner_meta.json ───────────────────────────────────────────────
    # The robot_ik_bundle dict carries some private (_source_*) keys that the
    # exporter uses to copy URDFs into the zip; strip them before emitting the
    # meta so they don't leak into the bundle's planner_meta.json. The robot
    # dicts in workspaces already have the public {urdf_path, urdf_package_dir,
    # ik_setting} fields attached by _slim_workspace.
    planner_meta = {
        "planner_id": planner.get("id"),
        "planner_name": planner.get("name"),
        "groups": groups,
        "workspaces": workspaces,
        "checkpoints": checkpoints_meta,
    }

    # ── build the zip ───────────────────────────────────────────────────
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, mode="w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        # 1. checkpoints/<id>/ — model files + per-checkpoint export_meta.json
        for cid, ckpt_dir in checkpoint_dirs.items():
            for entry in sorted(ckpt_dir.iterdir()):
                if entry.is_file():
                    zf.write(entry, arcname=f"checkpoints/{cid}/{entry.name}")
            zf.writestr(
                f"checkpoints/{cid}/export_meta.json",
                json.dumps(checkpoints_meta[cid], indent=2, ensure_ascii=False),
            )

        # 2. lerobot/ — vendored src tree
        _add_dir_to_zip(zf, LEROBOT_SRC_DIR, arc_root="lerobot/lerobot")

        # 3. easytrainer_runtime/ — SimpleAgent + SimpleEnv + image_parser + planner_engine
        _add_dir_to_zip(zf, RUNTIME_TEMPLATE_DIR, arc_root="easytrainer_runtime")

        # 4. urdfs/<robot_id>/ — bundled URDFs + mesh package dirs (for IK)
        for rid, ik_meta in robot_ik_bundle.items():
            src_pkg = ik_meta.get("_source_package_dir")
            src_urdf = ik_meta.get("_source_urdf_path")
            zip_pkg = ik_meta.get("_zip_pkg_dir")
            zip_urdf = ik_meta.get("_zip_urdf_path")
            if src_pkg:
                _add_dir_to_zip(zf, Path(src_pkg), arc_root=zip_pkg)
                # If the URDF lives outside the package_dir, write it
                # separately at the zip URDF path.
                try:
                    Path(src_urdf).resolve().relative_to(Path(src_pkg).resolve())
                    in_pkg = True
                except ValueError:
                    in_pkg = False
                if not in_pkg and src_urdf:
                    zf.write(src_urdf, arcname=zip_urdf)
            elif src_urdf:
                zf.write(src_urdf, arcname=zip_urdf)

        # 5. entry-point scripts + docs + docker test environment
        for tpl, arcname in (
            (INFERENCE_TEMPLATE, "inference.py"),
            (RUN_PLANNER_TEMPLATE, "run_planner.py"),
            (ROS_PLANNER_SERVICE_TEMPLATE, "ros_planner_service.py"),
            (README_PLANNER_TEMPLATE, "README.md"),
            (REQUIREMENTS_TEMPLATE, "requirements.txt"),
            (DOCKERFILE_TEMPLATE, "Dockerfile"),
            (DOCKER_COMPOSE_TEMPLATE, "docker-compose.yml"),
        ):
            if not tpl.exists():
                raise FileNotFoundError(f"Export template missing: {tpl}")
            zf.write(tpl, arcname=arcname)

        # 6. planner_meta.json
        zf.writestr(
            "planner_meta.json",
            json.dumps(planner_meta, indent=2, ensure_ascii=False),
        )

    buf.seek(0)

    raw_name = (planner.get("name") or "planner").strip()
    safe_name = "".join(c if (c.isalnum() or c in "-_") else "_" for c in raw_name)[:60]
    filename = f"planner_{planner.get('id')}_{safe_name}.zip"

    return buf, filename
