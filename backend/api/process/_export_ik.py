"""IK / URDF bundling helpers for the planner exporter.

For each robot referenced by a planner block, decide whether it has an IK
solver and, if so, return the URDF + ee definitions in a form that can be
written into the planner zip:

    {
        'urdf_path': 'urdfs/<robot_id>/<package_dir_basename>/<...>',
        'urdf_package_dir': 'urdfs/<robot_id>/<package_dir_basename>',
        'ik_setting': {
            'joints_to_lock': [...],
            'ee_definitions': [{'name', 'parent', 'offset'}],
            'gravity_compensate': float,
        },
        '_source_package_dir': <absolute host path to copy into the zip>,
    }

The exporter walks the package dir and copies its files under
``urdfs/<robot_id>/<package_dir_basename>/...`` so the URDF's relative
``package://`` mesh references keep working at runtime. ``_source_package_dir``
is private to the exporter — it's stripped before the dict ends up in
``planner_meta.json``.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


def _translate_ros2_path(path: str) -> str:
    """Custom robot URDFs are stored in the DB with paths as seen from the
    ROS 2 container (e.g. ``/root/ros2_ws/src/...``). The backend container
    mounts the same source tree at ``/opt/easytrainer/project/ros2/ros2_ws/``.
    Rewrite the prefix so the exporter can read the file."""
    if not path:
        return path
    data_root = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
    backend_ros2_root = os.path.join(data_root, 'project', 'ros2')
    prefixes = (
        ('/root/ros2_ws/', f'{backend_ros2_root}/ros2_ws/'),
        ('/root/robot_sdk/', f'{backend_ros2_root}/robot_sdk/'),
    )
    for src_prefix, dst_prefix in prefixes:
        if path.startswith(src_prefix):
            return dst_prefix + path[len(src_prefix):]
    return path


def _ik_block_from_manifest(robot_type: str) -> Optional[dict]:
    """Look up the ``ik`` section in installed module manifests for a built-in
    robot type. Returns None if the robot type has no manifest IK or no
    manifest at all (e.g. custom robot)."""
    from ...configs.module_loader import _iter_manifests, _resolve_manifest_paths

    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'robot':
            continue
        manifest = _resolve_manifest_paths(manifest)
        for entry in manifest.get('robots') or []:
            if entry.get('type') != robot_type:
                continue
            ik = entry.get('ik') or {}
            if not ik.get('urdf_path'):
                return None
            return ik
    return None


def _apply_ee_offset_override(ee_definitions: list, user_ee_offset: dict) -> list:
    """The agent UI lets the user override the per-EE offset stored in
    ``robot.settings.ee_offset`` (``{ee_name: [x, y, z]}``). Apply that
    override to each ee_definition's offset (3rd field)."""
    if not user_ee_offset:
        return ee_definitions
    out = []
    for item in ee_definitions:
        if isinstance(item, dict):
            name = item.get('name')
            parent = item.get('parent')
            offset = item.get('offset')
        else:
            name = item[0]
            parent = item[1]
            offset = item[2] if len(item) > 2 else None
        if name in user_ee_offset:
            offset = user_ee_offset[name]
        out.append({'name': name, 'parent': parent, 'offset': offset})
    return out


def collect_ik_info(robot: dict) -> Optional[dict]:
    """Return bundle-ready IK info for ``robot`` (a row from
    ``Robot.to_dict()``) or None if it has no IK solver.

    Resolves URDFs to absolute host paths so the exporter can copy them; the
    in-zip path rewrite happens later in ``rebase_for_bundle``.
    """
    if not robot.get('ik_available'):
        return None

    settings = robot.get('settings') or {}
    user_ee_offset = settings.get('ee_offset') or {}
    robot_type = robot.get('type')

    if robot_type == 'custom':
        urdf_path = settings.get('urdf_path')
        urdf_package_dir = settings.get('urdf_package_dir', '')
        ik_setting_raw = settings.get('ik_setting') or {}
        ee_definitions = ik_setting_raw.get('ee_definitions') or []
        joints_to_lock = ik_setting_raw.get('joints_to_lock') or []
        gravity_compensate = ik_setting_raw.get('gravity_compensate', 0.0)
    else:
        ik = _ik_block_from_manifest(robot_type)
        if ik is None:
            return None
        urdf_path = ik.get('urdf_path')
        urdf_package_dir = ik.get('urdf_package_dir', '')
        ee_definitions = ik.get('ee_definitions') or []
        joints_to_lock = ik.get('joints_to_lock') or []
        gravity_compensate = ik.get('gravity_compensate', 0.0)

    if not urdf_path:
        return None

    urdf_path = str(urdf_path)
    urdf_package_dir = str(urdf_package_dir or '')

    if not os.path.isabs(urdf_path):
        # Should not happen for module robots (placeholders are already
        # substituted by _resolve_manifest_paths). For custom robots we trust
        # the user but reject relative paths since we can't anchor them.
        return None

    # Custom robots store URDF paths as the ROS 2 container sees them
    # (/root/ros2_ws/...). Translate to the backend-container view so the
    # exporter can read the actual files.
    if not Path(urdf_path).is_file():
        translated = _translate_ros2_path(urdf_path)
        if translated != urdf_path and Path(translated).is_file():
            urdf_path = translated
        else:
            return None
    if urdf_package_dir:
        if not Path(urdf_package_dir).is_dir():
            translated = _translate_ros2_path(urdf_package_dir)
            if translated != urdf_package_dir and Path(translated).is_dir():
                urdf_package_dir = translated
            else:
                urdf_package_dir = ''

    ee_definitions = _apply_ee_offset_override(ee_definitions, user_ee_offset)

    return {
        'urdf_path': urdf_path,
        'urdf_package_dir': urdf_package_dir,
        'ik_setting': {
            'joints_to_lock': joints_to_lock,
            'ee_definitions': ee_definitions,
            'gravity_compensate': float(gravity_compensate or 0.0),
        },
    }


def rebase_for_bundle(robot_id: int, ik_info: dict) -> dict:
    """Compute the in-zip paths and return the dict that goes into
    planner_meta.json. Side effect: returned dict has ``_source_package_dir``
    and ``_source_urdf_path`` for the exporter to copy from.
    """
    urdf_path = Path(ik_info['urdf_path'])
    package_dir = Path(ik_info['urdf_package_dir']) if ik_info.get('urdf_package_dir') else None

    if package_dir is not None:
        pkg_name = package_dir.name or f"robot_{robot_id}"
        try:
            rel_urdf = urdf_path.resolve().relative_to(package_dir.resolve())
        except ValueError:
            # URDF lives outside the package dir — fallback: bundle the URDF
            # alongside the package as a sibling. We use the file basename.
            rel_urdf = Path(urdf_path.name)
        zip_pkg_dir = f"urdfs/{robot_id}/{pkg_name}"
        # If the URDF is outside the package dir we'll drop it at the package
        # root in the zip; otherwise preserve its relative position.
        if rel_urdf == Path(urdf_path.name):
            zip_urdf = f"{zip_pkg_dir}/{urdf_path.name}"
        else:
            zip_urdf = f"{zip_pkg_dir}/{rel_urdf.as_posix()}"
    else:
        pkg_name = f"robot_{robot_id}"
        zip_pkg_dir = f"urdfs/{robot_id}/{pkg_name}"
        zip_urdf = f"{zip_pkg_dir}/{urdf_path.name}"

    return {
        'urdf_path': zip_urdf,
        'urdf_package_dir': zip_pkg_dir,
        'ik_setting': ik_info['ik_setting'],
        '_source_package_dir': str(package_dir) if package_dir is not None else None,
        '_source_urdf_path': str(urdf_path),
        '_zip_pkg_dir': zip_pkg_dir,
        '_zip_urdf_path': zip_urdf,
    }
