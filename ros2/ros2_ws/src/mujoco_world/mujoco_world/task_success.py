"""Ground-truth success checks for tutorial MuJoCo scenes.

Single source of truth used by:
  * auto data collector — drop episodes where the final state is not a success
    so the training set never contains "almost worked" trajectories.
  * inference evaluator — compute success rate over N trials.
  * mujoco_world_node — publish live `/tutorial/success_json` and answer the
    `/tutorial/check_success` service.

A success check is a pure function of the latest object pose dict
(``{body_name: [x, y, z, qw, qx, qy, qz]}``) and the scene_id (we infer it
from the loaded scene file name when the node starts). Adding a new task
means adding one entry to ``SUCCESS_CHECKERS``.
"""

from __future__ import annotations

import math
import os
from typing import Callable, Dict, Tuple


def _quat_to_world_z(quat) -> Tuple[float, float, float]:
    """Return the world-frame direction the object's local +Z axis points to.

    quat is [w, x, y, z] (MuJoCo convention). Used to test "is the peg
    standing upright" without a full rotation matrix.
    """
    w, x, y, z = quat
    # Third column of the rotation matrix built from [w,x,y,z]
    rx = 2.0 * (x * z + w * y)
    ry = 2.0 * (y * z - w * x)
    rz = 1.0 - 2.0 * (x * x + y * y)
    return rx, ry, rz


def _check_peg_in_hole(poses: Dict[str, list]) -> Tuple[bool, dict]:
    """Peg bottom rests on plate top, inside the slot, axis ≈ vertical.

    Geometry recap (see scene_peg.xml):
      * peg is 60 mm tall, origin at the center → bottom = peg.z - 0.030.
      * hole_base body z is the freejoint origin. Plate top in world frame =
        hole_base.z + 0.005 (plate is a 10 mm-thick slab whose top sits 5 mm
        above the body origin).
      * Walls form an 18×18 mm opening centered on the body x/y.

    Tolerances:
      * xy radius: ≤ 5 mm from hole_center (slot is ±9 mm).
      * peg bottom: within ±3 mm of plate top — i.e. the peg is actually
        resting on the plate, not just hovering over it.
      * tilt: peg local +Z dotted with world +Z ≥ cos(15°) ≈ 0.966.
    """
    peg = poses.get("peg")
    hole = poses.get("hole_base")
    if peg is None or hole is None:
        return False, {"reason": "missing_pose"}

    hole_center_x = hole[0]
    hole_center_y = hole[1]
    plate_top_z = hole[2] + 0.005

    peg_bottom_z = peg[2] - 0.030

    dx = peg[0] - hole_center_x
    dy = peg[1] - hole_center_y
    dz = peg_bottom_z - plate_top_z
    xy_err = math.hypot(dx, dy)

    _, _, zdir = _quat_to_world_z(peg[3:7])
    tilt_cos = zdir

    xy_ok = xy_err <= 0.005
    z_ok = -0.003 <= dz <= 0.003
    tilt_ok = tilt_cos >= 0.966

    success = xy_ok and z_ok and tilt_ok
    return success, {
        "xy_err": round(xy_err, 5),
        "z_err": round(dz, 5),
        "tilt_cos": round(tilt_cos, 4),
        "xy_ok": xy_ok,
        "z_ok": z_ok,
        "tilt_ok": tilt_ok,
    }


def _check_cube_on_plate(poses: Dict[str, list]) -> Tuple[bool, dict]:
    """Default-scene success: cube center within 4 cm of plate center on xy
    and resting above plate top (z within ±2 cm of plate's top surface).

    Conservative defaults — the default scene was a demo, not a benchmark.
    """
    cube = poses.get("cube")
    plate = poses.get("plate")
    if cube is None or plate is None:
        return False, {"reason": "missing_pose"}
    dx = cube[0] - plate[0]
    dy = cube[1] - plate[1]
    xy_err = math.hypot(dx, dy)
    dz = cube[2] - plate[2]
    success = (xy_err <= 0.04) and (-0.02 <= dz <= 0.05)
    return success, {
        "xy_err": round(xy_err, 5),
        "z_err": round(dz, 5),
    }


SUCCESS_CHECKERS: Dict[str, Callable[[Dict[str, list]], Tuple[bool, dict]]] = {
    "peg": _check_peg_in_hole,
    "cube": _check_cube_on_plate,
}


def scene_id_from_path(scene_xml: str) -> str:
    """Map an MJCF path to a checker key. Falls back to 'cube'."""
    name = os.path.basename(scene_xml or "").lower()
    if "peg" in name:
        return "peg"
    return "cube"


def check_success(scene_id: str, poses: Dict[str, list]) -> Tuple[bool, dict]:
    checker = SUCCESS_CHECKERS.get(scene_id, _check_cube_on_plate)
    try:
        return checker(poses)
    except Exception as exc:
        return False, {"reason": f"checker_error: {exc!r}"}
