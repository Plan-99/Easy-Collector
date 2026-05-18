"""Pinocchio + pink IK solver for the exported planner.

Trimmed copy of ``ros2/ros2_bridge/ik_solver/pinocchio_solver/ik_solver.py`` +
``common_arm_ik.py``. The in-container version uses a ``parent_of_cwd`` hack to
make module-relative URDF paths work; the exported bundle pre-resolves the
paths against the bundle dir so we drop that logic entirely.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np
import pinocchio as pin
from pink import solve_ik
from pink.tasks import FrameTask
from pink.configuration import Configuration


# ──────────────────────────────────────────────────────────────────────────────
# Pose helpers
# ──────────────────────────────────────────────────────────────────────────────
def xyzrpy_to_se3(xyzrpy):
    xyz = np.array(xyzrpy[:3])
    rpy = np.array(xyzrpy[3:])
    rotation_matrix = pin.rpy.rpyToMatrix(rpy)
    return pin.SE3(rotation_matrix, xyz)


def se3_to_xyzaxayaz(se3_matrix):
    xyz = se3_matrix.translation
    rotation_matrix = se3_matrix.rotation
    axayaz = pin.log3(rotation_matrix)
    return np.concatenate([xyz, axayaz]).tolist()


def xyzaxayaz_to_se3(xyzaxayaz):
    xyz = np.array(xyzaxayaz[:3])
    axayaz = np.array(xyzaxayaz[3:])
    rotation_matrix = pin.exp3(axayaz)
    return pin.SE3(rotation_matrix, xyz)


# ──────────────────────────────────────────────────────────────────────────────
# IK_Solver
# ──────────────────────────────────────────────────────────────────────────────
class IK_Solver:
    """pink-based per-frame IK over a reduced Pinocchio model."""

    def __init__(self,
                 urdf_path: str,
                 joints_to_lock: Optional[Sequence[str]] = None,
                 ee_definitions: Optional[Sequence[tuple]] = None,
                 dt: float = 0.01,
                 solver: str = 'proxqp',
                 **kwargs):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        joints_to_lock = list(joints_to_lock or [])
        ee_definitions = list(ee_definitions or [])

        full_model = pin.buildModelFromUrdf(urdf_path)
        joints_to_lock_ids = [
            full_model.getJointId(jname)
            for jname in joints_to_lock
            if full_model.existJointName(jname)
        ]
        q_reference = pin.neutral(full_model)
        self.model = pin.buildReducedModel(full_model, joints_to_lock_ids, q_reference)
        self.data = self.model.createData()

        self.nq = self.model.nq
        self.nv = self.model.nv
        self.dt = dt
        self.solver = solver

        self.ee_names = []
        self.ee_frame_names = {}
        for entry in ee_definitions:
            name = entry[0]
            parent_or_frame = entry[1]
            offset = entry[2] if len(entry) > 2 else None
            self.ee_names.append(name)
            frame_name = name
            if offset is not None:
                parent_joint_name = parent_or_frame
                if not self.model.existJointName(parent_joint_name):
                    raise ValueError(f"Joint '{parent_joint_name}' not found in URDF.")
                parent_joint_id = self.model.getJointId(parent_joint_name)
                new_frame = pin.Frame(
                    frame_name, parent_joint_id,
                    pin.SE3(np.eye(3), np.array(offset)),
                    pin.FrameType.OP_FRAME,
                )
                if not self.model.existFrame(frame_name):
                    self.model.addFrame(new_frame)
            else:
                frame_name = parent_or_frame
                if not self.model.existFrame(frame_name):
                    raise ValueError(f"Frame '{frame_name}' not found in URDF.")
            self.ee_frame_names[name] = frame_name

        self.data = self.model.createData()

        self.tasks = {
            name: FrameTask(self.ee_frame_names[name],
                            position_cost=1.0, orientation_cost=1.0)
            for name in self.ee_names
        }
        self.q = pin.neutral(self.model)

    def solve_ik(self, target_poses: dict, current_lr_arm_motor_q=None,
                 current_lr_arm_motor_dq=None):
        if current_lr_arm_motor_q is not None:
            self.q = np.array(current_lr_arm_motor_q)
        for name, target_pose_vec in target_poses.items():
            if name in self.tasks:
                self.tasks[name].set_target(xyzaxayaz_to_se3(target_pose_vec))
        configuration = Configuration(self.model, self.data, self.q)
        try:
            velocity = solve_ik(configuration, self.tasks.values(), self.dt, solver=self.solver)
        except Exception:
            velocity = np.zeros(self.nv)
        self.q = configuration.integrate(velocity, self.dt)
        tau_ff = pin.rnea(self.model, self.data, self.q, velocity, np.zeros(self.nv))
        return self.q, tau_ff

    def get_ee_position(self, q_numeric):
        pin.forwardKinematics(self.model, self.data, np.array(q_numeric))
        pin.updateFramePlacements(self.model, self.data)
        poses = {}
        for name, frame_name in self.ee_frame_names.items():
            frame_id = self.model.getFrameId(frame_name)
            poses[name] = se3_to_xyzaxayaz(self.data.oMf[frame_id])
        return poses

    def reset_state(self, current_q):
        self.q = np.array(current_q).flatten()


# ──────────────────────────────────────────────────────────────────────────────
# Common_ArmIK — facade that pre-resolves URDF paths against a bundle root
# ──────────────────────────────────────────────────────────────────────────────
class Common_ArmIK(IK_Solver):
    """Wraps ``IK_Solver`` with bundle-root path resolution.

    ``urdf_path`` / ``urdf_package_dir`` may be either absolute or relative to
    ``bundle_dir`` (the unpacked planner zip root). Mesh ``package://`` URIs
    inside the URDF are resolved by Pinocchio against the parent of every entry
    in ``package_dir`` — we feed it both the directory itself and its parent so
    the common ``package://<pkg_name>/...`` form works regardless of how the
    URDF authors arranged things.
    """

    def __init__(self,
                 urdf_path: str,
                 urdf_package_dir=None,
                 bundle_dir: Optional[Path] = None,
                 joints_to_lock=None,
                 ee_definitions=None,
                 gravity_compensate: float = 0.0,
                 **kwargs):
        bundle_root = Path(bundle_dir) if bundle_dir is not None else Path.cwd()

        def _resolve(p) -> Path:
            p = Path(p)
            return p if p.is_absolute() else (bundle_root / p)

        urdf_abs = _resolve(urdf_path)
        if not urdf_abs.is_file():
            raise FileNotFoundError(
                f"URDF not found at {urdf_abs} (relative paths are resolved "
                f"against bundle_dir={bundle_root})"
            )

        package_dirs: list[str] = []
        if urdf_package_dir:
            items = (urdf_package_dir
                     if isinstance(urdf_package_dir, (list, tuple))
                     else [urdf_package_dir])
            seen: set[str] = set()
            for item in items:
                p = _resolve(item).resolve()
                for candidate in (p, p.parent):
                    s = str(candidate)
                    if s not in seen:
                        seen.add(s)
                        package_dirs.append(s)

        self.gravity_compensate = float(gravity_compensate or 0.0)

        if package_dirs:
            # pinocchio uses ROS_PACKAGE_PATH for package:// resolution if
            # buildModelFromUrdf doesn't take package_dirs. We patch the env
            # var for the lifetime of this constructor.
            prev = os.environ.get('ROS_PACKAGE_PATH', '')
            joined = ':'.join(package_dirs + ([prev] if prev else []))
            os.environ['ROS_PACKAGE_PATH'] = joined

        print(f"[IK_Solver] Initialized urdf={urdf_abs} "
              f"package_dirs={package_dirs} "
              f"joints_to_lock={joints_to_lock} "
              f"ee_definitions={ee_definitions} "
              f"gravity_compensate={self.gravity_compensate}")

        super().__init__(
            urdf_path=str(urdf_abs),
            joints_to_lock=joints_to_lock,
            ee_definitions=ee_definitions,
            **kwargs,
        )

    def solve_ik(self, target_poses: dict, **kwargs):
        if self.gravity_compensate != 0.0:
            target_poses = {
                name: [pose[0], pose[1], pose[2] + self.gravity_compensate]
                       + list(pose[3:])
                for name, pose in target_poses.items()
            }
        return super().solve_ik(target_poses, **kwargs)


def normalize_ee_definitions(ee_definitions) -> list:
    """Accept JSON-ish ``[{name, parent, offset}]`` or already-tuple form, and
    return the tuple form (``(name, parent, offset_or_None)``) that
    ``IK_Solver`` consumes."""
    out = []
    for item in (ee_definitions or []):
        if isinstance(item, dict):
            name = item.get('name')
            parent = item.get('parent')
            offset = item.get('offset')
        else:
            name = item[0]
            parent = item[1]
            offset = item[2] if len(item) > 2 else None
        if isinstance(offset, list):
            offset = np.array(offset).T
        out.append((name, parent, offset))
    return out
