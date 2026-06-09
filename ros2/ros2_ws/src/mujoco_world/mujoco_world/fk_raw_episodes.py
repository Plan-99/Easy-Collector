"""Forward-kinematics pass over raw_episodes/ to populate ee_pos.npy.

The tutorial collector only records (state, action) = joint positions; the
EasyTrainer training pipeline can train on action_key='relative_ee_pos' but
only when the dataset has an `observation.eepos` column. To get there we
read each raw episode's state.npy (qpos), run MuJoCo's kinematics on the
tutorial arm model, and write a sibling ``ee_pos.npy`` of shape
``(T, 6) = [x, y, z, rx, ry, rz]`` (axis-angle rotation vector) per frame.

Run inside the easytrainer_ros2 container — mujoco lives there.

    python3 -m mujoco_world.fk_raw_episodes \\
        --raw-dir /opt/easytrainer/training_data/raw_episodes \\
        --scene-xml /root/ros2_ws/src/mujoco_world/assets/scene_peg.xml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


def _qpos_adr(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise RuntimeError(f"joint not found in MJCF: {joint_name}")
    return int(model.jnt_qposadr[jid])


def _build_writer(scene_xml: Path):
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)
    qadrs = [_qpos_adr(model, j) for j in ARM_JOINTS]
    # gripper_mirror tracks gripper — set it too for visual fidelity (not needed
    # for the ee_site pose since ee_site is on gripper_base, but the mirror
    # joint exists in the model).
    mirror_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_mirror")
    mirror_adr = int(model.jnt_qposadr[mirror_id]) if mirror_id >= 0 else None
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    if site_id < 0:
        raise RuntimeError("ee_site not found in MJCF")

    def compute_eepos(state_arr: np.ndarray) -> np.ndarray:
        """state_arr: (T, 7) — joint angles [j1..j6, gripper]. Returns (T, 6)."""
        T = int(state_arr.shape[0])
        out = np.zeros((T, 6), dtype=np.float32)
        for t in range(T):
            for j, adr in enumerate(qadrs):
                data.qpos[adr] = float(state_arr[t, j])
            if mirror_adr is not None:
                # mirror = gripper (both slide in opposite directions in MJCF;
                # here we just match the magnitude — relevant only for visual).
                data.qpos[mirror_adr] = float(state_arr[t, 6])
            mujoco.mj_kinematics(model, data)
            xpos = np.array(data.site_xpos[site_id], dtype=np.float32)
            xmat = np.array(data.site_xmat[site_id], dtype=np.float32).reshape(3, 3)
            rvec = Rotation.from_matrix(xmat).as_rotvec().astype(np.float32)
            out[t, :3] = xpos
            out[t, 3:] = rvec
        return out

    return compute_eepos


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--raw-dir", required=True, type=Path)
    p.add_argument("--scene-xml", required=True, type=Path)
    p.add_argument("--force", action="store_true",
                    help="Recompute ee_pos.npy even if it already exists.")
    args = p.parse_args(argv)

    if not args.raw_dir.is_dir():
        print(f"[fk] raw_dir missing: {args.raw_dir}", file=sys.stderr)
        return 2
    if not args.scene_xml.is_file():
        print(f"[fk] scene_xml missing: {args.scene_xml}", file=sys.stderr)
        return 2

    print(f"[fk] loading MJCF: {args.scene_xml}", flush=True)
    compute = _build_writer(args.scene_xml)

    eps = sorted([d for d in args.raw_dir.iterdir()
                  if d.is_dir() and (d / "state.npy").is_file()])
    print(f"[fk] processing {len(eps)} episodes from {args.raw_dir}", flush=True)

    written = 0
    skipped = 0
    for d in eps:
        out_path = d / "ee_pos.npy"
        if out_path.exists() and not args.force:
            skipped += 1
            continue
        state = np.load(d / "state.npy")
        if state.ndim != 2 or state.shape[1] < 7:
            print(f"[fk] skip {d.name}: unexpected state shape {state.shape}",
                  file=sys.stderr)
            continue
        eep = compute(state[:, :7])
        np.save(out_path, eep)
        written += 1
        if written % 10 == 0:
            print(f"[fk]   wrote {written}/{len(eps)}", flush=True)

    print(f"[fk] done. wrote={written}, skipped(existing)={skipped}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
