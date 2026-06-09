#!/usr/bin/env python3
"""vision-reach M1 oracle — headless MuJoCo verification of `visual_reach`.

L1 = perception accuracy (estimated target/fingertip world XYZ vs ground-truth).
L2 = reach success: back-project target -> world, DLS-IK the EE (ee_site) to
     (target_xy, peg_top_z + hover), then measure the FINAL ee pose against the
     ground-truth peg. Pure kinematic placement (verifies the perceive->target->IK
     chain reaches above the real object; dynamic actuation is the planner's job).

Run inside the ros2 container (MUJOCO_GL=egl):
  python3 run_oracle.py --mode l2 --cam wrist_cam --n 20 --hover 0.06
"""
import os, argparse, json
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
import mujoco

SCENE = "/root/ros2_ws/src/mujoco_world/assets/scene_peg.xml"
W, H = 320, 240
PEG_HALF_H = 0.030  # peg box half-height (scene_peg.xml peg_geom size z)


def name2id(m, typ, n):
    return mujoco.mj_name2id(m, typ, n)


def intrinsics(m, cid):
    fovy = float(m.cam_fovy[cid])
    f = 0.5 * H / np.tan(np.deg2rad(fovy) * 0.5)
    return f, (W - 1) / 2.0, (H - 1) / 2.0


def backproject(u, v, z, f, cx, cy, cam_pos, cam_mat):
    p_cam = np.array([(u - cx) / f * z, -(v - cy) / f * z, -z])
    return cam_pos + cam_mat.reshape(3, 3) @ p_cam


def project(p_world, f, cx, cy, cam_pos, cam_mat):
    p_cam = cam_mat.reshape(3, 3).T @ (p_world - cam_pos)
    z = -p_cam[2]
    if z <= 1e-6:
        return None
    return np.array([p_cam[0] / z * f + cx, -p_cam[1] / z * f + cy])


def detect_red(rgb):
    r, g, b = rgb[..., 0].astype(int), rgb[..., 1].astype(int), rgb[..., 2].astype(int)
    mask = (r > 110) & (r - g > 50) & (r - b > 50)
    ys, xs = np.where(mask)
    if len(xs) < 8:
        return None, mask
    return np.array([xs.mean(), ys.mean()]), mask


def ik_move(m, d, ee_sid, target, arm_q, arm_dof, iters=200, tol=5e-4, damp=2e-3):
    for _ in range(iters):
        mujoco.mj_forward(m, d)
        err = target - d.site_xpos[ee_sid]
        if np.linalg.norm(err) < tol:
            break
        jacp = np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jacp, None, ee_sid)
        J = jacp[:, arm_dof]
        dq = J.T @ np.linalg.solve(J @ J.T + damp * np.eye(3), err)
        d.qpos[arm_q] += np.clip(dq, -0.3, 0.3)
        # clamp to joint ranges
        for k, qa in enumerate(arm_q):
            lo, hi = m.jnt_range[name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"joint{k+1}")]
            d.qpos[qa] = min(max(d.qpos[qa], lo), hi)
    mujoco.mj_forward(m, d)
    return d.site_xpos[ee_sid].copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="l2", choices=["l1", "l2"])
    ap.add_argument("--cam", default="wrist_cam")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--hover", type=float, default=0.06)
    ap.add_argument("--tol-xy", type=float, default=0.03)
    ap.add_argument("--tol-z", type=float, default=0.02)
    ap.add_argument("--target-rate", type=float, default=0.8)
    ap.add_argument("--save", default="/tmp/vr_oracle.png")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    key = name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
    cid = name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, args.cam)
    peg_bid = name2id(m, mujoco.mjtObj.mjOBJ_BODY, "peg")
    peg_q = m.jnt_qposadr[name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "peg_free")]
    ee_sid = name2id(m, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    arm_q = [m.jnt_qposadr[name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}")] for i in range(1, 7)]
    arm_dof = [m.jnt_dofadr[name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}")] for i in range(1, 7)]

    rgb_r = mujoco.Renderer(m, height=H, width=W)
    dep_r = mujoco.Renderer(m, height=H, width=W); dep_r.enable_depth_rendering()
    f = cx = cy = 0.0

    rng = np.random.default_rng(1)
    rows, l1, l2_ok = [], [], 0
    last = None
    for i in range(args.n):
        mujoco.mj_resetDataKeyframe(m, d, key)
        # peg within the observation cam FOV (R1: observe pose puts objects in view)
        px, py = 0.30 + rng.uniform(-0.035, 0.045), 0.01 + rng.uniform(-0.045, 0.05)
        d.qpos[peg_q:peg_q + 3] = [px, py, 0.035]
        d.qpos[peg_q + 3:peg_q + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(m, d)

        rgb_r.update_scene(d, camera=args.cam); rgb = rgb_r.render()
        dep_r.update_scene(d, camera=args.cam); depth = dep_r.render()
        cam_pos, cam_mat = d.cam_xpos[cid].copy(), d.cam_xmat[cid].copy()
        if f == 0.0:
            f, cx, cy = intrinsics(m, cid)
        peg_gt = d.xpos[peg_bid].copy()
        peg_top = peg_gt.copy(); peg_top[2] += PEG_HALF_H  # visible/approach reference

        cen, mask = detect_red(rgb)
        rec = {"i": i, "peg_xy": peg_gt[:2].round(4).tolist()}
        if cen is None:
            rec["status"] = "no_red_detected"; rows.append(rec)
            if last is None:
                last = (rgb, mask, None, None)
            continue
        u, v = int(round(cen[0])), int(round(cen[1]))
        zc = float(depth[v, u])
        est = backproject(cen[0], cen[1], zc, f, cx, cy, cam_pos, cam_mat)
        l1_err = float(np.linalg.norm(est - peg_top))  # vs visible top surface
        l1.append(l1_err)
        rec.update(status="ok", est=est.round(4).tolist(), l1_err=round(l1_err, 4))

        if args.mode == "l2":
            target = np.array([est[0], est[1], peg_top[2] + args.hover])
            ee_final = ik_move(m, d, ee_sid, target, arm_q, arm_dof)
            exy = float(np.linalg.norm(ee_final[:2] - peg_gt[:2]))
            ez = float(ee_final[2] - peg_top[2])  # want ~ +hover
            ok = (exy < args.tol_xy) and (abs(ez - args.hover) < args.tol_z)
            l2_ok += int(ok)
            rec.update(ee_final=ee_final.round(4).tolist(), reach_xy=round(exy, 4),
                       hover_z=round(ez, 4), pass_l2=bool(ok))
            ee_uv = project(ee_final, f, cx, cy, cam_pos, cam_mat)
        else:
            ee_uv = project(d.site_xpos[ee_sid], f, cx, cy, cam_pos, cam_mat)
        rows.append(rec)
        last = (rgb, mask, (u, v), ee_uv)

    if last is not None:
        rgb, mask, cen, ee_uv = last
        ov = rgb.copy(); ov[mask] = (0.5 * ov[mask] + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
        def stamp(img, uv, col, s=5):
            if uv is None: return
            x, y = int(uv[0]), int(uv[1])
            img[max(0, y-s):y+s, max(0, x-1):x+1] = col
            img[max(0, y-1):y+1, max(0, x-s):x+s] = col
        stamp(ov, cen, [255, 0, 0]); stamp(ov, ee_uv, [0, 0, 255])
        try:
            from PIL import Image; Image.fromarray(ov).save(args.save)
            print(f"[overlay] {args.save}")
        except Exception as e:
            print("[overlay] skip:", e)

    det = sum(1 for r in rows if r.get("status") == "ok")
    summary = {"mode": args.mode, "cam": args.cam, "n": args.n, "detected": det,
               "l1_mean_err_m": round(float(np.mean(l1)), 4) if l1 else None,
               "l1_max_err_m": round(float(np.max(l1)), 4) if l1 else None}
    if args.mode == "l2":
        rate = l2_ok / args.n
        summary.update(reach_success=l2_ok, reach_rate=round(rate, 3),
                       target_rate=args.target_rate, PASS=bool(rate >= args.target_rate))
    print("ROWS:", json.dumps(rows, ensure_ascii=False))
    print("SUMMARY:", json.dumps(summary, ensure_ascii=False))
    if args.mode == "l2":
        raise SystemExit(0 if summary.get("PASS") else 1)


if __name__ == "__main__":
    main()
