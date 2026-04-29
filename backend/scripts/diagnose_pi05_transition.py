"""PI05 transition frame 진단.

이전 replay 진단은 stationary frame(62.5%)이 평균을 낮춰서 transition 오차를 가렸음.
이 스크립트는 'moving frame'만 골라서 각 episode마다 transition 정확도를 측정.

Moving frame 정의:
  state delta가 큰 window의 중앙 frame들 (action-state magnitude 기준 상위 30%).

출력:
  - 각 episode마다 transition 시점의 norm_err 평균
  - 전체 dataset의 transition vs stationary norm_err 비교
  - 큐브 도달점 분포 vs 모델 예측 도달점 분포 (mode collapse 검출)

사용:
  docker exec easy_collector_service bash -lc "cd /root && python3 -m src.backend.scripts.diagnose_pi05_transition \\
      --ckpt 96 --dataset 22 --episodes 0 5 10 15 20 25 30 35 40 45 50 55 \\
      --task 'Pick the red cube and place it on the plate'"
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_lerobot_src = Path(__file__).resolve().parent.parent / "lerobot" / "src"
if str(_lerobot_src) not in sys.path:
    sys.path.insert(0, str(_lerobot_src))

import numpy as np
import pyarrow.parquet as pq
import torch
from PIL import Image

from lerobot.policies.pi05.modeling_pi05 import PI05Policy

from ..policies.utils import (
    make_easytrainer_processors,
    prepare_pi05_language_tokens,
    process_image,
)


CKPT_ROOT = Path("/root/src/backend/checkpoints")
DATASET_ROOT = Path("/opt/easytrainer/datasets")


def _read_video_frame(mp4_path: Path, frame_idx: int) -> Image.Image:
    import cv2
    cap = cv2.VideoCapture(str(mp4_path))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, bgr = cap.read()
    cap.release()
    if not ok:
        raise IndexError(f"could not read frame {frame_idx}")
    return Image.fromarray(bgr[..., ::-1])


def load_episode(dataset_id: int, episode_idx: int):
    ds_dir = DATASET_ROOT / str(dataset_id)
    parquet_path = ds_dir / "data" / "chunk-000" / f"episode_{episode_idx:06d}.parquet"
    df = pq.read_table(parquet_path).to_pandas()
    states = np.stack([np.asarray(s, dtype=np.float32) for s in df["observation.state"]])
    actions = np.stack([np.asarray(a, dtype=np.float32) for a in df["action"]])
    if "succeed" in df.columns:
        succeed = np.asarray(df["succeed"].values, dtype=np.float32)[:, None]
        actions = np.concatenate([actions, succeed], axis=-1)
    videos_root = ds_dir / "videos" / "chunk-000"
    sensor_dirs = sorted(p.name for p in videos_root.iterdir() if p.is_dir())
    return states, actions, sensor_dirs, videos_root


def find_transition_frames(states, actions, n_frames=3, chunk_size=20, min_frame=5, exclude_last=25):
    """state delta가 큰 frame을 transition frame으로 식별.

    chunk_size 만큼 미래에 있는 state 차이를 보고, 가장 movement가 큰 frame들을 선택.
    """
    L = len(states)
    if L < chunk_size + min_frame + exclude_last:
        return []
    # "이 frame에서 시작해서 chunk_size 동안 얼마나 움직이는가"
    movements = []
    for t in range(min_frame, L - chunk_size - exclude_last):
        # joint-space distance over chunk
        traj = states[t : t + chunk_size]
        d = np.abs(traj - states[t]).max(axis=0).sum()  # peak displacement summed
        movements.append((d, t))
    movements.sort(reverse=True)
    # Top N transition frames
    selected = [t for _, t in movements[:n_frames]]
    return sorted(selected)


def build_input(state_np, sensor_dirs, videos_root, episode_idx, frame_idx, task_text, cfg):
    qpos = torch.from_numpy(state_np).float().cuda().unsqueeze(0)
    inp = {"observation.state": qpos, "language_instruction": task_text}
    prepare_pi05_language_tokens(inp, cfg)
    for sensor_dir in sensor_dirs:
        mp4 = videos_root / sensor_dir / f"episode_{episode_idx:06d}.mp4"
        pil = _read_video_frame(mp4, frame_idx)
        t = process_image(pil, "resnet18", to_cuda=True, pixel_range="-11").unsqueeze(0)
        key = sensor_dir if sensor_dir.startswith("observation.images.") else f"observation.images.{sensor_dir}"
        inp[key] = t
    if "task" not in inp:
        inp["task"] = [task_text]
    return inp


@torch.no_grad()
def predict_chunk(policy, preprocessor, postprocessor, batch):
    policy.reset()
    if preprocessor is not None:
        batch = preprocessor(batch)
    first = policy.select_action(batch).squeeze(0)
    chunks = [first]
    q = policy._action_queue if hasattr(policy, "_action_queue") else policy._queues.get("action")
    while q is not None and len(q) > 0:
        chunks.append(q.popleft().squeeze(0))
    raw = torch.stack(chunks)
    if postprocessor is not None:
        raw = postprocessor(raw)
    return raw.cpu().numpy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", type=int, required=True)
    ap.add_argument("--dataset", type=int, required=True)
    ap.add_argument("--episodes", type=int, nargs="+", default=list(range(0, 58, 5)))
    ap.add_argument("--frames-per-ep", type=int, default=3, help="Top-N transition frames per episode")
    ap.add_argument("--task", type=str, default="Pick the red cube and place it on the plate")
    args = ap.parse_args()

    ckpt_dir = CKPT_ROOT / str(args.ckpt)
    print(f"\n{'='*80}\nPI05 TRANSITION DIAGNOSTIC  ckpt={args.ckpt}  ds={args.dataset}\n{'='*80}\n")

    policy = PI05Policy.from_pretrained(str(ckpt_dir))
    policy.cuda().eval()
    chunk_size = int(policy.config.chunk_size)

    import pickle
    with open(ckpt_dir / "dataset_stats.pkl", "rb") as f:
        stats = pickle.load(f)
    a_q01 = stats["action"]["q01"]
    a_q99 = stats["action"]["q99"]
    a_range = np.maximum(a_q99 - a_q01, 1e-2)
    a_mean = stats["action"]["mean"]

    all_norm_errs = []
    all_per_dim_errs = []
    cube_endpoint_gt = []   # GT chunk endpoint (cube reach position)
    cube_endpoint_pred = [] # Model predicted chunk endpoint
    per_episode_summary = []

    for ep in args.episodes:
        try:
            states, actions, sensor_dirs, videos_root = load_episode(args.dataset, ep)
        except FileNotFoundError:
            continue

        tx_frames = find_transition_frames(states, actions, n_frames=args.frames_per_ep, chunk_size=chunk_size)
        if not tx_frames:
            continue

        ep_errs = []
        for fr in tx_frames:
            pre, post = make_easytrainer_processors(
                policy_type="PI05", cfg=policy.config, pretrained_path=str(ckpt_dir)
            )
            gt_state = states[fr]
            gt_chunk = actions[fr : fr + chunk_size]
            inp = build_input(gt_state, sensor_dirs, videos_root, ep, fr, args.task, policy.config)
            pred_chunk = predict_chunk(policy, pre, post, inp)

            T = min(pred_chunk.shape[0], gt_chunk.shape[0])
            err = pred_chunk[:T] - gt_chunk[:T]
            abs_err = np.abs(err)
            norm_err = abs_err / a_range[None, :]
            mean_baseline_err = np.abs(a_mean[None, :] - gt_chunk[:T])

            ep_errs.append(norm_err.mean())
            all_norm_errs.append(norm_err.mean())
            all_per_dim_errs.append(norm_err.mean(axis=0))

            # Endpoint comparison (chunk 마지막 step = 해당 transition의 도달점)
            cube_endpoint_gt.append(gt_chunk[-1, :7])  # joint state portion
            cube_endpoint_pred.append(pred_chunk[-1, :7])

            # frame별 detail
            err_t0 = abs_err[0].mean()
            err_t10 = abs_err[10].mean() if T > 10 else 0
            err_t19 = abs_err[-1].mean()
            print(f"  ep{ep:2d} fr{fr:3d}: norm_err={norm_err.mean():.3f} "
                  f"(t=0:{err_t0:.3f}, t=10:{err_t10:.3f}, t=19:{err_t19:.3f}), "
                  f"baseline={mean_baseline_err.mean():.3f}, "
                  f"{'BETTER' if abs_err.mean() < mean_baseline_err.mean() else 'WORSE'}")

        if ep_errs:
            per_episode_summary.append((ep, np.mean(ep_errs)))

    print("\n" + "="*80)
    print(f"SUMMARY — Transition frames only (n={len(all_norm_errs)})")
    print("="*80)

    all_norm_errs = np.array(all_norm_errs)
    print(f"\n  Overall norm_err on transitions: mean={all_norm_errs.mean():.3f}, "
          f"median={np.median(all_norm_errs):.3f}, max={all_norm_errs.max():.3f}")

    if all_per_dim_errs:
        per_dim = np.stack(all_per_dim_errs).mean(axis=0)
        print(f"\n  Per-dim norm_err on transitions:")
        for i, e in enumerate(per_dim):
            label = f"joint {i}" if i < 6 else ("gripper" if i==6 else "succeed/done")
            warn = " 🔥" if e > 0.3 else (" ⚠️" if e > 0.15 else "")
            print(f"    {label}: {e:.3f}{warn}")

    # Endpoint clustering: mode collapse 검출
    if cube_endpoint_gt:
        gt_eps = np.stack(cube_endpoint_gt)
        pred_eps = np.stack(cube_endpoint_pred)
        gt_std = gt_eps.std(axis=0)
        pred_std = pred_eps.std(axis=0)
        print(f"\n  Endpoint diversity (GT vs PRED std per joint):")
        for j in range(6):
            ratio = pred_std[j] / max(gt_std[j], 1e-6)
            warn = " 🔥 mode collapse 의심" if ratio < 0.4 else (" ⚠️" if ratio < 0.7 else "")
            print(f"    joint {j}: GT_std={gt_std[j]:.3f}, PRED_std={pred_std[j]:.3f}, ratio={ratio:.2f}{warn}")
        # Endpoint-to-endpoint distance
        eps_dists = np.abs(gt_eps - pred_eps).mean(axis=-1)
        print(f"\n  GT vs PRED endpoint mean distance: {eps_dists.mean():.4f} (per-joint avg)")
        print(f"  Worst endpoint: ep at idx {eps_dists.argmax()} → diff={eps_dists.max():.4f}")

    print("\n" + "="*80)
    print("INTERPRETATION (transition frames만 보고)")
    print("="*80)
    if all_norm_errs.mean() < 0.10:
        print("  ✅ transition도 잘 학습됨. 다른 원인 (inference pipeline, etc.)")
    elif all_norm_errs.mean() < 0.20:
        print("  ⚠️ transition 어느 정도 학습됨. capacity나 학습 시간 부족 가능.")
    elif all_norm_errs.mean() < 0.40:
        print("  🔥 transition 학습 부실. mode collapse 또는 capacity 부족.")
    else:
        print("  🔥🔥 transition 거의 학습 안 됨. 근본적 문제 (data, capacity, 또는 pipeline).")
    if cube_endpoint_gt:
        avg_ratio = np.mean([pred_std[j]/max(gt_std[j],1e-6) for j in range(6)])
        if avg_ratio < 0.4:
            print(f"  🔥 Endpoint diversity ratio={avg_ratio:.2f} — Mode collapse 강함. cube 위치별 다른 reaching 못 함.")
        elif avg_ratio < 0.7:
            print(f"  ⚠️ Endpoint diversity ratio={avg_ratio:.2f} — 부분적 mode collapse.")
        else:
            print(f"  ✅ Endpoint diversity ratio={avg_ratio:.2f} — 모델이 cube 위치별 적절히 다른 trajectory 생성.")


if __name__ == "__main__":
    main()
