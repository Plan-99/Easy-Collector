"""PI05 학습 재현 진단.

목적: 학습 데이터의 frame을 그대로 모델에 입력해서 GT action chunk를 얼마나 정확히
재현하는지 측정. 모델이 자기가 학습한 데이터조차 재현 못 하면 학습/파이프라인 자체가 문제.

판단 기준:
  - normalized_err mean < 0.3 → 모델이 학습된 상태 (training fit OK)
  - normalized_err mean ~0.5~1.0 → underfit 또는 capacity 부족
  - normalized_err mean > 1.5 → 학습 실패 또는 inference 파이프라인 버그
  - GT action 평균과 model 출력이 비슷하면 → mode collapse (mean trajectory만 외움)

사용:
  docker exec easy_collector_service bash -lc "cd /root && python3 -m src.backend.scripts.diagnose_pi05_replay \\
      --ckpt 96 --dataset 22 --episode 0 --frames 30 60 100 --task 'Pick the red cube and place it on the plate'"
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
        raise IndexError(f"could not read frame {frame_idx} from {mp4_path}")
    return Image.fromarray(bgr[..., ::-1])  # BGR→RGB


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


def build_input(state_np, sensor_dirs, videos_root, episode_idx, frame_idx, task_text, cfg):
    qpos = torch.from_numpy(state_np).float().cuda().unsqueeze(0)
    inp = {"observation.state": qpos, "language_instruction": task_text}
    prepare_pi05_language_tokens(inp, cfg)
    for sensor_dir in sensor_dirs:
        mp4 = videos_root / sensor_dir / f"episode_{episode_idx:06d}.mp4"
        pil = _read_video_frame(mp4, frame_idx)
        t = process_image(pil, "resnet18", to_cuda=True, pixel_range="-11").unsqueeze(0)
        inp[sensor_dir if sensor_dir.startswith("observation.images.") else f"observation.images.{sensor_dir}"] = t
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
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--frames", type=int, nargs="+", default=[30, 60, 100, 150])
    ap.add_argument("--task", type=str, default="Pick the red cube and place it on the plate")
    args = ap.parse_args()

    ckpt_dir = CKPT_ROOT / str(args.ckpt)
    print(f"\n{'='*80}\nPI05 REPLAY DIAGNOSTIC  ckpt={args.ckpt}  ds={args.dataset}  ep={args.episode}\n{'='*80}\n")

    policy = PI05Policy.from_pretrained(str(ckpt_dir))
    policy.cuda().eval()
    chunk_size = int(policy.config.chunk_size)

    states, actions, sensor_dirs, videos_root = load_episode(args.dataset, args.episode)
    print(f"Episode length: {len(states)}, action_dim={actions.shape[-1]}, chunk_size={chunk_size}")
    print(f"Sensors: {sensor_dirs}\n")

    # Stats for per-dim normalization of error
    import pickle
    with open(ckpt_dir / "dataset_stats.pkl", "rb") as f:
        stats = pickle.load(f)
    a_q01 = stats["action"]["q01"]
    a_q99 = stats["action"]["q99"]
    a_range = np.maximum(a_q99 - a_q01, 1e-2)
    a_mean = stats["action"]["mean"]

    for frame_idx in args.frames:
        if frame_idx + chunk_size > len(states):
            print(f"\nframe {frame_idx}: 너무 늦은 frame — skip")
            continue
        # Re-create processors per call so RelativeActionsProcessorStep state cache is clean
        pre, post = make_easytrainer_processors(
            policy_type="PI05", cfg=policy.config, pretrained_path=str(ckpt_dir)
        )
        gt_state = states[frame_idx]
        gt_chunk = actions[frame_idx : frame_idx + chunk_size]  # (T, action_dim)

        inp = build_input(
            gt_state, sensor_dirs, videos_root, args.episode, frame_idx, args.task, policy.config
        )
        pred_chunk = predict_chunk(policy, pre, post, inp)

        T = min(pred_chunk.shape[0], gt_chunk.shape[0])
        err = pred_chunk[:T] - gt_chunk[:T]
        abs_err = np.abs(err)
        norm_err = abs_err / a_range[None, :]
        # Mean baseline: 항상 action 평균 출력하면 얼마나 틀리는지
        mean_baseline_err = np.abs(a_mean[None, :] - gt_chunk[:T])
        improvement = (mean_baseline_err.mean() - abs_err.mean()) / max(mean_baseline_err.mean(), 1e-6)

        print(f"\n{'─'*80}")
        print(f"Frame {frame_idx}  (state[:5]={np.round(gt_state[:5],3)})")
        print(f"{'─'*80}")
        print(f"  GT  action[t=0]  (frame {frame_idx}): {np.round(gt_chunk[0], 3)}")
        print(f"  PRED action[t=0]                 : {np.round(pred_chunk[0], 3)}")
        print(f"  GT  action[t=10] (frame {frame_idx+10}): {np.round(gt_chunk[10], 3) if len(gt_chunk)>10 else '-'}")
        print(f"  PRED action[t=10]                : {np.round(pred_chunk[10], 3) if len(pred_chunk)>10 else '-'}")
        print(f"  GT  action[t=19] (frame {frame_idx+19}): {np.round(gt_chunk[-1], 3)}")
        print(f"  PRED action[t=19]                : {np.round(pred_chunk[-1], 3)}")
        print(f"")
        print(f"  abs_err per-dim mean : {np.round(abs_err.mean(axis=0), 4)}")
        print(f"  abs_err per-dim max  : {np.round(abs_err.max(axis=0), 4)}")
        print(f"  norm_err per-dim mean: {np.round(norm_err.mean(axis=0), 3)}")
        print(f"  norm_err overall mean: {norm_err.mean():.3f}  (각 차원 q99-q01 범위 대비 에러 비율)")
        print(f"  vs predict-mean baseline: model_err={abs_err.mean():.4f}, baseline={mean_baseline_err.mean():.4f}, "
              f"{'BETTER' if improvement>0 else 'WORSE'} by {abs(improvement)*100:.1f}%")

        # Direction analysis: does the model push state in the right direction?
        gt_delta = gt_chunk[0] - gt_state[: gt_chunk.shape[-1]] if gt_chunk.shape[-1] <= gt_state.shape[0] else None
        pred_delta = pred_chunk[0] - gt_state[: pred_chunk.shape[-1]] if pred_chunk.shape[-1] <= gt_state.shape[0] else None
        if gt_delta is not None and pred_delta is not None:
            cos = np.dot(gt_delta, pred_delta) / (np.linalg.norm(gt_delta) * np.linalg.norm(pred_delta) + 1e-9)
            print(f"  GT vs PRED step-0 delta cosine: {cos:.3f}  (1.0=같은 방향, 0=직교, -1=반대방향)")

    print("\n" + "="*80)
    print("INTERPRETATION")
    print("="*80)
    print("""
  norm_err overall mean 기준:
    < 0.10  → 학습이 매우 잘 됨 (10% 이내 오차)
    0.10 ~ 0.30 → 정상 학습 (training fit OK)
    0.30 ~ 0.60 → 학습 일부 됐지만 부족 / capacity 부족
    > 0.60  → 학습 실패 또는 inference 파이프라인 버그

  vs predict-mean baseline:
    BETTER → 모델이 입력에 따라 다르게 행동 (정상)
    WORSE  → 모델이 평균 출력보다도 못함 = 심각한 학습 실패 또는 stats 오염

  cosine similarity (step-0 delta):
    > 0.7 → 같은 방향으로 움직임 (잘 학습됨)
    0.3 ~ 0.7 → 대충 비슷한 방향
    < 0.3 → 다른 방향 (mode collapse 또는 학습 부족)
    < 0   → 반대 방향 (심각)
""")


if __name__ == "__main__":
    main()
