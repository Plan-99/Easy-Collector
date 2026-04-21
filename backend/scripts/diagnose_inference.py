"""ACT 학습/추론 분포 일치 진단 스크립트.

목적: 학습된 체크포인트가 training data 자체에 대해서도 평균을 출력하는지,
아니면 training data에 대해서는 정답을 내고 inference 입력만 OOD인지 가른다.

사용:
  docker exec easy_collector_service python3 -m src.backend.scripts.diagnose_inference \
      --ckpt 31 --dataset 2 --episode 0 --frames 0 30 60 120

출력:
  각 frame마다
    - ground-truth state / action
    - 모델 입력 state (raw + normalized)
    - 모델 입력 image stats
    - 모델 출력 (raw + unnormalized)
    - per-joint absolute error vs ground truth
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import sys
from pathlib import Path

# lerobot 패키지 path 추가 (start_services.sh와 동일)
_lerobot_src = Path(__file__).resolve().parent.parent / "lerobot" / "src"
if str(_lerobot_src) not in sys.path:
    sys.path.insert(0, str(_lerobot_src))

import numpy as np
import pyarrow.parquet as pq
import torch
from PIL import Image

from lerobot.policies.act.modeling_act import ACTPolicy

# EasyTrainer 헬퍼 (preprocessor + image preprocessing)
from ..policies.utils import make_easytrainer_processors, process_image


CKPT_ROOT = Path("/root/backend/checkpoints")
DATASET_ROOT = Path("/opt/easytrainer/datasets")


def load_episode_frame(dataset_id: int, episode_idx: int, frame_idx: int):
    """파quet에서 state/action을 읽고, 디스크에서 sensor_*/episode/frame_XXXXXX.png를 불러온다."""
    ds_dir = DATASET_ROOT / str(dataset_id)
    parquet_path = ds_dir / "data" / "chunk-000" / f"episode_{episode_idx:06d}.parquet"
    if not parquet_path.exists():
        raise FileNotFoundError(parquet_path)
    df = pq.read_table(parquet_path).to_pandas()
    if frame_idx >= len(df):
        raise IndexError(f"frame {frame_idx} out of range (len={len(df)})")

    state = np.asarray(df["observation.state"].iloc[frame_idx], dtype=np.float32)
    action = np.asarray(df["action"].iloc[frame_idx], dtype=np.float32)
    if "succeed" in df.columns:
        succeed = float(df["succeed"].iloc[frame_idx])
        action = np.concatenate([action, np.array([succeed], dtype=np.float32)])

    # 이미지: images/observation.images.sensor_*/episode_XXXXXX/frame_XXXXXX.png
    images_root = ds_dir / "images"
    sensor_dirs = sorted(p.name for p in images_root.iterdir() if p.is_dir())
    images = {}
    for sensor_dir in sensor_dirs:
        # "observation.images.sensor_1" → "observation.images.sensor_1"
        frame_path = images_root / sensor_dir / f"episode_{episode_idx:06d}" / f"frame_{frame_idx:06d}.png"
        if not frame_path.exists():
            print(f"  [WARN] missing image: {frame_path}")
            continue
        img = Image.open(frame_path).convert("RGB")
        images[sensor_dir] = img  # PIL Image

    return state, action, images


def build_policy_input(state: np.ndarray, images: dict, vision_backbone: str = "resnet18"):
    """checkpoint_test.py와 동일한 방식으로 policy_input_t 구성."""
    qpos_t = torch.from_numpy(state).float().cuda().unsqueeze(0)  # (1, state_dim)
    policy_input = {"observation.state": qpos_t}
    img_stats = {}
    for sensor_key, pil_img in images.items():
        img_t = process_image(pil_img, vision_backbone, to_cuda=True)  # (3, H, W)
        img_t = img_t.unsqueeze(0)  # (1, 3, H, W)
        # 키 변환: "observation.images.sensor_1" 그대로 (디렉토리 이름이 이미 그 형태)
        policy_input[sensor_key] = img_t
        img_stats[sensor_key] = (
            float(img_t.min()),
            float(img_t.max()),
            float(img_t.mean()),
            float(img_t.std()),
        )
    return policy_input, img_stats


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", type=int, required=True, help="checkpoint id under /root/backend/checkpoints")
    parser.add_argument("--dataset", type=int, required=True, help="dataset id under /opt/easytrainer/datasets")
    parser.add_argument("--episode", type=int, default=0, help="episode index")
    parser.add_argument(
        "--frames",
        type=int,
        nargs="+",
        default=[0, 30, 60, 120, 200],
        help="frame indices to test",
    )
    parser.add_argument(
        "--vision-backbone",
        default="resnet18",
        help="must match the trained policy",
    )
    args = parser.parse_args()

    ckpt_dir = CKPT_ROOT / str(args.ckpt)
    if not ckpt_dir.exists():
        raise FileNotFoundError(ckpt_dir)

    print("=" * 80)
    print(f"DIAGNOSING checkpoint={args.ckpt} on dataset={args.dataset} episode={args.episode}")
    print("=" * 80)

    # ── 1. policy & preprocessor 로드 ──────────────────────────────────────
    print(f"\n[1] Loading policy from {ckpt_dir} ...")
    policy = ACTPolicy.from_pretrained(str(ckpt_dir))
    policy.cuda()
    policy.eval()
    print(f"    use_vae={policy.config.use_vae}, "
          f"chunk_size={policy.config.chunk_size}, "
          f"n_decoder_layers={policy.config.n_decoder_layers}")

    pre, post = make_easytrainer_processors(
        policy_type="ACT",
        cfg=policy.config,
        pretrained_path=str(ckpt_dir),
    )
    if pre is None:
        print("    [WARN] no preprocessor found in ckpt — using raw I/O")
    else:
        print("    preprocessor loaded ✓")

    # ── 2. training stats 로드 ────────────────────────────────────────────
    stats_path = ckpt_dir / "dataset_stats.pkl"
    with open(stats_path, "rb") as f:
        stats = pickle.load(f)
    state_mean = stats["observation.state"]["mean"]
    state_std = stats["observation.state"]["std"]
    action_mean = stats["action"]["mean"]
    action_std = stats["action"]["std"]
    print(f"\n[2] Training stats:")
    print(f"    state.mean = {np.round(state_mean, 3)}")
    print(f"    state.std  = {np.round(state_std, 3)}")
    print(f"    action.mean= {np.round(action_mean, 3)}")
    print(f"    action.std = {np.round(action_std, 3)}")

    # ── 3. 각 frame 추론 ─────────────────────────────────────────────────
    for frame_idx in args.frames:
        print("\n" + "─" * 80)
        print(f"FRAME {frame_idx}")
        print("─" * 80)

        try:
            gt_state, gt_action, images = load_episode_frame(
                args.dataset, args.episode, frame_idx
            )
        except (FileNotFoundError, IndexError) as e:
            print(f"  [SKIP] {e}")
            continue

        # OOD score: state가 training 분포에서 얼마나 벗어났는가 (z-score)
        state_z = (gt_state - state_mean[: len(gt_state)]) / (state_std[: len(gt_state)] + 1e-8)
        print(f"  GT state          : {np.round(gt_state, 4)}")
        print(f"  state z-score     : {np.round(state_z, 2)}  "
              f"(|max|={np.abs(state_z).max():.2f})")
        print(f"  GT action         : {np.round(gt_action, 4)}")

        # 입력 만들기
        policy_input, img_stats = build_policy_input(gt_state, images, args.vision_backbone)
        for k, (mn, mx, m, s) in img_stats.items():
            print(f"  img {k}: min={mn:.3f} max={mx:.3f} mean={m:.3f} std={s:.3f}")

        # preprocessor 적용
        if pre is not None:
            policy_input = pre(policy_input)
            norm_state = policy_input["observation.state"][0].cpu().numpy()
            print(f"  normalized state  : {np.round(norm_state, 3)}")

        # 모델 forward (predict_action_chunk: chunk 전체 출력)
        # n_action_steps=1 + temporal_ensemble_coeff 설정에 영향 없이 정확히 첫 chunk를 보고 싶음
        with torch.no_grad():
            policy.reset()
            # predict_action_chunk: (1, chunk_size, action_dim)
            raw_chunk = policy.predict_action_chunk(policy_input).squeeze(0).cpu()
            # 첫 step만 비교
            raw_first = raw_chunk[0]
            print(f"  RAW model out [0] (normalized space): {np.round(raw_first.numpy(), 3)}")

            # postprocessor (unnormalize)
            if post is not None:
                # postprocessor는 tensor → tensor (action_to_transition 사용)
                first_t = raw_first.cuda()
                unnorm = post(first_t).cpu().numpy()
            else:
                unnorm = raw_first.numpy()
        print(f"  UNNORMALIZED out  : {np.round(unnorm, 4)}")
        print(f"  GT action         : {np.round(gt_action, 4)}")
        err = unnorm - gt_action[: len(unnorm)]
        per_joint_abs = np.abs(err)
        print(f"  per-joint abs err : {np.round(per_joint_abs, 4)}")
        # 각 joint를 자기 std로 나눠서 normalized 단위 에러
        norm_err = per_joint_abs / (action_std[: len(unnorm)] + 1e-8)
        print(f"  per-joint norm err: {np.round(norm_err, 2)}  "
              f"(mean={norm_err.mean():.2f})")

        # 모델이 그냥 평균 출력 중인지 체크
        baseline_err = np.abs(action_mean[: len(unnorm)] - gt_action[: len(unnorm)])
        improvement = baseline_err.mean() - per_joint_abs.mean()
        print(f"  vs predict-mean baseline: model err {per_joint_abs.mean():.4f} "
              f"vs baseline {baseline_err.mean():.4f} "
              f"→ {'BETTER' if improvement > 0 else 'WORSE'} by {abs(improvement):.4f}")

    print("\n" + "=" * 80)
    print("INTERPRETATION GUIDE")
    print("=" * 80)
    print("""
- per-joint norm err < 0.3 (mean) on training frames → 모델이 학습된 상태
- per-joint norm err > 1.0 (mean) → 모델 학습 안 됨 또는 입력 OOD
- 'vs predict-mean baseline' → BETTER 가 나와야 함. WORSE면 모델이
  평균보다도 못한 출력 = 학습 실패 또는 inference 파이프라인 버그
- state z-score |max| > 3 → 이 frame이 training 분포 밖

만약 training frame에서도 모델이 실패하면 → 학습 자체 문제
training frame은 OK인데 라이브에서만 실패하면 → 라이브 입력 파이프라인 (이미지/state 처리) 문제
""")


if __name__ == "__main__":
    main()
