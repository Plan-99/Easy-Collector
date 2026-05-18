"""PI05 vision grounding 진단 스크립트.

목적: 학습된 PI05 정책이 실제로 image conditioning을 쓰는지, 아니면 state만으로
판단하고 vision은 거의 무시하고 있는지 정량 측정한다.

방법: 같은 state를 고정하고 이미지만 바꿔서 predicted action chunk가 얼마나
달라지는지 본다.
  - chunk_A : (state_fixed, image_A)        ← 실제 큐브 위치 A
  - chunk_B : (state_fixed, image_B)        ← 실제 큐브 위치 B (다른 frame/episode)
  - chunk_zero : (state_fixed, image_zeros) ← 검은 이미지
  - chunk_state2 : (state_other, image_A)   ← state만 다른 이미지 고정

해석:
  diff(A, B) ≪ diff(state1, state2)  → vision 거의 무시 (state-dominant)
  diff(A, B) ≈ diff(state1, state2)  → vision/state 둘 다 사용
  diff(A, zero) 작음                 → 이미지 channel 자체가 model에 안 들어감
  diff(A, B) 큼                      → vision은 잘 conditioning 중

사용:
  docker exec easy_collector_service python3 -m src.backend.scripts.diagnose_vision_grounding \\
      --ckpt 33 --dataset 22 \\
      --frame-a "0:30" --frame-b "5:30" \\
      --task "pick up the red cube and place it on the white plate"

  --frame-a/--frame-b 형식: "EPISODE:FRAME" (둘이 같은 episode여도 됨; cube 위치만 달라야 의미 있음)
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


def parse_frame_spec(s: str) -> tuple[int, int]:
    ep, fr = s.split(":")
    return int(ep), int(fr)


def _read_video_frame(mp4_path: Path, frame_idx: int) -> Image.Image:
    import cv2
    cap = cv2.VideoCapture(str(mp4_path))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, bgr = cap.read()
    cap.release()
    if not ok:
        raise IndexError(f"could not read frame {frame_idx} from {mp4_path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return Image.fromarray(rgb)


def load_episode_frame(dataset_id: int, episode_idx: int, frame_idx: int):
    ds_dir = DATASET_ROOT / str(dataset_id)
    parquet_path = ds_dir / "data" / "chunk-000" / f"episode_{episode_idx:06d}.parquet"
    df = pq.read_table(parquet_path).to_pandas()
    if frame_idx >= len(df):
        raise IndexError(f"frame {frame_idx} out of range (len={len(df)}) for episode {episode_idx}")

    state = np.asarray(df["observation.state"].iloc[frame_idx], dtype=np.float32)
    action = np.asarray(df["action"].iloc[frame_idx], dtype=np.float32)

    # Two storage layouts: PNG (older) or mp4 (current).
    images_root = ds_dir / "images"
    videos_root = ds_dir / "videos" / "chunk-000"
    sensor_dirs = []
    if images_root.exists():
        sensor_dirs = sorted(p.name for p in images_root.iterdir() if p.is_dir())
    if not sensor_dirs and videos_root.exists():
        sensor_dirs = sorted(p.name for p in videos_root.iterdir() if p.is_dir())

    images = {}
    for sensor_dir in sensor_dirs:
        png_path = images_root / sensor_dir / f"episode_{episode_idx:06d}" / f"frame_{frame_idx:06d}.png"
        mp4_path = videos_root / sensor_dir / f"episode_{episode_idx:06d}.mp4"
        if png_path.exists():
            images[sensor_dir] = Image.open(png_path).convert("RGB")
        elif mp4_path.exists():
            images[sensor_dir] = _read_video_frame(mp4_path, frame_idx)
        else:
            print(f"  [WARN] missing image (no PNG, no MP4): sensor={sensor_dir} ep={episode_idx} fr={frame_idx}")
    return state, action, images


def build_input(state_np, images_pil, task_text, cfg, vision_backbone, replace_image_with=None):
    """Build a batched, on-CUDA policy input.

    replace_image_with: None | 'zeros' | 'random' | dict[sensor_key→PIL.Image]
    """
    qpos = torch.from_numpy(state_np).float().cuda().unsqueeze(0)
    inp = {"observation.state": qpos, "language_instruction": task_text}
    prepare_pi05_language_tokens(inp, cfg)

    for sensor_key, pil in images_pil.items():
        if isinstance(replace_image_with, dict) and sensor_key in replace_image_with:
            pil = replace_image_with[sensor_key]
            t = process_image(pil, vision_backbone, to_cuda=True, pixel_range="-11").unsqueeze(0)
        elif replace_image_with == "zeros":
            t = process_image(pil, vision_backbone, to_cuda=True, pixel_range="-11").unsqueeze(0)
            t = torch.zeros_like(t)
        elif replace_image_with == "random":
            t = process_image(pil, vision_backbone, to_cuda=True, pixel_range="-11").unsqueeze(0)
            t = torch.empty_like(t).uniform_(-1.0, 1.0)
        else:
            t = process_image(pil, vision_backbone, to_cuda=True, pixel_range="-11").unsqueeze(0)
        # sensor_dir 이름이 이미 "observation.images.sensor_X" 형식이라 그대로 사용.
        key = sensor_key if sensor_key.startswith("observation.images.") else f"observation.images.{sensor_key}"
        inp[key] = t

    # Bridge for PI05 preprocessor (mirrors checkpoint_test.py).
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
    return raw.cpu().numpy()  # (chunk_size, action_dim)


def diff_stats(a, b, label):
    d = np.abs(a - b)
    print(f"  {label:<32} mean={d.mean():.4f}  max={d.max():.4f}  per-dim-mean={np.round(d.mean(axis=0), 4)}")
    return d.mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", type=int, required=True)
    ap.add_argument("--dataset", type=int, required=True)
    ap.add_argument("--frame-a", type=str, required=True, help="EP:FRAME (cube position A)")
    ap.add_argument("--frame-b", type=str, required=True, help="EP:FRAME (cube position B, different from A)")
    ap.add_argument("--task", type=str, default="pick up the red cube and place it on the white plate")
    ap.add_argument("--vision-backbone", default="resnet18", help="ignored for PI05 (uses [-1,1] resize)")
    args = ap.parse_args()

    ep_a, fr_a = parse_frame_spec(args.frame_a)
    ep_b, fr_b = parse_frame_spec(args.frame_b)

    ckpt_dir = CKPT_ROOT / str(args.ckpt)
    if not ckpt_dir.exists():
        raise FileNotFoundError(ckpt_dir)

    print("=" * 80)
    print(f"PI05 VISION GROUNDING DIAGNOSTIC")
    print(f"  ckpt={args.ckpt}  dataset={args.dataset}")
    print(f"  A = ep{ep_a} fr{fr_a}    B = ep{ep_b} fr{fr_b}")
    print(f"  task = {args.task!r}")
    print("=" * 80)

    print("\n[1] Loading PI05 policy + processors ...")
    policy = PI05Policy.from_pretrained(str(ckpt_dir))
    policy.cuda().eval()
    pre, post = make_easytrainer_processors(
        policy_type="PI05", cfg=policy.config, pretrained_path=str(ckpt_dir)
    )
    print(f"    chunk_size={policy.config.chunk_size}")
    if pre is None:
        print("    [WARN] preprocessor=None — falling back to raw I/O (suspicious!)")

    print(f"\n[2] Loading frame A (ep{ep_a} fr{fr_a}) and frame B (ep{ep_b} fr{fr_b}) ...")
    state_a, gt_a, imgs_a = load_episode_frame(args.dataset, ep_a, fr_a)
    state_b, gt_b, imgs_b = load_episode_frame(args.dataset, ep_b, fr_b)
    print(f"    state_a = {np.round(state_a, 3)}")
    print(f"    state_b = {np.round(state_b, 3)}")
    print(f"    |state_b - state_a| mean = {np.abs(state_b - state_a).mean():.4f}")
    if np.allclose(state_a, state_b, atol=1e-3):
        print("    (states near-identical — clean isolation of vision effect)")
    else:
        print("    (states differ — interpret with care; we'll still control by holding state_a fixed)")

    vb = args.vision_backbone

    print("\n[3] Predicting action chunks under controlled inputs ...")
    print("    All inputs use state=state_a (fixed). Only image / state varies.\n")

    # Reload processors per call because RelativeActionsProcessorStep caches state across calls.
    def make_pp():
        return make_easytrainer_processors(
            policy_type="PI05", cfg=policy.config, pretrained_path=str(ckpt_dir)
        )

    # 1) state_a + image_a
    pre1, post1 = make_pp()
    inp = build_input(state_a, imgs_a, args.task, policy.config, vb)
    chunk_AA = predict_chunk(policy, pre1, post1, inp)

    # 2) state_a + image_b   ← key comparison
    pre2, post2 = make_pp()
    inp = build_input(state_a, imgs_b, args.task, policy.config, vb)
    chunk_AB = predict_chunk(policy, pre2, post2, inp)

    # 3) state_a + zeros image
    pre3, post3 = make_pp()
    inp = build_input(state_a, imgs_a, args.task, policy.config, vb, replace_image_with="zeros")
    chunk_AZ = predict_chunk(policy, pre3, post3, inp)

    # 4) state_a + random image
    pre4, post4 = make_pp()
    inp = build_input(state_a, imgs_a, args.task, policy.config, vb, replace_image_with="random")
    chunk_AR = predict_chunk(policy, pre4, post4, inp)

    # 5) state_b + image_a   (vision held, state varies — gives a state-effect baseline)
    pre5, post5 = make_pp()
    inp = build_input(state_b, imgs_a, args.task, policy.config, vb)
    chunk_BA = predict_chunk(policy, pre5, post5, inp)

    print("[4] Pairwise chunk diffs (postprocessed action space, full chunk):")
    d_vision_real = diff_stats(chunk_AA, chunk_AB, "VISION effect (img_a vs img_b)")
    d_vision_zero = diff_stats(chunk_AA, chunk_AZ, "VISION effect (img_a vs zeros)")
    d_vision_rand = diff_stats(chunk_AA, chunk_AR, "VISION effect (img_a vs random)")
    d_state = diff_stats(chunk_AA, chunk_BA, "STATE  effect (state_a vs state_b)")

    print("\n[5] First action (immediate command) diff only:")
    diff_stats(chunk_AA[:1], chunk_AB[:1], "  step-0  vision A vs B")
    diff_stats(chunk_AA[:1], chunk_BA[:1], "  step-0  state  a vs b")

    print("\n" + "=" * 80)
    print("INTERPRETATION")
    print("=" * 80)
    ratio = d_vision_real / max(d_state, 1e-9)
    print(f"  vision/state ratio = {ratio:.3f}")
    if d_vision_real < 1e-3:
        print("  🔥 VISION 거의 무시. 큐브 위치 바꿔도 action 거의 동일.")
        print("     → multi_modal_projector를 LoRA target에 추가 + 데이터 다양성 점검 필요.")
    elif ratio < 0.2:
        print("  ⚠️  vision은 들어가지만 state가 dominate. spatial generalization 어려움.")
        print("     → LoRA capacity (vision pathway) 확장 + 다양한 cube 위치 데이터 추가.")
    elif ratio < 0.8:
        print("  ✅ vision/state 둘 다 conditioning에 기여. 적정 비율.")
    else:
        print("  ⚠️  vision이 너무 dominate. state가 거의 무시될 수 있음 (이번 케이스에선 드뭄).")

    if d_vision_zero < d_vision_real * 0.5:
        print("  ❗ zeros 이미지가 real B 이미지보다 영향 작음 — image channel routing 의심.")
    if d_vision_real < d_vision_rand * 0.5:
        print("  ❗ random 이미지가 real B 이미지보다 영향 큼 — feature가 OOD에 민감.")


if __name__ == "__main__":
    main()
