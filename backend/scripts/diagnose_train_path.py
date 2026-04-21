"""학습 path 정확 재현 진단.

목적: training loop와 동일하게 데이터 → preprocessor → policy.forward로 흘려서
- 보고된 train_loss와 같은 값이 나오는지
- train mode vs eval mode 출력 차이 (dropout 영향)
- predict_action_chunk vs forward의 동일성
- chunk position별 per-position loss 분석

사용:
  docker exec easy_collector_service python3 -m src.backend.scripts.diagnose_train_path --ckpt 31
"""
from __future__ import annotations

import argparse
import os
import pickle
import sys
from pathlib import Path

_lerobot_src = Path(__file__).resolve().parent.parent / "lerobot" / "src"
if str(_lerobot_src) not in sys.path:
    sys.path.insert(0, str(_lerobot_src))

import numpy as np
import torch
import torch.nn.functional as F

from lerobot.policies.act.modeling_act import ACTPolicy

from ..policies.utils import (
    EpisodicDataset,
    make_easytrainer_processors,
    forward_pass,
)


def _resolve_dataset_dir(dataset_id: int) -> str:
    return f"/opt/easytrainer/datasets/{dataset_id}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", type=int, required=True)
    parser.add_argument("--dataset", type=int, default=2)
    parser.add_argument("--num-episodes", type=int, default=32)
    parser.add_argument("--sensor-ids", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--seed", type=int, default=100)
    args = parser.parse_args()

    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    ckpt_dir = Path(f"/root/backend/checkpoints/{args.ckpt}")
    dataset_dir = _resolve_dataset_dir(args.dataset)

    print("=" * 80)
    print(f"TRAIN PATH DIAGNOSIS — ckpt={args.ckpt}, dataset={args.dataset}")
    print("=" * 80)

    # ── 1. policy 로드 ──────────────────────────────────────────────────
    policy = ACTPolicy.from_pretrained(str(ckpt_dir))
    policy.cuda()
    print(f"\n[1] Policy: use_vae={policy.config.use_vae}, "
          f"chunk_size={policy.config.chunk_size}, "
          f"dropout={policy.config.dropout}, "
          f"n_decoder_layers={policy.config.n_decoder_layers}")

    pre, post = make_easytrainer_processors(
        policy_type="ACT",
        cfg=policy.config,
        pretrained_path=str(ckpt_dir),
    )
    assert pre is not None, "preprocessor missing — ckpt was trained without normalize?"

    # ── 2. dataset ──────────────────────────────────────────────────────
    with open(ckpt_dir / "dataset_stats.pkl", "rb") as f:
        stats = pickle.load(f)

    # 학습과 동일한 split (seed 100, train_ratio 0.8)
    shuffled = np.random.permutation(args.num_episodes)
    split = max(1, int(0.8 * args.num_episodes))
    train_indices = shuffled[:split]
    print(f"\n[2] Train indices (first 10): {train_indices[:10].tolist()}")
    print(f"    Total train episodes: {len(train_indices)}")

    train_dataset = EpisodicDataset(
        episode_ids=train_indices,
        dataset_dir=dataset_dir,
        sensor_ids=args.sensor_ids,
        norm_stats=stats,
        chunk_size=policy.config.chunk_size,
        policy_type="ACT",
        vision_backbone="resnet18",
        n_obs_steps=1,
        action_key="qaction",
    )

    # ── 3. 한 batch 만들어서 학습 path 그대로 흘리기 ───────────────────
    print(f"\n[3] Loading 8 training samples (batch_size=8) ...")
    items = [train_dataset[i] for i in range(min(8, len(train_dataset)))]
    # 수동 collate (DataLoader 흉내)
    from torch.utils.data._utils.collate import default_collate
    batch = default_collate(items)
    print(f"    batch keys: {list(batch.keys())}")
    print(f"    state shape: {batch['observation.state'].shape}")
    print(f"    action shape: {batch['action'].shape}")
    print(f"    action_is_pad shape: {batch['action_is_pad'].shape}")
    print(f"    action raw[0,0]: {batch['action'][0, 0].numpy()}")
    print(f"    action raw[0,5]: {batch['action'][0, 5].numpy()}")

    # 4. forward_pass 그대로 (preprocessor 적용)
    print(f"\n[4] Running forward_pass exactly as training does (TRAIN mode):")
    policy.train()
    with torch.no_grad():  # gradient는 필요 없음
        loss_train, loss_dict_train = forward_pass(
            batch, policy, norm_stats=stats, preprocessor=pre
        )
    print(f"    train mode loss: {loss_train.item():.4f}")
    print(f"    loss_dict: {loss_dict_train}")

    # 5. EVAL mode에서 동일 batch
    print(f"\n[5] Same batch in EVAL mode:")
    policy.eval()
    with torch.no_grad():
        loss_eval, loss_dict_eval = forward_pass(
            batch, policy, norm_stats=stats, preprocessor=pre
        )
    print(f"    eval mode loss: {loss_eval.item():.4f}")
    print(f"    loss_dict: {loss_dict_eval}")
    print(f"    >>> train vs eval diff: {abs(loss_train.item() - loss_eval.item()):.4f}")
    print(f"    (큰 차이면 dropout 등이 추론을 망치고 있는 것)")

    # 6. predict_action_chunk vs forward 의 일치성 검증
    print(f"\n[6] predict_action_chunk vs forward (eval mode):")
    policy.eval()
    # 같은 batch를 preprocessor에 통과시킨 결과를 직접 사용
    data = {k: (v.cuda() if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
    data = pre(data)
    with torch.no_grad():
        # forward path
        loss_via_forward, _ = policy.forward(data)
        # predict_action_chunk path
        chunk = policy.predict_action_chunk(data)  # (B, chunk, action_dim)
    print(f"    forward path loss: {loss_via_forward.item():.4f}")
    print(f"    predict_action_chunk shape: {tuple(chunk.shape)}")

    # forward와 predict_action_chunk가 같은 출력을 내는지 직접 비교
    # forward 내부에서도 self.model(batch)[0]을 호출하므로 같아야 함
    target = data["action"]  # 이미 normalized 상태
    pad_mask = ~data["action_is_pad"]  # (B, chunk)
    per_position_l1 = (chunk - target).abs() * pad_mask.unsqueeze(-1).float()  # (B, chunk, action_dim)
    per_position_mean = per_position_l1.sum(dim=(0, 2)) / (pad_mask.float().sum(dim=0) * per_position_l1.shape[-1] + 1e-8)
    print(f"\n[7] Per-chunk-position L1 loss (eval mode, normalized space):")
    for i, v in enumerate(per_position_mean.cpu().numpy()):
        marker = " ← chunk[0]" if i == 0 else ""
        print(f"    pos {i:2d}: {v:.4f}{marker}")

    # 8. per-joint loss 분석 (전체 chunk 평균)
    per_joint_l1 = (chunk - target).abs() * pad_mask.unsqueeze(-1).float()
    per_joint_mean = per_joint_l1.sum(dim=(0, 1)) / (pad_mask.float().sum(dim=(0, 1)) + 1e-8)
    print(f"\n[8] Per-joint L1 loss (eval mode, normalized space, averaged over chunk):")
    for i, v in enumerate(per_joint_mean.cpu().numpy()):
        print(f"    joint {i}: {v:.4f}")

    # 9. 첫 chunk position만 따로 보고 raw 공간에서 비교
    print(f"\n[9] Sanity: first chunk position, sample 0, RAW space:")
    target_raw = batch["action"][0, 0].numpy()  # 원본
    chunk_first_norm = chunk[0, 0].cpu()  # normalized
    chunk_first_raw = post(chunk_first_norm.cuda()).cpu().numpy()
    print(f"    GT (raw)            : {np.round(target_raw, 4)}")
    print(f"    Model (raw, post-Q) : {np.round(chunk_first_raw, 4)}")
    print(f"    abs err per joint   : {np.round(np.abs(target_raw - chunk_first_raw), 4)}")

    # 10. 결론
    print("\n" + "=" * 80)
    print("INTERPRETATION")
    print("=" * 80)
    print(f"""
- 학습 시 보고되는 train_loss (마지막 epoch ~0.18) vs 여기서 측정한 train mode loss
  → 비슷하면 학습 path 정상, 다르면 보고 자체가 거짓

- train mode loss vs eval mode loss
  → 큰 차이 (>0.2) 나면 dropout이 추론을 망치고 있음 → dropout=0 권장

- chunk position별 loss 균등성
  → pos 0이 다른 pos보다 훨씬 크면 모델이 즉각 예측을 못 하고 미래만 잘 맞춤
  → 균등하면 chunk 전체적으로 학습됨

- per-joint loss
  → 특정 joint만 큰 값이면 그 joint 학습 실패
  → gripper(joint 6), succeed(joint 7) 보세요

- sample 0의 raw 공간 비교
  → GT와 거의 같으면 모델이 학습된 거. 차이 크면 진짜 학습 안 된 거.
""")


if __name__ == "__main__":
    main()
