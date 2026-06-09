# -*- coding: utf-8 -*-
"""학습 파라미터 → 예상 GPU VRAM 사용량(MiB) 추정.

**근사치**다. 정확한 peak VRAM 은 policy 구현·시퀀스 길이·옵티마이저·CUDA 단편화에
따라 달라지므로, 여기 값은 "동시 학습 admission 게이트 + UI 안내" 용도의 보수적
추정이다. 과소추정으로 OOM 이 나도 스케줄러가 실패를 fail 처리하지 않고 waiting
으로 requeue 하므로(자세한 건 training_scheduler) 안전망이 있다.

모델별 기준값은 RTX 4080(16GB)에서의 대략적 관찰을 바탕으로 한 상수이며,
환경변수로 덮어쓸 수 있다(EC_GPU_EST_*). 더 정확히 맞추려면 상수만 조정하면 된다.
"""
from __future__ import annotations

import os


# policy_type -> (base_mib, act_per_bs_at_224)
#   base_mib       : 모델 weight + 옵티마이저 state 등 batch 와 무관한 상주 메모리
#   act_per_bs_224 : 224×224 기준 batch 1 당 activation 메모리(MiB). batch_size 와
#                    이미지 픽셀 수에 비례한다고 가정.
_POLICY_PROFILE = {
    'ACT':       (1500, 90),
    'Diffusion': (2500, 110),
    'PI05':      (12000, 180),   # PaliGemma-3B 백본 — 상주 메모리가 큼
}
_DEFAULT_PROFILE = (2000, 100)


def _f(env, default):
    try:
        v = os.environ.get(env, '').strip()
        return float(v) if v else float(default)
    except (TypeError, ValueError):
        return float(default)


def estimate_gpu_mib(policy_type, train_settings):
    """(policy_type, train_settings dict) → 예상 VRAM MiB (int).

    고려 요소: policy 종류, batch_size, image_resolution, LoRA(use_peft),
    mixed precision(use_amp). gradient accumulation 은 micro-batch 메모리를
    바꾸지 않으므로 무시.
    """
    ts = train_settings or {}
    profile = _POLICY_PROFILE.get(policy_type, _DEFAULT_PROFILE)
    base, act_per_bs = profile

    # batch size
    try:
        bs = max(1, int(ts.get('batch_size', 32) or 32))
    except (TypeError, ValueError):
        bs = 32

    # image resolution (H, W) → 224² 대비 픽셀 스케일
    res = ts.get('image_resolution') or [224, 224]
    try:
        h, w = int(res[0]), int(res[1])
        res_scale = max(0.1, (h * w) / float(224 * 224))
    except (TypeError, ValueError, IndexError):
        res_scale = 1.0

    base = float(base)
    # LoRA: 백본을 얼리고 작은 어댑터만 학습 → 옵티마이저 state 가 급감.
    if policy_type == 'PI05' and bool(ts.get('use_peft')):
        base *= _f('EC_GPU_EST_LORA_FACTOR', 0.55)

    act = act_per_bs * bs * res_scale

    total = base + act
    # mixed precision(bf16): activation 메모리 절감.
    if bool(ts.get('use_amp')):
        total *= _f('EC_GPU_EST_AMP_FACTOR', 0.8)

    # 안전 마진(추정 불확실성 흡수).
    total *= _f('EC_GPU_EST_SAFETY', 1.15)
    return int(round(total))


def estimate_gpu_mib_for_checkpoint(checkpoint):
    """Checkpoint ORM row → 예상 VRAM MiB. policy 타입은 checkpoint.policy.type.
    추정 불가 시 None.
    """
    try:
        ts = checkpoint._get_json_field('train_settings') or {}
        policy = checkpoint.policy
        policy_type = policy.type if policy else None
        return estimate_gpu_mib(policy_type, ts)
    except Exception:
        return None
