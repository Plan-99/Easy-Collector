"""
Training worker - runs as a subprocess inside the training server.
Reads config from JSON file (no database dependency).
Reuses the same training logic as the local train.py.
"""
import torch
import math
import sys
import os
from pathlib import Path

# Add lerobot to path
_lerobot_src_dir = str(Path(__file__).resolve().parent / "lerobot" / "src")
if _lerobot_src_dir not in sys.path:
    sys.path.insert(0, _lerobot_src_dir)

import numpy as np
from tqdm import tqdm
import time
from copy import deepcopy
import argparse
import json
import shutil
import pickle

from policies.utils import (
    make_policy, make_optimizer, forward_pass, detach_dict,
    compute_dict_mean, set_seed, load_data, convert_lists_to_tuples,
    make_easytrainer_processors,
)

from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.pi05.configuration_pi05 import PI05Config
from lerobot.policies.pi05.modeling_pi05 import PI05Policy
from lerobot.utils.constants import (
    POLICY_PREPROCESSOR_DEFAULT_NAME,
    POLICY_POSTPROCESSOR_DEFAULT_NAME,
)
from lerobot.datasets.feature_utils import dataset_to_policy_features
from lerobot.configs.types import FeatureType
from safetensors.torch import load_file

import signal

# 저장(조기 종료) — training server 가 SIGUSR1 로 알린다. handler 는 flag 만 세팅하고
# 실제 저장은 train loop 가 flag 를 보고 정상 종료 경로(final validation → best
# checkpoint 저장 → exit 0)로 처리. stop(SIGTERM kill)과 달리 프로세스를 안 죽이고
# 그때까지의 best 를 정식 체크포인트로 남긴다.
_early_finish_requested = False


def _handle_early_finish(signum, frame):
    global _early_finish_requested
    _early_finish_requested = True
    print('[TRAIN] Early-finish (save) signal received — will run final '
          'validation, save best checkpoint, and finish.', flush=True)


def cycle(iterable):
    """무한 순환 iterator. itertools.cycle 은 첫 순회를 전부 메모리에 캐싱하므로
    DataLoader 엔 못 씀 — 소진 시 iter() 를 새로 만든다. persistent_workers=True 면
    worker 가 살아있어 재생성 비용은 거의 없음."""
    iterator = iter(iterable)
    while True:
        try:
            yield next(iterator)
        except StopIteration:
            iterator = iter(iterable)
            yield next(iterator)


def train(
    train_dataloader,
    val_dataloader,
    input_features,
    output_features,
    stats,
    policy_obj,
    train_settings_raw,
    load_model_path=None,
    checkpoint_dir=None,
):
    """Training function — full openpi-equivalent recipe (audit-doc bug #20-32 fixes).

    Mirrors backend/scripts/train.py (deleted by origin/integration_2 when training was
    split into this remote server). Includes:
      - Hybrid PI05 LoRA recipe (vision_tower attn+MLP + LM attn+MLP + expert attn+MLP
        LoRA, full-FT for projector + action heads, alpha=r, dropout=0)
      - pi05_base auto-load with key translator + tied embedding restoration
      - AdamW + warmup→cosine LR scheduler + gradient clipping
      - EMA shadow weights (default off; user opts in for openpi-recipe matching)
      - Smoothed val_loss best detection (default raw min for ACT/Diffusion compat)
      - Multi-pass validation (default single pass for ACT/Diffusion compat)
      - wrist_sensor_ids handling (popped + passed to load_data via main())
      - best_state_dict_cpu filter by requires_grad (covers full-FT modules)
    """
    seed = 100

    policy_settings = convert_lists_to_tuples(dict(policy_obj['settings']))
    train_settings = convert_lists_to_tuples(dict(train_settings_raw))

    set_seed(seed)

    # Step 기반 학습: num_steps = 총 optimizer update 수. 구버전 호환 — num_steps
    # 가 없으면 옛 num_epochs 값을 그대로 step 수로 사용. val_freq = 몇 step 마다
    # validation 할지(매 step validation 은 낭비).
    num_steps = train_settings.pop('num_steps', None)
    _legacy_num_epochs = train_settings.pop('num_epochs', None)
    if num_steps is None:
        num_steps = _legacy_num_epochs if _legacy_num_epochs is not None else 20000
    num_steps = int(num_steps)
    val_freq = int(train_settings.pop('val_freq', 200) or 200)
    train_settings.pop('batch_size', None)
    train_settings.pop('num_workers', None)
    train_settings.pop('action_type', None)
    train_settings.pop('has_succeed', None)
    train_settings.pop('use_relative_trajectory', None)
    # 메타 필드 — backend → training_server 전달용일 뿐 policy config 와 무관.
    # curriculum 흐름에서 ``train_settings`` 에 함께 묶여서 들어오므로 여기서 빼지
    # 않으면 ``ACTConfig(**train_settings)`` 가 ``unexpected keyword argument
    # 'server_url'`` 로 TypeError 를 던진다. (이 메시지가 그대로 사용자에게
    # 노출되어 "학습 서버 URL 이 없다" 로 오해를 일으킨다.)
    train_settings.pop('server_url', None)
    train_settings.pop('callback_url', None)
    policy_settings.pop('action_type', None)
    policy_settings.pop('obs_state_keys', None)
    # wrist_sensor_ids: read by main() before train(), but pop defensively in case
    # it survives into train_settings (would cause PI05Config to reject as unknown kwarg).
    train_settings.pop('wrist_sensor_ids', None)
    policy_settings.pop('wrist_sensor_ids', None)
    # 옛 policy DB row가 가지고 있을 수 있는 deprecated 필드 제거. PI05Config가
    # 더 이상 이 필드들을 안 가지므로 그대로 두면 unknown kwarg로 reject.
    for _dep in (
        'use_relative_actions', 'relative_joints_dim', 'relative_exclude_joints',
        'absolute_action_dims', 'relative_action_mask', 'action_feature_names',
    ):
        train_settings.pop(_dep, None)
        policy_settings.pop(_dep, None)

    # HuggingFace token for gated models (PaliGemma/Gemma).
    hf_token = policy_settings.pop('hf_token', None)
    if hf_token:
        os.environ['HF_TOKEN'] = hf_token
        os.environ['HUGGING_FACE_HUB_TOKEN'] = hf_token
        print(f'[TRAIN] HF token injected (len={len(hf_token)}, prefix={hf_token[:6]}...)', flush=True)

    use_peft = train_settings.pop('use_peft', False)
    peft_r = train_settings.pop('peft_r', 16)
    peft_alpha = train_settings.pop('peft_alpha', None)
    use_amp = train_settings.pop('use_amp', False)
    grad_accum_steps = train_settings.pop('grad_accum_steps', 1)
    # Unified resize target for the preprocessing pipeline (Common train setting,
    # not a policy config field) — lets users mix datasets with different camera
    # resolutions in one run. None → preprocessing falls back to (224, 224).
    # Pop here so it never reaches ACTConfig/DiffusionConfig/PI05Config (would be
    # rejected as an unknown kwarg).
    image_resolution = train_settings.pop('image_resolution', None)
    # EMA + smoothed validation defaults are SAFE-OFF (preserve prior ACT/Diffusion
    # behavior). PI05 LoRA users opt in via UI: ema_decay=0 (LoRA), 0.99 (full-FT only),
    # val_smooth_window=5 + val_n_passes=3 (small-dataset noise reduction).
    ema_decay = float(train_settings.pop('ema_decay', 0.0) or 0.0)
    val_smooth_window = int(train_settings.pop('val_smooth_window', 1) or 1)
    val_n_passes = int(train_settings.pop('val_n_passes', 1) or 1)

    def _log_gpu_mem(label):
        if torch.cuda.is_available():
            alloc = torch.cuda.memory_allocated() / 1024**3
            reserved = torch.cuda.memory_reserved() / 1024**3
            print(f'[GPU] {label}: alloc={alloc:.2f}GB, reserved={reserved:.2f}GB', flush=True)

    print(f'[TRAIN] Creating {policy_obj["type"]} policy... (use_amp={use_amp})', flush=True)
    _log_gpu_mem('before policy creation')

    if policy_obj['type'] == 'ACT':
        cfg = ACTConfig(input_features=input_features, output_features=output_features,
                        **policy_settings, **train_settings)
        policy = ACTPolicy(config=cfg)
        if use_amp:
            policy = policy.to(dtype=torch.bfloat16)
    elif policy_obj['type'] == 'Diffusion':
        cfg = DiffusionConfig(input_features=input_features, output_features=output_features,
                              **policy_settings, **train_settings)
        policy = DiffusionPolicy(config=cfg)
        if use_amp:
            policy = policy.to(dtype=torch.bfloat16)
    elif policy_obj['type'] == 'PI05':
        cfg = PI05Config(input_features=input_features, output_features=output_features,
                         **policy_settings, **train_settings)
        # PI05 has its own dtype handling (to_bfloat16_for_selected_params).
        # Always use bfloat16 for PI05 — float32 doubles VRAM.
        cfg.dtype = 'bfloat16'
        print(f'[TRAIN] PI05 config: device={cfg.device}, dtype={cfg.dtype}', flush=True)
        _log_gpu_mem('before PI05Policy init')
        policy = PI05Policy(config=cfg)

    _log_gpu_mem('after policy creation')
    print(f'[TRAIN] Policy created.', flush=True)

    policy.train()
    if not (policy_obj['type'] == 'PI05' and cfg.device in ('cuda', 'cuda:0')):
        policy.cuda()
    _log_gpu_mem('after policy on CUDA')

    # PI05: load pretrained weights into the VLM + action expert.
    # Two-tier strategy: (1) prefer lerobot/pi05_base (PI's PI0.5 post-pretraining
    # checkpoint), (2) fall back to PaliGemma-3B VLM-only.
    if policy_obj['type'] == 'PI05' and load_model_path is None:
        loaded_from = None
        _log_gpu_mem('before pretrained load')
        try:
            from huggingface_hub import hf_hub_download
            from safetensors.torch import load_file as _load_safetensors
            print('[TRAIN] Fetching lerobot/pi05_base (PI pre-trained PI0.5 checkpoint)...', flush=True)
            ckpt_path = hf_hub_download(repo_id="lerobot/pi05_base", filename="model.safetensors")
            pi05_sd = _load_safetensors(ckpt_path, device="cpu")
            target = policy.model
            adjusted = {}
            for k, v in pi05_sd.items():
                nk = k
                if nk.startswith('model.'):
                    nk = nk[len('model.'):]
                # Fold 'vision_tower.vision_model.X' → 'vision_tower.X' (transformers version diff)
                nk = nk.replace('vision_tower.vision_model.', 'vision_tower.')
                adjusted[nk] = v
            # Mirror tied embeddings (Gemma standard): lm_head ↔ embed_tokens.
            lm_head_key = 'paligemma_with_expert.paligemma.lm_head.weight'
            embed_key = 'paligemma_with_expert.paligemma.model.language_model.embed_tokens.weight'
            if lm_head_key in adjusted and embed_key not in adjusted:
                adjusted[embed_key] = adjusted[lm_head_key].clone()
            missing, unexpected = target.load_state_dict(adjusted, strict=False)
            print(f'[TRAIN] pi05_base loaded. missing={len(missing)} unexpected={len(unexpected)}', flush=True)
            if missing[:3]:
                print(f'  missing (first 3): {missing[:3]}', flush=True)
            if unexpected[:3]:
                print(f'  unexpected (first 3): {unexpected[:3]}', flush=True)
            del pi05_sd, adjusted
            loaded_from = 'lerobot/pi05_base'
        except Exception as e:
            print(f'[TRAIN] pi05_base load failed ({type(e).__name__}: {e}); '
                  f'falling back to PaliGemma-3B VLM-only init.', flush=True)

        if loaded_from is None:
            from transformers import PaliGemmaForConditionalGeneration
            print('[TRAIN] Loading PaliGemma-3B pretrained weights (VLM only)...', flush=True)
            pretrained = PaliGemmaForConditionalGeneration.from_pretrained(
                "google/paligemma-3b-pt-224", torch_dtype=torch.bfloat16, device_map="cpu",
            )
            pretrained_sd = pretrained.state_dict()
            del pretrained
            target_vocab = 257152
            vocab_keys = {'model.language_model.embed_tokens.weight', 'lm_head.weight'}
            for k in list(pretrained_sd.keys()):
                if k in vocab_keys and pretrained_sd[k].shape[0] > target_vocab:
                    pretrained_sd[k] = pretrained_sd[k][:target_vocab].contiguous()
            target = policy.model.paligemma_with_expert.paligemma
            missing, unexpected = target.load_state_dict(pretrained_sd, strict=False)
            print(f'[TRAIN] PaliGemma loaded. missing={len(missing)} unexpected={len(unexpected)}', flush=True)
            del pretrained_sd
            loaded_from = 'google/paligemma-3b-pt-224'

        # Restore the dtype split (vision float32, LLM bfloat16).
        policy.model.paligemma_with_expert.to_bfloat16_for_selected_params(cfg.dtype)
        torch.cuda.empty_cache()
        print(f'[TRAIN] Pretrained init source: {loaded_from}', flush=True)
        _log_gpu_mem('after pretrained load')

    if load_model_path is not None:
        print(f'[TRAIN] Loading pretrained weights from {load_model_path}...', flush=True)
        model_path = os.path.join(load_model_path, 'model.safetensors')
        state_dict = load_file(model_path, device='cuda')
        # strict 로드(기본). ACT state_dict 는 카메라를 sensor id 로 키잉하지 않고
        # (개수/순서 기반), 이 구성은 proprio 미사용(robot_state proj = (512,0)) 이라
        # 워크스페이스 디바이스(robot/sensor id) 를 바꿔도 weight 키 불일치가 없다.
        # 따라서 strict=True 가 정상 동작하며, 진짜 불일치(아키텍처/카메라 개수 변경
        # 등)는 조용히 넘어가지 않고 즉시 드러난다. 불일치가 실제로 생기면 strict=False
        # 로 덮지 말고 해당 체크포인트를 새 구성에 맞게 재학습/이전할 것.
        policy.load_state_dict(state_dict)
        print(f'[TRAIN] Pretrained weights loaded.', flush=True)

    policy.cuda()

    preprocessor, postprocessor = make_easytrainer_processors(
        policy_type=policy_obj['type'], cfg=cfg, dataset_stats=stats,
    )
    print('[TRAIN] Built preprocessor/postprocessor with dataset_stats.', flush=True)

    # PEFT/LoRA: hybrid recipe matching openpi gemma_2b_lora freeze filter.
    if use_peft:
        print(f'[TRAIN] Applying LoRA (r={peft_r}, alpha={peft_alpha or peft_r})...', flush=True)
        try:
            from peft import LoraConfig, get_peft_model
        except ImportError:
            raise ImportError("peft package not installed. Run 'pip install peft'.")

        if policy_obj['type'] == 'PI05':
            # Standard Gemma attention naming (paligemma language_model + gemma_expert)
            gemma_attn_suffixes = ('.self_attn.q_proj', '.self_attn.k_proj',
                                   '.self_attn.v_proj', '.self_attn.o_proj')
            gemma_mlp_suffixes = ('.mlp.gate_proj', '.mlp.up_proj', '.mlp.down_proj')
            # SigLIP vision_tower uses different naming
            siglip_attn_suffixes = ('.self_attn.q_proj', '.self_attn.k_proj',
                                    '.self_attn.v_proj', '.self_attn.out_proj')
            siglip_mlp_suffixes = ('.mlp.fc1', '.mlp.fc2')
            peft_target = []
            for name, module in policy.named_modules():
                if not isinstance(module, torch.nn.Linear):
                    continue
                # Gemma expert (action prediction backbone): attn + MLP.
                if '.gemma_expert.' in name and name.endswith(gemma_attn_suffixes + gemma_mlp_suffixes):
                    peft_target.append(name)
                # PaliGemma language_model: attn + MLP.
                elif '.paligemma.model.language_model.' in name and name.endswith(gemma_attn_suffixes + gemma_mlp_suffixes):
                    peft_target.append(name)
                # SigLIP vision_tower: attn + MLP (audit-doc step 6).
                elif '.paligemma.model.vision_tower.' in name and name.endswith(siglip_attn_suffixes + siglip_mlp_suffixes):
                    peft_target.append(name)

            # Full-FT modules (NOT LoRA-wrapped). openpi leaves these fully trainable.
            full_ft_substrings = (
                '.paligemma.model.multi_modal_projector.',
                '.action_in_proj', '.action_out_proj',
                '.time_mlp_in', '.time_mlp_out',
            )
            if not peft_target:
                raise RuntimeError('PI05 LoRA: no target modules found.')
            print(f'[TRAIN] LoRA targets: {len(peft_target)} modules ({peft_target[:3]} ...)', flush=True)
        else:
            peft_target = 'all-linear'
            full_ft_substrings = ()

        # alpha=r (scale 1.0) matches openpi; dropout=0 (openpi LoRA uses 0).
        _alpha = peft_alpha if peft_alpha else peft_r
        lora_config = LoraConfig(r=peft_r, lora_alpha=_alpha,
                                 target_modules=peft_target, lora_dropout=0.0)
        policy = get_peft_model(policy, lora_config)

        # Restore full-FT trainability (PEFT freezes all non-adapter params).
        n_full_ft = 0
        if full_ft_substrings:
            for pname, pparam in policy.named_parameters():
                if any(s in pname for s in full_ft_substrings):
                    pparam.requires_grad = True
                    n_full_ft += pparam.numel()
            print(f'[TRAIN] Full-FT params (no LoRA): {n_full_ft/1e6:.2f}M', flush=True)
        policy.print_trainable_parameters()
        print(f'[TRAIN] LoRA applied successfully.', flush=True)
        _log_gpu_mem('after LoRA')

    # EMA shadow weights (only trainable params; openpi LoRA preset uses 0=off).
    use_ema = ema_decay > 0
    ema_state: dict[str, torch.Tensor] = {}
    if use_ema:
        ema_state = {n: p.detach().clone() for n, p in policy.named_parameters() if p.requires_grad}
        n_ema = sum(t.numel() for t in ema_state.values())
        print(f'[TRAIN] EMA enabled: decay={ema_decay}, tracking {n_ema/1e6:.2f}M params', flush=True)
        _log_gpu_mem('after EMA init')

    # AdamW with config-driven betas/eps/weight_decay (audit-doc bug #17 fix).
    _cfg_betas = getattr(cfg, 'optimizer_betas', None)
    betas = tuple(_cfg_betas) if _cfg_betas is not None else (0.9, 0.999)
    optimizer = torch.optim.AdamW(
        filter(lambda p: p.requires_grad, policy.parameters()),
        lr=cfg.optimizer_lr,
        betas=betas,
        eps=getattr(cfg, 'optimizer_eps', 1e-8),
        weight_decay=getattr(cfg, 'optimizer_weight_decay', 0.0),
    )
    _log_gpu_mem('after optimizer creation')

    # LR scheduler: linear warmup → cosine decay (audit-doc bug #14 fix).
    warmup_steps = int(getattr(cfg, 'scheduler_warmup_steps', 0) or 0)
    decay_steps = int(getattr(cfg, 'scheduler_decay_steps', 0) or 0)
    decay_lr = float(getattr(cfg, 'scheduler_decay_lr', cfg.optimizer_lr) or cfg.optimizer_lr)
    peak_lr = float(cfg.optimizer_lr)
    sched_name = str(getattr(cfg, 'scheduler_name', '') or '').lower()

    if decay_steps <= 0 and sched_name == 'cosine':
        # step 기반: cosine decay 를 전체 학습 step(num_steps) 에 맞춘다.
        decay_steps = max(warmup_steps + 1, num_steps)

    def _lr_lambda(step: int) -> float:
        if warmup_steps > 0 and step < warmup_steps:
            return (step + 1) / warmup_steps
        if decay_steps <= warmup_steps:
            return 1.0
        progress = min(max((step - warmup_steps) / (decay_steps - warmup_steps), 0.0), 1.0)
        min_ratio = decay_lr / peak_lr if peak_lr > 0 else 0.0
        if sched_name == 'linear':
            return min_ratio + (1.0 - min_ratio) * (1.0 - progress)
        cosine = 0.5 * (1 + math.cos(math.pi * progress))
        return min_ratio + (1.0 - min_ratio) * cosine

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=_lr_lambda)
    _sched_desc = (f'warmup={warmup_steps} → '
                   + (f'{sched_name or "cosine"} decay to {decay_lr:.2e} over {decay_steps} steps'
                      if decay_steps > warmup_steps else 'constant'))
    print(f'[TRAIN] LR schedule ({policy_obj["type"]}): {_sched_desc} (peak {peak_lr:.2e})', flush=True)

    amp_dtype = torch.bfloat16 if use_amp else None
    print(f'[TRAIN] AMP={use_amp}, grad_accum={grad_accum_steps}', flush=True)

    train_history = []
    validation_history = []
    min_val_loss = np.inf
    recent_val_losses: list[float] = []
    min_smoothed_val = np.inf

    total_train_batches = len(train_dataloader)
    total_val_batches = len(val_dataloader)
    print(f'[TRAIN] Starting training: {num_steps} steps, val every {val_freq} steps, '
          f'{total_val_batches} val batches × {val_n_passes} pass(es), '
          f'smooth_window={val_smooth_window}', flush=True)

    best_state_dict_cpu = None
    best_step = -1

    def _swap_in_ema():
        if not use_ema:
            return None
        backup: dict[str, torch.Tensor] = {}
        with torch.no_grad():
            for n, p in policy.named_parameters():
                if n in ema_state:
                    backup[n] = p.data.clone()
                    p.data.copy_(ema_state[n])
        return backup

    def _restore_live(backup):
        if not use_ema or backup is None:
            return
        with torch.no_grad():
            for n, p in policy.named_parameters():
                if n in backup:
                    p.data.copy_(backup[n])

    grad_clip_norm = float(getattr(cfg, 'optimizer_grad_clip_norm', 0.0) or 0.0)
    trainable_params = [p for p in policy.parameters() if p.requires_grad]

    # empty_cache 빈도 — validation 마다 비우면 메모리가 매번 출렁이고 cudaMalloc
    # 재획득 overhead 도 큼. 매 validation 이 아니라 N 회마다 1번만 비운다(0.1배).
    _empty_cache_every = int(os.environ.get('EC_EMPTY_CACHE_EVERY', '10') or 10)
    _val_call_count = 0

    def _run_validation(step_done):
        """val_dataloader 전체 1회 평가 → val_loss 반환. best 면 best_state_dict_cpu
        갱신. EMA 가중치로 평가하고, 끝나면 학습 모드로 복원(캐시는 가끔만 비움)."""
        nonlocal best_state_dict_cpu, best_step, min_val_loss, min_smoothed_val, _val_call_count
        live_backup = _swap_in_ema()  # validate against EMA weights
        try:
            with torch.inference_mode():
                val_loss_sum = 0.0
                val_batch_count = 0
                for pass_idx in range(max(val_n_passes, 1)):
                    for batch_idx, data in enumerate(val_dataloader):
                        with torch.autocast('cuda', dtype=amp_dtype, enabled=use_amp):
                            loss, _ = forward_pass(data, policy, norm_stats=stats, preprocessor=preprocessor)
                        val_loss_sum += loss.item()
                        val_batch_count += 1
                cur_val_loss = val_loss_sum / max(val_batch_count, 1)
                validation_history.append(cur_val_loss)
                recent_val_losses.append(cur_val_loss)

                # Smoothed best detection (window=1 → raw min, preserves ACT/Diffusion behavior).
                is_best = False
                if len(recent_val_losses) >= val_smooth_window:
                    smoothed = sum(recent_val_losses[-val_smooth_window:]) / val_smooth_window
                    if smoothed < min_smoothed_val:
                        min_smoothed_val = smoothed
                        min_val_loss = smoothed
                        best_step = step_done
                        is_best = True
                elif cur_val_loss < min_val_loss:
                    min_val_loss = cur_val_loss
                    best_step = step_done
                    is_best = True

                if is_best:
                    # Snapshot EMA-loaded weights (currently in policy due to _swap_in_ema).
                    # Filter by requires_grad to capture both LoRA adapters AND full-FT modules.
                    if use_peft:
                        trainable_names = {n for n, p in policy.named_parameters() if p.requires_grad}
                        src = {k: v for k, v in policy.state_dict().items() if k in trainable_names}
                    else:
                        src = policy.state_dict()
                    best_state_dict_cpu = {k: v.detach().to('cpu', copy=True) for k, v in src.items()}
        finally:
            _restore_live(live_backup)
            policy.train()
            # 매 validation 이 아니라 N 회마다 한 번만 캐시 정리 (빈도 0.1배).
            _val_call_count += 1
            if _empty_cache_every > 0 and (_val_call_count % _empty_cache_every == 0):
                torch.cuda.empty_cache()
        return cur_val_loss

    # 학습 전 baseline validation (step 0 기준점).
    start_time = time.time()
    _baseline_val = _run_validation(0)
    print(f'  [VAL] step 0/{num_steps} (baseline) val_loss={_baseline_val:.4f}', flush=True)

    dl_iter = cycle(train_dataloader)
    policy.train()
    optimizer.zero_grad()
    _running_loss = 0.0
    _running_count = 0
    _last_log_time = time.time()
    _last_log_step = 0
    # 실제로 완주한 step 수. 정상 종료면 num_steps, 조기 종료면 break 시점.
    steps_completed = num_steps

    for step in range(num_steps):
        # 저장(조기 종료) 요청 시: 마지막 validation 한 번 더 → 직전 step 까지의
        # best 확정 후 loop 탈출 → 아래 정상 저장 경로로.
        if _early_finish_requested:
            print(f'[TRAIN] Early finish at step {step}/{num_steps} — '
                  f'final validation then saving best.', flush=True)
            steps_completed = step
            _run_validation(step)
            break

        # 한 optimizer update — grad_accum_steps 만큼 forward/backward 누적.
        _step_loss_sum = 0.0
        for _accum in range(grad_accum_steps):
            data = next(dl_iter)
            with torch.autocast('cuda', dtype=amp_dtype, enabled=use_amp):
                loss, _ = forward_pass(data, policy, norm_stats=stats, preprocessor=preprocessor)
            _step_loss_sum += loss.item()
            if grad_accum_steps > 1:
                loss = loss / grad_accum_steps
            loss.backward()

        if grad_clip_norm > 0:
            torch.nn.utils.clip_grad_norm_(trainable_params, grad_clip_norm)
        optimizer.step()
        scheduler.step()
        optimizer.zero_grad()
        if use_ema:
            with torch.no_grad():
                for n, p in policy.named_parameters():
                    if n in ema_state:
                        ema_state[n].mul_(ema_decay).add_(p.data, alpha=1.0 - ema_decay)

        _step_loss = _step_loss_sum / max(grad_accum_steps, 1)
        _cur_lr = optimizer.param_groups[0]['lr']
        train_history.append({'loss': _step_loss, 'lr': _cur_lr})
        _running_loss += _step_loss
        _running_count += 1

        step_done = step + 1
        if (step_done % val_freq == 0) or (step_done == num_steps):
            val_loss = _run_validation(step_done)
            train_loss_mean = _running_loss / max(_running_count, 1)
            _running_loss = 0.0
            _running_count = 0
            _now = time.time()
            _steps_per_sec = (step_done - _last_log_step) / max(_now - _last_log_time, 1e-6)
            _last_log_time = _now
            _last_log_step = step_done
            # epoch/total_epoch 키 유지 — training_server/app.py 의 progress 파서가
            # 이 키 비율로 progress 계산. step 의미로 재사용(UI 호환).
            train_log = {
                'epoch': step_done, 'total_epoch': num_steps,
                'step': step_done, 'total_step': num_steps,
                'val_loss': val_loss, 'train_loss': train_loss_mean,
                'train_time_sec': _now - start_time,
            }
            # 이 worker 가 잡은 GPU 메모리(MiB) — 서버가 job 별로 기록해 GPU 관리
            # 다이얼로그에서 nvidia-smi 프로세스를 체크포인트와 매칭하는 데 쓴다.
            try:
                if torch.cuda.is_available():
                    train_log['gpu_mib'] = int(torch.cuda.memory_reserved() / (1024 * 1024))
            except Exception:
                pass
            print(f"[TRAIN_LOG] {json.dumps(train_log)}", flush=True)
            print(f'  [STEP] {step_done}/{num_steps} train_loss={train_loss_mean:.4f} '
                  f'val_loss={val_loss:.4f} lr={_cur_lr:.2e} ({_steps_per_sec:.2f} step/s)', flush=True)

    # Save best checkpoint.
    ckpt_dir = checkpoint_dir
    # Restore best (EMA-snapshot) weights into policy. CPU snapshot only contains
    # trainable params; strict=False because base weights are unchanged.
    if best_state_dict_cpu is not None:
        policy.load_state_dict(best_state_dict_cpu, strict=False)
        best_state_dict_cpu = None
    elif use_ema:
        # No best snapshot taken — save final EMA weights as next-best.
        with torch.no_grad():
            for n, p in policy.named_parameters():
                if n in ema_state:
                    p.data.copy_(ema_state[n])
        print(f'[TRAIN] No best snapshot taken — saving final EMA weights instead.', flush=True)
    if use_ema:
        ema_state.clear()

    # PEFT: merge adapter weights into base model before saving (no peft dependency at inference).
    best_policy = policy
    if use_peft:
        best_policy = best_policy.merge_and_unload()

    best_policy.save_pretrained(ckpt_dir)

    config_path = os.path.join(ckpt_dir, 'config.json')
    with open(config_path, 'r') as f:
        saved_config = json.load(f)
    if 'type' not in saved_config:
        saved_config['type'] = best_policy.config.type
        with open(config_path, 'w') as f:
            json.dump(saved_config, f, indent=4)

    stats_path = os.path.join(ckpt_dir, 'dataset_stats.pkl')
    with open(stats_path, 'wb') as f:
        pickle.dump(stats, f)

    # Persist preprocessing metadata so inference can reproduce the exact resize
    # used during training. Kept in a sidecar file (not config.json) so the
    # vendored LeRobot dataclass loaders don't reject it as an unknown kwarg.
    if image_resolution is not None:
        try:
            _ir = (int(image_resolution[0]), int(image_resolution[1]))
        except Exception:
            _ir = (224, 224)
    else:
        _ir = (224, 224)
    with open(os.path.join(ckpt_dir, 'train_meta.json'), 'w') as f:
        json.dump({'image_resolution': list(_ir)}, f, indent=2)

    preprocessor.save_pretrained(ckpt_dir, config_filename=f"{POLICY_PREPROCESSOR_DEFAULT_NAME}.json")
    postprocessor.save_pretrained(ckpt_dir, config_filename=f"{POLICY_POSTPROCESSOR_DEFAULT_NAME}.json")

    _finish_kind = 'early-finish' if _early_finish_requested else 'completed'
    print(f'Training {_finish_kind} ({steps_completed}/{num_steps} steps):\n'
          f'Seed {seed}, val loss {min_val_loss:.6f} at step {best_step}', flush=True)
    return best_step, min_val_loss


def main(args):
    """Load config from JSON and run training."""
    # 저장(조기 종료) 시그널 등록 — training server 의 /api/train/save 가 이 worker
    # 메인 프로세스에 SIGUSR1 을 보낸다. train loop 가 flag 를 보고 best 저장 후 종료.
    try:
        signal.signal(signal.SIGUSR1, _handle_early_finish)
    except (ValueError, OSError) as e:
        print(f'[TRAIN][WARN] SIGUSR1 handler registration failed: {e}', flush=True)

    job_dir = args.job_dir
    checkpoint_dir = args.checkpoint_dir

    config_path = os.path.join(job_dir, 'train_config.json')
    with open(config_path, 'r') as f:
        config = json.load(f)

    policy = config['policy']
    train_settings = config['train_settings']
    dataset_dir = os.path.join(job_dir, 'dataset')

    # Find the actual dataset directory (might be nested)
    # Look for data.json to find the root
    actual_dataset_dir = _find_dataset_root(dataset_dir)
    if not actual_dataset_dir:
        print(f'[ERROR] Could not find dataset in {dataset_dir}', flush=True)
        sys.exit(1)

    # Check for base model
    base_model_dir = os.path.join(job_dir, 'base_model')
    load_model_path = None
    if os.path.exists(base_model_dir):
        # Find the model.safetensors
        for root, dirs, files in os.walk(base_model_dir):
            if 'model.safetensors' in files:
                load_model_path = root
                break

    # Count episodes
    from policies.utils import list_episodes, get_dataset_info, _read_json, INFO_PATH
    info = get_dataset_info(actual_dataset_dir)
    if not info:
        print(f'[ERROR] No dataset info found in {actual_dataset_dir}', flush=True)
        sys.exit(1)

    episodes = list_episodes(actual_dataset_dir)
    num_episodes = len(episodes)
    print(f'[INFO] Dataset: {num_episodes} episodes', flush=True)

    # Pre-decode videos to npy if needed
    _predecode_videos(actual_dataset_dir, info)

    # Extract settings
    batch_size = train_settings.get('batch_size', 32)
    action_key = policy['settings'].get('action_type') or train_settings.get('action_type', 'qaction')
    # NOTE: `or` 로 fallback 하면 빈 리스트 ([]) 가 falsy 라 ['qpos'] 로 덮여
    # 사용자의 "no-state" 의도가 무효화됨. 명시적 None 체크로 빈 리스트도 그대로 전달.
    _p_keys = policy['settings'].get('obs_state_keys')
    _t_keys = train_settings.get('obs_state_keys')
    if _p_keys is not None:
        obs_state_keys = _p_keys
    elif _t_keys is not None:
        obs_state_keys = _t_keys
    else:
        obs_state_keys = ['qpos']
    use_relative_trajectory = train_settings.get('use_relative_trajectory', False)
    sensor_ids = config.get('sensor_ids', [])

    # relative_joint_pos: 데이터 로더가 chunk-anchored joint delta로 미리 변환.
    # absolute로 유지할 dim(gripper/tool/done)은 dataset features에서 자동 검출.

    # wrist_sensor_ids: 콤마 sep 문자열로 들어옴 ("2,3"). 빈 문자열/None 모두 허용.
    # 컬러 jitter만 받고 spatial aug는 스킵 (audit-doc bug #31).
    _wrist_raw = (
        train_settings.get('wrist_sensor_ids')
        or policy['settings'].get('wrist_sensor_ids')
        or ''
    )
    if isinstance(_wrist_raw, (list, tuple)):
        wrist_sensor_ids = [int(x) for x in _wrist_raw if str(x).strip()]
    elif isinstance(_wrist_raw, str):
        wrist_sensor_ids = [int(x.strip()) for x in _wrist_raw.split(',') if x.strip()]
    else:
        wrist_sensor_ids = []

    if policy['type'] in ['ACT']:
        chunk_size = policy['settings']['chunk_size']
        vision_backbone = policy['settings']['vision_backbone']
    elif policy['type'] in ['Diffusion']:
        chunk_size = policy['settings']['horizon']
        vision_backbone = policy['settings']['vision_backbone']
    elif policy['type'] in ['PI05']:
        chunk_size = policy['settings']['chunk_size']
        vision_backbone = None

    num_workers = train_settings.get('num_workers', 4)
    n_obs_steps = policy['settings'].get('n_obs_steps', 1)
    # Resize target for the preprocessing pipeline. main() reads via .get() so it
    # stays in train_settings; train() pops from its own copy later (and that copy
    # is what writes train_meta.json).
    image_resolution = train_settings.get('image_resolution', None)

    # Detect succeed flag.
    # NOTE: LeRobot v2 layout 은 `data/chunk-XXX/...` (옛 v1 은 `parquet/chunk-XXX/...`).
    # 여기 inline path 가 v1 으로 남아있어서 sucess column 이 있는 데이터셋에도
    # has_succeed flag 가 set 되지 않았음. policies/utils.py 의 PARQUET_PATH_TEMPLATE
    # 와 같은 v2 layout 으로 통일.
    import pyarrow.parquet as pq
    PARQUET_PATH_TEMPLATE = "data/chunk-{chunk:03d}/episode_{ep:06d}.parquet"
    first_parquet = os.path.join(actual_dataset_dir, PARQUET_PATH_TEMPLATE.format(chunk=0, ep=0))
    if os.path.exists(first_parquet):
        _first_table = pq.read_table(first_parquet)
        if 'succeed' in _first_table.column_names:
            train_settings['has_succeed'] = True
            print('[INFO] succeed flag detected in dataset.', flush=True)

    # Multi-view: 데이터셋의 ``observation.images.*`` features 가 canonical
    # view_key 목록 (예: "5", "5_2", "7", "7_2"). task.sensor_ids 는 *물리*
    # ID 의 ordered list (중복 포함) 라 그대로 학습에 쓰면 EpisodicDataset 이
    # 같은 image_dict key 를 덮어써서 feature 수가 collapse 된다. 따라서
    # 항상 features 에서 view_key 를 뽑아 학습 input 의 ground truth 로 삼는다.
    # 정렬: (physical_sid, occurrence) 오름차순 — frontend / record / inference
    # 와 동일 ordering.
    features = info.get('features', {})
    _view_keys_from_features = [
        feat_key.replace('observation.images.sensor_', '')
        for feat_key in features
        if feat_key.startswith('observation.images.sensor_')
    ]

    def _view_key_sort_key(vk):
        s = str(vk)
        if '_' in s:
            head, tail = s.split('_', 1)
            try:
                return (int(head), int(tail))
            except ValueError:
                return (1 << 30, s)  # 비정형 키는 맨 뒤
        try:
            return (int(s), 1)  # occurrence 0 (suffix 없는 view) 는 1 로 처리해서
                                # "5" 가 "5_2" 보다 먼저 오게 — "_2" 는 (5,2)
        except ValueError:
            return (1 << 30, s)
    # 위 key 함수는 suffix 없는 view 를 (sid, 1) 로 두고, "_2" 는 (sid, 2),
    # "_3" 는 (sid, 3) 으로 두므로 "5", "5_2", "5_3", "7" 순서 보장.
    _view_keys_from_features.sort(key=_view_key_sort_key)

    if _view_keys_from_features:
        if sensor_ids and list(sensor_ids) != _view_keys_from_features:
            print(
                f'[INFO] Overriding config.sensor_ids={sensor_ids} with view_keys '
                f'derived from dataset features={_view_keys_from_features} '
                f'(multi-view canonical ordering).',
                flush=True,
            )
        sensor_ids = _view_keys_from_features
    elif not sensor_ids:
        sensor_ids = _view_keys_from_features
    print(f'[INFO] sensor_ids (view_keys) used for training: {sensor_ids}', flush=True)

    print(f"[CONFIG] obs_state_keys: {obs_state_keys}, action_key: {action_key}", flush=True)

    train_dataloader, val_dataloader, stats, input_features, output_features = load_data(
        actual_dataset_dir, policy['type'], num_episodes, sensor_ids,
        batch_size, batch_size, chunk_size, vision_backbone, num_workers,
        n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory,
        obs_state_keys=obs_state_keys,
        wrist_sensor_ids=wrist_sensor_ids,
        image_resolution=image_resolution,
    )

    # ── 학습 직전 state 구성 검증 (사용자가 의도한 obs_state_keys 가 실제 모델에
    # 어떻게 반영됐는지 한 줄로 확인 가능). 두 정보가 일치해야 함:
    #   1) obs_state_keys (전달된 키)
    #   2) stats['observation.state'].mean.shape (실제 학습되는 state 차원)
    try:
        _ss = stats.get('observation.state', {}).get('mean')
        _shape = tuple(getattr(_ss, 'shape', ())) if _ss is not None else None
    except Exception:
        _shape = None
    try:
        _input_state = input_features.get('observation.state')
        _input_shape = getattr(_input_state, 'shape', None)
    except Exception:
        _input_shape = None
    print(
        f"[VERIFY] obs_state_keys={obs_state_keys}  "
        f"stats.state.shape={_shape}  model.input.state.shape={_input_shape}",
        flush=True,
    )

    best_epoch, min_val_loss = train(
        train_dataloader, val_dataloader,
        input_features, output_features, stats,
        policy, train_settings,
        load_model_path=load_model_path,
        checkpoint_dir=checkpoint_dir,
    )

    # Save training result summary
    result = {
        'best_epoch': best_epoch,
        'loss': min_val_loss,
        'status': 'finished',
    }
    with open(os.path.join(checkpoint_dir, 'result.json'), 'w') as f:
        json.dump(result, f)

    print("Training process completed successfully.", flush=True)


def _find_dataset_root(search_dir):
    """Find the directory containing data.json (dataset root)."""
    for root, dirs, files in os.walk(search_dir):
        if 'data.json' in files:
            return root
    # Fallback: use search_dir itself if it has parquet/
    if os.path.exists(os.path.join(search_dir, 'parquet')):
        return search_dir
    # Check one level deep
    for d in os.listdir(search_dir):
        subdir = os.path.join(search_dir, d)
        if os.path.isdir(subdir) and os.path.exists(os.path.join(subdir, 'parquet')):
            return subdir
    return search_dir


def _predecode_videos(dataset_dir, info):
    """Pre-decode MP4 videos to numpy arrays for efficient loading."""
    features = info.get('features', {})
    for feat_key, feat in features.items():
        if not feat_key.startswith('observation.images.'):
            continue
        if feat.get('dtype') != 'video':
            continue

        videos_dir = os.path.join(dataset_dir, 'videos')
        if not os.path.exists(videos_dir):
            continue

        for chunk_dir in sorted(os.listdir(videos_dir)):
            feat_dir = os.path.join(videos_dir, chunk_dir, feat_key)
            if not os.path.exists(feat_dir):
                continue
            for mp4_file in sorted(os.listdir(feat_dir)):
                if not mp4_file.endswith('.mp4'):
                    continue
                npy_path = os.path.join(feat_dir, mp4_file[:-4] + '.npy')
                if os.path.exists(npy_path):
                    continue
                mp4_path = os.path.join(feat_dir, mp4_file)
                try:
                    from policies.utils import _decode_video_frames
                    frames = _decode_video_frames(mp4_path)
                    if frames:
                        arr = np.stack(frames)
                        np.save(npy_path, arr)
                        print(f'[INFO] Pre-decoded {mp4_path} -> {npy_path}', flush=True)
                except Exception as e:
                    print(f'[WARNING] Failed to pre-decode {mp4_path}: {e}', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--job_id', required=True)
    parser.add_argument('--job_dir', required=True)
    parser.add_argument('--checkpoint_dir', required=True)
    main(parser.parse_args())
