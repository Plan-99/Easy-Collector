import torch

import math
import sys
import os
from pathlib import Path

# 새 lerobot 패키지(src layout)를 sys.path에 추가
_lerobot_src_dir = str(Path(__file__).resolve().parent.parent / "lerobot" / "src")
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

# Import policies and utilities
from ..policies.utils import make_policy, make_optimizer, forward_pass, detach_dict, compute_dict_mean, set_seed, load_data, convert_lists_to_tuples, make_easytrainer_processors
from ..configs.global_configs import DATASET_DIR

# Import database and data models
from ..database.config.database import db as peewee_db
from ..database.models.robot_model import Robot
from ..database.models.policy_model import Policy
from ..database.models.task_model import Task
from ..database.models.gripper_model import Gripper
from ..database.models.sensor_model import Sensor
from ..database.models.checkpoint_model import Checkpoint

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

from .train_fiper import train_fiper
from ..api.process.generate_ood_features import generate_ood_features


def train(
    train_dataloader,
    val_dataloader,
    input_features,
    output_features,
    stats,
    policy_obj,
    checkpoint_obj,
    load_model=None,
    ):
    """Function to train the policy model."""
    seed = 100
    
    policy_settings = convert_lists_to_tuples(policy_obj['settings'])
    train_settings = convert_lists_to_tuples(checkpoint_obj['train_settings'])
    load_model_path = f"/root/src/backend/checkpoints/{load_model['id']}" if load_model else None

    set_seed(seed) # Set seed for reproducibility

    num_epochs = train_settings['num_epochs']
    del train_settings['num_epochs'] # Remove num_epochs from train_settings
    del train_settings['batch_size'] # Remove batch_size from train_settings
    del train_settings['num_workers'] # Remove num_workers from train_settings
    train_settings.pop('action_type', None) # Remove action_type (not a policy config param)
    train_settings.pop('has_succeed', None) # Remove has_succeed (not a policy config param)
    train_settings.pop('use_relative_trajectory', None) # Remove use_relative_trajectory (not a policy config param)
    policy_settings.pop('action_type', None) # Remove action_type (not a policy config param)
    policy_settings.pop('obs_state_keys', None) # Remove obs_state_keys (not a policy config param)

    # HuggingFace token for gated models (PaliGemma/Gemma). Pop before passing to
    # policy config and inject into env so HF libraries (AutoTokenizer, etc.) pick it up.
    hf_token = policy_settings.pop('hf_token', None)
    if hf_token:
        import os
        os.environ['HF_TOKEN'] = hf_token
        os.environ['HUGGING_FACE_HUB_TOKEN'] = hf_token
        print(f'[TRAIN] HF token injected (len={len(hf_token)}, prefix={hf_token[:6]}...)', flush=True)

    # PEFT/LoRA settings (pop before passing to policy config)
    use_peft = train_settings.pop('use_peft', False)
    peft_r = train_settings.pop('peft_r', 16)
    peft_alpha = train_settings.pop('peft_alpha', None)
    # Mixed precision & gradient accumulation settings
    use_amp = train_settings.pop('use_amp', False)
    grad_accum_steps = train_settings.pop('grad_accum_steps', 1)
    # EMA + smoothed validation settings (pop before passing to policy config).
    # IMPORTANT: openpi explicitly DISABLES EMA for LoRA presets (ema_decay=None in
    # `gemma_2b_lora`/`gemma_300m_lora` configs) — only full-FT presets use 0.99/0.999.
    # Reason: LoRA-B starts at zero, so EMA(0.99) systematically drags the trained
    # delta back toward zero ("99% old, 1% new"), under-training the LoRA path.
    # Default to 0 when use_peft is on, 0.99 otherwise (matches openpi behavior).
    # User can still override via UI/train_settings.
    _ema_default = 0.0 if train_settings.get('use_peft', False) else 0.99
    ema_decay = float(train_settings.pop('ema_decay', _ema_default) or 0.0)
    val_smooth_window = int(train_settings.pop('val_smooth_window', 5))
    val_n_passes = int(train_settings.pop('val_n_passes', 3))

    
    def _log_gpu_mem(label):
        if torch.cuda.is_available():
            alloc = torch.cuda.memory_allocated() / 1024**3
            reserved = torch.cuda.memory_reserved() / 1024**3
            print(f'[GPU] {label}: alloc={alloc:.2f}GB, reserved={reserved:.2f}GB', flush=True)

    print(f'[TRAIN] Creating {policy_obj["type"]} policy... (use_amp={use_amp})', flush=True)
    _log_gpu_mem('before policy creation')
    if policy_obj['type'] == 'ACT':
        cfg = ACTConfig(
            input_features=input_features,
            output_features=output_features,
            **policy_settings,
            **train_settings,
        )
        policy = ACTPolicy(config=cfg)
        if use_amp:
            policy = policy.to(dtype=torch.bfloat16)
    elif policy_obj['type'] == 'Diffusion':
        cfg = DiffusionConfig(
            input_features=input_features,
            output_features=output_features,
            **policy_settings,
            **train_settings,
        )
        policy = DiffusionPolicy(config=cfg)
        if use_amp:
            policy = policy.to(dtype=torch.bfloat16)
    elif policy_obj['type'] == 'PI05':
        cfg = PI05Config(
            input_features=input_features,
            output_features=output_features,
            **policy_settings,
            **train_settings,
        )
        # PI05 has its own dtype handling (to_bfloat16_for_selected_params)
        # Always use bfloat16 for PI05 — float32 doubles VRAM (~10GB vs ~5GB)
        # and offers no training quality benefit for this architecture
        cfg.dtype = 'bfloat16'
        print(f'[TRAIN] PI05 config: device={cfg.device}, dtype={cfg.dtype}', flush=True)
        _log_gpu_mem('before PI05Policy init')
        policy = PI05Policy(config=cfg)
    _log_gpu_mem('after policy creation')
    print(f'[TRAIN] Policy created.', flush=True)

    policy.train()
    if not (policy_obj['type'] == 'PI05' and cfg.device in ('cuda', 'cuda:0')):
        # PI05 already on CUDA via config.device; only move others
        policy.cuda()
    _log_gpu_mem('after policy on CUDA')
    print(f'[TRAIN] Policy on CUDA.', flush=True)

    # PI05: load pretrained weights into the VLM + action expert.
    # Two-tier strategy:
    #   1) Prefer lerobot/pi05_base — Physical Intelligence's public checkpoint
    #      AFTER the 280k-step pre-training stage (paper §IV.C). This closes
    #      the gap for users who cannot run full pre-training themselves.
    #   2) Fallback to google/paligemma-3b-pt-224 — VLM backbone only. Action
    #      expert + action/time projections stay random (paper-correct for
    #      post-training start, but with no pre-training benefit).
    if policy_obj['type'] == 'PI05' and load_model_path is None:
        loaded_from = None
        _log_gpu_mem('before pretrained load')
        try:
            from huggingface_hub import hf_hub_download
            from safetensors.torch import load_file as _load_safetensors
            print('[TRAIN] Fetching lerobot/pi05_base (PI pre-trained PI0.5 checkpoint)...', flush=True)
            ckpt_path = hf_hub_download(
                repo_id="lerobot/pi05_base",
                filename="model.safetensors",
            )
            # Load to CPU; load_state_dict will copy per-tensor to the target device.
            pi05_sd = _load_safetensors(ckpt_path, device="cpu")
            # Three key-format differences between pi05_base (older transformers) and
            # our current transformers version:
            #   1) SigLIP used to nest embeddings/encoder under an extra 'vision_model.'
            #      level; new transformers exposes them directly on the vision tower.
            #   2) pi05_base keys are at PI05Pytorch level (no outer 'model.' prefix),
            #      while we load into policy.model which IS that level — no prefix
            #      adjustment needed, but we strip any stray leading 'model.' just in case.
            #   3) Gemma ties embed_tokens with lm_head, so pi05_base only stored
            #      lm_head.weight. We materialize them as separate tensors via the
            #      meta-device rebuild, so copy lm_head → embed_tokens to restore parity.
            target = policy.model
            adjusted = {}
            for k, v in pi05_sd.items():
                nk = k
                if nk.startswith('model.'):
                    nk = nk[len('model.'):]
                # Fold 'vision_tower.vision_model.X' → 'vision_tower.X'
                nk = nk.replace('vision_tower.vision_model.', 'vision_tower.')
                adjusted[nk] = v
            # Mirror tied embeddings (Gemma standard): lm_head ↔ embed_tokens.
            lm_head_key = 'paligemma_with_expert.paligemma.lm_head.weight'
            embed_key = 'paligemma_with_expert.paligemma.model.language_model.embed_tokens.weight'
            if lm_head_key in adjusted and embed_key not in adjusted:
                adjusted[embed_key] = adjusted[lm_head_key].clone()
            missing, unexpected = target.load_state_dict(adjusted, strict=False)
            print(
                f'[TRAIN] pi05_base loaded. missing={len(missing)} unexpected={len(unexpected)}',
                flush=True,
            )
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
                "google/paligemma-3b-pt-224",
                torch_dtype=torch.bfloat16,
                device_map="cpu",  # stage on CPU to avoid a 2nd 6GB GPU copy
            )
            pretrained_sd = pretrained.state_dict()
            del pretrained
            # PaliGemma-3B-pt-224 uses vocab=257216, our model uses 257152 to stay
            # compatible with lerobot/pi05_base. Slice off the 64 extra token rows
            # in embed_tokens and lm_head (PI05 doesn't use them).
            target_vocab = 257152
            vocab_keys = {
                'model.language_model.embed_tokens.weight',
                'lm_head.weight',
            }
            for k in list(pretrained_sd.keys()):
                if k in vocab_keys and pretrained_sd[k].shape[0] > target_vocab:
                    pretrained_sd[k] = pretrained_sd[k][:target_vocab].contiguous()
            target = policy.model.paligemma_with_expert.paligemma
            missing, unexpected = target.load_state_dict(pretrained_sd, strict=False)
            print(
                f'[TRAIN] PaliGemma loaded. missing={len(missing)} unexpected={len(unexpected)}',
                flush=True,
            )
            if missing[:3]:
                print(f'  missing (first 3): {missing[:3]}', flush=True)
            if unexpected[:3]:
                print(f'  unexpected (first 3): {unexpected[:3]}', flush=True)
            del pretrained_sd
            loaded_from = 'google/paligemma-3b-pt-224'

        # Restore the dtype split (vision float32, LLM bfloat16) that
        # PI05Policy.__init__ set up. Loading may have overwritten some params.
        policy.model.paligemma_with_expert.to_bfloat16_for_selected_params(cfg.dtype)
        torch.cuda.empty_cache()
        print(f'[TRAIN] Pretrained init source: {loaded_from}', flush=True)
        _log_gpu_mem('after pretrained load')

    if load_model_path is not None:
        print(f'[TRAIN] Loading pretrained weights from {load_model_path}...', flush=True)
        model_path = os.path.join(load_model_path, 'model.safetensors')
        state_dict = load_file(model_path, device='cuda')
        policy.load_state_dict(state_dict)
        print(f'[TRAIN] Pretrained weights loaded.', flush=True)

    policy.cuda() # Move model to GPU

    # Build pre/post processors so action/state get normalized per cfg.normalization_mapping.
    # Without this, raw L1 loss is dominated by large-magnitude joints and tiny-scale joints
    # (e.g. gripper in [0, 0.05]) never get learned. The processor pipeline applies
    # Normalize to inputs (state, images) and target action during training; at inference
    # checkpoint_test loads the same pipeline and Unnormalize the model output.
    preprocessor, postprocessor = make_easytrainer_processors(
        policy_type=policy_obj['type'],
        cfg=cfg,
        dataset_stats=stats,
    )
    print('[TRAIN] Built preprocessor/postprocessor with dataset_stats.', flush=True)

    # PEFT/LoRA: wrap policy with low-rank adapters to reduce GPU memory usage
    if use_peft:
        print(f'[TRAIN] Applying LoRA (r={peft_r}, alpha={peft_alpha or peft_r})...', flush=True)
        try:
            from peft import LoraConfig, get_peft_model
        except ImportError:
            raise ImportError(
                "peft 패키지가 설치되어 있지 않습니다. "
                "Docker 컨테이너에서 'pip install peft' 실행 후 다시 시도하세요."
            )
        # Policy-specific LoRA target modules
        # PI05/PI0: mirror openpi's gemma_2b_lora freeze filter — `nnx.All('*llm*', Not('*lora*'))`.
        # In openpi, modules under PaliGemma/llm get LoRA on attn+ffn, while everything else
        # (vision_tower, multi_modal_projector, action/time projections) is FULLY trainable.
        # We replicate that hybrid recipe: LoRA on LLM/expert backbones, full-FT on small heads
        # and vision projector. SigLIP vision_tower gets attention-only LoRA (full-FT would be
        # 412M params; attention LoRA captures most of the adaptation benefit while staying
        # within single-GPU memory).
        if policy_obj['type'] == 'PI05':
            # Standard Gemma attention naming (paligemma language_model + gemma_expert)
            gemma_attn_suffixes = (
                '.self_attn.q_proj', '.self_attn.k_proj',
                '.self_attn.v_proj', '.self_attn.o_proj',
            )
            gemma_mlp_suffixes = (
                '.mlp.gate_proj', '.mlp.up_proj', '.mlp.down_proj',
            )
            # SigLIP vision_tower uses different naming: out_proj (not o_proj) for attention
            # and fc1/fc2 (not gate/up/down) for MLP.
            siglip_attn_suffixes = (
                '.self_attn.q_proj', '.self_attn.k_proj',
                '.self_attn.v_proj', '.self_attn.out_proj',
            )
            siglip_mlp_suffixes = ('.mlp.fc1', '.mlp.fc2')
            peft_target = []
            for name, module in policy.named_modules():
                if not isinstance(module, torch.nn.Linear):
                    continue
                # Gemma expert (action prediction backbone): attn + MLP.
                if '.gemma_expert.' in name and name.endswith(gemma_attn_suffixes + gemma_mlp_suffixes):
                    peft_target.append(name)
                # PaliGemma language_model: attn + MLP. MLP path was the missing piece —
                # openpi puts LoRA on both attn and ffn ("attn"/"ffn" lora_configs). Without
                # MLP LoRA the prefix can't reshape its task-specific reasoning; only attn
                # patterns adapt, which limits how vision/language tokens are integrated.
                elif '.paligemma.model.language_model.' in name and name.endswith(gemma_attn_suffixes + gemma_mlp_suffixes):
                    peft_target.append(name)
                # SigLIP vision_tower: attn + MLP. attn-only LoRA (previous version) let token
                # interaction patterns adapt but the FEATURE TRANSFORMATION itself stayed
                # frozen — that's mostly in fc1/fc2 (each ~5M params per layer). Transition
                # diagnostic showed cube xy localization (joint 0 + joint 5) was the most
                # inaccurate dim, consistent with vision features not adapting to recognize
                # tabletop cube positions at fine spatial resolution. Adding MLP LoRA gives
                # the vision feature extraction itself task-specific spatial precision while
                # still preserving the SigLIP pretrained init via low-rank delta.
                elif '.paligemma.model.vision_tower.' in name and name.endswith(siglip_attn_suffixes + siglip_mlp_suffixes):
                    peft_target.append(name)

            # Full-FT modules (NOT LoRA-wrapped). openpi leaves these fully trainable in its
            # LoRA configs — they're outside the '*llm*' freeze filter. r=64 LoRA on tiny
            # heads (~1M params each) is more constraint than capacity; full FT is cheaper
            # (4.5M extra trainable params total) AND more expressive.
            full_ft_substrings = (
                '.paligemma.model.multi_modal_projector.',
                '.action_in_proj',
                '.action_out_proj',
                '.time_mlp_in',
                '.time_mlp_out',
            )

            if not peft_target:
                raise RuntimeError('PI05 LoRA: no target modules found. Check model structure.')
            print(f'[TRAIN] LoRA targets: {len(peft_target)} modules ({peft_target[:3]} ...)', flush=True)
        else:
            peft_target = 'all-linear'
            full_ft_substrings = ()

        # openpi LoRA configs use scale = α/r = 1.0 (rank=alpha=16 or 32). Default alpha=peft_r
        # (instead of 2× as before): scale=2.0 amplified noisy LoRA updates and contributed to
        # erratic vision conditioning observed in earlier runs. UI can still override.
        _alpha = peft_alpha if peft_alpha else peft_r
        lora_config = LoraConfig(
            r=peft_r,
            lora_alpha=_alpha,
            target_modules=peft_target,
            lora_dropout=0.0,  # openpi uses 0; LoRA dropout adds gradient noise on small datasets
        )
        policy = get_peft_model(policy, lora_config)

        # Restore full-FT trainability AFTER PEFT applies. PEFT's mark_only_adapters_as_trainable
        # sets all non-adapter base params to requires_grad=False — we override for the small heads
        # that should be fully trainable per openpi's recipe. Optimizer is created later with
        # filter(requires_grad), so it picks up these params correctly.
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

    # EMA shadow weights: mirror openpi's gemma_2b_lora ema_decay=0.99. Updated
    # per optimizer step; validation runs against EMA-loaded weights for less
    # noisy val_loss; final checkpoint saves EMA weights. Only trainable params
    # are tracked to keep memory cost manageable (~145M × 2B bf16 = ~290MB extra).
    use_ema = ema_decay > 0
    ema_state: dict[str, torch.Tensor] = {}
    if use_ema:
        ema_state = {
            n: p.detach().clone()
            for n, p in policy.named_parameters()
            if p.requires_grad
        }
        n_ema = sum(t.numel() for t in ema_state.values())
        print(f'[TRAIN] EMA enabled: decay={ema_decay}, tracking {n_ema/1e6:.2f}M params', flush=True)
        _log_gpu_mem('after EMA init')

    # Only optimize trainable params (with PEFT, base model is frozen → saves optimizer memory).
    # AdamW replaces plain Adam (Adam treats weight_decay as L2 regularization on gradients,
    # which interacts badly with momentum on transformers). Defaults preserve previous
    # behavior for ACT (which has no betas configured): (0.9, 0.999), torch's Adam default.
    # Note: before this change, weight_decay was silently ignored because it wasn't passed;
    # now it's applied per the config field. ACT/Diffusion UI values now actually take effect.
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

    # LR scheduler: linear warmup → cosine decay. Per-policy behavior:
    #   - PI05:      warmup + cosine decay to scheduler_decay_lr (paper recipe).
    #   - Diffusion: warmup (scheduler_warmup_steps=500). If scheduler_name='cosine'
    #                we decay over the remainder of training (num_epochs*batches/grad_accum).
    #   - ACT:       no scheduler fields → warmup=0, _lr_lambda returns 1.0 for all steps
    #                (constant LR, matches pre-change behavior).
    warmup_steps = int(getattr(cfg, 'scheduler_warmup_steps', 0) or 0)
    decay_steps = int(getattr(cfg, 'scheduler_decay_steps', 0) or 0)
    decay_lr = float(getattr(cfg, 'scheduler_decay_lr', cfg.optimizer_lr) or cfg.optimizer_lr)
    peak_lr = float(cfg.optimizer_lr)
    sched_name = str(getattr(cfg, 'scheduler_name', '') or '').lower()

    # For Diffusion-style configs with scheduler_name='cosine' but no decay_steps,
    # treat the full training horizon as the decay window so 'cosine' actually decays.
    if decay_steps <= 0 and sched_name == 'cosine':
        est_steps_per_epoch = max(1, len(train_dataloader) // max(1, grad_accum_steps))
        decay_steps = max(warmup_steps + 1, num_epochs * est_steps_per_epoch)

    def _lr_lambda(step: int) -> float:
        # Linear warmup
        if warmup_steps > 0 and step < warmup_steps:
            return (step + 1) / warmup_steps
        # No decay configured → hold peak LR (ACT default, Diffusion='constant')
        if decay_steps <= warmup_steps:
            return 1.0
        # 'linear' name → linear decay to min_ratio
        progress = (step - warmup_steps) / (decay_steps - warmup_steps)
        progress = min(max(progress, 0.0), 1.0)
        min_ratio = decay_lr / peak_lr if peak_lr > 0 else 0.0
        if sched_name == 'linear':
            return min_ratio + (1.0 - min_ratio) * (1.0 - progress)
        # Default (PI05, Diffusion 'cosine'): cosine decay from 1.0 → min_ratio
        cosine = 0.5 * (1 + math.cos(math.pi * progress))
        return min_ratio + (1.0 - min_ratio) * cosine

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=_lr_lambda)
    _sched_desc = (
        f'warmup={warmup_steps} → '
        + (f'{sched_name or "cosine"} decay to {decay_lr:.2e} over {decay_steps} steps'
           if decay_steps > warmup_steps else 'constant')
    )
    print(f'[TRAIN] LR schedule ({policy_obj["type"]}): {_sched_desc} (peak {peak_lr:.2e})', flush=True)

    # Mixed precision: autocast dtype for forward pass
    # PI05 handles its own dtype internally, so autocast is mainly for ACT/Diffusion
    amp_dtype = torch.bfloat16 if use_amp else None
    print(f'[TRAIN] AMP={use_amp}, grad_accum={grad_accum_steps}', flush=True)

    train_history = []
    validation_history = []
    min_val_loss = np.inf
    # Smoothed best-epoch detection: maintain rolling buffer of last K val_losses,
    # compare smoothed average against min_smoothed. Suppresses single-epoch outliers
    # (ckpt 96's epoch 98 dipped to 0.014 vs window median 0.07 — a noise lottery).
    recent_val_losses: list[float] = []
    min_smoothed_val = np.inf

    # Count total batches for progress logging
    total_train_batches = len(train_dataloader)
    total_val_batches = len(val_dataloader)
    print(
        f'[TRAIN] Starting training: {num_epochs} epochs, {total_train_batches} train batches, '
        f'{total_val_batches} val batches × {val_n_passes} pass(es), '
        f'smooth_window={val_smooth_window}',
        flush=True,
    )

    # Best checkpoint: keep only a CPU state_dict snapshot, never a second live copy
    # of the model on GPU. deepcopy(policy) doubled VRAM for a 4B-param model and
    # was the main source of mid-training OOMs.
    best_state_dict_cpu = None
    best_epoch = -1

    def _swap_in_ema():
        """Move live trainable params to backup, load EMA into model. Returns backup dict."""
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

    # Main training loop
    start_time = time.time()
    for epoch in range(num_epochs):

        # --- Validation Step ---
        # NOTE: Keep policy in train mode during validation forward pass.
        # New lerobot ACT's VAE encoder only runs in training mode;
        # eval mode returns None for mu/log_sigma, causing KLD calculation to fail.
        live_backup = _swap_in_ema()  # Validate against EMA weights (smoother)
        with torch.inference_mode():
            val_loss_sum = 0.0
            val_batch_count = 0
            # Multi-pass validation: each pass re-iterates the dataloader, drawing
            # fresh random chunks per episode. N passes ≈ N× more samples evaluated
            # → reduces per-epoch val_loss variance proportional to 1/sqrt(N).
            for pass_idx in range(max(val_n_passes, 1)):
                for batch_idx, data in enumerate(val_dataloader):
                    with torch.autocast('cuda', dtype=amp_dtype, enabled=use_amp):
                        loss, _ = forward_pass(data, policy, norm_stats=stats, preprocessor=preprocessor)
                    val_loss_sum += loss.item()
                    val_batch_count += 1
                    if pass_idx == 0 and (
                        (batch_idx + 1) % max(1, total_val_batches // 5) == 0
                        or batch_idx == total_val_batches - 1
                    ):
                        print(
                            f'  [VAL] epoch {epoch}/{num_epochs} pass {pass_idx+1}/{val_n_passes} '
                            f'batch {batch_idx+1}/{total_val_batches} loss={loss.item():.4f}',
                            flush=True,
                        )

            epoch_val_loss = val_loss_sum / max(val_batch_count, 1)
            validation_history.append(epoch_val_loss)

            # Smoothed best detection. For early epochs (buffer not full), fall back
            # to raw val_loss so we always have *some* best snapshot even if training
            # ends before val_smooth_window epochs.
            recent_val_losses.append(epoch_val_loss)
            is_best = False
            if len(recent_val_losses) >= val_smooth_window:
                smoothed = sum(recent_val_losses[-val_smooth_window:]) / val_smooth_window
                if smoothed < min_smoothed_val:
                    min_smoothed_val = smoothed
                    min_val_loss = smoothed  # report smoothed as the "min"
                    best_epoch = epoch
                    is_best = True
            elif epoch_val_loss < min_val_loss:
                # Pre-window fallback. Once buffer fills, smoothed criterion takes over
                # and may move best_epoch forward — that's intended.
                min_val_loss = epoch_val_loss
                best_epoch = epoch
                is_best = True

            if is_best:
                # Snapshot the EMA-loaded weights (currently in policy since we swapped
                # before validation). Filter by requires_grad to pick up both LoRA
                # adapters AND manually-unfrozen full-FT modules — earlier code that
                # filtered by 'lora' substring missed the full-FT modules and produced
                # frankenstein checkpoints (best LoRA + last full-FT).
                if use_peft:
                    trainable_names = {n for n, p in policy.named_parameters() if p.requires_grad}
                    src = {k: v for k, v in policy.state_dict().items() if k in trainable_names}
                else:
                    src = policy.state_dict()
                best_state_dict_cpu = {k: v.detach().to('cpu', copy=True) for k, v in src.items()}

        _restore_live(live_backup)
        live_backup = None

        # Release val-phase activations/cached allocator blocks before train phase
        # so the caching allocator can reclaim memory instead of fragmenting.
        torch.cuda.empty_cache()

        # --- Training Step ---
        policy.train() # Set model to training mode
        epoch_start = time.time()
        optimizer.zero_grad()
        grad_clip_norm = float(getattr(cfg, 'optimizer_grad_clip_norm', 0.0) or 0.0)
        trainable_params = [p for p in policy.parameters() if p.requires_grad]
        for batch_idx, data in enumerate(train_dataloader):

            with torch.autocast('cuda', dtype=amp_dtype, enabled=use_amp):
                loss, _ = forward_pass(data, policy, norm_stats=stats, preprocessor=preprocessor)

            # Gradient accumulation: scale loss and delay optimizer step
            if grad_accum_steps > 1:
                loss = loss / grad_accum_steps
            loss.backward()

            if (batch_idx + 1) % grad_accum_steps == 0 or (batch_idx + 1) == total_train_batches:
                # Gradient clipping prevents occasional spikes (VLA training is noisy
                # with small batch sizes; without clipping a single bad batch can
                # kick the model into a bad region and drive loss up).
                if grad_clip_norm > 0:
                    torch.nn.utils.clip_grad_norm_(trainable_params, grad_clip_norm)
                optimizer.step()
                scheduler.step()  # advance warmup/cosine schedule per optimizer step
                optimizer.zero_grad()
                # EMA update: ema = decay * ema + (1-decay) * live. Per optimizer step
                # (after grad_accum boundary) so update frequency tracks effective batch.
                if use_ema:
                    with torch.no_grad():
                        for n, p in policy.named_parameters():
                            if n in ema_state:
                                ema_state[n].mul_(ema_decay).add_(p.data, alpha=1.0 - ema_decay)

            train_history.append({
                'loss': loss.item() * (grad_accum_steps if grad_accum_steps > 1 else 1),
                'lr': optimizer.param_groups[0]['lr'],
            })

            # Log every ~20% of batches (at least every batch for small datasets)
            if (batch_idx + 1) % max(1, total_train_batches // 5) == 0 or batch_idx == total_train_batches - 1:
                elapsed = time.time() - epoch_start
                batches_per_sec = (batch_idx + 1) / elapsed if elapsed > 0 else 0
                print(
                    f'  [BATCH] epoch {epoch}/{num_epochs} batch {batch_idx+1}/{total_train_batches} '
                    f'loss={train_history[-1]["loss"]:.4f} lr={train_history[-1]["lr"]:.2e} '
                    f'({batches_per_sec:.2f} batch/s)',
                    flush=True,
                )

        epoch_summary = compute_dict_mean(train_history[(batch_idx+1)*epoch:(batch_idx+1)*(epoch+1)])
        train_time_sec = time.time() - start_time
        train_log = {
            'epoch': epoch,
            'total_epoch': num_epochs,
            'val_loss': epoch_val_loss,
            'train_loss': epoch_summary["loss"],
            'train_time_sec': train_time_sec,
        }
        print(f"[TRAIN_LOG] {json.dumps(train_log)}", flush=True)

        
    # save dataset stats
    ckpt_dir = f"/root/src/backend/checkpoints/{checkpoint_obj['id']}"

    # Load the best-validation weights back into the live policy. This avoids
    # keeping a second GPU copy around (the old deepcopy path was the main OOM
    # source). The CPU snapshot only contains trainable weights, so we use
    # strict=False; base-model weights already match between snapshot-time and now
    # since they're frozen when use_peft / train_expert_only is enabled.
    # Snapshot was taken with EMA weights loaded (we swap before validation), so
    # this restores the smoothed weights at the smoothed-best epoch.
    if best_state_dict_cpu is not None:
        policy.load_state_dict(best_state_dict_cpu, strict=False)
        # Free CPU snapshot memory before the merge/save burst
        best_state_dict_cpu = None
    elif use_ema:
        # No best snapshot was taken (e.g., num_epochs < val_smooth_window and no
        # raw fallback hit either). Save the final EMA weights as the next-best
        # option — they're at least the smoothed end-of-training state, never
        # worse than the live final-epoch weights.
        with torch.no_grad():
            for n, p in policy.named_parameters():
                if n in ema_state:
                    p.data.copy_(ema_state[n])
        print(f'[TRAIN] No best snapshot taken — saving final EMA weights instead.', flush=True)
    # Free EMA buffer before merge/save burst
    if use_ema:
        ema_state.clear()

    # PEFT: merge adapter weights into base model before saving
    # This produces a standard checkpoint that loads without peft dependency
    best_policy = policy
    if use_peft:
        best_policy = best_policy.merge_and_unload()

    best_policy.save_pretrained(ckpt_dir)

    # draccus.dump doesn't serialize the 'type' property, but from_pretrained needs it
    config_path = os.path.join(ckpt_dir, 'config.json')
    with open(config_path, 'r') as f:
        saved_config = json.load(f)
    if 'type' not in saved_config:
        saved_config['type'] = best_policy.config.type
        with open(config_path, 'w') as f:
            json.dump(saved_config, f, indent=4)

    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'wb') as f:
        pickle.dump(stats, f)

    # Save processor pipelines so checkpoint_test.py can rebuild the same
    # Normalize/Unnormalize transforms via make_pre_post_processors(pretrained_path=ckpt_dir).
    preprocessor.save_pretrained(
        ckpt_dir, config_filename=f"{POLICY_PREPROCESSOR_DEFAULT_NAME}.json"
    )
    postprocessor.save_pretrained(
        ckpt_dir, config_filename=f"{POLICY_POSTPROCESSOR_DEFAULT_NAME}.json"
    )

    # torch.save(best_state_dict, ckpt_path)

    print(f'Training finished:\nSeed {seed}, val loss {min_val_loss:.6f} at epoch {best_epoch}')

    # Preserve the (epoch, val_loss, _) tuple interface expected by main().
    # The third slot used to be a deepcopy'd policy but is no longer consumed.
    return best_epoch, min_val_loss, None


def main(args):
    """Main execution function to load configs and start training."""
    
    # Setup database connection using Peewee
    peewee_db.connect(reuse_if_open=True)
    
    # Create a temporary directory to store dataset files
    temp_dir = os.path.join(DATASET_DIR, "tmp")
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
    os.makedirs(temp_dir)
    
    try:
        # Fetch configurations from the database
        # task = Task.find(args.task_id).to_dict()
        # policy = Policy.find(args.policy_id).to_dict()
        checkpoint = Checkpoint.find(args.checkpoint_id)
        checkpoint.update({'status': 'training'})
        task = Task.find(checkpoint.task_id).to_dict()
        policy = Policy.find(checkpoint.policy_id).to_dict()
        load_model = Checkpoint.find(checkpoint.load_model_id).to_dict() if checkpoint.load_model_id else None
        # load_model = Checkpoint.find(args.load_model_id).to_dict() if args.load_model_id else None

        # Merge LeRobot datasets into temp directory
        from ..api.process.lerobot_io import (
            list_episodes, get_dataset_info, _read_json, _write_json,
            _read_jsonl, _write_jsonl, _append_jsonl,
            PARQUET_PATH_TEMPLATE, IMAGE_PATH_TEMPLATE, INFO_PATH,
            EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH, DEFAULT_CHUNK_SIZE,
        )
        import pyarrow.parquet as pq
        import pyarrow as pa

        dataset_ids = checkpoint.dataset_info.keys()
        episode_counter = 0

        # Initialize temp dataset from first source
        first_ds_id = list(dataset_ids)[0]
        first_ds_path = os.path.join(DATASET_DIR, str(first_ds_id))
        first_info = get_dataset_info(first_ds_path)
        if first_info:
            tmp_info = dict(first_info)
            tmp_info["total_episodes"] = 0
            tmp_info["total_frames"] = 0
            tmp_info["total_chunks"] = 0
            tmp_info["total_tasks"] = 0
            tmp_info["splits"] = {}
            _write_json(tmp_info, os.path.join(temp_dir, INFO_PATH))
            for p in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
                fp = os.path.join(temp_dir, p)
                os.makedirs(os.path.dirname(fp), exist_ok=True)
                open(fp, "w").close()
            # Copy tasks from first source
            src_tasks = _read_jsonl(os.path.join(first_ds_path, TASKS_PATH))
            if src_tasks:
                _write_jsonl(src_tasks, os.path.join(temp_dir, TASKS_PATH))

        for ds_id in dataset_ids:
            dataset_path = os.path.join(DATASET_DIR, str(ds_id))
            ds_info = get_dataset_info(dataset_path)
            if ds_info is None:
                continue
            episodes = list_episodes(dataset_path)

            for ep_entry in episodes:
                ep_idx = ep_entry["episode_index"]
                chunk = ep_idx // ds_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
                src_parquet = os.path.join(dataset_path, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=ep_idx))
                if not os.path.exists(src_parquet):
                    continue

                table = pq.read_table(src_parquet)
                df = table.to_pandas()
                num_frames = len(df)

                tmp_info = _read_json(os.path.join(temp_dir, INFO_PATH))
                global_start = tmp_info["total_frames"]
                new_chunk = episode_counter // tmp_info.get("chunks_size", DEFAULT_CHUNK_SIZE)

                df["index"] = np.arange(global_start, global_start + num_frames, dtype=np.int64)
                df["episode_index"] = episode_counter

                # Copy videos (MP4) or images (legacy PNG)
                features = ds_info.get("features", {})
                for feat_key, feat in features.items():
                    if not feat_key.startswith("observation.images."):
                        continue
                    if feat.get("dtype") == "video":
                        # Copy MP4 video file
                        src_video = os.path.join(
                            dataset_path, "videos", f"chunk-{chunk:03d}", feat_key,
                            f"episode_{ep_idx:06d}.mp4"
                        )
                        dst_video = os.path.join(
                            temp_dir, "videos", f"chunk-{new_chunk:03d}", feat_key,
                            f"episode_{episode_counter:06d}.mp4"
                        )
                        os.makedirs(os.path.dirname(dst_video), exist_ok=True)
                        if os.path.exists(src_video):
                            shutil.copy2(src_video, dst_video)
                            # Pre-decode MP4 to mmap-friendly .npy beside it so the
                            # dataloader can numpy-slice frames instead of re-decoding
                            # the entire video on every epoch.
                            from ..api.process.lerobot_io import _decode_video_frames
                            frames = _decode_video_frames(dst_video)
                            if frames:
                                arr = np.stack(frames)  # (T, H, W, 3) uint8
                                np.save(dst_video[:-4] + ".npy", arr)
                    elif feat.get("dtype") == "image" and feat_key in df.columns:
                        # Legacy: copy embedded image paths
                        sensor_name = feat_key.replace("observation.images.", "")
                        s_id = sensor_name.replace("sensor_", "")
                        new_paths = []
                        for frame_idx, old_rel_path in enumerate(df[feat_key]):
                            old_abs = os.path.join(dataset_path, old_rel_path)
                            new_rel = IMAGE_PATH_TEMPLATE.format(sid=s_id, ep=episode_counter, frame=frame_idx)
                            new_abs = os.path.join(temp_dir, new_rel)
                            os.makedirs(os.path.dirname(new_abs), exist_ok=True)
                            if os.path.exists(old_abs):
                                shutil.copy2(old_abs, new_abs)
                            new_paths.append(new_rel)
                        df[feat_key] = new_paths

                dest_parquet = os.path.join(temp_dir, PARQUET_PATH_TEMPLATE.format(chunk=new_chunk, ep=episode_counter))
                os.makedirs(os.path.dirname(dest_parquet), exist_ok=True)
                pq.write_table(pa.Table.from_pandas(df), dest_parquet)

                # Copy episode stats
                src_stats = _read_jsonl(os.path.join(dataset_path, EPISODES_STATS_PATH))
                ep_stats = {}
                for s in src_stats:
                    if s.get("episode_index") == ep_idx:
                        ep_stats = s.get("stats", {})
                        break

                tmp_info["total_episodes"] = episode_counter + 1
                tmp_info["total_frames"] = global_start + num_frames
                tmp_info["total_chunks"] = new_chunk + 1
                tmp_info["splits"] = {"train": f"0:{episode_counter + 1}"}
                _write_json(tmp_info, os.path.join(temp_dir, INFO_PATH))

                _append_jsonl(
                    {"episode_index": episode_counter, "length": num_frames, "tasks": ep_entry.get("tasks", [""])},
                    os.path.join(temp_dir, EPISODES_PATH),
                )
                _append_jsonl(
                    {"episode_index": episode_counter, "stats": ep_stats},
                    os.path.join(temp_dir, EPISODES_STATS_PATH),
                )

                episode_counter += 1
                print(f"Copied episode {ep_idx} from dataset {ds_id} -> episode_{episode_counter - 1}")

        checkpoint = checkpoint.to_dict()

        batch_size = checkpoint['train_settings']['batch_size']
        action_key = policy['settings'].get('action_type') or checkpoint['train_settings'].get('action_type', 'qaction')
        _obs_keys = policy['settings'].get('obs_state_keys')
        if _obs_keys is None:
            _obs_keys = checkpoint['train_settings'].get('obs_state_keys')
        obs_state_keys = _obs_keys if _obs_keys is not None else ['qpos']
        use_relative_trajectory = checkpoint['train_settings'].get('use_relative_trajectory', False)
        # PI05 'use_relative_actions' lives in policy settings (not train settings).
        # The preprocessor transforms action→delta BEFORE normalize, so stats must be
        # computed on the same delta form — otherwise normalization collapses learning.
        use_relative_actions = policy['settings'].get('use_relative_actions', False)
        # Mask configuration for which dims become delta. Priority: explicit mask >
        # absolute_dims > joints_dim. All optional — defaults preserve old behavior.
        relative_joints_dim = policy['settings'].get('relative_joints_dim')
        relative_action_mask = policy['settings'].get('relative_action_mask')
        absolute_action_dims = policy['settings'].get('absolute_action_dims')
        # task settings에 설정된 센서만 학습에 사용
        sensor_settings = task.get('settings', {}).get('sensors', {})
        if sensor_settings:
            sensor_ids = [sid for sid in task['sensor_ids'] if str(sid) in sensor_settings]
        else:
            sensor_ids = task['sensor_ids']
        if policy['type'] in ['ACT']:
            chunk_size = policy['settings']['chunk_size']
            vision_backbone = policy['settings']['vision_backbone']
        elif policy['type'] in ['Diffusion']:
            chunk_size = policy['settings']['horizon']
            vision_backbone = policy['settings']['vision_backbone']
        elif policy['type'] in ['PI05']:
            chunk_size = policy['settings']['chunk_size']
            vision_backbone = None
        num_workers = checkpoint['train_settings']['num_workers']
        n_obs_steps = policy['settings']['n_obs_steps']  # Default to 1 if not specified


        # succeed 플래그 자동 감지: 첫 번째 에피소드의 parquet에서 확인
        first_parquet = os.path.join(temp_dir, PARQUET_PATH_TEMPLATE.format(chunk=0, ep=0))
        if os.path.exists(first_parquet):
            _first_table = pq.read_table(first_parquet)
            _has_succeed = 'succeed' in _first_table.column_names
            if _has_succeed:
                ts = checkpoint['train_settings']
                ts['has_succeed'] = True
                Checkpoint.find(args.checkpoint_id).update({'train_settings': ts})
                checkpoint['train_settings']['has_succeed'] = True
                print('[INFO] succeed flag detected in dataset, saved to train_settings.')

        # wrist_sensor_ids: optional list of sensor IDs that are wrist-mounted.
        # openpi skips spatial augmentation (crop+rotate) on wrist cams since they
        # encode end-effector geometry; random crop destroys the pixel↔gripper-pose
        # invariance the model relies on for fine cube approach. EasyTrainer's previous
        # infrastructure (audit-doc bug #24) was never wired through load_data —
        # this is the wiring. Read from train_settings or policy settings (UI).
        # Accept str ("1,3"), list ([1, 3]), or None.
        # Use .pop() (not .get()) to remove the key from the dicts — train() later
        # passes train_settings/policy_settings as **kwargs to PI05Config which would
        # reject this unknown kwarg with TypeError.
        _wrist_raw = (
            checkpoint['train_settings'].pop('wrist_sensor_ids', None)
            or policy['settings'].pop('wrist_sensor_ids', None)
            or None
        )
        if isinstance(_wrist_raw, str):
            _wrist_raw = [int(x.strip()) for x in _wrist_raw.split(',') if x.strip()]
        wrist_sensor_ids = list(_wrist_raw) if _wrist_raw else None
        if wrist_sensor_ids:
            print(f"[CONFIG] wrist_sensor_ids: {wrist_sensor_ids} (skip spatial aug)")

        # Load data from the temporary directory
        print(f"[CONFIG] obs_state_keys: {obs_state_keys}, action_key: {action_key}")
        train_dataloader, val_dataloader, stats, input_features, output_features = load_data(temp_dir, policy['type'], episode_counter, sensor_ids, batch_size, batch_size, chunk_size, vision_backbone, num_workers, n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, use_relative_actions=use_relative_actions, relative_joints_dim=relative_joints_dim, relative_action_mask=relative_action_mask, absolute_action_dims=absolute_action_dims, wrist_sensor_ids=wrist_sensor_ids)
        # Start the training process
        best_epoch, min_val_loss, best_state_dict = train(
            train_dataloader,
            val_dataloader,
            input_features,
            output_features,
            stats,
            policy,
            checkpoint,
            load_model=load_model,
        )

        # train_fiper(checkpoint, task, chunk_size)
        
        Checkpoint.find(args.checkpoint_id).update({
            'status': 'finished',
            'best_epoch': best_epoch,
            'loss': min_val_loss,
        })

        # OOD feature 생성
        try:
            generate_ood_features(checkpoint, policy, task)
        except Exception as ood_e:
            print(f'[WARN] OOD feature generation failed: {ood_e}')

        print("Training process completed successfully.")
        
    except Exception as e:
        import traceback
        error_msg = traceback.format_exc()
        print(f"[ERROR] Training process failed:\n{error_msg}")
        Checkpoint.find(args.checkpoint_id).delete()
        raise e

    finally:
        # Clean up the temporary directory
        shutil.rmtree(temp_dir)
    
    
# Script entry point
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    # parser.add_argument('--task_id', required=True)
    # parser.add_argument('--policy_id', required=True)
    # parser.add_argument('--load_model_id', default=None, required=False)
    parser.add_argument('--checkpoint_id', required=True)
    # parser.add_argument('--dataset_ids', required=True)
    
    # # Add arguments for training parameters
    # # This is a bit of a hack to get all the training parameters from the command line
    # # A better way would be to pass a config file
    # for key, value in ACTConfig.model_fields.items():
    #     if key not in ['input_features', 'output_features']:
    #         parser.add_argument(f'--{key}', type=type(value.default), default=value.default)

    main(parser.parse_args())
    sys.exit(0)
