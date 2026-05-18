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

    num_epochs = train_settings.pop('num_epochs')
    train_settings.pop('batch_size', None)
    train_settings.pop('num_workers', None)
    train_settings.pop('action_type', None)
    train_settings.pop('has_succeed', None)
    train_settings.pop('use_relative_trajectory', None)
    policy_settings.pop('action_type', None)
    policy_settings.pop('obs_state_keys', None)
    # wrist_sensor_ids: read by main() before train(), but pop defensively in case
    # it survives into train_settings (would cause PI05Config to reject as unknown kwarg).
    train_settings.pop('wrist_sensor_ids', None)
    policy_settings.pop('wrist_sensor_ids', None)

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
        est_steps_per_epoch = max(1, len(train_dataloader) // max(1, grad_accum_steps))
        decay_steps = max(warmup_steps + 1, num_epochs * est_steps_per_epoch)

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
    print(f'[TRAIN] Starting training: {num_epochs} epochs, {total_train_batches} train batches, '
          f'{total_val_batches} val batches × {val_n_passes} pass(es), '
          f'smooth_window={val_smooth_window}', flush=True)

    best_state_dict_cpu = None
    best_epoch = -1

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

    start_time = time.time()
    for epoch in range(num_epochs):
        # --- Validation ---
        live_backup = _swap_in_ema()  # validate against EMA weights
        with torch.inference_mode():
            val_loss_sum = 0.0
            val_batch_count = 0
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
                        print(f'  [VAL] epoch {epoch}/{num_epochs} pass {pass_idx+1}/{val_n_passes} '
                              f'batch {batch_idx+1}/{total_val_batches} loss={loss.item():.4f}', flush=True)

            epoch_val_loss = val_loss_sum / max(val_batch_count, 1)
            validation_history.append(epoch_val_loss)
            recent_val_losses.append(epoch_val_loss)

            # Smoothed best detection (window=1 → raw min, preserves ACT/Diffusion behavior).
            is_best = False
            if len(recent_val_losses) >= val_smooth_window:
                smoothed = sum(recent_val_losses[-val_smooth_window:]) / val_smooth_window
                if smoothed < min_smoothed_val:
                    min_smoothed_val = smoothed
                    min_val_loss = smoothed
                    best_epoch = epoch
                    is_best = True
            elif epoch_val_loss < min_val_loss:
                min_val_loss = epoch_val_loss
                best_epoch = epoch
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

        _restore_live(live_backup)
        live_backup = None
        torch.cuda.empty_cache()

        # --- Training ---
        policy.train()
        epoch_start = time.time()
        optimizer.zero_grad()
        grad_clip_norm = float(getattr(cfg, 'optimizer_grad_clip_norm', 0.0) or 0.0)
        trainable_params = [p for p in policy.parameters() if p.requires_grad]
        for batch_idx, data in enumerate(train_dataloader):
            with torch.autocast('cuda', dtype=amp_dtype, enabled=use_amp):
                loss, _ = forward_pass(data, policy, norm_stats=stats, preprocessor=preprocessor)

            if grad_accum_steps > 1:
                loss = loss / grad_accum_steps
            loss.backward()

            if (batch_idx + 1) % grad_accum_steps == 0 or (batch_idx + 1) == total_train_batches:
                if grad_clip_norm > 0:
                    torch.nn.utils.clip_grad_norm_(trainable_params, grad_clip_norm)
                optimizer.step()
                scheduler.step()
                optimizer.zero_grad()
                # EMA update per optimizer step.
                if use_ema:
                    with torch.no_grad():
                        for n, p in policy.named_parameters():
                            if n in ema_state:
                                ema_state[n].mul_(ema_decay).add_(p.data, alpha=1.0 - ema_decay)

            train_history.append({
                'loss': loss.item() * (grad_accum_steps if grad_accum_steps > 1 else 1),
                'lr': optimizer.param_groups[0]['lr'],
            })

            if (batch_idx + 1) % max(1, total_train_batches // 5) == 0 or batch_idx == total_train_batches - 1:
                elapsed = time.time() - epoch_start
                batches_per_sec = (batch_idx + 1) / elapsed if elapsed > 0 else 0
                print(f'  [BATCH] epoch {epoch}/{num_epochs} batch {batch_idx+1}/{total_train_batches} '
                      f'loss={train_history[-1]["loss"]:.4f} lr={train_history[-1]["lr"]:.2e} '
                      f'({batches_per_sec:.2f} batch/s)', flush=True)

        epoch_summary = compute_dict_mean(train_history[(batch_idx+1)*epoch:(batch_idx+1)*(epoch+1)])
        train_time_sec = time.time() - start_time
        train_log = {
            'epoch': epoch, 'total_epoch': num_epochs,
            'val_loss': epoch_val_loss, 'train_loss': epoch_summary["loss"],
            'train_time_sec': train_time_sec,
        }
        print(f"[TRAIN_LOG] {json.dumps(train_log)}", flush=True)

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

    preprocessor.save_pretrained(ckpt_dir, config_filename=f"{POLICY_PREPROCESSOR_DEFAULT_NAME}.json")
    postprocessor.save_pretrained(ckpt_dir, config_filename=f"{POLICY_POSTPROCESSOR_DEFAULT_NAME}.json")

    print(f'Training finished:\nSeed {seed}, val loss {min_val_loss:.6f} at epoch {best_epoch}')
    return best_epoch, min_val_loss


def main(args):
    """Load config from JSON and run training."""
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
    obs_state_keys = policy['settings'].get('obs_state_keys') or train_settings.get('obs_state_keys', ['qpos'])
    use_relative_trajectory = train_settings.get('use_relative_trajectory', False)
    sensor_ids = config.get('sensor_ids', [])

    # PI05 delta-mode 기준 (audit-doc bug #11 fix). stats가 raw action이 아니라
    # chunk-wise delta 분포로 계산돼야 함. ACT/Diffusion은 use_relative_actions
    # 미사용이라 무해.
    use_relative_actions = policy['settings'].get('use_relative_actions', False)
    absolute_action_dims = policy['settings'].get('absolute_action_dims', None)
    relative_action_mask = policy['settings'].get('relative_action_mask', None)
    relative_joints_dim = policy['settings'].get('relative_joints_dim', None)

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

    # Auto-detect sensor_ids from dataset features if not provided
    if not sensor_ids:
        features = info.get('features', {})
        for feat_key in features:
            if feat_key.startswith('observation.images.sensor_'):
                sid = feat_key.replace('observation.images.sensor_', '')
                try:
                    sensor_ids.append(int(sid))
                except ValueError:
                    sensor_ids.append(sid)
        print(f'[INFO] Auto-detected sensor_ids: {sensor_ids}', flush=True)

    print(f"[CONFIG] obs_state_keys: {obs_state_keys}, action_key: {action_key}", flush=True)

    train_dataloader, val_dataloader, stats, input_features, output_features = load_data(
        actual_dataset_dir, policy['type'], num_episodes, sensor_ids,
        batch_size, batch_size, chunk_size, vision_backbone, num_workers,
        n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory,
        obs_state_keys=obs_state_keys,
        use_relative_actions=use_relative_actions,
        relative_joints_dim=relative_joints_dim,
        relative_action_mask=relative_action_mask,
        absolute_action_dims=absolute_action_dims,
        wrist_sensor_ids=wrist_sensor_ids,
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
