"""
Training worker - runs as a subprocess inside the training server.
Reads config from JSON file (no database dependency).
Reuses the same training logic as the local train.py.
"""
import torch
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
    """Training function - same logic as local train.py but without DB."""
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

    use_peft = train_settings.pop('use_peft', False)
    peft_r = train_settings.pop('peft_r', 16)
    peft_alpha = train_settings.pop('peft_alpha', None)
    use_amp = train_settings.pop('use_amp', False)
    grad_accum_steps = train_settings.pop('grad_accum_steps', 1)

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
        if use_amp:
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

    if use_peft:
        print(f'[TRAIN] Applying LoRA (r={peft_r}, alpha={peft_alpha or peft_r})...', flush=True)
        from peft import LoraConfig, get_peft_model
        if policy_obj['type'] == 'PI05':
            peft_target = [
                r".*\.gemma_expert\..*\.self_attn\.(q_proj|v_proj)",
                r"model\.(state_proj|action_in_proj|action_out_proj|action_time_mlp_in|action_time_mlp_out)",
            ]
        else:
            peft_target = 'all-linear'
        lora_config = LoraConfig(r=peft_r, lora_alpha=peft_alpha if peft_alpha else peft_r,
                                 target_modules=peft_target, lora_dropout=0.05)
        policy = get_peft_model(policy, lora_config)
        policy.print_trainable_parameters()
        print(f'[TRAIN] LoRA applied successfully.', flush=True)

    optimizer = torch.optim.Adam(
        filter(lambda p: p.requires_grad, policy.parameters()), lr=cfg.optimizer_lr,
    )

    amp_dtype = torch.bfloat16 if use_amp else None
    print(f'[TRAIN] AMP={use_amp}, grad_accum={grad_accum_steps}', flush=True)

    train_history = []
    validation_history = []
    min_val_loss = np.inf
    best_ckpt_info = None

    total_train_batches = len(train_dataloader)
    total_val_batches = len(val_dataloader)
    print(f'[TRAIN] Starting training: {num_epochs} epochs, {total_train_batches} train batches, {total_val_batches} val batches', flush=True)

    start_time = time.time()
    for epoch in range(num_epochs):
        # Validation
        with torch.inference_mode():
            val_loss_sum = 0
            for batch_idx, data in enumerate(val_dataloader):
                with torch.autocast('cuda', dtype=amp_dtype, enabled=use_amp):
                    loss, _ = forward_pass(data, policy, norm_stats=stats, preprocessor=preprocessor)
                val_loss_sum += loss.item()
                if (batch_idx + 1) % max(1, total_val_batches // 5) == 0 or batch_idx == total_val_batches - 1:
                    print(f'  [VAL] epoch {epoch}/{num_epochs} batch {batch_idx+1}/{total_val_batches} loss={loss.item():.4f}', flush=True)

            epoch_val_loss = val_loss_sum / (batch_idx + 1)
            validation_history.append(epoch_val_loss)

            if epoch_val_loss < min_val_loss:
                min_val_loss = epoch_val_loss
                best_ckpt_info = (epoch, min_val_loss, deepcopy(policy))

        # Training
        policy.train()
        epoch_start = time.time()
        optimizer.zero_grad()
        for batch_idx, data in enumerate(train_dataloader):
            with torch.autocast('cuda', dtype=amp_dtype, enabled=use_amp):
                loss, _ = forward_pass(data, policy, norm_stats=stats, preprocessor=preprocessor)

            if grad_accum_steps > 1:
                loss = loss / grad_accum_steps
            loss.backward()

            if (batch_idx + 1) % grad_accum_steps == 0 or (batch_idx + 1) == total_train_batches:
                optimizer.step()
                optimizer.zero_grad()

            train_history.append({'loss': loss.item() * (grad_accum_steps if grad_accum_steps > 1 else 1)})

            if (batch_idx + 1) % max(1, total_train_batches // 5) == 0 or batch_idx == total_train_batches - 1:
                elapsed = time.time() - epoch_start
                batches_per_sec = (batch_idx + 1) / elapsed if elapsed > 0 else 0
                print(f'  [BATCH] epoch {epoch}/{num_epochs} batch {batch_idx+1}/{total_train_batches} loss={train_history[-1]["loss"]:.4f} ({batches_per_sec:.2f} batch/s)', flush=True)

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

    # Save best checkpoint
    ckpt_dir = checkpoint_dir
    best_epoch, min_val_loss, best_policy = best_ckpt_info

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

    # Detect succeed flag
    import pyarrow.parquet as pq
    PARQUET_PATH_TEMPLATE = "parquet/chunk-{chunk:03d}/episode_{ep:06d}.parquet"
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
