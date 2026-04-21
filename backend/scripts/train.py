import torch

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
from orator import DatabaseManager
from orator.orm import Model
from ..database.models.robot_model import Robot
from ..database.models.policy_model import Policy
from ..database.models.task_model import Task
from ..database.models.gripper_model import Gripper
from ..database.models.sensor_model import Sensor
from ..database.models.checkpoint_model import Checkpoint
from ..database.config.database import DATABASES

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
    load_model_path = f"/root/backend/checkpoints/{load_model['id']}" if load_model else None

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

    # PEFT/LoRA settings (pop before passing to policy config)
    use_peft = train_settings.pop('use_peft', False)
    peft_r = train_settings.pop('peft_r', 16)
    peft_alpha = train_settings.pop('peft_alpha', None)
    # Mixed precision & gradient accumulation settings
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
        # config.dtype='bfloat16' creates model in bf16 (~5GB instead of ~10GB)
        if use_amp:
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
        # PI05/PI0: only target Gemma expert attention + action projections (model is too large for all-linear)
        # ACT/Diffusion: all linear layers are fine (smaller models)
        if policy_obj['type'] == 'PI05':
            peft_target = [
                r".*\.gemma_expert\..*\.self_attn\.(q_proj|v_proj)",
                r"model\.(state_proj|action_in_proj|action_out_proj|action_time_mlp_in|action_time_mlp_out)",
            ]
        else:
            peft_target = 'all-linear'

        lora_config = LoraConfig(
            r=peft_r,
            lora_alpha=peft_alpha if peft_alpha else peft_r,
            target_modules=peft_target,
            lora_dropout=0.05,
        )
        policy = get_peft_model(policy, lora_config)
        policy.print_trainable_parameters()
        print(f'[TRAIN] LoRA applied successfully.', flush=True)
        _log_gpu_mem('after LoRA')

    # Only optimize trainable params (with PEFT, base model is frozen → saves optimizer memory)
    optimizer = torch.optim.Adam(
        filter(lambda p: p.requires_grad, policy.parameters()),
        lr=cfg.optimizer_lr,
    )
    _log_gpu_mem('after optimizer creation')

    # Mixed precision: autocast dtype for forward pass
    # PI05 handles its own dtype internally, so autocast is mainly for ACT/Diffusion
    amp_dtype = torch.bfloat16 if use_amp else None
    print(f'[TRAIN] AMP={use_amp}, grad_accum={grad_accum_steps}', flush=True)

    train_history = []
    validation_history = []
    min_val_loss = np.inf
    best_ckpt_info = None

    # Count total batches for progress logging
    total_train_batches = len(train_dataloader)
    total_val_batches = len(val_dataloader)
    print(f'[TRAIN] Starting training: {num_epochs} epochs, {total_train_batches} train batches, {total_val_batches} val batches', flush=True)

    # Main training loop
    start_time = time.time()
    for epoch in range(num_epochs):

        # --- Validation Step ---
        # NOTE: Keep policy in train mode during validation forward pass.
        # New lerobot ACT's VAE encoder only runs in training mode;
        # eval mode returns None for mu/log_sigma, causing KLD calculation to fail.
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

        # --- Training Step ---
        policy.train() # Set model to training mode
        epoch_start = time.time()
        optimizer.zero_grad()
        for batch_idx, data in enumerate(train_dataloader):

            with torch.autocast('cuda', dtype=amp_dtype, enabled=use_amp):
                loss, _ = forward_pass(data, policy, norm_stats=stats, preprocessor=preprocessor)

            # Gradient accumulation: scale loss and delay optimizer step
            if grad_accum_steps > 1:
                loss = loss / grad_accum_steps
            loss.backward()

            if (batch_idx + 1) % grad_accum_steps == 0 or (batch_idx + 1) == total_train_batches:
                optimizer.step()
                optimizer.zero_grad()

            train_history.append({'loss': loss.item() * (grad_accum_steps if grad_accum_steps > 1 else 1)})

            # Log every ~20% of batches (at least every batch for small datasets)
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

        
    # save dataset stats
    ckpt_dir = f"/root/backend/checkpoints/{checkpoint_obj['id']}"

    # Save the best checkpoint after training is finished
    best_epoch, min_val_loss, best_policy = best_ckpt_info

    # PEFT: merge adapter weights into base model before saving
    # This produces a standard checkpoint that loads without peft dependency
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

    return best_ckpt_info


def main(args):
    """Main execution function to load configs and start training."""
    
    # Setup database connection using Orator
    db = DatabaseManager(DATABASES)
    Model.set_connection_resolver(db)
    
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

        # Load data from the temporary directory
        print(f"[CONFIG] obs_state_keys: {obs_state_keys}, action_key: {action_key}")
        train_dataloader, val_dataloader, stats, input_features, output_features = load_data(temp_dir, policy['type'], episode_counter, sensor_ids, batch_size, batch_size, chunk_size, vision_backbone, num_workers, n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys)
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
