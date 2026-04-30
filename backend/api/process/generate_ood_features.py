import os
import shutil
import torch
import numpy as np

from lerobot.policies.act.modeling_act import ACTPolicy
from ...policies.utils import forward_pass, FullScanDataset, get_norm_stats
from ...configs.global_configs import DATASET_DIR, get_checkpoint_dir
from torch.utils.data import DataLoader


def generate_ood_features(checkpoint, policy_obj, task, task_control=None):
    """기존 체크포인트에서 모델을 로드하고, 학습 데이터를 순회하여 OOD reference feature를 생성."""
    ckpt_dir = get_checkpoint_dir(checkpoint['id'])

    # 모델 로드
    policy = ACTPolicy.from_pretrained(ckpt_dir)
    policy.cuda()
    policy.eval()
    print(f'[OOD] Loaded model from {ckpt_dir}')

    if not hasattr(policy, 'enable_feature_caching'):
        print('[OOD] Policy does not support feature caching, skipping.')
        return

    # 데이터셋 준비 (train.py와 동일한 방식)
    temp_dir = os.path.join(DATASET_DIR, 'tmp_ood')
    os.makedirs(temp_dir, exist_ok=True)

    try:
        dataset_ids = checkpoint.get('dataset_info', {}).keys()
        episode_counter = 0
        for ds_id in dataset_ids:
            dataset_path = os.path.join(DATASET_DIR, str(ds_id))
            if not os.path.exists(dataset_path):
                continue
            episode_files = [f for f in os.listdir(dataset_path) if f.startswith('episode_') and os.path.isfile(os.path.join(dataset_path, f))]
            for ep_file in episode_files:
                src = os.path.join(dataset_path, ep_file)
                dst = os.path.join(temp_dir, f'episode_{episode_counter}.hdf5')
                shutil.copy(src, dst)
                episode_counter += 1

        if episode_counter == 0:
            print('[OOD] No episodes found, skipping.')
            return

        policy_settings = policy_obj.get('settings', {})
        action_key = policy_settings.get('action_type') or checkpoint.get('train_settings', {}).get('action_type', 'qaction')
        _obs_keys = policy_settings.get('obs_state_keys')
        if _obs_keys is None:
            _obs_keys = checkpoint.get('train_settings', {}).get('obs_state_keys')
        obs_state_keys = _obs_keys if _obs_keys is not None else ['qpos']
        use_relative_trajectory = checkpoint.get('train_settings', {}).get('use_relative_trajectory', False)
        sensor_ids = task.get('sensor_ids', [])
        sensor_settings = task.get('settings', {}).get('sensors', {})
        if sensor_settings:
            sensor_ids = [sid for sid in sensor_ids if str(sid) in sensor_settings]

        policy_type = policy_obj['type']
        if policy_type in ['ACT']:
            chunk_size = policy_settings.get('chunk_size', 15)
            vision_backbone = policy_settings.get('vision_backbone', 'resnet18')
        elif policy_type in ['Diffusion']:
            chunk_size = policy_settings.get('horizon', 16)
            vision_backbone = policy_settings.get('vision_backbone', 'resnet18')
        elif policy_type in ['PI0']:
            chunk_size = policy_settings.get('chunk_size', 50)
            vision_backbone = None
        else:
            chunk_size = 15
            vision_backbone = 'resnet18'

        n_obs_steps = policy_settings.get('n_obs_steps', 1)
        batch_size = checkpoint.get('train_settings', {}).get('batch_size', 32)

        # norm_stats 계산
        norm_stats, skipped = get_norm_stats(
            temp_dir, episode_counter, action_key=action_key,
            use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys,
        )

        # 전체 스텝을 순차 순회하는 FullScanDataset 사용
        all_indices = np.array([i for i in range(episode_counter) if i not in skipped])
        dataset = FullScanDataset(
            all_indices, temp_dir, sensor_ids, norm_stats, chunk_size, policy_type,
            vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key,
            use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys,
        )
        dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=0)

        # Feature 수집
        print(f'[OOD] Collecting features from {len(dataset)} samples...')
        policy.enable_feature_caching(True)
        all_image_feats = []
        all_state_feats = []
        with torch.no_grad():
            for batch_idx, data in enumerate(dataloader):
                forward_pass(data, policy)
                img_feat, state_feat = policy.get_cached_features()
                if img_feat is not None:
                    all_image_feats.append(img_feat.cpu())
                if state_feat is not None:
                    all_state_feats.append(state_feat.cpu())
                if (batch_idx + 1) % 50 == 0:
                    print(f'[OOD] Batch {batch_idx + 1}/{len(dataloader)} done')
        policy.enable_feature_caching(False)

        ood_data = {}
        k = 5
        if all_image_feats:
            img_feats = torch.cat(all_image_feats, dim=0)
            ood_data['image_features'] = img_feats.numpy()
            img_dists = torch.cdist(img_feats, img_feats)
            img_dists.fill_diagonal_(float('inf'))
            img_knn = img_dists.topk(k, largest=False).values.mean(dim=1)
            ood_data['image_dist_mean'] = np.array([img_knn.mean().item()])
            ood_data['image_dist_std'] = np.array([img_knn.std().item()])
            ood_data['image_dist_sorted'] = np.sort(img_knn.numpy())
            print(f'[OOD] Image stats: mean={img_knn.mean():.4f}, std={img_knn.std():.4f}')
        if all_state_feats:
            state_feats = torch.cat(all_state_feats, dim=0)
            ood_data['state_features'] = state_feats.numpy()
            state_dists = torch.cdist(state_feats, state_feats)
            state_dists.fill_diagonal_(float('inf'))
            state_knn = state_dists.topk(k, largest=False).values.mean(dim=1)
            ood_data['state_dist_mean'] = np.array([state_knn.mean().item()])
            ood_data['state_dist_std'] = np.array([state_knn.std().item()])
            ood_data['state_dist_sorted'] = np.sort(state_knn.numpy())
            print(f'[OOD] State stats: mean={state_knn.mean():.4f}, std={state_knn.std():.4f}')

        if ood_data:
            np.savez(os.path.join(ckpt_dir, 'ood_features.npz'), **ood_data)
            print(f'[OOD] Features saved: image={ood_data.get("image_features", np.array([])).shape}, state={ood_data.get("state_features", np.array([])).shape}')
        else:
            print('[OOD] No features collected.')

    except Exception as e:
        import traceback
        print(f'[OOD ERROR] {traceback.format_exc()}')

    finally:
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir)

    # GPU 메모리 정리
    del policy
    import gc
    gc.collect()
    torch.cuda.empty_cache()
    print('[OOD] Done.')
