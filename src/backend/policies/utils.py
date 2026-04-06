from .policies import ACTPolicy, CNNMLPPolicy
from einops import rearrange
import cv2
import numpy as np
import torch
import os
from torch.utils.data import TensorDataset, DataLoader
import IPython
import numpy as np
import json
from types import SimpleNamespace
from ..lerobot.configs.types import PolicyFeature, FeatureType
from torchvision import transforms
from transformers import AutoImageProcessor
from PIL import Image
from scipy.spatial.transform import Rotation
import pyarrow.parquet as pq
from ..api.process.lerobot_io import (
    read_episode, list_episodes, get_dataset_info, get_norm_stats_from_dataset,
    _parse_image_value,
    PARQUET_PATH_TEMPLATE, IMAGE_PATH_TEMPLATE, DEFAULT_CHUNK_SIZE,
)


def delta_to_relative_trajectory(deltas: np.ndarray) -> np.ndarray:
    """UMI 방식의 relative trajectory 라벨로 변환.

    deltas: [T, D] sequential ee_delta. 앞 6차원은 [dx, dy, dz, dax, day, daz],
            7차원 이후는 tool joint (absolute 값)으로 변환 없이 그대로 유지.
    반환값: [T, D] 각 row i = 현재 위치 기준 i+1 step 후의 누적 displacement + tool

    UMI 방식: action[i] = T_now→t+i+1 (현재 pose 기준 절대 상대 위치)
    모든 waypoint가 동일한 기준점(현재 EE)에서 독립적으로 계산되므로
    오차가 누적되지 않음.
    """
    T = len(deltas)
    relative = np.zeros_like(deltas)

    # Translation: 단순 누적합
    relative[:, :3] = np.cumsum(deltas[:, :3], axis=0)

    # Rotation (axis-angle): proper 회전 합성
    cumulative = Rotation.identity()
    for i in range(T):
        cumulative = cumulative * Rotation.from_rotvec(deltas[i, 3:6])
        relative[i, 3:6] = cumulative.as_rotvec()

    # Tool 차원 (6 이후): absolute 값이므로 변환 없이 그대로 복사
    if deltas.shape[1] > 6:
        relative[:, 6:] = deltas[:, 6:]

    return relative


def relative_trajectory_to_delta(waypoints: np.ndarray) -> np.ndarray:
    """relative trajectory → sequential delta 역변환 (inference 시 사용).

    waypoints: [T, D] relative trajectory. 앞 6차원은 (T_now→t+i),
               7차원 이후는 tool joint (absolute 값)으로 변환 없이 그대로 유지.
    반환값: [T, D] sequential deltas + tool
    """
    T = len(waypoints)
    deltas = np.zeros_like(waypoints)

    # Translation: 연속 차분
    deltas[0, :3] = waypoints[0, :3]
    deltas[1:, :3] = np.diff(waypoints[:, :3], axis=0)

    # Rotation: 연속 회전 차분
    deltas[0, 3:6] = waypoints[0, 3:6]
    for i in range(1, T):
        r_prev = Rotation.from_rotvec(waypoints[i - 1, 3:6])
        r_curr = Rotation.from_rotvec(waypoints[i, 3:6])
        deltas[i, 3:6] = (r_prev.inv() * r_curr).as_rotvec()

    # Tool 차원: 그대로 복사
    if waypoints.shape[1] > 6:
        deltas[:, 6:] = waypoints[:, 6:]

    return deltas




e = IPython.embed


class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone='resnet18', n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None):
        super(EpisodicDataset).__init__()
        self.episode_ids = episode_ids
        self.dataset_dir = dataset_dir
        self.sensor_ids = sensor_ids
        self.norm_stats = norm_stats
        self.chunk_size = chunk_size
        self.n_obs_steps = n_obs_steps
        self.policy_type = policy_type
        self.vision_backbone = vision_backbone
        self.action_key = action_key
        self.use_relative_trajectory = use_relative_trajectory
        self.obs_state_keys = obs_state_keys if obs_state_keys is not None else ['qpos']
        self.info = None

        # Pre-load episode metadata (lengths) for efficient sampling
        self._ep_cache = {}
        self._dataset_info = get_dataset_info(dataset_dir)

        self.__getitem__(0) # initialize self.info

    def __len__(self):
        return len(self.episode_ids)

    def _load_episode_parquet(self, episode_id):
        """Load parquet data for an episode, with caching."""
        if episode_id in self._ep_cache:
            return self._ep_cache[episode_id]

        chunk = episode_id // self._dataset_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
        parquet_path = os.path.join(
            self.dataset_dir,
            PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_id),
        )
        table = pq.read_table(parquet_path)
        df = table.to_pandas()

        state_data = np.array(df["observation.state"].tolist(), dtype=np.float32)
        action_data = np.array(df["action"].tolist(), dtype=np.float32)

        # Get language instruction from tasks
        task_index = int(df["task_index"].iloc[0])
        from ..api.process.lerobot_io import _read_jsonl, TASKS_PATH
        tasks = _read_jsonl(os.path.join(self.dataset_dir, TASKS_PATH))
        language_instruction = ""
        for t in tasks:
            if t.get("task_index") == task_index:
                language_instruction = t.get("task", "")
                break

        # Get image data per sensor — from parquet (legacy) or video files
        image_data = {}
        image_cols = [col for col in df.columns if col.startswith("observation.images.")]
        if image_cols:
            # Legacy image mode: images embedded in parquet
            for col in image_cols:
                sensor_name = col.replace("observation.images.", "")
                image_data[sensor_name] = list(df[col])
        else:
            # Video mode: decode frames from MP4 files
            features = self._dataset_info.get("features", {})
            for feat_key, feat in features.items():
                if not feat_key.startswith("observation.images."):
                    continue
                sensor_name = feat_key.replace("observation.images.", "")
                video_path = os.path.join(
                    self.dataset_dir, "videos", f"chunk-{chunk:03d}", feat_key,
                    f"episode_{episode_id:06d}.mp4"
                )
                if os.path.exists(video_path):
                    from ..api.process.lerobot_io import _decode_video_frames
                    image_data[sensor_name] = _decode_video_frames(video_path)
                else:
                    image_data[sensor_name] = []

        succeed = np.array(df["succeed"].tolist(), dtype=np.float32) if "succeed" in df.columns else None

        result = {
            "state_data": state_data,
            "action_data": action_data,
            "language_instruction": language_instruction,
            "image_data": image_data,
            "succeed": succeed,
            "episode_len": len(df),
        }
        # Cache only a limited number
        if len(self._ep_cache) > 50:
            self._ep_cache.pop(next(iter(self._ep_cache)))
        self._ep_cache[episode_id] = result
        return result

    def __getitem__(self, index):
        episode_id = self.episode_ids[index]
        ep = self._load_episode_parquet(episode_id)

        episode_len = ep["episode_len"]
        state_data = ep["state_data"]   # (T, state_dim)
        action_data = ep["action_data"]  # (T, action_dim)
        language_instruction = ep["language_instruction"]

        action_dim = action_data.shape[1]

        # succeed 처리
        expected_action_dim = self.norm_stats['action']['mean'].shape[-1]
        any_has_succeed = (expected_action_dim > action_dim)
        if any_has_succeed:
            action_dim = expected_action_dim

        original_action_shape = (self.chunk_size, action_dim)

        if episode_len <= self.chunk_size + self.n_obs_steps - 1:
            start_ts = self.n_obs_steps - 1
        else:
            start_ts = np.random.choice(np.arange(self.n_obs_steps - 1, episode_len - self.chunk_size))
        end_ts = start_ts + self.chunk_size

        obs_step_start = start_ts - self.n_obs_steps + 1

        # Observation states
        qpos = []
        for i in range(self.n_obs_steps):
            idx = max(0, min(obs_step_start + i, episode_len - 1))
            qpos.append(state_data[idx])

        # Images
        image_dict = {}
        for sensor_id in self.sensor_ids:
            key = f"sensor_{sensor_id}"
            image_dict[key] = []
            img_vals = ep["image_data"].get(key, [])
            for i in range(self.n_obs_steps):
                idx = max(0, min(obs_step_start + i, episode_len - 1))
                if idx < len(img_vals):
                    img_array = _parse_image_value(img_vals[idx], self.dataset_dir)
                else:
                    img_array = np.zeros((224, 224, 3), dtype=np.uint8)
                image_dict[key].append(img_array)

        # Actions
        actual_end = min(episode_len, end_ts)
        action = action_data[start_ts:actual_end]

        if any_has_succeed:
            if ep["succeed"] is not None:
                succeed = ep["succeed"][start_ts:actual_end].reshape(-1, 1)
            else:
                succeed = np.zeros((action.shape[0], 1), dtype=np.float32)
            action = np.concatenate([action, succeed], axis=1)

        action_len = min(self.chunk_size, episode_len - start_ts)

        padded_action = np.zeros(original_action_shape, dtype=np.float32)
        if self.action_key == 'relative_ee_pos' or (self.use_relative_trajectory and self.action_key == 'ee_delta_action'):
            padded_action[:action_len] = delta_to_relative_trajectory(action[:action_len])
        else:
            padded_action[:action_len] = action
        is_pad = np.zeros(self.chunk_size)
        is_pad[action_len:] = 1

        item = dict()
        item['language_instruction'] = language_instruction
        for sensor_id in self.sensor_ids:
            processed_images = []
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                image = process_image(image)
                processed_images.append(image)

            if self.policy_type in ['PI0']:
                item[f"observation.images.sensor_{sensor_id}"] = torch.stack(processed_images).squeeze()
            else:
                item[f"observation.images.sensor_{sensor_id}"] = torch.stack(processed_images)

        if self.policy_type in ['PI0']:
            item["observation.state"] = torch.from_numpy(np.concatenate(qpos)).float()
        else:
            item["observation.state"] = torch.from_numpy(np.array(qpos)).float()

        item["action"] = torch.from_numpy(padded_action).float()
        item["action_is_pad"] = torch.from_numpy(is_pad).bool()
        item['next.done'] = torch.from_numpy(np.zeros(1, dtype=np.bool_)).bool()

        if self.info is None:
            self.info = dict()
            for key, val in item.items():
                if key.startswith("observation.images"):
                    self.info[key] = PolicyFeature(FeatureType.VISUAL, shape=val[0].shape)
                if key == "observation.state":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=val[0].shape)
                if key == "action":
                    self.info[key] = PolicyFeature(FeatureType.ACTION, shape=val[0].shape)
                if key == "language_instruction":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=(1,))

        return item



def process_image(image, vision_backbone='resnet18', to_cuda=False):
    if vision_backbone not in VISION_BACKBONE_MAP:
        tensor_transform = transforms.ToTensor()
        image = tensor_transform(image)
    else:
        image_processor = AutoImageProcessor.from_pretrained(VISION_BACKBONE_MAP[vision_backbone])
        image = image_processor(image)['pixel_values'][0]  # Assuming the image is a PIL Image or numpy array

    return image.cuda() if to_cuda else image  # Add batch dimension


def get_norm_stats(dataset_dir, num_episodes, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None):
    """Compute normalization stats from LeRobot dataset parquet files."""
    if obs_state_keys is None:
        obs_state_keys = ['qpos']

    dataset_info = get_dataset_info(dataset_dir)
    all_qpos_data = []
    all_action_data = []
    observation_image_keys = []
    cnt = 0
    skipped_episodes = []

    # Check if any episode has succeed
    any_has_succeed = False
    expected_action_dim = None

    for episode_idx in range(num_episodes):
        chunk = episode_idx // dataset_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
        parquet_path = os.path.join(dataset_dir, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_idx))
        if not os.path.exists(parquet_path):
            skipped_episodes.append(episode_idx)
            continue

        table = pq.read_table(parquet_path)
        df = table.to_pandas()

        state_data = np.array(df["observation.state"].tolist(), dtype=np.float32)
        action_data = np.array(df["action"].tolist(), dtype=np.float32)

        # Check succeed
        if "succeed" in df.columns:
            any_has_succeed = True

    # Second pass: collect data
    for episode_idx in range(num_episodes):
        if episode_idx in skipped_episodes:
            continue
        chunk = episode_idx // dataset_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
        parquet_path = os.path.join(dataset_dir, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_idx))
        table = pq.read_table(parquet_path)
        df = table.to_pandas()

        qpos = np.array(df["observation.state"].tolist(), dtype=np.float32)
        action = np.array(df["action"].tolist(), dtype=np.float32)

        if qpos.ndim == 1:
            qpos = qpos.reshape(1, -1)
        if action.ndim == 1:
            action = action.reshape(1, -1)

        # succeed 플래그
        if any_has_succeed:
            if "succeed" in df.columns:
                succeed = np.array(df["succeed"].tolist(), dtype=np.float32).reshape(-1, 1)
            else:
                succeed = np.zeros((action.shape[0], 1), dtype=np.float32)
            action = np.concatenate([action, succeed], axis=1)

        if expected_action_dim is None:
            expected_action_dim = action.shape[1]
        elif action.shape[1] != expected_action_dim:
            print(f'[WARN] episode_{episode_idx} action dim {action.shape[1]} != expected {expected_action_dim}, skipping.')
            skipped_episodes.append(episode_idx)
            continue

        if action_key == 'relative_ee_pos' or (use_relative_trajectory and action_key == 'ee_delta_action'):
            action = delta_to_relative_trajectory(action)

        # Collect image keys from parquet columns
        for col in df.columns:
            if col.startswith("observation.images."):
                sensor_name = col.replace("observation.images.", "")
                if sensor_name not in observation_image_keys:
                    observation_image_keys.append(sensor_name)

        cnt += qpos.shape[0]
        all_qpos_data.append(torch.from_numpy(qpos))
        all_action_data.append(torch.from_numpy(action))

    # Also collect image keys from features (video mode: not in parquet columns)
    if dataset_info:
        for feat_key, feat in dataset_info.get("features", {}).items():
            if feat_key.startswith("observation.images.") and feat.get("dtype") in ("video", "image"):
                sensor_name = feat_key.replace("observation.images.", "")
                if sensor_name not in observation_image_keys:
                    observation_image_keys.append(sensor_name)

    if skipped_episodes:
        print(f'[WARN] Skipped incompatible episodes: {skipped_episodes}')

    all_qpos_data = torch.cat(all_qpos_data, dim=0)
    all_action_data = torch.cat(all_action_data, dim=0)

    action_min = all_action_data.min(dim=0)[0]
    action_max = all_action_data.max(dim=0)[0]
    action_mean = all_action_data.mean(dim=0)
    action_std = all_action_data.std(dim=0)
    action_std = torch.clip(action_std, 1e-2, np.inf)

    qpos_min = all_qpos_data.min(dim=0)[0]
    qpos_max = all_qpos_data.max(dim=0)[0]
    qpos_mean = all_qpos_data.mean(dim=0)
    qpos_std = all_qpos_data.std(dim=0)
    qpos_std = torch.clip(qpos_std, 1e-2, np.inf)

    stats = {
        "action": {
            "min": action_min.numpy(),
            "max": action_max.numpy(),
            "mean": action_mean.numpy().squeeze(),
            "std": action_std.numpy().squeeze(),
            "count": np.array([cnt]),
        },
        "observation.state": {
            "min": qpos_min.numpy(),
            "max": qpos_max.numpy(),
            "mean": qpos_mean.numpy().squeeze(),
            "std": qpos_std.numpy().squeeze(),
            "count": np.array([cnt]),
        },
    }

    for key in observation_image_keys:
        stats[f"observation.images.{key}"] = {
            "min": np.array([[[0.0]], [[0.0]], [[0.0]]]),
            "max": np.array([[[1.0]], [[1.0]], [[1.0]]]),
            "mean": np.array([[[0.5]], [[0.5]], [[0.5]]]),
            "std": np.array([[[0.25]], [[0.25]], [[0.25]]]),
            "count": np.array([cnt]),
        }

    return stats, skipped_episodes


class FullScanDataset(EpisodicDataset):
    """OOD feature 수집용: 모든 에피소드의 모든 스텝을 순차적으로 반환."""

    def __init__(self, *args, **kwargs):
        self._index_map = None
        super().__init__(*args, **kwargs)
        self._build_index_map()

    def _build_index_map(self):
        """(global_index) -> (episode_id, start_ts) 매핑 테이블 구축."""
        index_map = []
        for ep_idx, ep_id in enumerate(self.episode_ids):
            ep = self._load_episode_parquet(ep_id)
            episode_len = ep["episode_len"]

            stride = 57
            offset = (ep_idx * 17) % stride
            start = self.n_obs_steps - 1 + offset
            for ts in range(start, max(self.n_obs_steps, episode_len - self.chunk_size), stride):
                index_map.append((ep_id, ts))
        self._index_map = index_map
        print(f'[FullScanDataset] Total samples: {len(self._index_map)}')

    def __len__(self):
        if self._index_map is None:
            return len(self.episode_ids)
        return len(self._index_map)

    def __getitem__(self, index):
        if self._index_map is None:
            return super().__getitem__(index)

        ep_id, start_ts = self._index_map[index]
        ep = self._load_episode_parquet(ep_id)

        episode_len = ep["episode_len"]
        state_data = ep["state_data"]
        action_data = ep["action_data"]
        language_instruction = ep["language_instruction"]

        action_dim = action_data.shape[1]
        expected_action_dim = self.norm_stats['action']['mean'].shape[-1]
        any_has_succeed = (expected_action_dim > action_dim)
        if any_has_succeed:
            action_dim = expected_action_dim

        original_action_shape = (self.chunk_size, action_dim)
        end_ts = start_ts + self.chunk_size
        obs_step_start = start_ts - self.n_obs_steps + 1

        qpos = []
        for i in range(self.n_obs_steps):
            idx = max(0, min(obs_step_start + i, episode_len - 1))
            qpos.append(state_data[idx])

        image_dict = {}
        for sensor_id in self.sensor_ids:
            key = f"sensor_{sensor_id}"
            image_dict[key] = []
            img_vals = ep["image_data"].get(key, [])
            for i in range(self.n_obs_steps):
                idx = max(0, min(obs_step_start + i, episode_len - 1))
                if idx < len(img_vals):
                    img_array = _parse_image_value(img_vals[idx], self.dataset_dir)
                else:
                    img_array = np.zeros((224, 224, 3), dtype=np.uint8)
                image_dict[key].append(img_array)

        actual_end = min(episode_len, end_ts)
        action = action_data[start_ts:actual_end]

        if any_has_succeed:
            if ep["succeed"] is not None:
                succeed = ep["succeed"][start_ts:actual_end].reshape(-1, 1)
            else:
                succeed = np.zeros((action.shape[0], 1), dtype=np.float32)
            action = np.concatenate([action, succeed], axis=1)

        action_len = min(self.chunk_size, episode_len - start_ts)

        padded_action = np.zeros(original_action_shape, dtype=np.float32)
        if self.action_key == 'relative_ee_pos' or (self.use_relative_trajectory and self.action_key == 'ee_delta_action'):
            padded_action[:action_len] = delta_to_relative_trajectory(action[:action_len])
        else:
            padded_action[:action_len] = action
        is_pad = np.zeros(self.chunk_size)
        is_pad[action_len:] = 1

        item = dict()
        item['language_instruction'] = language_instruction
        for sensor_id in self.sensor_ids:
            processed_images = []
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                image = process_image(image)
                processed_images.append(image)
            if self.policy_type in ['PI0']:
                item[f"observation.images.sensor_{sensor_id}"] = torch.stack(processed_images).squeeze()
            else:
                item[f"observation.images.sensor_{sensor_id}"] = torch.stack(processed_images)

        if self.policy_type in ['PI0']:
            item["observation.state"] = torch.from_numpy(np.concatenate(qpos)).float()
        else:
            item["observation.state"] = torch.from_numpy(np.array(qpos)).float()

        item["action"] = torch.from_numpy(padded_action).float()
        item["action_is_pad"] = torch.from_numpy(is_pad).bool()
        item['next.done'] = torch.from_numpy(np.zeros(1, dtype=np.bool_)).bool()

        if self.info is None:
            self.info = dict()
            for key, val in item.items():
                if key.startswith("observation.images"):
                    self.info[key] = PolicyFeature(FeatureType.VISUAL, shape=val[0].shape)
                if key == "observation.state":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=val[0].shape)
                if key == "action":
                    self.info[key] = PolicyFeature(FeatureType.ACTION, shape=val[0].shape)
                if key == "language_instruction":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=(1,))

        return item


def load_data(dataset_dir, policy_type, num_episodes, sensor_ids, batch_size_train, batch_size_val, chunk_size, vision_backbone='resnet18', num_workers=1, n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None):
    if obs_state_keys is None:
        obs_state_keys = ['qpos']
    print(f'\nData from: {dataset_dir}\n')
    # obtain train test split
    train_ratio = 0.8
    shuffled_indices = np.random.permutation(num_episodes)
    split = max(1, int(train_ratio * num_episodes))
    train_indices = shuffled_indices[:split]
    # 에피소드가 적으면 train 데이터를 val에도 재사용
    val_indices = shuffled_indices[split:] if split < num_episodes else shuffled_indices

    # obtain normalization stats for qpos and action
    norm_stats, skipped_episodes = get_norm_stats(dataset_dir, num_episodes, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys)

    # 호환 불가능한 에피소드 제외
    if skipped_episodes:
        valid_mask = ~np.isin(shuffled_indices, skipped_episodes)
        valid_indices = shuffled_indices[valid_mask]
        split = max(1, int(train_ratio * len(valid_indices)))
        train_indices = valid_indices[:split]
        val_indices = valid_indices[split:] if split < len(valid_indices) else valid_indices

    # construct dataset and dataloader
    train_dataset = EpisodicDataset(train_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys)
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size_train, shuffle=True, pin_memory=True, num_workers=num_workers)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size_val, shuffle=True, pin_memory=True, num_workers=num_workers)

    input_features = {k: v for k, v in train_dataset.info.items() if k.startswith("observation")}
    output_features = {k: v for k, v in train_dataset.info.items() if k.startswith("action")}

    return train_dataloader, val_dataloader, norm_stats, input_features, output_features



# Computes the mean of a list of dictionaries, where each dictionary represents an epoch's metrics.
def compute_dict_mean(epoch_dicts):
    result = {k: None for k in epoch_dicts[0]}
    num_items = len(epoch_dicts)
    for k in result:
        value_sum = 0
        for epoch_dict in epoch_dicts:
            value_sum += epoch_dict[k]
        result[k] = value_sum / num_items
    return result


# Detaches all tensors in a dictionary.
def detach_dict(d):
    new_d = dict()
    for k, v in d.items():
        new_d[k] = v.detach()
    return new_d

def set_seed(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)


def zoom_image(img, point, size):
    height, width = img.shape[:2]

    
    # 중심 좌표를 기준으로 관심 영역 크기 계산
    half_width = int(size[0] / 2)
    half_height = int(size[1] / 2)

    x1 = max(point[0] - half_width, 0)
    y1 = max(point[1] - half_height, 0)
    x2 = min(point[0] + half_width, width)
    y2 = min(point[1] + half_height, height)

    # 관심 영역 크롭
    cropped = img[y1:y2, x1:x2]
    
    return cropped



def input_caching(prompt):
    cache_file_path = "input_cache.json"
    if os.path.exists(cache_file_path):
        with open(cache_file_path, "r") as f:
            cache = json.load(f)
    else:
        cache = {}
    default = cache.get(prompt, "")
    def prefill_hook():
        readline.insert_text(default)  # 기본값 입력
        readline.redisplay()          # 화면에 표시
    readline.set_pre_input_hook(prefill_hook)

    answer = input(prompt)

    cache[prompt] = answer

    with open(cache_file_path, "w") as f:
        json.dump(cache, f, indent=4)

    return answer


def ros_image_to_numpy(image_msg):
    if isinstance(image_msg, CompressedImage):
        # 압축 이미지 처리
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_array = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 기본 BGR 형태로 디코딩됨
        image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)  # RGB로 변환
        image_array = image_array[:, :, ::-1]  # BGR -> RGB
        return image_array

    # 일반 Image 메시지 처리
    encoding_to_dtype = {
        'rgb8': ('uint8', 3),
        'bgr8': ('uint8', 3),
        'mono8': ('uint8', 1),
        'mono16': ('uint16', 1),
        'rgba8': ('uint8', 4),
        'bgra8': ('uint8', 4),
    }

    if image_msg.encoding not in encoding_to_dtype:
        raise ValueError(f"Unsupported encoding: {image_msg.encoding}")
    
    dtype, channels = encoding_to_dtype[image_msg.encoding]
    data = np.frombuffer(image_msg.data, dtype=dtype)
    image_array = data.reshape((image_msg.height, image_msg.width, channels))
    
    if image_msg.encoding == 'bgr8':
        image_array = image_array[:, :, ::-1]  # BGR -> RGB
    elif image_msg.encoding == 'bgra8':
        image_array = image_array[:, :, [2, 1, 0, 3]]  # BGRA -> RGBA
    
    return image_array


def rescale_val(val, origin_rng, rescaled_rng):
    return rescaled_rng[0] + (rescaled_rng[1] - rescaled_rng[0]) * ((val - origin_rng[0]) / (origin_rng[1] - origin_rng[0]))


def make_policy(ckpt_path, seed, learning_rate, lr_backbone, policy_obj, task, robot, sensors, gripper=None):
    args_override = policy_obj['settings']
    if policy_obj['type'] == 'ACT':
        args_override['ckpt_dir'] = ckpt_path
        args_override['policy_class'] = policy_obj['type']
        args_override['task_name'] = task['name']
        args_override['seed'] = seed
        args_override['state_dim'] = robot['joint_dim']
        if gripper is not None:
            args_override['state_dim'] += 1 # gripper state dim
        args_override['num_queries'] = int(policy_obj['settings']['chunk_size'])
        
        args_override['learning_rate'] = learning_rate
        args_override['lr_backbone'] = lr_backbone
        # args_override[''] = int(policy_obj['settings']['lr_backbone'])
        
        
        sensor_names = [sensor['name'] for sensor in sensors]
        args_override['camera_names'] = sensor_names
        
        policy = ACTPolicy(args_override)
    else:
        raise NotImplementedError
    return policy


def make_optimizer(policy_class, policy):
    if policy_class == 'ACT':
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer

def forward_pass(batch, policy):
    data = {k: (v.cuda() if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
    return policy.forward(data)
    # image_data, qpos_data, action_data, is_pad = data
    # image_data, qpos_data, action_data, is_pad = image_data.cuda(), qpos_data.cuda(), action_data.cuda(), is_pad.cuda()
    # return policy(qpos_data, image_data, action_data, is_pad) # TODO remove None


def convert_lists_to_tuples(obj):
    """
    딕셔너리나 리스트 내부의 모든 리스트를 재귀적으로 튜플로 변환합니다.
    """
    # 입력된 객체가 딕셔너리일 경우
    if isinstance(obj, dict):
        return {key: convert_lists_to_tuples(value) for key, value in obj.items()}
    # 입력된 객체가 리스트일 경우
    elif isinstance(obj, list):
        return tuple(convert_lists_to_tuples(item) for item in obj)
    # 딕셔너리나 리스트가 아니면 그대로 반환
    else:
        return obj
    

VISION_BACKBONE_MAP = {
    'dinov2': 'facebook/dinov2-base',
    'dinov3': 'facebook/dinov3-vitb16-pretrain-lvd1689m',
}