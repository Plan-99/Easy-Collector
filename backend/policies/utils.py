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
from lerobot.configs.types import PolicyFeature, FeatureType
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
    def __init__(self, episode_ids, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone='resnet18', n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, augment=False, wrist_sensor_ids=None):
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
        # Image augmentation: PI0.5 paper Appendix E recipe (RandomCrop 95% + Resize
        # + Rotate ±5° + ColorJitter). Applied on PIL images before process_image
        # so it interacts correctly with any downstream normalization. Train-only;
        # val/eval datasets instantiate with augment=False.
        # Wrist cameras: openpi skips spatial transforms on wrist views because
        # crop/rotate breaks the gripper↔observation geometric coupling. Only
        # ColorJitter is applied. wrist_sensor_ids defaults to empty (all sensors
        # get the full aug pipeline — backward compatible). User must configure
        # which sensor IDs are wrist-mounted to opt into the differentiated path.
        self.augment = augment
        self.wrist_sensor_ids = set(int(x) for x in (wrist_sensor_ids or []))
        if augment:
            self._image_augment_full = transforms.Compose([
                transforms.RandomResizedCrop(size=(224, 224), scale=(0.9025, 1.0), ratio=(0.95, 1.05)),
                transforms.RandomRotation(degrees=5),
                transforms.ColorJitter(brightness=0.3, contrast=0.4, saturation=0.5),
            ])
            self._image_augment_color_only = transforms.ColorJitter(
                brightness=0.3, contrast=0.4, saturation=0.5
            )
        else:
            self._image_augment_full = None
            self._image_augment_color_only = None
        self.info = None

        # Pre-load episode metadata (lengths) for efficient sampling
        self._ep_cache = {}
        self._dataset_info = get_dataset_info(dataset_dir)

        # Pre-load episode → tasks mapping from episodes.jsonl (lerobot standard).
        # Each line: {"episode_index": N, "length": L, "tasks": ["pick up the cup", ...]}
        # PI05 uses the first entry of `tasks` per episode as its language instruction.
        from ..api.process.lerobot_io import _read_jsonl, EPISODES_PATH
        episodes_meta = _read_jsonl(os.path.join(dataset_dir, EPISODES_PATH))
        self._episode_tasks = {
            int(e.get("episode_index")): (e.get("tasks") or [""])
            for e in episodes_meta
            if e.get("episode_index") is not None
        }

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

        # Language instruction from episodes.jsonl (per-episode `tasks` list).
        # Use the first entry; most datasets have a single task per episode.
        ep_tasks = self._episode_tasks.get(int(episode_id), [""])
        language_instruction = ep_tasks[0] if ep_tasks else ""

        # Get image data per sensor — from parquet (legacy), video files, or PNG dir.
        # CRITICAL: silent fallback to np.zeros((224,224,3)) was masking missing images
        # and causing the model to train on all-black frames. Each sensor MUST resolve
        # to actual images here; an assertion below catches the failure mode loudly.
        image_data = {}
        image_cols = [col for col in df.columns if col.startswith("observation.images.")]
        if image_cols:
            # Legacy image mode: images embedded in parquet
            for col in image_cols:
                sensor_name = col.replace("observation.images.", "")
                image_data[sensor_name] = list(df[col])
        else:
            # Video / PNG mode: prefer pre-decoded .npy (mmap'd), then mp4, then PNG dir.
            features = self._dataset_info.get("features", {})
            for feat_key, feat in features.items():
                if not feat_key.startswith("observation.images."):
                    continue
                sensor_name = feat_key.replace("observation.images.", "")
                video_path = os.path.join(
                    self.dataset_dir, "videos", f"chunk-{chunk:03d}", feat_key,
                    f"episode_{episode_id:06d}.mp4"
                )
                npy_path = video_path[:-4] + ".npy"
                # PNG fallback: images/{feat_key}/episode_{id:06d}/frame_{idx:06d}.png
                # lerobot_io.py writes frames here when video encoding isn't used.
                png_dir = os.path.join(
                    self.dataset_dir, "images", feat_key, f"episode_{episode_id:06d}"
                )
                if os.path.exists(npy_path):
                    image_data[sensor_name] = np.load(npy_path, mmap_mode="r")
                elif os.path.exists(video_path):
                    from ..api.process.lerobot_io import _decode_video_frames
                    image_data[sensor_name] = _decode_video_frames(video_path)
                elif os.path.isdir(png_dir):
                    # Build a list of frame_XXXXXX.png paths in order. _parse_image_value
                    # accepts string paths and decodes via PIL.
                    n_frames = len(state_data)
                    image_data[sensor_name] = [
                        os.path.join(png_dir, f"frame_{i:06d}.png") for i in range(n_frames)
                    ]
                else:
                    image_data[sensor_name] = []

        # Loud assertion: if any sensor came back empty for an episode that has frames,
        # the dataset is silently feeding all-black images. Fail early instead of
        # training on garbage.
        for sensor_name, vals in image_data.items():
            if len(vals) == 0 and len(state_data) > 0:
                raise RuntimeError(
                    f"[EpisodicDataset] No image source found for sensor '{sensor_name}' "
                    f"in episode {episode_id} (dataset_dir={self.dataset_dir}). "
                    f"Checked: parquet column, .npy, .mp4, PNG dir. "
                    f"Training would silently use all-zero frames."
                )

        succeed = np.array(df["succeed"].tolist(), dtype=np.float32) if "succeed" in df.columns else None

        result = {
            "state_data": state_data,
            "action_data": action_data,
            "language_instruction": language_instruction,
            "image_data": image_data,
            "succeed": succeed,
            "episode_len": len(df),
        }
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
            # Pick aug based on whether this sensor is configured as wrist-mounted.
            # Wrist cams: ColorJitter only (preserve spatial geometry).
            # Other cams: full aug (crop+rotate+jitter).
            if int(sensor_id) in getattr(self, 'wrist_sensor_ids', set()):
                aug = getattr(self, '_image_augment_color_only', None)
            else:
                aug = getattr(self, '_image_augment_full', None)
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                if aug is not None:
                    image = aug(image)
                # PI05's SigLIP vision tower expects [-1, 1] (PaliGemma / openpi convention).
                # ToTensor alone yields [0, 1] and breaks pretrained visual features.
                _pixel_range = '-11' if self.policy_type == 'PI05' else '01'
                image = process_image(image, self.vision_backbone, pixel_range=_pixel_range)
                processed_images.append(image)

            # New lerobot expects [C, H, W] per sample (collate → [batch, C, H, W])
            stacked = torch.stack(processed_images)  # [n_obs_steps, C, H, W]
            if self.n_obs_steps == 1:
                item[f"observation.images.sensor_{sensor_id}"] = stacked.squeeze(0)  # [C, H, W]
            else:
                item[f"observation.images.sensor_{sensor_id}"] = stacked

        if self.n_obs_steps == 1:
            item["observation.state"] = torch.from_numpy(qpos[0]).float()
        elif self.policy_type in ['PI0', 'PI05']:
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
                    self.info[key] = PolicyFeature(FeatureType.VISUAL, shape=val.shape)
                if key == "observation.state":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=val.shape)
                if key == "action":
                    self.info[key] = PolicyFeature(FeatureType.ACTION, shape=val[0].shape)
                if key == "language_instruction":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=(1,))

        return item



def process_image(image, vision_backbone='resnet18', to_cuda=False, pixel_range='01'):
    """Preprocess an image into a model-ready tensor.

    pixel_range: '01' → standard torchvision ToTensor output in [0, 1].
                 '-11' → scaled to [-1, 1] (mandatory for PI05 / PaliGemma / SigLIP —
                         openpi preprocess_observation explicitly does `img / 255 * 2 - 1`).
                         Feeding [0, 1] into pi05_base shifts the vision feature
                         distribution and wrecks pretrained visual grounding; LoRA
                         can't fully recover it, which matches the "moves but misses"
                         symptom during inference.
    """
    if not isinstance(image, Image.Image):
        image = Image.fromarray(np.array(image))
    if vision_backbone not in VISION_BACKBONE_MAP:
        image_transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
        ])
        image = image_transform(image)
        if pixel_range == '-11':
            image = image * 2.0 - 1.0
    else:
        image_processor = AutoImageProcessor.from_pretrained(VISION_BACKBONE_MAP[vision_backbone])
        image = image_processor(image)['pixel_values'][0]  # backbone's own normalization

    return image.cuda() if to_cuda else image


def get_norm_stats(dataset_dir, num_episodes, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, use_relative_actions=False, relative_joints_dim=None, relative_action_mask=None, absolute_action_dims=None, chunk_size=50):
    """Compute normalization stats from LeRobot dataset parquet files.

    When ``use_relative_actions=True`` the pipeline transforms action to
    (action - state) for dimensions shared with state BEFORE normalization. Stats
    must therefore be computed on those delta values, not raw absolute targets —
    otherwise the normalizer divides tiny deltas by absolute-scale q99/q01 and the
    training signal collapses (every normalized target ends up near a constant).

    ``relative_joints_dim`` mirrors openpi's ``make_bool_mask(N, -1)`` — when set,
    only the first N dims are converted to delta. For a 6-DOF arm + gripper + done
    token, use relative_joints_dim=6 (gripper and done stay absolute).
    """
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

        # use_relative_actions: stats MUST match what RelativeActionsProcessorStep
        # actually produces during training/inference, not naive per-timestep deltas.
        # The pipeline does:
        #     delta[t in chunk] = action[t] - state[chunk_start]   (single state, all T)
        # NOT:
        #     delta[t] = action[t] - state[t]                      (per-step, tiny)
        # The chunk-wise delta accumulates over the chunk → magnitudes 10-50× larger
        # than per-step deltas. Computing stats with per-step deltas underestimates
        # q99-q01, normalization explodes target values to [-99, 99], gradients break,
        # loss never decreases. THIS WAS THE "loss won't go down with delta mode" bug.
        if use_relative_actions:
            episode_len = action.shape[0]
            state_dim = qpos.shape[1]
            action_dim = action.shape[1]

            # Build effective mask once. Truthy checks so default empty list / 0 falls through.
            if relative_action_mask:
                m = list(relative_action_mask)
                m = m + [False] * (action_dim - len(m)) if len(m) < action_dim else m[:action_dim]
                effective_mask = [bool(x) for x in m]
            elif absolute_action_dims:
                effective_mask = [True] * action_dim
                for idx in absolute_action_dims:
                    if 0 <= int(idx) < action_dim:
                        effective_mask[int(idx)] = False
            elif relative_joints_dim and relative_joints_dim > 0:
                n = min(relative_joints_dim, action_dim)
                effective_mask = [True] * n + [False] * (action_dim - n)
            else:
                shared = min(state_dim, action_dim)
                effective_mask = [True] * shared + [False] * (action_dim - shared)

            # Sample chunk-wise deltas: for every valid chunk start in the episode,
            # compute (action[start:start+chunk_size] - state[start]) for delta dims.
            # Concatenate all such chunks → matches the distribution the pipeline produces.
            delta_chunks = []
            cs = max(1, int(chunk_size))
            for start in range(episode_len):
                end = min(start + cs, episode_len)
                chunk = action[start:end].copy()  # (T, action_dim)
                state_at_start = qpos[start]      # (state_dim,)
                for i, is_delta in enumerate(effective_mask):
                    if is_delta and i < state_dim:
                        chunk[:, i] = chunk[:, i] - state_at_start[i]
                delta_chunks.append(chunk)
            # Replace `action` with concatenated chunk-deltas so downstream stats
            # (min/max/mean/std/q01/q99) are computed on the right distribution.
            action = np.concatenate(delta_chunks, axis=0) if delta_chunks else action

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
    # q01/q99 for QUANTILES normalization (Pi0.5). Using torch.quantile.
    action_q01 = torch.quantile(all_action_data.float(), 0.01, dim=0).to(all_action_data.dtype)
    action_q99 = torch.quantile(all_action_data.float(), 0.99, dim=0).to(all_action_data.dtype)
    # Guard against near-constant columns (e.g. a rarely-triggered done flag whose
    # q01 == q99 == 0). Without this the normalizer falls back to eps=1e-8 denominator
    # and the single non-zero value becomes ~1e8 in normalized space — destroys training.
    _range = action_q99 - action_q01
    _min_range = torch.clamp(_range, min=1e-2)
    action_q99 = action_q01 + _min_range

    qpos_min = all_qpos_data.min(dim=0)[0]
    qpos_max = all_qpos_data.max(dim=0)[0]
    qpos_mean = all_qpos_data.mean(dim=0)
    qpos_std = all_qpos_data.std(dim=0)
    qpos_std = torch.clip(qpos_std, 1e-2, np.inf)
    qpos_q01 = torch.quantile(all_qpos_data.float(), 0.01, dim=0).to(all_qpos_data.dtype)
    qpos_q99 = torch.quantile(all_qpos_data.float(), 0.99, dim=0).to(all_qpos_data.dtype)
    _qrange = qpos_q99 - qpos_q01
    _qmin_range = torch.clamp(_qrange, min=1e-2)
    qpos_q99 = qpos_q01 + _qmin_range

    stats = {
        "action": {
            "min": action_min.numpy(),
            "max": action_max.numpy(),
            "mean": action_mean.numpy().squeeze(),
            "std": action_std.numpy().squeeze(),
            "q01": action_q01.numpy(),
            "q99": action_q99.numpy(),
            "count": np.array([cnt]),
        },
        "observation.state": {
            "min": qpos_min.numpy(),
            "max": qpos_max.numpy(),
            "mean": qpos_mean.numpy().squeeze(),
            "std": qpos_std.numpy().squeeze(),
            "q01": qpos_q01.numpy(),
            "q99": qpos_q99.numpy(),
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
            # Pick aug based on whether this sensor is configured as wrist-mounted.
            # Wrist cams: ColorJitter only (preserve spatial geometry).
            # Other cams: full aug (crop+rotate+jitter).
            if int(sensor_id) in getattr(self, 'wrist_sensor_ids', set()):
                aug = getattr(self, '_image_augment_color_only', None)
            else:
                aug = getattr(self, '_image_augment_full', None)
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                if aug is not None:
                    image = aug(image)
                # PI05's SigLIP vision tower expects [-1, 1] (PaliGemma / openpi convention).
                # ToTensor alone yields [0, 1] and breaks pretrained visual features.
                _pixel_range = '-11' if self.policy_type == 'PI05' else '01'
                image = process_image(image, self.vision_backbone, pixel_range=_pixel_range)
                processed_images.append(image)

            # New lerobot expects [C, H, W] per sample (collate → [batch, C, H, W])
            stacked = torch.stack(processed_images)  # [n_obs_steps, C, H, W]
            if self.n_obs_steps == 1:
                item[f"observation.images.sensor_{sensor_id}"] = stacked.squeeze(0)  # [C, H, W]
            else:
                item[f"observation.images.sensor_{sensor_id}"] = stacked

        if self.n_obs_steps == 1:
            item["observation.state"] = torch.from_numpy(qpos[0]).float()
        elif self.policy_type in ['PI0', 'PI05']:
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
                    self.info[key] = PolicyFeature(FeatureType.VISUAL, shape=val.shape)
                if key == "observation.state":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=val.shape)
                if key == "action":
                    self.info[key] = PolicyFeature(FeatureType.ACTION, shape=val[0].shape)
                if key == "language_instruction":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=(1,))

        return item


def load_data(dataset_dir, policy_type, num_episodes, sensor_ids, batch_size_train, batch_size_val, chunk_size, vision_backbone='resnet18', num_workers=1, n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, use_relative_actions=False, relative_joints_dim=None, relative_action_mask=None, absolute_action_dims=None, wrist_sensor_ids=None):
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
    norm_stats, skipped_episodes = get_norm_stats(dataset_dir, num_episodes, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, use_relative_actions=use_relative_actions, relative_joints_dim=relative_joints_dim, relative_action_mask=relative_action_mask, absolute_action_dims=absolute_action_dims, chunk_size=chunk_size)

    # 호환 불가능한 에피소드 제외
    if skipped_episodes:
        valid_mask = ~np.isin(shuffled_indices, skipped_episodes)
        valid_indices = shuffled_indices[valid_mask]
        split = max(1, int(train_ratio * len(valid_indices)))
        train_indices = valid_indices[:split]
        val_indices = valid_indices[split:] if split < len(valid_indices) else valid_indices

    # construct dataset and dataloader. Image augmentation (paper Appendix E:
    # RandomCrop 95% + Rotate ±5° + ColorJitter) is applied only to PI05 training
    # splits — ACT/Diffusion have their own expected input distributions and val
    # needs deterministic data for honest loss comparison.
    train_augment = (policy_type == 'PI05')
    # wrist_sensor_ids: openpi skips spatial aug (crop+rotate) on wrist cams since
    # they encode end-effector geometry — random crop destroys gripper↔scene spatial
    # prior and breaks pretrained feature extraction. EasyTrainer's previous
    # augmentation infrastructure (added in audit-doc bug #24) was never wired through
    # load_data — that's now fixed by passing it from train.py via the train config.
    train_dataset = EpisodicDataset(train_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, augment=train_augment, wrist_sensor_ids=wrist_sensor_ids)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, augment=False, wrist_sensor_ids=wrist_sensor_ids)
    # Keep workers alive across epochs so EpisodicDataset._ep_cache (and OS page
    # cache backing the mmap'd frame .npy files) survives between epochs.
    loader_kwargs = dict(
        shuffle=True,
        pin_memory=True,
        num_workers=num_workers,
    )
    if num_workers > 0:
        loader_kwargs["persistent_workers"] = True
        loader_kwargs["prefetch_factor"] = 4
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size_train, **loader_kwargs)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size_val, **loader_kwargs)

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

_pi05_tokenizer = None

def _get_pi05_tokenizer():
    """Lazy-load PaliGemma tokenizer for PI05."""
    global _pi05_tokenizer
    if _pi05_tokenizer is None:
        from transformers import AutoTokenizer
        _pi05_tokenizer = AutoTokenizer.from_pretrained("google/paligemma-3b-pt-224")
    return _pi05_tokenizer


def prepare_pi05_language_tokens(batch, config, norm_stats=None):
    """Convert language_instruction + state → tokenized language tokens for PI05.

    Replicates the logic of Pi05PrepareStateTokenizerProcessorStep + TokenizerProcessorStep.
    """
    tokenizer = _get_pi05_tokenizer()
    max_length = getattr(config, 'tokenizer_max_length', 200)
    max_state_dim = getattr(config, 'max_state_dim', 32)

    # Get language instruction (string or list of strings)
    lang = batch.get('language_instruction', '')
    if isinstance(lang, str):
        lang = [lang]

    # Get state for discretization
    state = batch.get('observation.state')
    prompts = []
    for i, task_text in enumerate(lang):
        cleaned = task_text.strip().replace("_", " ").replace("\n", " ") if task_text else ""
        state_str = ""
        if state is not None:
            s = state[i] if state.dim() > 1 else state
            s_np = s.cpu().numpy().flatten()
            # Normalize to [-1, 1] using min-max if norm_stats available
            if norm_stats and 'observation.state' in norm_stats:
                s_min = norm_stats['observation.state']['min']
                s_max = norm_stats['observation.state']['max']
                s_range = s_max - s_min
                s_range[s_range < 1e-6] = 1.0
                s_np = 2.0 * (s_np - s_min) / s_range - 1.0
                s_np = np.clip(s_np, -1.0, 1.0)
            # Pad to max_state_dim
            if len(s_np) < max_state_dim:
                s_np = np.concatenate([s_np, np.zeros(max_state_dim - len(s_np))])
            # Discretize into 256 bins
            bins = np.linspace(-1, 1, 257)[:-1]
            discretized = np.digitize(s_np, bins) - 1
            state_str = " ".join(map(str, discretized.astype(int)))
        prompt = f"Task: {cleaned}, State: {state_str};\nAction: "
        prompts.append(prompt)

    tokenized = tokenizer(
        prompts,
        max_length=max_length,
        truncation=True,
        padding="max_length",
        return_tensors="pt",
    )

    device = state.device if state is not None else torch.device('cuda')
    batch['observation.language.tokens'] = tokenized['input_ids'].to(device)
    batch['observation.language.attention_mask'] = tokenized['attention_mask'].to(dtype=torch.bool, device=device)

    return batch


def forward_pass(batch, policy, norm_stats=None, preprocessor=None):
    data = {k: (v.cuda() if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
    # PI05 preprocessor pipeline expects complementary_data['task']. EasyTrainer
    # datasets store the same field under 'language_instruction', so bridge the
    # two so Pi05PrepareStateTokenizerProcessorStep can find the prompt.
    if 'task' not in data:
        if 'language_instruction' in data:
            data['task'] = data['language_instruction']
        elif hasattr(policy, 'config') and hasattr(policy.config, 'tokenizer_max_length'):
            # Determine batch size to build a placeholder list of the right length.
            bsz = None
            for v in data.values():
                if isinstance(v, torch.Tensor) and v.dim() >= 1:
                    bsz = v.shape[0]
                    break
            data['task'] = [''] * (bsz or 1)
    # Apply input preprocessor (Normalize state/action/images per cfg.normalization_mapping)
    # so the model trains on normalized targets and the gradient is balanced across joints.
    # The PI05 preprocessor pipeline includes Pi05PrepareStateTokenizerProcessorStep +
    # TokenizerProcessorStep which write `observation.language.tokens` correctly using
    # QUANTILE-normalized state and native state_dim (audit-doc bug #23 fix).
    if preprocessor is not None:
        data = preprocessor(data)
    elif hasattr(policy, 'config') and hasattr(policy.config, 'tokenizer_max_length'):
        # Fallback for the rare case where preprocessor failed to load (e.g. corrupted
        # checkpoint json). Use the legacy `prepare_pi05_language_tokens` to produce
        # *some* language tokens — the result is divergent from training (min-max norm,
        # padded state) and the model will perform poorly, but at least it won't crash.
        # Normal training/inference always has preprocessor and skips this branch.
        data = prepare_pi05_language_tokens(data, policy.config, norm_stats=norm_stats)
    return policy.forward(data)


def make_easytrainer_processors(policy_type, cfg, dataset_stats=None, pretrained_path=None):
    """Per-policy dispatch for building / loading the LeRobot Normalize+Unnormalize pipeline.

    Bypasses ``lerobot.policies.factory.make_pre_post_processors`` because that module
    eagerly imports ``lerobot.envs.configs`` → robots → motors, none of which we install
    in the EasyTrainer container. We just call the per-policy ``make_*_pre_post_processors``
    builders directly so the dependency surface stays minimal.

    Args:
        policy_type: 'ACT' | 'Diffusion' | 'PI05'
        cfg: the policy config (ACTConfig / DiffusionConfig / PI05Config)
        dataset_stats: numpy stats dict from get_norm_stats() — used when building from scratch.
        pretrained_path: when set, load saved processor json/safetensors from this dir
            instead of building. ``dataset_stats`` is ignored in this branch.

    Returns:
        (preprocessor, postprocessor) PolicyProcessorPipeline tuple, or (None, None)
        on a load failure when pretrained_path is set (caller falls back to raw I/O).
    """
    from lerobot.processor.pipeline import PolicyProcessorPipeline
    from lerobot.processor.converters import (
        batch_to_transition,
        transition_to_batch,
        policy_action_to_transition,
        transition_to_policy_action,
    )
    from lerobot.utils.constants import (
        POLICY_PREPROCESSOR_DEFAULT_NAME,
        POLICY_POSTPROCESSOR_DEFAULT_NAME,
    )

    if pretrained_path is not None:
        # Force-import the policy-specific processor modules before deserializing the
        # saved pipeline JSON. ProcessorStepRegistry is populated by @register decorators
        # at import time — without this, loading a PI05 checkpoint fails with
        # "Processor step 'pi05_prepare_state_tokenizer_processor_step' not found in
        # registry" and the code silently falls back to raw I/O (no normalization).
        # That skipped normalization is what makes the robot behave erratically at
        # inference even though training looked fine.
        if policy_type == 'PI05':
            import lerobot.policies.pi05.processor_pi05  # noqa: F401
        elif policy_type == 'ACT':
            import lerobot.policies.act.processor_act  # noqa: F401
        elif policy_type == 'Diffusion':
            import lerobot.policies.diffusion.processor_diffusion  # noqa: F401
        try:
            preprocessor = PolicyProcessorPipeline.from_pretrained(
                pretrained_model_name_or_path=pretrained_path,
                config_filename=f"{POLICY_PREPROCESSOR_DEFAULT_NAME}.json",
                to_transition=batch_to_transition,
                to_output=transition_to_batch,
            )
            postprocessor = PolicyProcessorPipeline.from_pretrained(
                pretrained_model_name_or_path=pretrained_path,
                config_filename=f"{POLICY_POSTPROCESSOR_DEFAULT_NAME}.json",
                to_transition=policy_action_to_transition,
                to_output=transition_to_policy_action,
            )
            # Re-wire the relative↔absolute pair after deserialization. AbsoluteActionsProcessorStep's
            # get_config only persists `enabled` (not relative_step, which is a live cross-pipeline
            # reference), so a freshly loaded postprocessor has relative_step=None and would either
            # crash (use_relative_actions=True) or silently skip delta→absolute conversion.
            # Find the RelativeActionsProcessorStep inside the preprocessor and rebind it.
            try:
                from lerobot.processor.relative_action_processor import (
                    RelativeActionsProcessorStep,
                    AbsoluteActionsProcessorStep,
                )
                _rel_step = None
                for s in getattr(preprocessor, 'steps', []):
                    if isinstance(s, RelativeActionsProcessorStep):
                        _rel_step = s
                        break
                if _rel_step is not None:
                    for s in getattr(postprocessor, 'steps', []):
                        if isinstance(s, AbsoluteActionsProcessorStep):
                            s.relative_step = _rel_step
            except Exception as _wire_err:
                print(f'[WARN] Could not rewire AbsoluteActionsProcessorStep.relative_step: {_wire_err}')
            return preprocessor, postprocessor
        except Exception as e:
            print(f'[WARN] make_easytrainer_processors: failed to load processors from '
                  f'{pretrained_path} ({type(e).__name__}: {e}). Returning (None, None).')
            return None, None

    # Build from scratch using per-policy factories
    if policy_type == 'ACT':
        from lerobot.policies.act.processor_act import make_act_pre_post_processors
        return make_act_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    if policy_type == 'Diffusion':
        from lerobot.policies.diffusion.processor_diffusion import make_diffusion_pre_post_processors
        return make_diffusion_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    if policy_type == 'PI05':
        from lerobot.policies.pi05.processor_pi05 import make_pi05_pre_post_processors
        return make_pi05_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    raise ValueError(f'make_easytrainer_processors: unsupported policy_type {policy_type!r}')


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