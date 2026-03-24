from .policies import ACTPolicy, CNNMLPPolicy
from einops import rearrange
import cv2
import numpy as np
import torch
import os
import h5py
from torch.utils.data import TensorDataset, DataLoader
import IPython
import numpy as np
import json
import readline
from sensor_msgs.msg import CompressedImage
from types import SimpleNamespace
from ..lerobot.configs.types import PolicyFeature, FeatureType
from torchvision import transforms
from transformers import AutoImageProcessor
from PIL import Image
from scipy.spatial.transform import Rotation


def delta_to_relative_trajectory(deltas: np.ndarray) -> np.ndarray:
    """UMI 방식의 relative trajectory 라벨로 변환.

    deltas: [T, 6] sequential ee_delta [dx, dy, dz, dax, day, daz]
    반환값: [T, 6] 각 row i = 현재 위치 기준 i+1 step 후의 누적 displacement

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
        cumulative = cumulative * Rotation.from_rotvec(deltas[i, 3:])
        relative[i, 3:] = cumulative.as_rotvec()

    return relative


def relative_trajectory_to_delta(waypoints: np.ndarray) -> np.ndarray:
    """relative trajectory → sequential delta 역변환 (inference 시 사용).

    waypoints: [T, 6] relative trajectory (T_now→t+i)
    반환값: [T, 6] sequential deltas
    """
    T = len(waypoints)
    deltas = np.zeros_like(waypoints)

    # Translation: 연속 차분
    deltas[0, :3] = waypoints[0, :3]
    deltas[1:, :3] = np.diff(waypoints[:, :3], axis=0)

    # Rotation: 연속 회전 차분
    deltas[0, 3:] = waypoints[0, 3:]
    for i in range(1, T):
        r_prev = Rotation.from_rotvec(waypoints[i - 1, 3:])
        r_curr = Rotation.from_rotvec(waypoints[i, 3:])
        deltas[i, 3:] = (r_prev.inv() * r_curr).as_rotvec()

    return deltas




e = IPython.embed


class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone='resnet18', n_obs_steps=1, action_key='qaction', use_relative_trajectory=False):
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
        self.info = None
        self.__getitem__(0) # initialize self.is_sim

    def __len__(self):
        return len(self.episode_ids)

    def __getitem__(self, index):
        sample_full_episode = False # hardcode

        episode_id = self.episode_ids[index]
        dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_id}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            # is_sim = root.attrs['sim']

            action_dim = 0
            is_mixed = (self.action_key == 'ee_delta_action')
            if is_mixed:
                ee_robots = set(root['ee_delta_action'].keys()) if 'ee_delta_action' in root else set()
                for robot_key in root['qaction'].keys():
                    src = root['ee_delta_action'] if robot_key in ee_robots else root['qaction']
                    item = src[robot_key]
                    leaves = list(item.values()) if isinstance(item, h5py.Group) else [item]
                    for leaf in leaves:
                        episode_len = leaf.shape[0]
                        action_dim += leaf.shape[1]
            else:
                for key in root[self.action_key].keys():
                    item = root[self.action_key][key]
                    leaves = list(item.values()) if isinstance(item, h5py.Group) else [item]
                    for leaf in leaves:
                        episode_len = leaf.shape[0]
                        action_dim += leaf.shape[1]

            original_action_shape = (self.chunk_size, action_dim)

            language_instruction = root['language_instruction'][()]
            if isinstance(language_instruction, bytes):
                language_instruction = language_instruction.decode('utf-8')

            if sample_full_episode:
                start_ts = 0
            else:
                start_ts = np.random.choice(np.arange(self.n_obs_steps - 1, episode_len - self.chunk_size))
            end_ts = start_ts + self.chunk_size

            obs_step_start = start_ts - self.n_obs_steps + 1
            qpos = []
            if is_mixed:
                for i in range(self.n_obs_steps):
                    qpos.append(get_concatenated_mixed_pos(root, '/observations/ee_delta', '/observations/qpos', target_id=obs_step_start + i))
            else:
                obs_state_path = '/observations/qpos'
                for i in range(self.n_obs_steps):
                    qpos.append(get_concatenated_pos(root[obs_state_path], target_id=obs_step_start + i))

            image_dict = dict()
            for sensor_id in self.sensor_ids:
                image_dict[f"sensor_{sensor_id}"] = []
                for i in range(self.n_obs_steps):
                    image_dict[f"sensor_{sensor_id}"].append(root[f'/observations/images/sensor_{sensor_id}'][obs_step_start + i])


            if is_mixed:
                action = get_concatenated_mixed_pos(root, 'ee_delta_action', 'qaction', target_id=None, target_range=[start_ts, min(episode_len, end_ts)])
            else:
                action = get_concatenated_pos(root[self.action_key], target_id=None, target_range=[start_ts, min(episode_len, end_ts)])
            
            action_len = min(self.chunk_size, episode_len - start_ts) # hack, to make timesteps more aligned

        padded_action = np.zeros(original_action_shape, dtype=np.float32)

        if self.use_relative_trajectory and self.action_key == 'ee_delta_action':
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
            # image = torch.from_numpy(np.array(image_dict[f"sensor_{sensor_id}"]))
            # image = torch.einsum('n h w c -> n c h w', image)  # channel last to channel first
            # image = image / 255.0  # normalize image

            if self.policy_type in ['PI0']:
                item[f"observation.images.sensor_{sensor_id}"] = torch.stack(processed_images).squeeze()  # add time dim
                print(item[f"observation.images.sensor_{sensor_id}"].shape)
            else:
                item[f"observation.images.sensor_{sensor_id}"] = torch.stack(processed_images)
                
        if self.policy_type in ['PI0']:
            item["observation.state"] = torch.from_numpy(np.concatenate(qpos)).float()
        else:
            item["observation.state"] = torch.from_numpy(np.array(qpos)).float()

        item["action"] = torch.from_numpy(padded_action).float()
        item["action_is_pad"] = torch.from_numpy(is_pad).bool()
        item['next.done'] = torch.from_numpy(np.zeros(1, dtype=np.bool_)).bool()  # dummy done tensor

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
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=(1,)) # dummy shape

        return item



def process_image(image, vision_backbone='resnet18', to_cuda=False):
    if vision_backbone not in VISION_BACKBONE_MAP:
        tensor_transform = transforms.ToTensor()
        image = tensor_transform(image)
    else:
        image_processor = AutoImageProcessor.from_pretrained(VISION_BACKBONE_MAP[vision_backbone])
        image = image_processor(image)['pixel_values'][0]  # Assuming the image is a PIL Image or numpy array

    return image.cuda() if to_cuda else image  # Add batch dimension


def get_norm_stats(dataset_dir, num_episodes, action_key='qaction', use_relative_trajectory=False):
    all_qpos_data = []
    all_action_data = []
    # episode 길이 관련 코드 수정
    observation_image_keys = []
    cnt = 0
    is_mixed = (action_key == 'ee_delta_action')
    for episode_idx in range(num_episodes):
        dataset_path = os.path.join(dataset_dir, f'episode_{episode_idx}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            if is_mixed:
                qpos = get_concatenated_mixed_pos(root, '/observations/ee_delta', '/observations/qpos')
                action = get_concatenated_mixed_pos(root, 'ee_delta_action', 'qaction')
            else:
                qpos = get_concatenated_pos(root['/observations/qpos'])
                action = get_concatenated_pos(root[action_key])
            if use_relative_trajectory and action_key == 'ee_delta_action':
                action = delta_to_relative_trajectory(action)
            observation_image_keys = list(root['/observations/images'].keys())
        
        cnt += qpos.shape[0]
        all_qpos_data.append(torch.from_numpy(qpos))
        all_action_data.append(torch.from_numpy(action))
    all_qpos_data = torch.stack(all_qpos_data)
    all_action_data = torch.stack(all_action_data)

    # normalize action data
    action_min = all_action_data.view(-1, action.shape[-1]).min(dim=0)[0]
    action_max = all_action_data.view(-1, action.shape[-1]).max(dim=0)[0]
    action_mean = all_action_data.mean(dim=[0, 1], keepdim=True)
    action_std = all_action_data.std(dim=[0, 1], keepdim=True)
    action_std = torch.clip(action_std, 1e-2, np.inf) # clipping

    # normalize qpos data
    qpos_min = all_qpos_data.view(-1, qpos.shape[-1]).min(dim=0)[0]
    qpos_max = all_qpos_data.view(-1, qpos.shape[-1]).max(dim=0)[0]
    qpos_mean = all_qpos_data.mean(dim=[0, 1], keepdim=True)
    qpos_std = all_qpos_data.std(dim=[0, 1], keepdim=True)
    qpos_std = torch.clip(qpos_std, 1e-2, np.inf) # clipping

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
            "min": np.array([[[0.0]], [[0.0]], [[0.0]]]),  # Assuming images are normalized between 0 and 1
            "max": np.array([[[1.0]], [[1.0]], [[1.0]]]),  # Assuming images are normalized between 0 and 1
            "mean": np.array([[[0.5]], [[0.5]], [[0.5]]]),  # Assuming images are normalized between 0 and 1
            "std": np.array([[[0.25]], [[0.25]], [[0.25]]]),  # Assuming images are normalized between 0 and 1
            "count": np.array([cnt]),
        }
    

    # stats = {"qaction_mean": action_mean.numpy().squeeze(), "qaction_std": action_std.numpy().squeeze(),
    #          "qpos_mean": qpos_mean.numpy().squeeze(), "qpos_std": qpos_std.numpy().squeeze(),
    #          "example_qpos": qpos, 'example_action': action }
    
    return stats


def get_concatenated_pos(pos_path, target_id=None, target_range=None):
    """robot_N 키 아래 dataset이 있거나, robot_N/ee_name 처럼 한 단계 더 nested된 구조 모두 처리."""
    pos_list = []
    for key in pos_path.keys():
        item = pos_path[key]
        leaves = list(item.values()) if isinstance(item, h5py.Group) else [item]
        for leaf in leaves:
            if target_id is None and target_range is None:
                pos_list.append(leaf[()])
            elif target_range is None:
                pos_list.append(leaf[target_id])
            else:
                pos_list.append(leaf[target_range[0]:target_range[1]])
    if not pos_list:
        return np.array([])
    axis = 0 if (target_range is None and target_id is not None) else 1
    return np.concatenate(pos_list, axis=axis)


def get_concatenated_mixed_pos(root, primary_path, fallback_path, target_id=None, target_range=None):
    """ee_delta_action 학습 시, single_arm은 primary_path(ee_delta_action/ee_delta)에서,
    tool은 fallback_path(qaction/qpos)에서 읽어 concatenate."""
    primary_robots = set(root[primary_path].keys()) if primary_path in root else set()

    pos_list = []
    for robot_key in root[fallback_path].keys():
        if robot_key in primary_robots:
            item = root[primary_path][robot_key]
        else:
            item = root[fallback_path][robot_key]

        leaves = list(item.values()) if isinstance(item, h5py.Group) else [item]
        for leaf in leaves:
            if target_id is None and target_range is None:
                pos_list.append(leaf[()])
            elif target_range is None:
                pos_list.append(leaf[target_id])
            else:
                pos_list.append(leaf[target_range[0]:target_range[1]])
    if not pos_list:
        return np.array([])
    axis = 0 if (target_range is None and target_id is not None) else 1
    return np.concatenate(pos_list, axis=axis)


def load_data(dataset_dir, policy_type, num_episodes, sensor_ids, batch_size_train, batch_size_val, chunk_size, vision_backbone='resnet18', num_workers=1, n_obs_steps=1, action_key='qaction', use_relative_trajectory=False):
    print(f'\nData from: {dataset_dir}\n')
    # obtain train test split
    train_ratio = 0.8
    shuffled_indices = np.random.permutation(num_episodes)
    train_indices = shuffled_indices[:int(train_ratio * num_episodes)]
    val_indices = shuffled_indices[int(train_ratio * num_episodes):]

    # obtain normalization stats for qpos and action
    norm_stats = get_norm_stats(dataset_dir, num_episodes, action_key=action_key, use_relative_trajectory=use_relative_trajectory)

    # construct dataset and dataloader
    train_dataset = EpisodicDataset(train_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory)
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