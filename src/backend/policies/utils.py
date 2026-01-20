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


e = IPython.embed


class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone='resnet18', n_obs_steps=1, model_input=['qpos', 'vision'], model_output=['qaction']):
        super(EpisodicDataset).__init__()
        self.episode_ids = episode_ids
        self.dataset_dir = dataset_dir
        self.sensor_ids = sensor_ids
        self.norm_stats = norm_stats
        self.chunk_size = chunk_size
        self.n_obs_steps = n_obs_steps
        self.policy_type = policy_type
        self.vision_backbone = vision_backbone
        self.model_input = model_input
        self.model_output = model_output
        self.info = None
        
        self.key_mapping = {
            'joint': '/observations/qpos',
            'qpos': '/observations/qpos',
            'qvel': '/observations/qvel',
            'eepos': '/observations/eepos',
            'eef_vel': '/observations/eef_vel',
            'eetarget': '/eetarget',
            'eetarget_delta': '/eetarget_delta',
            'qaction': '/qaction',
            'qaction_delta': '/qaction_delta',
        }
        self.__getitem__(0) # initialize self.info

    def __len__(self):
        return len(self.episode_ids)

    def __getitem__(self, index):
        sample_full_episode = False # hardcode

        episode_id = self.episode_ids[index]
        dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_id}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            qaction_path = root.get('/qaction')
            if qaction_path is None:
                raise KeyError("'/qaction' group not found in HDF5 file.")
            
            action_dim = 0
            for key in qaction_path.keys():
                episode_len = qaction_path[key].shape[0]
                action_dim += qaction_path[key].shape[1]

            language_instruction = root['language_instruction'][()]
            if isinstance(language_instruction, bytes):
                language_instruction = language_instruction.decode('utf-8')
                
            if sample_full_episode:
                start_ts = 0
            else:
                start_ts = np.random.choice(np.arange(self.n_obs_steps - 1, episode_len - self.chunk_size))
            end_ts = start_ts + self.chunk_size
            obs_step_start = start_ts - self.n_obs_steps + 1

            # Process state inputs
            state_parts = []
            for i in range(self.n_obs_steps):
                current_step_state_parts = []
                for key in self.model_input:
                    if key in self.key_mapping:
                        hdf5_path = self.key_mapping[key]
                        if hdf5_path in root:
                            current_step_state_parts.append(get_concatenated_pos(root[hdf5_path], target_id=obs_step_start + i))
                if current_step_state_parts:
                    state_parts.append(np.concatenate(current_step_state_parts))

            if not state_parts: # Vision-only case or no state inputs
                for _ in range(self.n_obs_steps):
                    state_parts.append(np.zeros(1, dtype=np.float32))
            
            # Process image inputs if 'vision' is in model_input
            image_dict = dict()
            if 'vision' in self.model_input:
                for sensor_id in self.sensor_ids:
                    image_dict[f"sensor_{sensor_id}"] = []
                    for i in range(self.n_obs_steps):
                        image_dict[f"sensor_{sensor_id}"].append(root[f'/observations/images/sensor_{sensor_id}'][obs_step_start + i])

            # Process output actions
            action_parts = []
            for key in self.model_output:
                if key in self.key_mapping:
                    hdf5_path = self.key_mapping[key]
                    if hdf5_path in root:
                         action_parts.append(get_concatenated_pos(root[hdf5_path], target_id=None, target_range=[start_ts, min(episode_len, end_ts)]))

            if not action_parts:
                raise ValueError("No valid model_output keys found in HDF5 file.")

            action = np.concatenate(action_parts, axis=1)
            
            action_dim = action.shape[1]
            original_action_shape = (self.chunk_size, action_dim)
            action_len = min(self.chunk_size, episode_len - start_ts)

            padded_action = np.zeros(original_action_shape, dtype=np.float32)
            padded_action[:action_len] = action
            is_pad = np.zeros(self.chunk_size)
            is_pad[action_len:] = 1

        item = dict()
        item['language_instruction'] = language_instruction
        
        if 'vision' in self.model_input:
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

        qpos = state_parts
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
        image = image_processor(image)['pixel_values'][0]

    return image.cuda() if to_cuda else image


def get_norm_stats(dataset_dir, num_episodes, model_input=['qpos'], model_output=['qaction']):
    all_state_data = []
    all_action_data = []

    key_mapping = {
        'joint': '/observations/qpos',
        'qpos': '/observations/qpos',
        'qvel': '/observations/qvel',
        'eepos': '/observations/eepos',
        'eef_vel': '/observations/eef_vel',
        'eetarget': '/eetarget',
        'eetarget_delta': '/eetarget_delta',
        'qaction': '/qaction',
        'qaction_delta': '/qaction_delta',
    }
    
    observation_image_keys = []
    
    for episode_idx in range(num_episodes):
        dataset_path = os.path.join(dataset_dir, f'episode_{episode_idx}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            # state data
            state_parts = []
            for key in model_input:
                if key in key_mapping and key_mapping[key] in root:
                    state_parts.append(get_concatenated_pos(root[key_mapping[key]]))
            
            if state_parts:
                state = np.concatenate(state_parts, axis=1)
                all_state_data.append(torch.from_numpy(state))

            # action data
            action_parts = []
            for key in model_output:
                if key in key_mapping and key_mapping[key] in root:
                    action_parts.append(get_concatenated_pos(root[key_mapping[key]]))
            
            if not action_parts:
                raise ValueError("No valid model_output keys found for stats calculation.")

            action = np.concatenate(action_parts, axis=1)
            all_action_data.append(torch.from_numpy(action))
            
            if 'vision' in model_input and '/observations/images' in root:
                observation_image_keys = list(root['/observations/images'].keys())
    
    stats = {}
    
    # Action stats (calculated first to get total_steps)
    if not all_action_data:
        raise ValueError("No action data found to calculate stats.")
        
    all_action_data_cat = torch.cat(all_action_data, dim=0)
    action_min = all_action_data_cat.min(dim=0)[0]
    action_max = all_action_data_cat.max(dim=0)[0]
    action_mean = all_action_data_cat.mean(dim=0)
    action_std = all_action_data_cat.std(dim=0)
    action_std = torch.clip(action_std, 1e-2, np.inf)
    total_steps = all_action_data_cat.shape[0]

    stats["action"] = {
        "min": action_min.numpy(),
        "max": action_max.numpy(),
        "mean": action_mean.numpy(),
        "std": action_std.numpy(),
        "count": np.array([total_steps]),
    }

    # State stats
    if all_state_data:
        all_state_data_cat = torch.cat(all_state_data, dim=0)
        state_min = all_state_data_cat.min(dim=0)[0]
        state_max = all_state_data_cat.max(dim=0)[0]
        state_mean = all_state_data_cat.mean(dim=0)
        state_std = all_state_data_cat.std(dim=0)
        state_std = torch.clip(state_std, 1e-2, np.inf)

        stats["observation.state"] = {
            "min": state_min.numpy(),
            "max": state_max.numpy(),
            "mean": state_mean.numpy(),
            "std": state_std.numpy(),
            "count": np.array([total_steps]),
        }
    else: # No state data, create dummy stats
        stats["observation.state"] = {
            "min": np.zeros(1, dtype=np.float32),
            "max": np.zeros(1, dtype=np.float32),
            "mean": np.zeros(1, dtype=np.float32),
            "std": np.ones(1, dtype=np.float32),
            "count": np.array([total_steps]),
        }

    # Image stats
    if 'vision' in model_input:
        for key in observation_image_keys:
            stats[f"observation.images.{key}"] = {
                "mean": np.array([[[0.5]], [[0.5]], [[0.5]]]),
                "std": np.array([[[0.5]], [[0.5]], [[0.5]]]),
                "count": np.array([total_steps]),
            }
    
    return stats


def get_concatenated_pos(pos_path, target_id=None, target_range=None):
    def find_datasets_recursive(group, data_list):
        sorted_keys = sorted(group.keys())
        for key in sorted_keys:
            item = group[key]
            if isinstance(item, h5py.Dataset):
                if target_id is None and target_range is None:
                    data_list.append(item[()])
                elif target_range is None:
                    data_list.append(item[target_id])
                else:
                    data_list.append(item[target_range[0]:target_range[1]])
            elif isinstance(item, h5py.Group):
                find_datasets_recursive(item, data_list)

    pos_list = []
    find_datasets_recursive(pos_path, pos_list)

    if not pos_list:
        return np.array([])

    if target_id is None and target_range is None:
        return np.concatenate(pos_list, axis=1)
    elif target_range is None:
        return np.concatenate(pos_list, axis=0)
    else:
        return np.concatenate(pos_list, axis=1)


def load_data(dataset_dir, policy_type, num_episodes, sensor_ids, batch_size_train, batch_size_val, 
              chunk_size, vision_backbone='resnet18', 
              num_workers=1, n_obs_steps=1, 
              model_input=['joint', 'vision'], model_output='qaction'):
    
    print(f'\nData from: {dataset_dir}\n')

    if isinstance(model_output, str):
        model_output = [model_output]

    # obtain train test split
    train_ratio = 0.8
    shuffled_indices = np.random.permutation(num_episodes)
    train_indices = shuffled_indices[:int(train_ratio * num_episodes)]
    val_indices = shuffled_indices[int(train_ratio * num_episodes):]

    # obtain normalization stats
    norm_stats = get_norm_stats(dataset_dir, num_episodes, model_input, model_output)

    # construct dataset and dataloader
    train_dataset = EpisodicDataset(train_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, model_input=model_input, model_output=model_output)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, model_input=model_input, model_output=model_output)
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size_train, shuffle=True, pin_memory=True, num_workers=num_workers, prefetch_factor=1)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size_val, shuffle=True, pin_memory=True, num_workers=num_workers, prefetch_factor=1)

    input_features = {k: v for k, v in train_dataset.info.items() if k.startswith("observation")}
    output_features = {k: v for k, v in train_dataset.info.items() if k.startswith("action")}

    return train_dataloader, val_dataloader, norm_stats, input_features, output_features



# Computes the mean of a list of dictionaries, where each dictionary represents an epoch's metrics.
def compute_dict_mean(epoch_dicts):
    if not epoch_dicts:
        return {}
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
        readline.insert_text(default)
        readline.redisplay()
    readline.set_pre_input_hook(prefill_hook)

    answer = input(prompt)

    cache[prompt] = answer

    with open(cache_file_path, "w") as f:
        json.dump(cache, f, indent=4)

    return answer


def ros_image_to_numpy(image_msg):
    if isinstance(image_msg, CompressedImage):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_array = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
        image_array = image_array[:, :, ::-1]
        return image_array

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
        image_array = image_array[:, :, ::-1]
    elif image_msg.encoding == 'bgra8':
        image_array = image_array[:, :, [2, 1, 0, 3]]
    
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
            args_override['state_dim'] += 1
        args_override['num_queries'] = int(policy_obj['settings']['chunk_size'])
        
        args_override['learning_rate'] = learning_rate
        args_override['lr_backbone'] = lr_backbone
        
        
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


def convert_lists_to_tuples(obj):
    if isinstance(obj, dict):
        return {key: convert_lists_to_tuples(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return tuple(convert_lists_to_tuples(item) for item in obj)
    else:
        return obj
    

VISION_BACKBONE_MAP = {
    'dinov2': 'facebook/dinov2-base',
    'dinov3': 'facebook/dinov3-vitb16-pretrain-lvd1689m',
}