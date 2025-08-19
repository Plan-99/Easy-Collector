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


e = IPython.embed


class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, sensor_ids, norm_stats, chunk_size, n_obs_steps=1):
        super(EpisodicDataset).__init__()
        self.episode_ids = episode_ids
        self.dataset_dir = dataset_dir
        self.sensor_ids = sensor_ids
        self.norm_stats = norm_stats
        self.chunk_size = chunk_size
        self.n_obs_steps = n_obs_steps
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
            for key in root['qaction'].keys():
                episode_len = root['qaction'][key].shape[0]
                action_dim += root['qaction'][key].shape[1]

            original_action_shape = (self.chunk_size, action_dim)
                
            if sample_full_episode:
                start_ts = 0
            else:
                start_ts = np.random.choice(np.arange(self.n_obs_steps - 1, episode_len - self.chunk_size))
            end_ts = start_ts + self.chunk_size

            obs_step_start = start_ts - self.n_obs_steps + 1
            qpos = []
            for i in range(self.n_obs_steps):
                qpos.append(get_concatenated_pos(root['/observations/qpos'], target_id=obs_step_start + i))

            image_dict = dict()
            for sensor_id in self.sensor_ids:
                image_dict[f"sensor_{sensor_id}"] = []
                for i in range(self.n_obs_steps):
                    image_dict[f"sensor_{sensor_id}"].append(root[f'/observations/images/sensor_{sensor_id}'][obs_step_start + i])


            action = get_concatenated_pos(root['qaction'], target_id=None, target_range=[start_ts, min(episode_len, end_ts)])
            
            action_len = min(self.chunk_size, episode_len - start_ts) # hack, to make timesteps more aligned

        padded_action = np.zeros(original_action_shape, dtype=np.float32)

        padded_action[:action_len] = action
        is_pad = np.zeros(self.chunk_size)
        is_pad[action_len:] = 1

        # # new axis for different cameras
        # all_cam_images = []
        # for sensor_id in self.sensor_ids:
        #     all_cam_images.append(image_dict[f"sensor_{sensor_id}"])
        # all_cam_images = np.stack(all_cam_images, axis=0)

        # # construct observations
        # image_data = torch.from_numpy(all_cam_images)
        # qpos_data = torch.from_numpy(qpos).float()
        # action_data = torch.from_numpy(padded_action).float()
        # is_pad = torch.from_numpy(is_pad).bool()

        # # channel last
        # image_data = torch.einsum('k h w c -> k c h w', image_data)

        # # normalize image and change dtype to float
        # image_data = image_data / 255.0
        # action_data = (action_data - self.norm_stats["qpos_mean"]) / self.norm_stats["qpos_std"]
        # qpos_data = (qpos_data - self.norm_stats["qpos_mean"]) / self.norm_stats["qpos_std"]

        # return image_data, qpos_data, action_data, is_pad


        item = dict()
        for sensor_id in self.sensor_ids:
            image = torch.from_numpy(np.array(image_dict[f"sensor_{sensor_id}"]))
            image = torch.einsum('n h w c -> n c h w', image)  # channel last to channel first
            image = image / 255.0  # normalize image
            item[f"observation.images.sensor_{sensor_id}"] = image
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

        return item




def get_norm_stats(dataset_dir, num_episodes):
    all_qpos_data = []
    all_action_data = []
    # episode 길이 관련 코드 수정
    observation_image_keys = []
    cnt = 0
    for episode_idx in range(num_episodes):
        dataset_path = os.path.join(dataset_dir, f'episode_{episode_idx}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            qpos = get_concatenated_pos(root['/observations/qpos'])
            action = get_concatenated_pos(root['qaction'])
            observation_image_keys = list(root['/observations/images'].keys())
        
        cnt += qpos.shape[0]
        all_qpos_data.append(torch.from_numpy(qpos))
        all_action_data.append(torch.from_numpy(action))
    all_qpos_data = torch.stack(all_qpos_data)
    all_action_data = torch.stack(all_action_data)

    # normalize action data
    action_min = all_action_data.view(-1, 7).min(dim=0)[0]
    action_max = all_action_data.view(-1, 7).max(dim=0)[0]
    action_mean = all_action_data.mean(dim=[0, 1], keepdim=True)
    action_std = all_action_data.std(dim=[0, 1], keepdim=True)
    action_std = torch.clip(action_std, 1e-2, np.inf) # clipping

    # normalize qpos data
    qpos_min = all_qpos_data.view(-1, 7).min(dim=0)[0]
    qpos_max = all_qpos_data.view(-1, 7).max(dim=0)[0]
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
    pos_list = []
    for key in pos_path.keys():
        if target_id is None and target_range is None:
            pos_list.append(pos_path[key][()])
            if len(pos_list) > 0:
                pos = np.concatenate(pos_list, axis=0)
        elif target_range is None:
            pos_list.append(pos_path[key][target_id])
            if len(pos_list) > 0:
                pos = np.concatenate(pos_list, axis=0)
        else:
            pos_list.append(pos_path[key][target_range[0]:target_range[1]])
            if len(pos_list) > 0:
                pos = np.concatenate(pos_list, axis=0)
    return pos


def load_data(dataset_dir, num_episodes, sensor_ids, batch_size_train, batch_size_val, chunk_size, num_workers=1, n_obs_steps=1):
    print(f'\nData from: {dataset_dir}\n')
    # obtain train test split
    train_ratio = 0.8
    shuffled_indices = np.random.permutation(num_episodes)
    train_indices = shuffled_indices[:int(train_ratio * num_episodes)]
    val_indices = shuffled_indices[int(train_ratio * num_episodes):]

    # obtain normalization stats for qpos and action
    norm_stats = get_norm_stats(dataset_dir, num_episodes)

    # construct dataset and dataloader
    train_dataset = EpisodicDataset(train_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, n_obs_steps=n_obs_steps)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, n_obs_steps=n_obs_steps)
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size_train, shuffle=True, pin_memory=True, num_workers=num_workers, prefetch_factor=1)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size_val, shuffle=True, pin_memory=True, num_workers=num_workers, prefetch_factor=1)

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



# def fetch_image_with_config(image, config, memory=None, yolo_config=None, no_yolo=False, no_zoom=False):
#     if memory is None:
#         memory = {
#             'is_first_image': True,
#             'fixed_boxes': [],
#             'last_box': {}
#         }

#     if 'zoom' in config and not no_zoom:
#         size = config['zoom']['size']
#         point = config['zoom']['point']
#         image = zoom_image(image, point, size)
#     if 'resize' in config:
#         size = config['resize']['size']
#         image = cv2.resize(image, size)
#     if 'masked_yolo' in config and not no_yolo:
#         classes = config['masked_yolo']['classes']
#         masked_image = np.zeros_like(image)
#         show_boxes = []
#         yolo_model = yolo_config['model']

#         results = yolo_model(image, conf=yolo_config['conf'])
#         result = results[0]
#         boxes = result.boxes
#         names = result.names
#         masked_image = np.zeros_like(image)

#         for class_name, class_config in classes.items():
#             class_boxes = []
#             last_box = memory['last_box'][class_name] if class_name in memory['last_box'] else None
#             for box in boxes:
#                 box_id = int(box.cls.item())
#                 if box_id == class_config['id']:
#                     class_boxes.append(box)

#             if class_config['is_fixed_mask']:

#                 if memory['is_first_image']:

#                     if class_config['show_id'] == -1:
#                         memory['fixed_boxes'] += class_boxes
#                         memory['is_first_image'] = False
#                     elif len(class_boxes) > class_config['show_id']:
#                         memory['fixed_boxes'].append(class_boxes[class_config['show_id']])
#                         memory['is_first_image'] = False

#                 show_boxes += memory['fixed_boxes']

#             else:
#                 if class_config['show_id'] == -1:
#                     show_boxes += class_boxes
#                 elif len(class_boxes) > class_config['show_id']:
#                     show_boxes.append(class_boxes[class_config['show_id']])

#             if class_config['keep_last_box']:
#                 if last_box is not None:
#                     show_boxes.append(last_box)
#                 if len(class_boxes) > 0:
#                     memory['last_box'][class_name] = class_boxes[0]

#         if len(show_boxes) > 0:
#             masked_image = mask_outside_boxes(image, show_boxes, padding=yolo_config['padding'])
#         image = masked_image

    
#     return image, memory


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

# def get_image(ts, sensor_ids, camera_config, memories, yolo_config=None):
#     curr_images = []
#     raw_images = []

#     for index, cam_name in enumerate(camera_names):
#         image = ts.observation['images'][cam_name]

#         # if cam_name in camera_config:
#         #     image, memories[index] = fetch_image_with_config(image, camera_config[cam_name], memories[index], yolo_config)


#         raw_images.append(image)
#         curr_image = rearrange(image, 'h w c -> c h w')
#         curr_images.append(curr_image)

#     if len(raw_images):           
#         # 이미지 크기 맞추기 (최대 크기로 맞추거나 다른 방식으로 조정)
#         max_height = 480
#         max_width = 640
#         resized_images = [cv2.resize(img, (max_width, max_height)) for img in raw_images]

#         # 이미지를 가로로 나열
#         combined_image = cv2.hconcat(resized_images)

#     else:
#         print("No images to display.")


#     curr_image = np.stack(curr_images, axis=0)
#     curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)

#     return curr_image, memories


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