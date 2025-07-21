import argparse
import sys
import os
import torch
import pickle
import tqdm
import numpy as np
import time

from ..database.models.robot_model import Robot
from ..database.models.policy_model import Policy
from ..database.models.task_model import Task
from ..database.models.gripper_model import Gripper
from ..database.models.sensor_model import Sensor
from ..database.models.checkpoint_model import Checkpoint

from ..env.env import Env
from ..env.agent import Agent

from ..policies.utils import make_policy, make_optimizer, forward_pass, detach_dict, compute_dict_mean, set_seed, load_data

def test(
    checkpoint,
    task,
    policy_obj,
    robots,
    sensors_ls,
    gripper=None
    ):
    
    seed = 1
    
    ckpt_dir = "/root/src/backend/checkpoints"
    ckpt_path = os.path.join(ckpt_dir, checkpoint['id'])
    policy = make_policy(ckpt_dir, seed, policy_obj, task, robots[0], sensors_ls, gripper)
    
    temporal_agg = True
    
    loading_status = policy.load_state_dict(torch.load(ckpt_path))
    print(loading_status)
    policy.cuda()
    policy.eval()
    print(f'Loaded: {ckpt_path}')
    
    
    state_dim = 0
    for robot in robots:
        state_dim += robot['state_dim']
    
    if policy_obj.type == 'ACT':
        stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
        with open(stats_path, 'rb') as f:
            stats = pickle.load(f)

        pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
        post_process = lambda a: a * stats['action_std'] + stats['action_mean']
        
        if temporal_agg:
            query_frequency = 1
        num_queries = policy.settings['chunk_size']
        
        max_timesteps = 2000
        
        env = Env(robots, sensors_ls)
        
        ts = env.reset()
        timesteps = [ts]
        actions = []
        
        if temporal_agg:
            all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()
        
        for t in tqdm(range(max_timesteps)):
            start = time.time()
            ### process previous timestep to get qpos and image_list
            timesteps.append(ts)
            obs = ts.observation
                
            cur_qpos = np.array(obs['robot_state'])
            
            robot_input_raw = cur_qpos
            robot_input = pre_process(robot_input_raw)
            robot_input = torch.from_numpy(robot_input).float().cuda().unsqueeze(0)
            
            curr_image = ts.observation['images']
            
            with torch.inference_mode():
                ### query policy
                if policy_obj.type == "ACT":
                    if t % query_frequency == 0:
                        all_actions = policy(robot_input, curr_image)
                    if temporal_agg:
                        all_time_actions[[t], t:t+num_queries] = all_actions
                        actions_for_curr_step = all_time_actions[:, t]
                        actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                        actions_for_curr_step = actions_for_curr_step[actions_populated]
                        k = 0.01
                        exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                        # raw_action = all_actions[:, t % query_frequency]
                    else:
                        raw_action = all_actions[:, t % query_frequency]
                else:
                    raise NotImplementedError
            
            # curr_image, memories = get_image(ts, camera_names, config['camera_config'], yolo_config, memories)
            
            
# def get_image(ts, camera_names, camera_config, memories, yolo_config=None):
#     curr_images = []
#     raw_images = []

#     for index, cam_name in enumerate(camera_names):
#         image = ts.observation['images'][cam_name]

#         if cam_name in camera_config:
#             image, memories[index] = fetch_image_with_config(image, camera_config[cam_name], memories[index], yolo_config)


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

#         # # 단일 창에 표시
#         # cv2.imshow("Combined Image", combined_image)
#         # cv2.waitKey(1)
#     else:
#         print("No images to display.")


#     curr_image = np.stack(curr_images, axis=0)
#     curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)

#     return curr_image, memories
        


def main(args):
    checkpoint = Checkpoint.find(args.checkpoint_id)
    task = Task.find(checkpoint['task_id'])
    policy = Policy.find(checkpoint['policy_id'])
    robots = [Robot.find(rid).to_dict() for rid in task['robot_ids']]
    sensors_ls = [Sensor.find(sid).to_dict() for sid in task['sensor_ids']]
    
    test(
        checkpoint,
        task,
        policy,
        robots,
        sensors_ls
    )
    
    


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint_id', required=True)
    
    main(parser.parse_args())
    sys.exit(0)
