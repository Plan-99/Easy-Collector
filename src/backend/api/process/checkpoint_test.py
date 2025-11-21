import os
import torch
import pickle
import numpy as np
import time
import cv2
from einops import rearrange
from ...lerobot.configs.types import PolicyFeature, FeatureType
from ...utils.image_parser import fetch_image_with_config

from ...policies.utils import make_policy, VISION_BACKBONE_MAP, process_image
from ...env.env import Env

from ...lerobot.policies.act.modeling_act import ACTPolicy
from ...lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from ...lerobot.policies.pi0.modeling_pi0 import PI0Policy

import torchvision.transforms as transforms
from transformers import AutoImageProcessor


def checkpoint_test(
    node,
    checkpoint,
    task,
    policy_obj,
    agents,
    sensors,
    socketio_instance,
    task_control,
    max_timesteps,
    ):
    
    try:
        if checkpoint['is_base_model']:
            if policy_obj['type'] == 'PI0':
                ckpt_dir = "lerobot/pi0"
        else:
            ckpt_dir = os.path.join("/root/src/backend/checkpoints", str(checkpoint['id']))

        # policy = make_policy(ckpt_dir, seed, 0, 0, policy_obj, task, robots[0], sensors, gripper)
        if policy_obj['type'] == 'ACT':
            policy = ACTPolicy.from_pretrained(ckpt_dir)
        elif policy_obj['type'] == 'Diffusion':
            policy = DiffusionPolicy.from_pretrained(ckpt_dir)
        elif policy_obj['type'] == 'PI0':
            policy = PI0Policy.from_pretrained(ckpt_dir)
        
        socketio_instance.emit('log_checkpoint_test', {
            'log': f'Loaded Policy from {ckpt_dir}',
            'type': 'stdout '
        })
        
        state_dim = 0
        for agent in agents:
            state_dim += agent.joint_len

        env = Env(node, agents, sensors)

        vision_backbone = policy_obj.get('vision_backbone')


    except Exception as e:
        import traceback
        error_string = traceback.format_exc()
        print(error_string)
        socketio_instance.emit('log_checkpoint_test', {
            'log': f'Error in Checkpoint Test: {error_string}',
            'type': 'stdout '
        })
        return
            
    policy.reset()
    
    print(policy.config.output_features)
    
    # if temporal_agg:
    #     all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()
        
    socketio_instance.emit('checkpoint_test_progress', {
        'progress': 0,
        'type': 'stdout '
    })
    
    home_pose = task['home_pose']

    if home_pose is not None:
        for agent in env.agents:
            agent.move_to(home_pose[str(agent.id)])

    time.sleep(3)
        
    socketio_instance.emit('log_checkpoint_test', {
        'log': 'Robot moved to homepose',
        'type': 'stdout '
    })
    
    ts = env.reset()

    timesteps = [ts]
    
    while True:

        try:
            start = time.time()
            state = {}

            obs = ts.observation
            qpos_list = [item['qpos'] for item in obs['robot_states'].values()]
            qpos = np.concatenate(qpos_list)
            qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)

            state['observation.state'] = qpos
            for sensor in sensors:
                image = obs['images'][f'sensor_{sensor["id"]}']
                image = fetch_image_with_config(image, {
                    'resize': task['sensor_img_size'],
                })
                image = process_image(image, vision_backbone, to_cuda=True)
                # image = image / 255.0
                # image = torch.from_numpy(image).float().cuda().unsqueeze(0)
                # image = rearrange(image, 'b h w c -> b c h w')
                state[f'observation.images.sensor_{sensor["id"]}'] = image.unsqueeze(0)


            # Prepare input features dynamically if base model
            if checkpoint['is_base_model']:
                state['language_instruction'] = [task['name']]
                info = dict()
                for key, val in state.items():
                    if key.startswith("observation.images"):
                        info[key] = PolicyFeature(FeatureType.VISUAL, shape=val[0].shape)
                    if key == "observation.state":
                        info[key] = PolicyFeature(FeatureType.STATE, shape=val[0].shape)     
                info['action'] = PolicyFeature(FeatureType.ACTION, shape=[7])      
                policy.config.input_features = {k: v for k, v in info.items() if k.startswith("observation")}
                policy.config.output_features = {k: v for k, v in info.items() if k.startswith("action")}
            


            with torch.inference_mode():
                action = policy.select_action(state)

            # Prepare the action for the environment
            action = action.squeeze(0).to("cpu").numpy()
            start_action_id = 0
            for agent in env.agents:
                target_qpos = action[start_action_id:start_action_id + agent.joint_len]
                # print(f"To:{target_qpos} / Now:{qpos_list}")
                start_action_id += agent.joint_len
                agent.move_step(target_qpos)


            # time.sleep(0.03)  # Simulate processing time

            ts = env.record_step()
            timesteps.append(ts)
            
            if task_control['stop']:
                return
                            
            end = time.time()
            print(f"Uncertainty: {policy.uncertainty}")
            print(f"Step Time: {end - start}")
            socketio_instance.emit('checkpoint_test_step', {
                'uncertainty': policy.uncertainty,
                'step_time': end - start,
            })

        except Exception as e:
            import traceback
            error_string = traceback.format_exc()
            socketio_instance.emit('log_checkpoint_test', {
                'log': f'Error in Checkpoint Test: {error_string}',
                'type': 'stdout '
            })
            return

            # curr_image, memories = get_image(ts, camera_names, config['camera_config'], yolo_config, memories)