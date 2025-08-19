import os
import torch
import pickle
import numpy as np
import time
import cv2
from einops import rearrange
from ...utils.image_parser import fetch_image_with_config

from ...policies.utils import make_policy
from ...env.env import Env

from ...lerobot.policies.pi0.modeling_pi0 import PI0Policy
from ...lerobot.configs.types import PolicyFeature, FeatureType

def test_vla(
    node,
    model,
    policy_obj,
    robots,
    sensors,
    image_size,
    prompt,
    socketio_instance,
    task_control
):  
    max_timesteps = 1000
    
    try:
        if policy_obj['type'] == 'PI0':
            policy = PI0Policy.from_pretrained("lerobot/pi0")
        
        socketio_instance.emit('log_test_vla', {
            'log': f'Loaded Policy from PI0',
            'type': 'stdout '
        })
        
        state_dim = 0
        for robot in robots:
            state_dim += robot['joint_dim']

            
        env = Env(node, robots, sensors)
        
    except Exception as e:
        import traceback
        error_string = traceback.format_exc()
        print(error_string)
        socketio_instance.emit('log_test_vla', {
            'log': f'Error in Checkpoint Test: {error_string}',
            'type': 'stdout '
        })
        return
            
    while True:
        policy.reset()
        
        # if temporal_agg:
        #     all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()
            
        socketio_instance.emit('test_vla_progress', {
            'progress': 0,
            'type': 'stdout '
        })
        
            
        home_pose = {"1": [0, 0.6, -0.6, 0, 0, 0, 0]}
        for agent in env.agents:
            agent.move_to(home_pose[str(agent.id)])
            
        socketio_instance.emit('log_test_vla', {
            'log': 'Robot moved to homepose',
            'type': 'stdout '
        })
        
        ts = env.reset()
        

        timesteps = [ts]
        
        for t in range(max_timesteps):

            try:
                state = {}

                obs = ts.observation
                qpos_list = [item['qpos'] for item in obs['robot_states'].values()]
                qpos = np.concatenate(qpos_list)
                qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
                
                state['task'] = [prompt]
                state['observation.state'] = qpos
                
                # policy.config.input_features = [for key in state.keys() if key.startswith('observation.images')]
                
                
                for sensor in sensors:
                    image = obs['images'][f'sensor_{sensor["id"]}']
                    image = fetch_image_with_config(image, {
                        'resize': image_size,
                    })
                    image = image / 255.0
                    image = torch.from_numpy(image).float().cuda().unsqueeze(0)
                    image = rearrange(image, 'b h w c -> b c h w')
                    state[f'observation.images.sensor_{sensor["id"]}'] = image
                    
                    
                
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
                    print(f"To:{target_qpos} / Now:{qpos_list}")
                    start_action_id += agent.joint_len
                    agent.move_step(target_qpos)
                    
                    
                socketio_instance.emit('test_vla_progress', {
                    'progress': (t+1) / max_timesteps,
                    'type': 'stdout '
                })

                time.sleep(0.1)  # Simulate processing time

                ts = env.record_step()
                timesteps.append(ts)
                
                if task_control['stop']:
                    return
            
            except Exception as e:
                import traceback
                error_string = traceback.format_exc()
                socketio_instance.emit('log_test_vla', {
                    'log': f'Error in Checkpoint Test: {error_string}',
                    'type': 'stdout '
                })
                return