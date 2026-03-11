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

from .oti_rl import SACAgent, ReplayBuffer, UncertaintySubscriber
from rclpy.executors import SingleThreadedExecutor

import torchvision.transforms as transforms
from transformers import AutoImageProcessor

import rclpy
import re

import gc
import threading


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
    move_homepose=False,
    hz=10,
    ):

    oti_rl = False
    move_reward = 1.0
    
    # --- 1. 초기 설정 ---
    try:
        gc.collect()
        torch.cuda.empty_cache()

        from concurrent.futures import ThreadPoolExecutor
        thread_pool = ThreadPoolExecutor(max_workers=len(agents))
        executor = None
        
        # 기본 정책 로드
        if checkpoint['is_base_model']:
            ckpt_dir = "lerobot/pi0_base"
        else:
            ckpt_dir = os.path.join("/root/src/backend/checkpoints", str(checkpoint['id']))

        if policy_obj['type'] == 'ACT':
            policy = ACTPolicy.from_pretrained(ckpt_dir)
        elif policy_obj['type'] == 'Diffusion':
            policy = DiffusionPolicy.from_pretrained(ckpt_dir)
        elif policy_obj['type'] == 'PI0':
            policy = PI0Policy.from_pretrained(ckpt_dir)
        
        policy.cuda()
        policy.eval()
        print(f'Loaded Policy from {ckpt_dir}, hz: {hz}')
        
        # 환경 및 RL 에이전트 초기화
        state_dim = sum(agent.joint_len for agent in agents)
        env = Env(node, agents, sensors)
        vision_backbone = policy_obj.get('vision_backbone')
        episode_len = task.get('episode_len', 300) * 1.5

        # OTI-RL 관련 요소들 조건부 초기화
        if oti_rl:
            rl_agent = SACAgent(state_dim=state_dim, action_dim=state_dim)
            replay_buffer = ReplayBuffer(capacity=10000)
            noise_reward_coeff = 0.0
            uncertainty_penalty_coeff = 1.0
            move_reward_coeff = 50.0
            rl_batch_size = 128

            # --- Load pre-trained RL model if specified ---
            load_rl_checkpoint_step = 0
            if load_rl_checkpoint_step and load_rl_checkpoint_step > 0:
                rl_model_dir = os.path.join("/root/src/backend/fiper/data", str(task['id']), "rl_models")
                
                actor_path = os.path.join(rl_model_dir, f"actor_step_{load_rl_checkpoint_step}.pth")
                critic1_path = os.path.join(rl_model_dir, f"critic1_step_{load_rl_checkpoint_step}.pth")
                critic2_path = os.path.join(rl_model_dir, f"critic2_step_{load_rl_checkpoint_step}.pth")

                if os.path.exists(actor_path):
                    try:
                        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
                        rl_agent.actor.load_state_dict(torch.load(actor_path, map_location=device))
                        rl_agent.critic1.load_state_dict(torch.load(critic1_path, map_location=device))
                        rl_agent.critic2.load_state_dict(torch.load(critic2_path, map_location=device))
                        
                        rl_agent.target_critic1.load_state_dict(rl_agent.critic1.state_dict())
                        rl_agent.target_critic2.load_state_dict(rl_agent.critic2.state_dict())

                        print(f"Successfully loaded RL models from step {load_rl_checkpoint_step}")

                    except Exception as e:
                        print(f"[ERROR] Error loading RL models: {str(e)}")
                else:
                    print(f"[ERROR] RL model checkpoint not found at: {actor_path}")

            uncertainty_subscriber = UncertaintySubscriber()
            executor = SingleThreadedExecutor()
            executor.add_node(uncertainty_subscriber)

    except Exception as e:
        import traceback
        error_string = traceback.format_exc()
        print(f"[ERROR] Error in Initialization: {error_string}")
        return

    # --- 3. 메인 루프 ---
    try:
        noise_t_raw = None
        
        # --- Find the last saved RL step to resume numbering ---
        last_step = 0
        uncertainty_entered_step = 0
        uncertainty_mode_timer = 0
        if oti_rl:
            rl_model_dir = os.path.join("/root/src/backend/fiper/data", str(task['id']), "rl_models")
            if os.path.exists(rl_model_dir):
                try:
                    step_numbers = [
                        int(re.search(r'actor_step_(\d+)\.pth', f).group(1))
                        for f in os.listdir(rl_model_dir)
                        if re.search(r'actor_step_(\d+)\.pth', f)
                    ]
                    if step_numbers:
                        last_step = max(step_numbers)
                        print(f"Found last RL model checkpoint at step {last_step}. Resuming from there.")
                except Exception as e:
                    print(f"Could not parse last RL step, starting from 0. Error: {e}")

        step_num = last_step
        home_pose = task.get('home_pose')
        episode_reward = 0.0

        # --- Pre-define frequencies for smooth probing motion ---
        joint_len = sum(agent.joint_len for agent in agents)
        probing_freqs = np.linspace(0.1, 0.4, joint_len) * 2 * np.pi

        policy.reset()
        if home_pose is not None:
            for agent in env.agents:
                agent.move_to(home_pose[str(agent.id)])
        time.sleep(8)
        ts = env.reset()
        print('Robot moved to homepose')
        
        start = time.time()
        while not task_control['stop']:
            if step_num % episode_len == 0 and step_num != 0 and move_homepose: 
                print(f"Episode finished. Total Reward: {episode_reward:.4f}")
                episode_reward = 0.0
                policy.reset()
                if home_pose is not None:
                    for agent in env.agents:
                        agent.move_to(home_pose[str(agent.id)])
                time.sleep(6)
                ts = env.reset()
                print('Robot moved to homepose')

            # 일정 스텝마다 강제 메모리 정리 (예: 100스텝마다)
            if step_num % 100 == 0:
                gc.collect()
                torch.cuda.empty_cache()
                
            # === a. 현재 상태(state_t) 계산 ===
            obs_t = ts.observation
            with torch.no_grad():
                qpos_t = torch.from_numpy(np.concatenate([item['qpos'] for item in obs_t['robot_states'].values()])).float().cuda().unsqueeze(0)

                policy_input_t = {'observation.state': qpos_t}
                for sensor in sensors:
                    image = obs_t['images'][f'sensor_{sensor["id"]}']
                    sensor_id = str(sensor['id'])
                    image = fetch_image_with_config(image, {
                        'resize': task['sensor_img_size'][str(sensor_id)],
                        'cropped_area': task['sensor_cropped_area'][str(sensor_id)],
                        'rotate': task['sensor_rotate'][str(sensor_id)]
                    })
                    image = process_image(image, vision_backbone, to_cuda=True)
                    policy_input_t[f'observation.images.sensor_{sensor["id"]}'] = image.unsqueeze(0)

                state_t = policy.select_action(policy_input_t).squeeze(0).cpu().numpy()

                if noise_t_raw is None:
                    noise_t_raw = np.zeros_like(state_t)

            # === b. 최종 행동(final_action) 결정 ===
            if oti_rl:
                executor.spin_once(timeout_sec=0.01)
                uncertainty = uncertainty_subscriber.latest_score

                if uncertainty > 1.2:
                    if uncertainty_mode_timer == 0:
                        uncertainty_entered_step = step_num
                    if uncertainty_mode_timer % 30 == 0:
                        policy.reset()
                        amplitude_vector = rl_agent.select_action(state_t)
                    
                    amplitude_scale = np.linalg.norm(amplitude_vector) * 0.15 
                    noise_t = amplitude_scale * np.sin(probing_freqs * step_num)
                    uncertainty_mode_timer += 1
                else:
                    noise_t = np.zeros_like(state_t)
                    uncertainty_mode_timer = 0

                noise_t[-1] = 0 # Gripper에는 노이즈 적용 안함
                final_action = state_t + noise_t
            else:
                final_action = state_t

            # === c. 로봇 제어 (필터 없이 즉시 반영) ===
            start_action_id = 0
            for agent in env.agents:
                target_qpos = final_action[start_action_id : start_action_id + agent.joint_len]
                thread_pool.submit(agent.move_joint_step, target_qpos)
                start_action_id += agent.joint_len
            
            ts_next = env.record_step()

            # === d. OTI-RL 학습 ===
            if oti_rl:
                executor.spin_once(timeout_sec=0.01)
                uncertainty = uncertainty_subscriber.latest_score
                noise_reward = noise_reward_coeff * np.linalg.norm(noise_t)
                uncertainty_penalty = -uncertainty_penalty_coeff * uncertainty
                move_reward = move_reward_coeff * np.linalg.norm(np.concatenate([item['qpos'] for item in ts_next.observation['robot_states'].values()]) - 
                                                                 np.concatenate([item['qpos'] for item in ts.observation['robot_states'].values()]))
                reward_t = noise_reward + uncertainty_penalty + move_reward

                with torch.no_grad():
                    obs_t1 = ts_next.observation
                    qpos_t1 = torch.from_numpy(np.concatenate([item['qpos'] for item in obs_t1['robot_states'].values()])).float().cuda().unsqueeze(0)
                    policy_input_t1 = {'observation.state': qpos_t1}
                    for sensor in sensors:
                        sensor_id = str(sensor['id'])
                        image = obs_t1['images'][f'sensor_{sensor_id}']
                        image = fetch_image_with_config(image, {
                            'resize': task['sensor_img_size'][sensor_id],
                            'cropped_area': task['sensor_cropped_area'][sensor_id].get('cropped_area', None),
                            'rotate': task['sensor_rotate'][sensor_id]
                        })
                        image = process_image(image, vision_backbone, to_cuda=True)
                        policy_input_t1[f'observation.images.sensor_{sensor_id}'] = image.unsqueeze(0)
                    state_t1 = policy.select_action(policy_input_t1).squeeze(0).cpu().numpy()
                
                replay_buffer.push(state_t, noise_t, reward_t, state_t1, done=False)
                if len(replay_buffer) > rl_batch_size:
                    rl_agent.update_parameters(replay_buffer, rl_batch_size)

                # 모델 저장 로직
                if step_num > 0 and step_num % 2000 == 0:
                    save_dir = os.path.join("/root/src/backend/fiper/data", str(task['id']), "rl_models")
                    os.makedirs(save_dir, exist_ok=True)

                    actor_path = os.path.join(save_dir, f"actor_step_{step_num}.pth")
                    critic1_path = os.path.join(save_dir, f"critic1_step_{step_num}.pth")
                    critic2_path = os.path.join(save_dir, f"critic2_step_{step_num}.pth")

                    torch.save(rl_agent.actor.state_dict(), actor_path)
                    torch.save(rl_agent.critic1.state_dict(), critic1_path)
                    torch.save(rl_agent.critic2.state_dict(), critic2_path)

                    print(f"RL models saved at step {step_num} to {save_dir}")

                episode_reward += reward_t

            time.sleep(max(0, (1.0 / hz) - (time.time() - start)))  # Loop at specified Hz
            print(f"Time: -------------------{time.time() - start}" )
            start = time.time()
            step_num += 1
            ts = ts_next

    except Exception as e:
        import traceback
        error_string = traceback.format_exc()
        print(f"Error in main loop: {error_string}")

    finally:
        thread_pool.shutdown(wait=False)
        # --- 4. 종료 처리 ---
        if oti_rl and 'uncertainty_subscriber' in locals():
            uncertainty_subscriber.destroy_node()
            if executor is not None:
                executor.shutdown()
        
        gc.collect()
        torch.cuda.empty_cache()

    return
