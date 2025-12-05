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

    oti_rl = True
    move_reward = 1.0
    
    # --- 1. 초기 설정 ---
    try:

        gc.collect()
        torch.cuda.empty_cache()
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
        socketio_instance.emit('log_checkpoint_test', { 'log': f'Loaded Policy from {ckpt_dir}' })
        
        # 환경 및 RL 에이전트 초기화
        state_dim = sum(agent.joint_len for agent in agents)
        env = Env(node, agents, sensors)
        vision_backbone = policy_obj.get('vision_backbone')

        smooth_factor = 0.9

        # OTI-RL 관련 요소들 조건부 초기화
        if oti_rl:
            rl_agent = SACAgent(state_dim=state_dim, action_dim=state_dim)
            replay_buffer = ReplayBuffer(capacity=10000)
            noise_reward_coeff = 0.0
            uncertainty_penalty_coeff = 1.0
            move_reward_coeff = 50.0
            rl_batch_size = 128

            # --- Load pre-trained RL model if specified ---
            load_rl_checkpoint_step = 0  # <--- 불러올 스텝 번호를 여기에 지정 (0 또는 None이면 로드 안 함)
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
                        
                        # 타겟 네트워크에도 동일한 가중치를 복사하여 학습 안정성 확보
                        rl_agent.target_critic1.load_state_dict(rl_agent.critic1.state_dict())
                        rl_agent.target_critic2.load_state_dict(rl_agent.critic2.state_dict())

                        log_msg = f"Successfully loaded RL models from step {load_rl_checkpoint_step}"
                        print(log_msg)
                        socketio_instance.emit('log_checkpoint_test', {'log': log_msg})
                    except Exception as e:
                        log_msg = f"Error loading RL models: {e}"
                        print(log_msg)
                        socketio_instance.emit('log_checkpoint_test', {'log': log_msg})
                else:
                    log_msg = f"RL model checkpoint not found at: {actor_path}"
                    print(log_msg)
                    socketio_instance.emit('log_checkpoint_test', {'log': log_msg})         

            # rclpy.init() is now managed by the main application process.
            # if not rclpy.ok():
            #     rclpy.init(args=None)
            uncertainty_subscriber = UncertaintySubscriber()

            executor = SingleThreadedExecutor()
            executor.add_node(uncertainty_subscriber)

    except Exception as e:
        import traceback
        error_string = traceback.format_exc()
        socketio_instance.emit('log_checkpoint_test', { 'log': f'Error in Initialization: {error_string}'})
        if oti_rl and 'uncertainty_subscriber' in locals() and rclpy.ok():
            # This was a bug, shutdown should not be called here.
            # It's already commented out in oti_rl.py, but ensuring it's not here.
            pass
        return

    # --- 3. 메인 루프 ---
    try:
        noise_t_raw = None
        
        # --- Find the last saved RL step to resume numbering ---
        last_step = 22000
        uncertainty_entered_step = 0
        uncertainty_mode_timer = 0
        if oti_rl:
            rl_model_dir = os.path.join("/root/src/backend/fiper/data", str(task['id']), "rl_models")
            if os.path.exists(rl_model_dir):
                try:
                    # Find all step numbers from filenames like 'actor_step_12000.pth'
                    step_numbers = [
                        int(re.search(r'actor_step_(\d+)\.pth', f).group(1))
                        for f in os.listdir(rl_model_dir)
                        if re.search(r'actor_step_(\d+)\.pth', f)
                    ]
                    if step_numbers:
                        last_step = max(step_numbers)
                        log_msg = f"Found last RL model checkpoint at step {last_step}. Resuming from there."
                        print(log_msg)
                        socketio_instance.emit('log_checkpoint_test', {'log': log_msg})
                except Exception as e:
                    print(f"Could not parse last RL step, starting from 0. Error: {e}")

        step_num = last_step
        home_pose = task.get('home_pose')
        episode_reward = 0.0

        # --- Pre-define frequencies for smooth probing motion ---
        joint_len = sum(agent.joint_len for agent in agents)
        # Create a unique frequency for each joint for complex, smooth exploration
        probing_freqs = np.linspace(0.1, 0.4, joint_len) * 2 * np.pi
        smoothed_amplitude_scale = 0.0 # For smoothing the noise amplitude

        policy.reset()
        if home_pose is not None:
            for agent in env.agents:
                agent.move_to(home_pose[str(agent.id)])
        time.sleep(3)
        ts = env.reset()
        # Initialize smoothed_action with the robot's initial joint positions for EMA filtering
        initial_qpos = np.concatenate([item['qpos'] for item in ts.observation['robot_states'].values()])
        smoothed_action = initial_qpos
        socketio_instance.emit('log_checkpoint_test', { 'log': 'Robot moved to homepose' })
        
        while not task_control['stop']:
            if step_num % (max_timesteps * 2) == 0 and oti_rl and step_num != 0: # Reset episode every 2*max_timesteps
                print(f"Total Reward: {episode_reward:.4f}")
                socketio_instance.emit('log_record_episode', { 'log': f'Episode finished. Total Reward: {episode_reward:.4f}' })
                episode_reward = 0.0
                policy.reset()
                if home_pose is not None:
                    for agent in env.agents:
                        agent.move_to(home_pose[str(agent.id)])
                time.sleep(3)
                ts = env.reset()
                # Initialize smoothed_action with the robot's initial joint positions for EMA filtering
                initial_qpos = np.concatenate([item['qpos'] for item in ts.observation['robot_states'].values()])
                smoothed_action = initial_qpos
                socketio_instance.emit('log_checkpoint_test', { 'log': 'Robot moved to homepose' })
                
            # === a. 현재 상태(state_t) 계산 (항상 실행) ===
            obs_t = ts.observation
            with torch.no_grad():
                qpos_t = torch.from_numpy(np.concatenate([item['qpos'] for item in obs_t['robot_states'].values()])).float().cuda().unsqueeze(0)

                policy_input_t = {'observation.state': qpos_t}
                for sensor in sensors:
                    image = obs_t['images'][f'sensor_{sensor["id"]}']
                    image = fetch_image_with_config(image, {'resize': task['sensor_img_size']})
                    image = process_image(image, vision_backbone, to_cuda=True)
                    policy_input_t[f'observation.images.sensor_{sensor["id"]}'] = image.unsqueeze(0)

                state_t = policy.select_action(policy_input_t).squeeze(0).cpu().numpy()

                if noise_t_raw is None:
                    noise_t_raw = np.zeros_like(state_t)

            # === b. 최종 행동(final_action) 결정 (조건부 로직) ===
            if oti_rl:
                executor.spin_once(timeout_sec=0.01)
                uncertainty = uncertainty_subscriber.latest_score

                # --- State-based noise generation logic ---

                # 1. High uncertainty is the highest priority: Stop any other exploratory action.
                if uncertainty > 1.2:
                    # --- Generate a smooth, continuous "probing" motion using sine waves ---
                    # The RL agent determines the overall magnitude of the exploration.
                    if uncertainty_mode_timer == 0:
                        uncertainty_entered_step = step_num
                    if uncertainty_mode_timer % 30 == 0:
                        policy.reset()  # Reset policy internal states to avoid bias from previous steps
                        amplitude_vector = rl_agent.select_action(state_t)
                    # Use the norm of the agent's output as a single scaling factor for the amplitude.
                    amplitude_scale = np.linalg.norm(amplitude_vector) * 0.15 # Adjust 0.15 to control max exploration magnitude
                    # Generate a smooth, multi-dimensional sinusoidal noise using the pre-defined frequencies.
                    # noise_t = amplitude_scale * np.sin(probing_freqs * step_num)
                    noise_t = amplitude_scale * np.sin(probing_freqs * step_num)
                    
                    uncertainty_mode_timer += 1
                # elif move_reward < 0.1 and not escape_mode_active:
                #     print(f"ENTERING ESCAPE MODE due to low move_reward: {move_reward:.4f}")
                #     escape_mode_active = True
                #     escape_mode_timer = 20  # Set duration for 30 steps
                #     # Generate a base noise direction for the entire maneuver
                #     noise_t_raw = rl_agent.select_action(state_t)
                #     noise_t = np.zeros_like(state_t) # Start with zero noise on the first step of entering
                # elif escape_mode_active and escape_mode_timer > 0:
                #     # We are IN escape mode, continue the sinusoidal wiggle
                #     oscillation_progress = (20 - escape_mode_timer) / 20.0
                #     oscillation_factor = np.sin(oscillation_progress * 2 * np.pi)
                    
                #     noise_t = 0.3 * noise_t_raw * oscillation_factor
                    
                #     escape_mode_timer -= 1
                #     if escape_mode_timer == 0:
                #         print("EXITING ESCAPE MODE")
                #         escape_mode_active = False  # Exit mode when timer is up
                else:
                    noise_t = np.zeros_like(state_t)
                    uncertainty_mode_timer = 0  # Reset uncertainty timer when not in high uncertainty mode

                noise_t[-1] = 0 # Do not apply noise to the gripper
                final_action = state_t + noise_t
            else:
                final_action = state_t

            # === c. 로봇 제어 (EMA 필터 적용) ===
            smoothed_action = smooth_factor * smoothed_action + (1.0 - smooth_factor) * final_action

            start_action_id = 0
            for agent in env.agents:
                target_qpos = smoothed_action[start_action_id : start_action_id + agent.joint_len]
                agent.move_step(target_qpos)
                start_action_id += agent.joint_len
            
            time.sleep(0.01)
            ts_next = env.record_step()

            # === d. OTI-RL 학습 (조건부 실행) ===
            if oti_rl:
                # 보상 계산
                executor.spin_once(timeout_sec=0.01)
                uncertainty = uncertainty_subscriber.latest_score
                noise_reward = noise_reward_coeff * np.linalg.norm(noise_t)
                uncertainty_penalty = -uncertainty_penalty_coeff * uncertainty
                move_reward = move_reward_coeff * np.linalg.norm(np.concatenate([item['qpos'] for item in ts_next.observation['robot_states'].values()]) - 
                                                                 np.concatenate([item['qpos'] for item in ts.observation['robot_states'].values()]))
                reward_t = noise_reward + uncertainty_penalty + move_reward

                # 다음 상태(state_t+1) 계산
                with torch.no_grad():
                    obs_t1 = ts_next.observation
                    qpos_t1 = torch.from_numpy(np.concatenate([item['qpos'] for item in obs_t1['robot_states'].values()])).float().cuda().unsqueeze(0)
                    policy_input_t1 = {'observation.state': qpos_t1}
                    for sensor in sensors:
                        image = obs_t1['images'][f'sensor_{sensor["id"]}']
                        image = fetch_image_with_config(image, {'resize': task['sensor_img_size']})
                        image = process_image(image, vision_backbone, to_cuda=True)
                        policy_input_t1[f'observation.images.sensor_{sensor["id"]}'] = image.unsqueeze(0)
                    state_t1 = policy.select_action(policy_input_t1).squeeze(0).cpu().numpy()
                
                # 리플레이 버퍼 저장 및 학습
                replay_buffer.push(state_t, noise_t, reward_t, state_t1, done=False)
                if len(replay_buffer) > rl_batch_size:
                    rl_agent.update_parameters(replay_buffer, rl_batch_size)
                    # print(f"OTI-RL updated at step {step_num}")

                # Model Saving Logic
                if step_num > 0 and step_num % 2000 == 0:

                    # os.makedirs(os.path.join("src/backend/fiper/data", str(task['id']), "rl_models"), exist_ok=True)
                    save_dir = os.path.join("/root/src/backend/fiper/data", str(task['id']), "rl_models")
                    os.makedirs(save_dir, exist_ok=True)

                    actor_path = os.path.join(save_dir, f"actor_step_{step_num}.pth")
                    critic1_path = os.path.join(save_dir, f"critic1_step_{step_num}.pth")
                    critic2_path = os.path.join(save_dir, f"critic2_step_{step_num}.pth")

                    torch.save(rl_agent.actor.state_dict(), actor_path)
                    torch.save(rl_agent.critic1.state_dict(), critic1_path)
                    torch.save(rl_agent.critic2.state_dict(), critic2_path)

                    print(f"RL models saved at step {step_num} to {save_dir}")

                    socketio_instance.emit('log_checkpoint_test', {
                        'log': f"RL models saved at step {step_num} to {save_dir}"
                    })

                episode_reward += reward_t
                # socketio_instance.emit('log_checkpoint_test', {
                #     'log': f'T:{step_num}, Reward:{reward_t:.4f} (Noise:{noise_reward:.4f}, Uncert:{uncertainty_penalty:.4f}), Total:{episode_reward:.4f}'
                # })

                # print(f"T:{step_num}, Reward:{reward_t:.4f} (Noise:{noise_reward:.4f}, Uncert:{uncertainty_penalty:.4f}), Move: {move_reward:.4f} Total:{episode_reward:.4f}")

            # 다음 스텝 준비
            
            step_num += 1
            ts = ts_next


    except Exception as e:
        import traceback
        error_string = traceback.format_exc()
        print(f"Error in main loop: {error_string}")
        socketio_instance.emit('log_checkpoint_test', { 'log': f'Error in main loop: {error_string}'})

    finally:
        # --- 4. 종료 처리 ---
        if oti_rl and 'uncertainty_subscriber' in locals():
            uncertainty_subscriber.destroy_node()
            # The main application will handle the final rclpy shutdown.
            # if rclpy.ok():
            #     rclpy.shutdown()
            if executor is not None:
                executor.shutdown()
        
        gc.collect()
        torch.cuda.empty_cache()

    return
