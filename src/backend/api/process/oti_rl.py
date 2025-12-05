import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal
import numpy as np
import random
from collections import deque
import time
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from ...lerobot.policies.act.modeling_act import ACTPolicy
from ...lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from ...lerobot.policies.pi0.modeling_pi0 import PI0Policy
from ...utils.image_parser import fetch_image_with_config
from ...policies.utils import process_image
from ...env.env import Env

# --- ROS2 Subscriber for Uncertainty Score ---

class UncertaintySubscriber(Node):
    """ROS2 노드를 상속하여 uncertainty score를 구독하는 클래스."""
    def __init__(self):
        super().__init__('uncertainty_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            '/failure_detection/uncertainty_score',
            self.listener_callback,
            10)
        self.latest_score = 0.0

    def listener_callback(self, msg):
        self.latest_score = msg.data

# --- Replay Buffer ---

class ReplayBuffer:
    """경험(Experience)을 저장하고 샘플링하는 리플레이 버퍼 클래스."""
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        state, action, reward, next_state, done = zip(*random.sample(self.buffer, batch_size))
        return np.array(state), np.array(action), reward, np.array(next_state), done

    def __len__(self):
        return len(self.buffer)

# --- SAC (Soft Actor-Critic) Agent for Continuous Actions ---

LOG_SIG_MAX = 2
LOG_SIG_MIN = -20
epsilon = 1e-6

class Actor(nn.Module):
    """상태를 받아 행동(noise)의 분포(mean, std)를 출력하는 액터 네트워크."""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Actor, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.mean_linear = nn.Linear(hidden_dim, action_dim)
        self.log_std_linear = nn.Linear(hidden_dim, action_dim)

    def forward(self, state):
        x = self.net(state)
        mean = self.mean_linear(x)
        log_std = self.log_std_linear(x)
        log_std = torch.clamp(log_std, min=LOG_SIG_MIN, max=LOG_SIG_MAX)
        return mean, log_std

    def sample(self, state):
        mean, log_std = self.forward(state)
        std = log_std.exp()
        normal = Normal(mean, std)
        # Reparameterization Trick
        x_t = normal.rsample()
        y_t = torch.tanh(x_t)
        action = y_t
        log_prob = normal.log_prob(x_t)
        # Enforcing Action Bound
        log_prob -= torch.log((1 - y_t.pow(2)) + epsilon)
        log_prob = log_prob.sum(1, keepdim=True)
        mean = torch.tanh(mean)
        return action, log_prob, mean

class Critic(nn.Module):
    """상태와 행동(noise)을 받아 Q-값을 출력하는 크리틱 네트워크."""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Critic, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, state, action):
        x = torch.cat([state, action], 1)
        return self.net(x)

class SACAgent:
    """연속 행동 공간을 위한 SAC 에이전트."""
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, tau=0.005, alpha=0.2):
        self.gamma = gamma
        self.tau = tau
        self.alpha = alpha

        # Actor
        self.actor = Actor(state_dim, action_dim).cuda()
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)

        # Critic
        self.critic1 = Critic(state_dim, action_dim).cuda()
        self.critic2 = Critic(state_dim, action_dim).cuda()
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=lr)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=lr)
        
        # Target Critic
        self.target_critic1 = Critic(state_dim, action_dim).cuda()
        self.target_critic2 = Critic(state_dim, action_dim).cuda()
        self.target_critic1.load_state_dict(self.critic1.state_dict())
        self.target_critic2.load_state_dict(self.critic2.state_dict())

    def select_action(self, state, evaluate=False):
        state = torch.FloatTensor(state).unsqueeze(0).cuda()
        if evaluate:
            _, _, action = self.actor.sample(state)
        else:
            action, _, _ = self.actor.sample(state)
        return action.detach().cpu().numpy()[0]

    def update_parameters(self, memory, batch_size):
        if len(memory) < batch_size:
            return

        states, actions, rewards, next_states, dones = memory.sample(batch_size)

        states = torch.FloatTensor(states).cuda()
        actions = torch.FloatTensor(actions).cuda()
        rewards = torch.FloatTensor(rewards).cuda().unsqueeze(1)
        next_states = torch.FloatTensor(next_states).cuda()
        dones = torch.FloatTensor(np.float32(dones)).cuda().unsqueeze(1)

        with torch.no_grad():
            next_state_actions, next_state_log_pi, _ = self.actor.sample(next_states)
            q1_next_target = self.target_critic1(next_states, next_state_actions)
            q2_next_target = self.target_critic2(next_states, next_state_actions)
            min_q_next_target = torch.min(q1_next_target, q2_next_target) - self.alpha * next_state_log_pi
            next_q_value = rewards + (1 - dones) * self.gamma * min_q_next_target

        q1 = self.critic1(states, actions)
        q2 = self.critic2(states, actions)
        q1_loss = F.mse_loss(q1, next_q_value)
        q2_loss = F.mse_loss(q2, next_q_value)

        self.critic1_optimizer.zero_grad()
        q1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        q2_loss.backward()
        self.critic2_optimizer.step()

        pi, log_pi, _ = self.actor.sample(states)
        q1_pi = self.critic1(states, pi)
        q2_pi = self.critic2(states, pi)
        min_q_pi = torch.min(q1_pi, q2_pi)
        
        actor_loss = ((self.alpha * log_pi) - min_q_pi).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Soft update for target critics
        for target_param, param in zip(self.target_critic1.parameters(), self.critic1.parameters()):
            target_param.data.copy_(param.data * self.tau + target_param.data * (1.0 - self.tau))
        for target_param, param in zip(self.target_critic2.parameters(), self.critic2.parameters()):
            target_param.data.copy_(param.data * self.tau + target_param.data * (1.0 - self.tau))

# --- Main Training Loop ---

# def oti_rl(
#     node,
#     checkpoint,
#     task,
#     policy_obj,
#     agents,
#     sensors,
#     socketio_instance,
#     task_control,
#     max_timesteps,
#     ):
#     """OTI-RL 메인 학습 루프 (Continuous Noise)."""
    
#     # --- 1. ROS2 및 환경 설정 ---
#     if not rclpy.ok():
#         rclpy.init(args=None)
#     uncertainty_subscriber = UncertaintySubscriber()
    
#     # --- 2. 기본 정책(Base Policy) 로드 ---
#     try:
#         if checkpoint['is_base_model']:
#             ckpt_dir = "lerobot/pi0_base"
#         else:
#             ckpt_dir = os.path.join("/root/src/backend/checkpoints", str(checkpoint['id']))

#         if policy_obj['type'] == 'ACT':
#             base_policy = ACTPolicy.from_pretrained(ckpt_dir)
#         elif policy_obj['type'] == 'Diffusion':
#             base_policy = DiffusionPolicy.from_pretrained(ckpt_dir)
#         elif policy_obj['type'] == 'PI0':
#             base_policy = PI0Policy.from_pretrained(ckpt_dir)
#         base_policy.cuda()
#         base_policy.eval()
        
#         socketio_instance.emit('log_oti_rl', {'log': f'Base Policy loaded from {ckpt_dir}'})

#     except Exception as e:
#         socketio_instance.emit('log_oti_rl', {'log': f'Error loading base policy: {e}'})
#         return

#     # --- 3. RL 에이전트 및 환경 초기화 ---
#     env = Env(node, agents, sensors)
#     action_dim = sum(agent.joint_len for agent in agents)
#     state_dim = action_dim # 상태는 기본 정책의 액션이므로 차원이 동일
    
#     # action_dim은 noise 벡터의 차원
#     rl_agent = SACAgent(state_dim=state_dim, action_dim=action_dim)
#     replay_buffer = ReplayBuffer(capacity=10000)
#     batch_size = 128
    
#     # 하이퍼파라미터 (튜닝 필요)
#     noise_reward_coeff = 0.5
#     uncertainty_penalty_coeff = 1.0

#     ts = env.reset()
#     episode_reward = 0
    
#     # --- 4. 메인 학습 루프 ---
#     try:
#         for t in range(max_timesteps):
#             if task_control['stop']:
#                 break
            
#             # --- a. 상태(State) 결정: 기본 정책의 Action ---
#             obs = ts.observation
#             qpos = torch.from_numpy(np.concatenate([item['qpos'] for item in obs['robot_states'].values()])).float().cuda().unsqueeze(0)
            
#             policy_input = {'observation.state': qpos}
#             vision_backbone = policy_obj.get('vision_backbone')
#             for sensor in sensors:
#                 image = obs['images'][f'sensor_{sensor["id"]}']
#                 image = fetch_image_with_config(image, {'resize': task['sensor_img_size']})
#                 image = process_image(image, vision_backbone, to_cuda=True)
#                 policy_input[f'observation.images.sensor_{sensor["id"]}'] = image.unsqueeze(0)

#             with torch.no_grad():
#                 base_action = base_policy.select_action(policy_input)
#             state = base_action.squeeze(0).cpu().numpy()

#             # --- b. RL 에이전트의 행동(Noise) 선택 ---
#             noise = rl_agent.select_action(state)

#             # --- c. 최종 행동 계산 및 실행 ---
#             final_action = state + noise
#             # 중요: 로봇의 물리적 한계를 넘지 않도록 final_action을 클리핑해야 할 수 있습니다.
#             # final_action = np.clip(final_action, env.action_space.low, env.action_space.high)
            
#             start_action_id = 0
#             for agent in env.agents:
#                 target_qpos = final_action[start_action_id : start_action_id + agent.joint_len]
#                 agent.move_step(target_qpos)
#                 start_action_id += agent.joint_len
            
#             time.sleep(0.1)

#             # --- d. 보상(Reward) 계산 ---
#             rclpy.spin_once(uncertainty_subscriber, timeout_sec=0.01)
#             uncertainty = uncertainty_subscriber.latest_score
            
#             noise_reward = noise_reward_coeff * np.linalg.norm(noise)
#             uncertainty_penalty = -uncertainty_penalty_coeff * uncertainty
#             reward = noise_reward + uncertainty_penalty

#             # --- e. 다음 상태 관측 및 저장 ---
#             ts_next = env.record_step()
#             obs_next = ts_next.observation
#             qpos_next = torch.from_numpy(np.concatenate([item['qpos'] for item in obs_next['robot_states'].values()])).float().cuda().unsqueeze(0)
            
#             policy_input_next = {'observation.state': qpos_next}
#             for sensor in sensors:
#                 image_next = obs_next['images'][f'sensor_{sensor["id"]}']
#                 image_next = fetch_image_with_config(image_next, {'resize': task['sensor_img_size']})
#                 image_next = process_image(image_next, vision_backbone, to_cuda=True)
#                 policy_input_next[f'observation.images.sensor_{sensor["id"]}'] = image_next.unsqueeze(0)
                
#             with torch.no_grad():
#                 base_action_next = base_policy.select_action(policy_input_next)
#             next_state = base_action_next.squeeze(0).cpu().numpy()
            
#             done = False 
            
#             replay_buffer.push(state, noise, reward, next_state, done)
            
#             # --- f. RL 에이전트 학습 ---
#             rl_agent.update_parameters(replay_buffer, batch_size)
            
#             ts = ts_next
#             episode_reward += reward
            
#             socketio_instance.emit('log_oti_rl', {'log': f"T: {t}, Reward: {reward:.4f}, Noise Mag: {np.linalg.norm(noise):.4f}, Uncertainty: {uncertainty:.4f}"})
    
#     finally:
#         # --- 5. 종료 처리 ---
#         uncertainty_subscriber.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()
#         socketio_instance.emit('log_oti_rl', {'log': "OTI-RL training finished."})

#     return


# import os
# import torch
# import pickle
# import numpy as np
# import time
# import cv2
# from einops import rearrange
# from ...lerobot.configs.types import PolicyFeature, FeatureType
# from ...utils.image_parser import fetch_image_with_config

# from ...policies.utils import make_policy, VISION_BACKBONE_MAP, process_image
# from ...env.env import Env

# from ...lerobot.policies.act.modeling_act import ACTPolicy
# from ...lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
# from ...lerobot.policies.pi0.modeling_pi0 import PI0Policy

# from .oti_rl import SACAgent, ReplayBuffer, UncertaintySubscriber

# import torchvision.transforms as transforms
# from transformers import AutoImageProcessor

# import rclpy

# import gc