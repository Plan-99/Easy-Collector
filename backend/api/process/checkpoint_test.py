import os
import torch
import pickle
import numpy as np
import time
import cv2
from einops import rearrange
from lerobot.configs.types import PolicyFeature, FeatureType
from ...utils.image_parser import fetch_image_with_config

from ...policies.utils import VISION_BACKBONE_MAP, process_image, relative_trajectory_to_delta, make_easytrainer_processors, _absolute_action_dims_from_features
from ...configs.global_configs import resolve_checkpoint_dir
from collections import deque
from ...bridge.remote_env import RemoteEnv

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

from .oti_rl import SACAgent, ReplayBuffer
from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb

import torchvision.transforms as transforms
from transformers import AutoImageProcessor

import re

import gc
import threading


def _move_to_homepose(agents, home_pose, task_control, socketio_instance, timeout=30.0, duration=5.0, settle_sec=0.0):
    """home_pose가 None이거나 매핑이 없으면 즉시 리턴.

    moving_homepose 이벤트로 프론트엔드 오버레이를 켰다 끈다.

    settle_sec: home 도달 후 추가로 대기할 시간 (s). 기본 0. 그리퍼/팔 안정화나
    물체가 정착할 시간이 필요한 시나리오에서 사용. stop 신호 시엔 무시.

    NOTE: 이전엔 폴링 후 `time.sleep(0.5)` 안정화 + sequential move_to 호출이 있었는데,
    호출 측에서 어차피 wait_for_images + joint_states polling 으로 자연 대기하므로
    redundant. agent.move_to 는 thread_pool로 동시 호출해 RPC 누적 시간을 단축.
    """
    if home_pose is None:
        return
    socketio_instance.emit('moving_homepose', {'moving': True})
    try:
        # agent별 move_to를 동시 호출 (sequential RPC 누적 방지)
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=max(1, len(agents))) as _pool:
            _futs = []
            for agent in agents:
                target = home_pose.get(str(agent.id))
                if target is not None:
                    _futs.append(_pool.submit(agent.move_to, target, duration=duration))
            for _f in _futs:
                try:
                    _f.result(timeout=5.0)
                except Exception as _e:
                    print(f"[checkpoint_test] move_to dispatch failed: {_e}", flush=True)
        start_wait = time.time()
        while time.time() - start_wait < timeout:
            if task_control.get('stop'):
                # 비동기 move_to 가 background 에서 명령을 계속 쏘지 못하게.
                for a in agents:
                    try:
                        a.cancel_move_to()
                    except Exception:
                        pass
                return
            if not any(a.is_moving for a in agents):
                break
            time.sleep(0.1)
        # 도달 후 추가 안정화 대기. stop 신호가 들어오면 즉시 빠져나감.
        if settle_sec and settle_sec > 0:
            _settle_end = time.time() + float(settle_sec)
            while time.time() < _settle_end:
                if task_control.get('stop'):
                    return
                time.sleep(min(0.1, _settle_end - time.time()))
    finally:
        socketio_instance.emit('moving_homepose', {'moving': False})


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
    move_homepose_duration=5.0,
    move_homepose_settle_sec=0.0,
    hz=10,
    re_inference_steps=1,
    temporal_ensemble_coeff=0.01,
    action_type=None,
    language_instruction=None,  # optional VLA prompt for PI0.5; falls back to task['name']
    inference_episode_len=None,  # planner feature: inference 시 별도 episode_len 지정
    preloaded=None,
    go_home_first=True,
    ):

    agents = sorted(agents, key=lambda a: a.id)
    oti_rl = False
    move_reward = 1.0

    # --- 1. 초기 설정 ---
    try:
        gc.collect()
        torch.cuda.empty_cache()

        from concurrent.futures import ThreadPoolExecutor
        thread_pool = ThreadPoolExecutor(max_workers=len(agents))
        executor = None
        
        ckpt_dir = resolve_checkpoint_dir(checkpoint['id'])

        action_key = action_type or policy_obj.get('settings', {}).get('action_type') or checkpoint.get('train_settings', {}).get('action_type', 'qaction')
        # 신/구 action_key 별칭 통일: 'qaction'→'joint', 'ee_delta_action'→'ee_delta'.
        # 아래에서 분기 비교할 때 양쪽 모두 인식되도록 normalize.
        _ACTION_KEY_ALIAS = {'qaction': 'joint', 'joint': 'joint',
                              'ee_delta_action': 'ee_delta', 'ee_delta': 'ee_delta',
                              'relative_ee_pos': 'relative_ee_pos',
                              'relative_joint_pos': 'relative_joint_pos'}
        action_key_norm = _ACTION_KEY_ALIAS.get(action_key, action_key)
        _obs_keys = policy_obj.get('settings', {}).get('obs_state_keys')
        if _obs_keys is None:
            _obs_keys = checkpoint.get('train_settings', {}).get('obs_state_keys')
        obs_state_keys = _obs_keys if _obs_keys is not None else ['qpos']
        use_relative_trajectory = checkpoint.get('train_settings', {}).get('use_relative_trajectory', False)
        has_succeed = checkpoint.get('train_settings', {}).get('has_succeed', False)

        # Either reuse a CPU-resident model from the planner prefetch cache, or
        # build everything fresh from disk. Per-block config (temporal ensemble,
        # n_action_steps) is applied AFTER this branch so the same cached policy
        # can be driven with different settings across blocks.
        if preloaded is not None:
            policy = preloaded['policy']
            preprocessor = preloaded['preprocessor']
            postprocessor = preloaded['postprocessor']
            ood_cpu = preloaded.get('ood') or {}
            print(f'[INFER] Using preloaded checkpoint {checkpoint["id"]} from CPU cache')
        else:
            if policy_obj['type'] == 'ACT':
                policy = ACTPolicy.from_pretrained(ckpt_dir)
            elif policy_obj['type'] == 'Diffusion':
                policy = DiffusionPolicy.from_pretrained(ckpt_dir)
            elif policy_obj['type'] == 'PI05':
                from lerobot.policies.pi05.modeling_pi05 import PI05Policy
                policy = PI05Policy.from_pretrained(ckpt_dir)

            # Load preprocessor/postprocessor saved alongside the model. These wrap input
            # observations with Normalize and the model output action with Unnormalize, using
            # the dataset stats captured at training time. Required for correct predictions
            # on tiny-scale joints (e.g. gripper) — without normalization the L1 loss is
            # dominated by larger joints and the gripper is essentially untrained.
            # Older checkpoints (no processor files) return (None, None) so the call site
            # can stay uniform and fall back to raw I/O.
            preprocessor, postprocessor = make_easytrainer_processors(
                policy_type=policy_obj['type'],
                cfg=policy.config,
                pretrained_path=ckpt_dir,
            )
            ood_cpu = {}
            ood_features_path = os.path.join(ckpt_dir, 'ood_features.npz')
            if os.path.exists(ood_features_path) and hasattr(policy, 'enable_feature_caching'):
                ood_data = np.load(ood_features_path)
                if 'image_features' in ood_data:
                    ood_cpu['image_feats'] = torch.from_numpy(ood_data['image_features']).float()
                if 'state_features' in ood_data:
                    ood_cpu['state_feats'] = torch.from_numpy(ood_data['state_features']).float()
                if 'image_dist_sorted' in ood_data:
                    ood_cpu['image_dist_sorted'] = ood_data['image_dist_sorted']
                if 'state_dist_sorted' in ood_data:
                    ood_cpu['state_dist_sorted'] = ood_data['state_dist_sorted']

        # Per-block re-inference / temporal ensemble setup. Always reapply because
        # the same cached policy may be reused with different params across blocks.
        # ACT uses ``config.temporal_ensemble_coeff`` (not the attribute) to choose
        # between ensembler and action-queue branches in select_action/reset, so we
        # toggle the config flag rather than nulling the attribute.
        if policy_obj['type'] == 'ACT':
            # re_inference_steps=1: temporal ensemble 활성화 (매 스텝 추론 + 가중 평균)
            # re_inference_steps>1: temporal ensemble 비활성화, N스텝마다 재추론
            if re_inference_steps == 1:
                from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
                policy.config.temporal_ensemble_coeff = temporal_ensemble_coeff
                policy.temporal_ensembler = ACTTemporalEnsembler(temporal_ensemble_coeff, policy.config.chunk_size)
            else:
                policy.config.temporal_ensemble_coeff = None
                policy.config.n_action_steps = re_inference_steps
                policy._action_queue = deque([], maxlen=re_inference_steps)
        elif policy_obj['type'] == 'Diffusion':
            policy.config.n_action_steps = re_inference_steps
            policy._queues['action'] = deque(maxlen=re_inference_steps)

        policy.cuda()
        policy.eval()
        print(f'Loaded Policy from {ckpt_dir}, hz: {hz}, re_inference_steps: {re_inference_steps}, action_key: {action_key}')

        if preprocessor is None:
            print(f'[INFER][WARN] No processor pipeline found at {ckpt_dir}. '
                  f'Falling back to raw model I/O — retrain to enable normalization.')
        else:
            print(f'[INFER] Loaded preprocessor/postprocessor from {ckpt_dir} '
                  f'(action_key_norm={action_key_norm})')

        # OOD reference tensors — keep originals on CPU (so the cache survives),
        # ship a CUDA copy into the loop. The CUDA copies are freed in finally.
        ood_image_feats = ood_cpu['image_feats'].cuda() if ood_cpu.get('image_feats') is not None else None
        ood_state_feats = ood_cpu['state_feats'].cuda() if ood_cpu.get('state_feats') is not None else None
        ood_image_dist_sorted = ood_cpu.get('image_dist_sorted')
        ood_state_dist_sorted = ood_cpu.get('state_dist_sorted')
        if (ood_image_feats is not None or ood_state_feats is not None) and hasattr(policy, 'enable_feature_caching'):
            policy.enable_feature_caching(True)
            print(f'[OOD] Loaded reference features: image={ood_image_feats.shape if ood_image_feats is not None else None}, state={ood_state_feats.shape if ood_state_feats is not None else None}')

        # Grad-CAM 활성화
        gradcam_enabled = hasattr(policy, 'enable_gradcam')
        if gradcam_enabled:
            policy.enable_gradcam(True)
            print('[Grad-CAM] Enabled')

        # 환경 및 RL 에이전트 초기화
        state_dim = sum(agent.joint_len for agent in agents)
        tutorial = any((s.get('settings') or {}).get('is_tutorial') for s in (sensors or []))
        env = RemoteEnv(agents, sensors, tutorial=tutorial)
        vision_backbone = policy_obj.get('vision_backbone')
        # 추론 1 에피소드 길이 — move_homepose가 켜져 있으면 이 step 만큼 추론한 뒤
        # 다시 home으로 돌아간다. 프론트에서 명시한 값이 있으면 그것을 쓰고,
        # 없으면 task의 episode_len * 2를 기본값으로 사용.
        _base_ep_len = int(task.get('episode_len', 300))
        if inference_episode_len:
            episode_len = max(1, int(inference_episode_len))
        else:
            episode_len = max(1, _base_ep_len * 2)

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

            # UncertaintySubscriber는 ROS2 컨테이너에서 gRPC로 관리
            bridge_client = get_bridge_client()
            bridge_client.uncertainty.StartSubscriber(pb.Empty())

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
        if go_home_first:
            _move_to_homepose(env.agents, home_pose, task_control, socketio_instance, duration=move_homepose_duration, settle_sec=move_homepose_settle_sec)
            if task_control['stop']:
                return
            print('Robot moved to homepose')
        # Block until every sensor has produced its first frame; otherwise the
        # first iteration of the main loop hits ``None`` images.
        env.wait_for_images(timeout=10.0)
        # Robots may not have published their first joint_states yet either —
        # poll until each agent has one before the inference loop reads it.
        # NOTE: `agent.joint_states` (cache) 는 subscribe_state_stream gRPC stream
        # 의 callback 에서만 채워진다. planner 가 그 stream 을 구독하지 않은 채로
        # checkpoint block 에 들어오면 캐시가 영원히 None 이라 10s timeout 까지
        # idle. 캐시 대신 get_joint_states() RPC 결과로 판정 → 구독 여부와 무관.
        _deadline = time.time() + 10.0
        while time.time() < _deadline:
            ok = True
            for a in agents:
                js = a.get_joint_states()
                if js is None:
                    ok = False
                    break
                a.joint_states = js  # cache 도 채워둠 (다른 컴포넌트 호환)
            if ok:
                break
            time.sleep(0.05)
        ts = env.reset()
        if move_homepose:
            socketio_instance.emit('inference_progress', {
                'progress': 0.0, 'step': 0, 'episode_len': episode_len,
            })

        prev_qpos_dict = {}
        for agent in agents:
            if agent.role != 'tool' and agent.ik_solver is not None:
                prev_qpos_dict[agent.id] = ts.observation['robot_states'][agent.id]['qpos']
        rel_action_queue = deque()  # relative_ee_pos용 delta action queue
        # PI05 + relative_joint_pos: whole-chunk absolute joint targets cached after one
        # inference (delta + current qpos). Subsequent loop iterations pop from here
        # without re-inferring.
        pi05_chunk_queue = deque()
        # Soft start: 첫 추론 cycle 의 cmd 는 정책의 raw 첫 chunk[0] 이라 학습 home
        # 을 가리킬 수 있고, 추론 cadence(100ms) 안에서 큰 delta 가 되어 ServoJ
        # cmdT=5ms 안에 큰 가속도 요구 → 덜컹. 첫 cycle 만 move_to 의 smoothstep 으로
        # 부드럽게 도달시킨 뒤, main loop 2 번째 cycle 부터는 robot 이 이미 target 근처
        # 라 작은 delta 가 만들어져 keyboard teleop 처럼 자연스러움. qaction 경로만
        # 해당 — ee_delta/relative_ee_pos 는 본질적으로 delta 라 첫 cmd 가 작음.
        first_step = True
        first_step_duration = 0.3
        start = time.time()
        while not task_control['stop']:
            if step_num % episode_len == 0 and step_num != 0 and move_homepose:
                print(f"Episode finished. Total Reward: {episode_reward:.4f}")
                episode_reward = 0.0
                policy.reset()
                rel_action_queue.clear()
                pi05_chunk_queue.clear()  # PI0.5 chunk queue도 reset 시 비우기
                _move_to_homepose(env.agents, home_pose, task_control, socketio_instance, duration=move_homepose_duration, settle_sec=move_homepose_settle_sec)
                if task_control['stop']:
                    return
                ts = env.reset()
                for agent in agents:
                    if agent.role != 'tool' and agent.ik_solver is not None:
                        prev_qpos_dict[agent.id] = ts.observation['robot_states'][agent.id]['qpos']
                # 새 episode 시작 — soft start 다시 활성화 (홈에서 정책 첫 cmd 까지 부드럽게).
                first_step = True
                print('Robot moved to homepose')
                socketio_instance.emit('inference_progress', {
                    'progress': 0.0, 'step': 0, 'episode_len': episode_len,
                })

            if move_homepose:
                ep_step = step_num % episode_len
                # progress emit은 5 step마다(또는 episode 막판) — 너무 자주 보내면 UI 깜빡임.
                if ep_step % 5 == 0 or ep_step == episode_len - 1:
                    socketio_instance.emit('inference_progress', {
                        'progress': ep_step / episode_len,
                        'step': ep_step,
                        'episode_len': episode_len,
                    })

            # 일정 스텝마다 강제 메모리 정리 (예: 100스텝마다)
            if step_num % 100 == 0:
                gc.collect()
                torch.cuda.empty_cache()
                
            # === a. 현재 상태(state_t) 계산 ===
            obs_t = ts.observation

            # relative_ee_pos: queue에 남은 delta가 있으면 추론 없이 꺼내 씀
            if action_key_norm == 'relative_ee_pos' and len(rel_action_queue) > 0:
                state_t = rel_action_queue.popleft()
            # PI05 + relative_joint_pos: 같은 chunk의 나머지 absolute joint targets
            # 사용 (chunk-inference 시점의 qpos로 이미 더해진 상태). 재추론 없이 pop.
            elif policy_obj['type'] == 'PI05' and action_key_norm == 'relative_joint_pos' and len(pi05_chunk_queue) > 0:
                state_t = pi05_chunk_queue.popleft()
            else:
                with torch.no_grad():
                    # UMI relative trajectory: 매 step마다 현재 EE pose 기준으로 재예측.
                    # temporal ensemble이 다른 기준 frame의 waypoint를 섞지 않도록 policy를 reset.
                    if use_relative_trajectory and action_key_norm == 'ee_delta':
                        policy.reset()
                    if action_key_norm in ('ee_delta', 'relative_ee_pos'):
                        # single_arm: 실제 EE 포즈 변화량 (closed-loop) + tool qpos, tool-only: 현재 qpos
                        obs_parts = []
                        for agent in env.agents:
                            if agent.role != 'tool' and agent.ik_solver is not None:
                                current_qpos = obs_t['robot_states'][agent.id]['qpos']
                                if agent.id in prev_qpos_dict:
                                    fk_delta = agent.compute_fk_delta(current_qpos, prev_qpos_dict[agent.id])
                                    if fk_delta is not None:
                                        ee_obs = np.concatenate([fk_delta[name] for name in agent.ee_names])
                                    else:
                                        ee_obs = np.zeros(len(agent.ee_names) * 6)
                                else:
                                    ee_obs = np.zeros(len(agent.ee_names) * 6)
                                # tool_inner: tool joint qpos를 proprioception에 append
                                if agent.tool_inner:
                                    _, tool_pos = agent.get_joint_and_tool_pos(current_qpos)
                                    if tool_pos is not None:
                                        ee_obs = np.concatenate([ee_obs, np.array(tool_pos)])
                                obs_parts.append(ee_obs)
                            else:
                                obs_parts.append(obs_t['robot_states'][agent.id]['qpos'])
                        qpos_np = np.concatenate(obs_parts)
                    else:
                        qpos_np = np.concatenate([item['qpos'] for item in obs_t['robot_states'].values()])
                    # qpos가 obs_state_keys에 없으면 0으로 채움
                    if 'qpos' not in obs_state_keys:
                        qpos_np = np.zeros_like(qpos_np)
                    # obs_state_keys 에 따라 qvel, qeffort, eepos append.
                    # 학습 시 _build_obs_state 와 동일한 순서로 concat 해야 dim 맞음:
                    # qpos → qvel → qeffort → eepos.
                    extra_obs = []
                    if 'qvel' in obs_state_keys:
                        extra_obs.extend([np.array(agent.get_joint_vel()) for agent in env.agents])
                    if 'qeffort' in obs_state_keys:
                        extra_obs.extend([np.array(agent.get_joint_effort()) for agent in env.agents])
                    if 'eepos' in obs_state_keys:
                        # EE 절대좌표 (x,y,z,rx,ry,rz). tool_inner면 tool joint 도 append.
                        for agent in env.agents:
                            if agent.role == 'tool' or agent.ik_solver is None:
                                continue
                            ee_dict = agent.get_ee_position() or {}
                            for ee_name in (agent.ee_names or []):
                                vals = ee_dict.get(ee_name)
                                if vals:
                                    extra_obs.append(np.array(vals, dtype=np.float32))
                    if extra_obs:
                        qpos_np = np.concatenate([qpos_np] + extra_obs)
                    qpos_t = torch.from_numpy(qpos_np).float().cuda().unsqueeze(0)
                    policy_input_t = {'observation.state': qpos_t}
                    # PI05 needs tokenized language inputs. Prefer user-provided
                    # `language_instruction` (from UI); fall back to task.name if empty
                    # or None so the behavior stays backward-compatible.
                    if policy_obj['type'] == 'PI05':
                        _lang = language_instruction if (language_instruction and str(language_instruction).strip()) else task.get('name', '')
                        policy_input_t['language_instruction'] = _lang
                        # NOTE: do NOT call prepare_pi05_language_tokens here. It writes
                        # `observation.language.tokens` using a divergent (min-max + padded)
                        # state representation that is then overwritten by the preprocessor's
                        # Pi05PrepareStateTokenizerProcessorStep + TokenizerProcessorStep using
                        # the correct quantile + native-dim representation. Removing this dead
                        # call eliminates a foot-gun that would silently revert to bad behavior
                        # if anyone ever short-circuited the preprocessor.
                    print(f"[INPUT] state: {qpos_t.shape} = {qpos_t[0].cpu().numpy()}")
                    for sensor in sensors:
                        image = obs_t['images'][f'sensor_{sensor["id"]}']
                        sensor_id = str(sensor['id'])
                        image = fetch_image_with_config(image, {
                            'sensor_id': str(sensor_id),
                            'sam3': (task.get('sensor_sam3') or {}).get(str(sensor_id)),
                            'resize': task['sensor_img_size'][str(sensor_id)],
                            'cropped_area': task['sensor_cropped_area'][str(sensor_id)],
                            'rotate': task['sensor_rotate'][str(sensor_id)]
                        })
                        # BGR → RGB. ros_image_to_numpy() returns BGR (cv2/OpenCV convention),
                        # but training data on disk is saved as RGB (lerobot_io.py applies
                        # cv2.cvtColor(BGR2RGB) before mp4/png write). process_image() then
                        # passes through PIL.Image.fromarray which interprets uint8 HxWx3 as RGB.
                        # Without this flip, R and B channels are swapped at inference vs training
                        # → red cube looks blue to SigLIP → robot goes to wrong location even though
                        # joint motions look like training (state pathway is unaffected). The export
                        # template (export_templates/ros_inference.py:269-272) already had this fix;
                        # the live inference path (this file) was missing it.
                        if hasattr(image, 'ndim') and image.ndim == 3 and image.shape[2] == 3:
                            image = image[:, :, ::-1]
                            import numpy as _np
                            image = _np.ascontiguousarray(image)
                        # PI05/PaliGemma pretrained weights expect [-1, 1]-ranged pixels.
                        _pixel_range = '-11' if policy_obj['type'] == 'PI05' else '01'
                        image = process_image(image, vision_backbone, to_cuda=True, pixel_range=_pixel_range)
                        policy_input_t[f'observation.images.sensor_{sensor["id"]}'] = image.unsqueeze(0)

                    # Normalize observation inputs (state, images) using train-time stats.
                    # Shapes are already batched + on CUDA, so AddBatchDim/DeviceProcessor
                    # are no-ops; only NormalizerProcessorStep does meaningful work.
                    if preprocessor is not None:
                        # PI05 preprocessor pipeline expects complementary_data['task'].
                        # Training bridged 'language_instruction' → 'task' inside forward_pass;
                        # inference goes through preprocessor() directly, so mirror that
                        # bridge here too. Must be a list (of length=batch) because
                        # Pi05PrepareStateTokenizerProcessorStep iterates over it — if we
                        # pass a bare string, iteration would go char-by-char and break.
                        if policy_obj['type'] == 'PI05' and 'task' not in policy_input_t:
                            _lang = policy_input_t.get('language_instruction') or task.get('name', '')
                            if isinstance(_lang, str):
                                _lang = [_lang]
                            policy_input_t['task'] = _lang
                        policy_input_t = preprocessor(policy_input_t)

                    if action_key_norm == 'relative_ee_pos':
                        # full chunk를 한번에 받아서 delta로 변환 후 queue에 넣음
                        policy.reset()
                        if hasattr(policy, 'predict_action_chunk'):
                            # ACT: predict_action_chunk → (1, chunk_size, action_dim)
                            raw_actions = policy.predict_action_chunk(policy_input_t).squeeze(0)
                        else:
                            # Diffusion/PI0 등: select_action이 이미 single action을 반환
                            # n_action_steps=chunk_size로 설정했으므로 queue를 채운 뒤 전부 꺼냄
                            raw_list = []
                            first = policy.select_action(policy_input_t).squeeze(0)
                            raw_list.append(first)
                            while len(policy._action_queue if hasattr(policy, '_action_queue') else policy._queues.get('action', [])) > 0:
                                q = policy._action_queue if hasattr(policy, '_action_queue') else policy._queues['action']
                                raw_list.append(q.popleft().squeeze(0))
                            raw_actions = torch.stack(raw_list)
                        # Unnormalize the predicted action chunk back to robot units
                        if postprocessor is not None:
                            raw_actions = postprocessor(raw_actions)
                        raw_np = raw_actions.cpu().numpy()
                        # DexUMI 학습 — relative trajectory 는 current EE local frame
                        # 이라 world frame 으로 풀려면 현재 EE pose 가 필요. 단일 arm 가정
                        # (multi-EE 는 추후). tool_inner 면 tool joint 차원도 append.
                        _curr_eepos = None
                        for _agent in env.agents:
                            if _agent.role == 'tool' or _agent.ik_solver is None:
                                continue
                            _ee_dict = _agent.get_ee_position() or {}
                            _ee_name = _agent.ee_names[0] if _agent.ee_names else None
                            if _ee_name and _ee_name in _ee_dict:
                                _curr_eepos = np.array(_ee_dict[_ee_name], dtype=np.float32)
                                break
                        deltas = relative_trajectory_to_delta(raw_np, current_eepos=_curr_eepos)
                        state_t = deltas[0]
                        for i in range(1, len(deltas)):
                            rel_action_queue.append(deltas[i])
                    elif policy_obj['type'] == 'PI05' and action_key_norm == 'relative_joint_pos':
                        # 데이터가 chunk-anchored delta로 학습됐고, runtime processor는
                        # raw → normalize → model → unnormalize 만 함. 모델 출력 =
                        # real-unit delta. robot의 current qpos를 직접 가져와서 delta에
                        # 더해서 absolute joint target 복원 (relative_ee_pos가 current_eepos를
                        # forward kinematics로 가져오는 패턴과 대칭).
                        policy.reset()
                        first = policy.select_action(policy_input_t).squeeze(0)
                        chunk_list = [first]
                        q = policy._action_queue if hasattr(policy, '_action_queue') else policy._queues.get('action')
                        while q and len(q) > 0:
                            chunk_list.append(q.popleft().squeeze(0))
                        raw_actions = torch.stack(chunk_list)
                        if postprocessor is not None:
                            raw_actions = postprocessor(raw_actions)
                        raw_np = raw_actions.cpu().numpy()  # (chunk_size, action_dim) — delta
                        # Robot의 current qpos를 anchor로. obs_state_keys와 무관하게
                        # robot_states에서 직접 읽어옴.
                        anchor_qpos = np.concatenate([
                            item['qpos'] for item in obs_t['robot_states'].values()
                        ])
                        # absolute dims (gripper/tool/done)는 학습 시점 dataset features
                        # 에서 자동 식별. agent.joint_names + has_succeed로 안전하게 재구성.
                        action_dim = raw_np.shape[1]
                        anchor_dim = anchor_qpos.shape[0]
                        _action_names = []
                        for _ag in sorted(agents, key=lambda a: a.id):
                            for _jn in (getattr(_ag, 'joint_names', None) or []):
                                _action_names.append(f"robot_{_ag.id}_{_jn}")
                        _abs_dims = _absolute_action_dims_from_features(_action_names, has_succeed)
                        if _abs_dims:
                            effective_mask = [True] * action_dim
                            for idx in _abs_dims:
                                if 0 <= int(idx) < action_dim:
                                    effective_mask[int(idx)] = False
                        else:
                            shared = min(anchor_dim, action_dim)
                            effective_mask = [True] * shared + [False] * (action_dim - shared)
                        for i, is_delta in enumerate(effective_mask):
                            if is_delta and i < anchor_dim:
                                raw_np[:, i] = raw_np[:, i] + anchor_qpos[i]
                        state_t = raw_np[0]
                        for i in range(1, len(raw_np)):
                            pi05_chunk_queue.append(raw_np[i])
                    else:
                        # relative trajectory 모드: select_action이 반환하는 값은 chunk[0] = T_now→1 = 즉각 delta
                        raw_action = policy.select_action(policy_input_t)
                        # Unnormalize back to robot units before downstream slicing/publishing
                        if postprocessor is not None:
                            raw_action = postprocessor(raw_action)
                        state_t = raw_action.squeeze(0).cpu().numpy()

                    if noise_t_raw is None:
                        noise_t_raw = np.zeros_like(state_t)


            # === OOD scoring (추론이 실행된 스텝에서만) ===
            if (ood_image_feats is not None or ood_state_feats is not None) and hasattr(policy, 'get_cached_features'):
                img_feat, state_feat = policy.get_cached_features()
                if img_feat is not None or state_feat is not None:
                    ood_scores = {}
                    k = 5
                    if img_feat is not None and ood_image_feats is not None:
                        dists = torch.cdist(img_feat, ood_image_feats)  # (1, N)
                        raw_dist = float(dists.topk(k, largest=False).values.mean())
                        if ood_image_dist_sorted is not None and len(ood_image_dist_sorted) > 0:
                            idx = int(np.searchsorted(ood_image_dist_sorted, raw_dist))
                            percentile = idx / len(ood_image_dist_sorted)
                            ood_scores['image'] = round(min(percentile, 1.0), 3)
                            print(f"[OOD DEBUG] image raw_dist={raw_dist:.4f}, searchsorted_idx={idx}/{len(ood_image_dist_sorted)}, percentile={percentile:.4f}, ref_range=[{ood_image_dist_sorted[0]:.4f}, {ood_image_dist_sorted[-1]:.4f}]")
                        else:
                            ood_scores['image'] = raw_dist
                    if state_feat is not None and ood_state_feats is not None:
                        dists = torch.cdist(state_feat, ood_state_feats)  # (1, N)
                        raw_dist = float(dists.topk(k, largest=False).values.mean())
                        if ood_state_dist_sorted is not None and len(ood_state_dist_sorted) > 0:
                            idx = int(np.searchsorted(ood_state_dist_sorted, raw_dist))
                            percentile = idx / len(ood_state_dist_sorted)
                            ood_scores['state'] = round(min(percentile, 1.0), 3)
                            print(f"[OOD DEBUG] state raw_dist={raw_dist:.4f}, searchsorted_idx={idx}/{len(ood_state_dist_sorted)}, percentile={percentile:.4f}, ref_range=[{ood_state_dist_sorted[0]:.4f}, {ood_state_dist_sorted[-1]:.4f}]")
                        else:
                            ood_scores['state'] = raw_dist
                    socketio_instance.emit('ood_score', ood_scores)

            # === Grad-CAM (10스텝마다) ===
            if gradcam_enabled and step_num % 10 == 0 and 'policy_input_t' in dir():
                try:
                    heatmaps = policy.compute_gradcam(policy_input_t)
                    gradcam_payload = {}
                    for cam_idx, heatmap in heatmaps.items():
                        # 히트맵을 원본 이미지에 overlay하여 base64로 변환
                        sensor = sensors[cam_idx] if cam_idx < len(sensors) else None
                        if sensor is not None:
                            import base64
                            sensor_id = str(sensor['id'])
                            resize = task['sensor_img_size'].get(sensor_id, [640, 480])
                            w, h = resize[0], resize[1]
                            heatmap_resized = cv2.resize(heatmap, (w, h))
                            heatmap_color = cv2.applyColorMap((heatmap_resized * 255).astype(np.uint8), cv2.COLORMAP_JET)
                            _, buf = cv2.imencode('.jpg', heatmap_color, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            b64 = base64.b64encode(buf).decode('utf-8')
                            gradcam_payload[f'sensor_{sensor["id"]}'] = b64
                    if gradcam_payload:
                        socketio_instance.emit('gradcam', gradcam_payload)
                except Exception as e:
                    print(f'[Grad-CAM] Error: {e}')

            # === b. 최종 행동(final_action) 결정 ===
            if oti_rl:
                uncertainty = bridge_client.uncertainty.GetLatestScore(pb.Empty()).score

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
            # succeed 플래그 분리 (action 마지막 1차원)
            if has_succeed:
                succeed_val = final_action[-1]
                final_action = final_action[:-1]
                socketio_instance.emit('inference_succeed', {'succeed': bool(succeed_val > 0.5), 'score': round(float(succeed_val), 4)})
                # Planner "until done": signal the outer loop when score exceeds threshold.
                done_threshold = task_control.get('done_threshold') if isinstance(task_control, dict) else None
                if done_threshold is not None and float(succeed_val) > float(done_threshold):
                    task_control['done'] = True
                    break

            # prev_qpos 갱신: 다음 스텝에서 실제 delta 계산에 사용
            for agent in env.agents:
                if agent.role != 'tool' and agent.ik_solver is not None:
                    prev_qpos_dict[agent.id] = obs_t['robot_states'][agent.id]['qpos']

            start_action_id = 0
            for agent in env.agents:
                if action_key_norm in ('ee_delta', 'relative_ee_pos') and agent.role != 'tool' and agent.ik_solver is not None:
                    ee_delta_dim = len(agent.ee_names) * 6
                    # tool_inner: ee_delta(6) + tool_abs 차원 추가
                    _, sample_tool = agent.get_joint_and_tool_pos([0.0] * agent.joint_len)
                    tool_dim = len(sample_tool) if agent.tool_inner and sample_tool else 0
                    total_dim = ee_delta_dim + tool_dim
                    agent_action = final_action[start_action_id : start_action_id + total_dim]

                    ee_delta_dict = {
                        ee_name: agent_action[i * 6 : (i + 1) * 6].tolist()
                        for i, ee_name in enumerate(agent.ee_names)
                    }

                    tool_positions = agent_action[ee_delta_dim:].tolist() if tool_dim > 0 else None
                    thread_pool.submit(agent.move_ee_delta_step, ee_delta_dict, None, tool_positions)
                    start_action_id += total_dim
                else:
                    target_qpos = final_action[start_action_id : start_action_id + agent.joint_len]
                    if first_step and agent.role != 'tool':
                        # 첫 cycle 만 move_to (smoothstep) 로 부드럽게. tool(그리퍼) 은
                        # SDK 자체 velocity profile 이라 그대로 step.
                        target_list = target_qpos.tolist() if hasattr(target_qpos, 'tolist') else list(target_qpos)
                        agent.move_to(target_list, duration=first_step_duration)
                    else:
                        thread_pool.submit(agent.move_joint_step, target_qpos)
                    start_action_id += agent.joint_len

            # Soft start: 첫 cycle 의 move_to 가 완료될 때까지 대기. 이후 cycle 들은
            # step_num 진행대로 normal loop 진행.
            if first_step:
                _soft_start_deadline = time.time() + first_step_duration + 2.0
                while time.time() < _soft_start_deadline:
                    if task_control['stop']:
                        for a in env.agents:
                            try:
                                a.cancel_move_to()
                            except Exception:
                                pass
                        return
                    if not any(a.is_moving for a in env.agents):
                        break
                    time.sleep(0.05)
                first_step = False
                start = time.time()  # loop cadence 재기준
                print('[INFER] Soft start 완료 — 정상 추론 loop 진입')

            ts_next = env.record_step()

            # === d. OTI-RL 학습 ===
            if oti_rl:
                uncertainty = bridge_client.uncertainty.GetLatestScore(pb.Empty()).score
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
                            'sensor_id': sensor_id,
                            'sam3': (task.get('sensor_sam3') or {}).get(sensor_id),
                            'resize': task['sensor_img_size'][sensor_id],
                            'cropped_area': task['sensor_cropped_area'][sensor_id].get('cropped_area', None),
                            'rotate': task['sensor_rotate'][sensor_id]
                        })
                        # BGR → RGB (same fix as the main inference path above; ros_image_to_numpy
                        # returns BGR but training data on disk is RGB).
                        if hasattr(image, 'ndim') and image.ndim == 3 and image.shape[2] == 3:
                            image = image[:, :, ::-1]
                            image = np.ascontiguousarray(image)
                        # PI05/PaliGemma pretrained weights expect [-1, 1]-ranged pixels.
                        _pixel_range = '-11' if policy_obj['type'] == 'PI05' else '01'
                        image = process_image(image, vision_backbone, to_cuda=True, pixel_range=_pixel_range)
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
        if oti_rl and 'bridge_client' in locals():
            try:
                bridge_client.uncertainty.StopSubscriber(pb.Empty())
            except Exception:
                pass

        # Drop GPU copies of the OOD tensors (originals stay on CPU in preloaded).
        try:
            del ood_image_feats
            del ood_state_feats
        except NameError:
            pass

        # If the policy came from the planner prefetch cache, return it to CPU
        # so VRAM is freed for the next block. Otherwise let it go out of scope.
        if preloaded is not None and 'policy' in locals():
            try:
                policy.cpu()
            except Exception:
                pass
        elif 'policy' in locals():
            del policy

        gc.collect()
        torch.cuda.empty_cache()

    return

