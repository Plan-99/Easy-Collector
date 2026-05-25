import os
import json
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
    action_hz=None,       # 로봇 명령 송신 rate. 미지정 시 hz 와 동일 (back-compat).
    inference_hz=None,    # 모델 inference 호출 rate. 미지정 시 action_hz / re_inference_steps.
    ):

    agents = sorted(agents, key=lambda a: a.id)
    oti_rl = False
    move_reward = 1.0

    # action_hz / inference_hz 정규화.
    # - hz (legacy) 는 두 값의 fallback. 명시 안 한 경우 모두 동일하게 설정 → 기존 동작.
    # - inference_hz 미지정 + re_inference_steps>1 이면 action_hz / re_inference_steps
    #   (예: action_hz=10, re_inference_steps=8 → inference_hz≈1.25 = 한 chunk 다 쓰고 재추론).
    # 분리값을 지정하면 action thread 는 매 1/action_hz 마다 한 step 진행, inference
    # 는 1/inference_hz 마다만 호출됨 (느린 모델 latency 흡수).
    if action_hz is None:
        action_hz = float(hz)
    else:
        action_hz = float(action_hz)
    if inference_hz is None:
        # back-compat: re_inference_steps 가 inference 간격을 step 단위로 표현.
        _ris = max(1, int(re_inference_steps))
        inference_hz = action_hz / _ris
    else:
        inference_hz = float(inference_hz)
    action_dt = 1.0 / action_hz
    inference_dt = 1.0 / inference_hz
    print(
        f'[INFER] action_hz={action_hz:.2f} (dt={action_dt*1000:.0f}ms), '
        f'inference_hz={inference_hz:.2f} (dt={inference_dt*1000:.0f}ms), '
        f're_inference_steps={re_inference_steps}',
        flush=True,
    )

    # --- 1. 초기 설정 ---
    try:
        gc.collect()
        torch.cuda.empty_cache()

        from concurrent.futures import ThreadPoolExecutor
        thread_pool = ThreadPoolExecutor(max_workers=len(agents))
        executor = None
        
        ckpt_dir = resolve_checkpoint_dir(checkpoint['id'])

        # Read training-time preprocessing metadata (image_resolution) so the
        # inference resize matches what the model was trained on. Falls back to
        # (224, 224) when the sidecar is absent (older checkpoints).
        image_resolution = (224, 224)
        _meta_path = os.path.join(ckpt_dir, 'train_meta.json')
        if os.path.exists(_meta_path):
            try:
                with open(_meta_path, 'r') as _f:
                    _ir = (json.load(_f) or {}).get('image_resolution')
                if isinstance(_ir, (list, tuple)) and len(_ir) == 2:
                    image_resolution = (int(_ir[0]), int(_ir[1]))
            except Exception as _e:
                print(f'[INFER][WARN] failed to read train_meta.json: {_e}; falling back to (224, 224)')

        action_key = action_type or policy_obj.get('settings', {}).get('action_type') or checkpoint.get('train_settings', {}).get('action_type', 'qaction')
        # 신/구 action_key 별칭 통일: 'qaction'→'joint', 'ee_delta_action'→'ee_delta'.
        # 아래에서 분기 비교할 때 양쪽 모두 인식되도록 normalize.
        _ACTION_KEY_ALIAS = {'qaction': 'joint', 'joint': 'joint',
                              'ee_delta_action': 'ee_delta', 'ee_delta': 'ee_delta',
                              'relative_ee_pos': 'relative_ee_pos',
                              'relative_joint_pos': 'relative_joint_pos'}
        action_key_norm = _ACTION_KEY_ALIAS.get(action_key, action_key)
        # 빈 리스트 [] 는 명시적 "no-state" 의도 — 학습 측 _build_obs_state 가
        # 그대로 빈 state 로 처리하므로 추론도 동일. None 일 때만 ['qpos'] 로 default.
        _p_keys = policy_obj.get('settings', {}).get('obs_state_keys')
        _t_keys = checkpoint.get('train_settings', {}).get('obs_state_keys')
        if _p_keys is not None:
            obs_state_keys = _p_keys
        elif _t_keys is not None:
            obs_state_keys = _t_keys
        else:
            obs_state_keys = ['qpos']
        use_relative_trajectory = checkpoint.get('train_settings', {}).get('use_relative_trajectory', False)

        # Scheduled-waypoint 모드: DexUMI 스타일 absolute-time waypoint scheduling.
        # 모든 action_key 에 대해 기본 활성화 — 매 명령을 (target, t_start + dt) 로
        # ec_joint_waypoint 큐에 push, interpolation_node 가 200Hz 로 보간해 출력.
        # frame dt 와 추론 hz 가 어긋나도 robot 이 학습 분포를 그대로 traverse,
        # inference jitter 에 강함. 비상시 OFF: 환경변수 EC_SCHEDULED_WAYPOINTS=0
        import os as _os
        use_scheduled_waypoints = (
            _os.environ.get('EC_SCHEDULED_WAYPOINTS', '').lower() not in ('0', 'false', 'no')
        )
        if use_scheduled_waypoints:
            print(f'[INFER] Scheduled-waypoint mode (action_key={action_key_norm}, target = t_start + {action_dt:.3f}s)', flush=True)
        else:
            print('[INFER] Scheduled-waypoint mode DISABLED via EC_SCHEDULED_WAYPOINTS=0', flush=True)
        # NOTE: use_relative_actions 플래그는 upstream 에서 action_key_norm ==
        # 'relative_joint_pos' 로 대체됨 (commit 2881984). 아래 PI05 chunk-state
        # 매핑 분기도 그 기준으로 통일.
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
        # DIAG: 모델이 observation.state 를 실제로 입력으로 받는지 확인 (ACT 의
        # robot_state_feature property — input_features 에 observation.state 가
        # 있으면 truthy).
        try:
            _rsf = getattr(policy.config, 'robot_state_feature', None)
            _inp = dict(getattr(policy.config, 'input_features', {}) or {})
            _inp_summary = {k: getattr(v, 'shape', None) for k, v in _inp.items()}
            print(
                f'[INFER policy] robot_state_feature={_rsf} '
                f'input_features={_inp_summary}',
                flush=True,
            )
        except Exception as _ex:
            print(f'[INFER policy] diag failed: {_ex}', flush=True)

        # relative_ee_pos / ee_delta 추론은 action 이 EE pose (6 또는 6+tool 차원).
        # 옛 코드 (DexUMI 도입 전) 로 학습된 checkpoint 의 action 차원은 joint 기준
        # (7~8) 일 수 있어 새 추론 로직과 맞지 않음 → 명시적 에러로 막는다.
        if action_key_norm in ('ee_delta', 'relative_ee_pos'):
            _expected_ee_dim = 0
            for _a in agents:
                if _a.role == 'tool' or getattr(_a, 'ik_solver', None) is None:
                    continue
                _per_ee = 6 + (1 if getattr(_a, 'tool_inner', False) else 0)
                _expected_ee_dim += _per_ee * max(1, len(getattr(_a, 'ee_names', []) or []))
            # 별도 tool agent (그리퍼) joint dim 도 eepos/action 에 포함됨. lerobot_io
            # 가 ee_names_list 끝에 추가하는 것과 동일.
            for _a in agents:
                if _a.role == 'tool' and getattr(_a, 'ik_solver', None) is None:
                    _expected_ee_dim += getattr(_a, 'joint_len', 0)
            try:
                _model_action_dim = policy.config.output_features['action'].shape[0]
            except Exception:
                _model_action_dim = None
            if _model_action_dim is not None and _model_action_dim != _expected_ee_dim:
                # has_succeed 면 +1 허용.
                if not (has_succeed and _model_action_dim == _expected_ee_dim + 1):
                    raise RuntimeError(
                        f'[INFER] action_key={action_key} (EE 기반) 인데 checkpoint 의 '
                        f'output action dim = {_model_action_dim} 이고 expected EE dim = '
                        f'{_expected_ee_dim} (또는 +1 if succeed). 학습 시점에 DexUMI 스타일 '
                        f'relative trajectory 로직이 없던 옛 코드로 학습됐을 가능성. '
                        f'최신 코드로 재학습 필요.'
                    )

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

        # Inference-time vision map: register persistent attention hooks once
        # so each inference forward populates a holder. Frontend toggles the
        # method via task_control['vision_map_method'] = 'attention'|'gradcam'|None.
        # Only ACT supports this (others have no comparable cross-attention).
        _vm_holder = None
        _vm_ordered_sensors: list = []
        if policy_obj['type'] == 'ACT':
            try:
                from .vision_map import make_attention_holder
                _vm_holder = make_attention_holder(policy)
                _vm_ordered_sensors = [
                    key.replace('observation.images.', '')
                    for key in policy.config.image_features
                ]
                print(f'[InferenceVisionMap] hooks registered (sensors={_vm_ordered_sensors})')
            except Exception as e:
                print(f'[InferenceVisionMap] hook registration failed: {e}')
                _vm_holder = None
        # Per-sensor (H, W) of the latest raw camera frame — populated each
        # step. Used to upscale the heatmap PNG to match the live WebRTC feed.
        _vm_orig_sizes: dict = {}

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
        # 시간 기반 inference cadence: 마지막 inference 시각 추적. 두 큐(rel_action_queue,
        # pi05_chunk_queue) path 모두 (1) 큐가 비었거나 (2) inference_dt 가 지났으면
        # 재추론. 후자가 발동되면 queue 의 잔여 chunk 는 폐기되고 새 chunk 로 교체.
        last_inference_time = 0.0

        # Background inference (relative_ee_pos 전용): 모델 forward 가 action loop 를
        # block 하지 않도록 별도 thread 에서 실행. 결과는 매 iter 시작에 polling 으로
        # 수거해 rel_action_queue 를 갱신. 큐가 비어 있고 inference 도 아직 안 끝났으면
        # 그 iter 는 action skip (cold start / inference 늦을 때만 발생).
        # PI05 use_relative_actions 및 일반 ACT online 경로는 기존대로 synchronous 유지
        # (변경 폭 최소화).
        inference_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='infer')
        _inference_future = None

        # 아래 nested function 은 closure 로 env, policy, preprocessor, postprocessor,
        # agents, sensors, task, policy_obj, language_instruction, obs_state_keys,
        # vision_backbone, action_key_norm, use_relative_trajectory 를 캡쳐. 호출 시
        # obs_snap (ts.observation 스냅샷) 만 인자로 전달.
        def _do_rel_ee_inference(obs_snap):
            """relative_ee_pos chunk 한 번 추론 — worker thread 에서 실행.

            obs_snap: dict-like observation snapshot (immutable read).
                      agent.get_joint_vel/effort/get_ee_position/get_joint_states 는
                      gRPC 호출이라 thread 에서 안전 (호출 시점의 fresh 값 가져옴).
            Returns: deltas np.ndarray (chunk_size, action_dim).
            """
            with torch.no_grad():
                if use_relative_trajectory and action_key_norm == 'ee_delta':
                    policy.reset()
                # 학습 측 _build_obs_state 와 동일 semantic — arm features 만 user-selected,
                # tool pose 는 obs_state_keys 와 무관하게 마지막에 항상 추가.
                _arms = [a for a in env.agents if not (a.role == 'tool' and a.ik_solver is None)]
                _tools = [a for a in env.agents if a.role == 'tool' and a.ik_solver is None]
                _parts = []
                for _key in obs_state_keys:
                    if _key == 'qpos':
                        _arm_qpos = []
                        for _a in _arms:
                            _st = obs_snap['robot_states'].get(_a.id) or obs_snap['robot_states'].get(str(_a.id))
                            if _st is not None and _st.get('qpos') is not None:
                                _arm_qpos.append(np.array(_st['qpos'], dtype=np.float32))
                        if _arm_qpos:
                            _parts.append(np.concatenate(_arm_qpos))
                    elif _key == 'qvel':
                        _arm_qvel = [np.array(_a.get_joint_vel() or [], dtype=np.float32) for _a in _arms]
                        if any(len(x) > 0 for x in _arm_qvel):
                            _parts.append(np.concatenate(_arm_qvel))
                    elif _key == 'qeffort':
                        _arm_eff = [np.array(_a.get_joint_effort() or [], dtype=np.float32) for _a in _arms]
                        if any(len(x) > 0 for x in _arm_eff):
                            _parts.append(np.concatenate(_arm_eff))
                    elif _key == 'eepos':
                        _ee_parts = []
                        for _a in _arms:
                            if _a.ik_solver is None:
                                continue
                            _ee_dict = _a.get_ee_position() or {}
                            for _ee_name in sorted(_a.ee_names or []):
                                _vals = _ee_dict.get(_ee_name)
                                if _vals:
                                    _ee_parts.append(np.array(_vals, dtype=np.float32))
                        if _ee_parts:
                            _parts.append(np.concatenate(_ee_parts))
                # 항상 마지막에 tool_pose append — 분리형 tool agent 의 qpos.
                for _a in _tools:
                    _t_qpos = _a.get_joint_states()
                    if _t_qpos:
                        _parts.append(np.array(_t_qpos, dtype=np.float32))
                _qpos_np = np.concatenate(_parts) if _parts else np.zeros(0, dtype=np.float32)
                _qpos_t = torch.from_numpy(_qpos_np).float().cuda().unsqueeze(0)
                _policy_input = {'observation.state': _qpos_t}
                # DIAG: 매번 찍으면 로그 폭주 — 처음 한 번만, 그 이후엔 1000번에 한 번.
                if not hasattr(_do_rel_ee_inference, '_state_log_count'):
                    _do_rel_ee_inference._state_log_count = 0
                _do_rel_ee_inference._state_log_count += 1
                if _do_rel_ee_inference._state_log_count == 1 or \
                   _do_rel_ee_inference._state_log_count % 1000 == 0:
                    _state_vals = _qpos_t[0].cpu().numpy()
                    print(
                        f"[INPUT bg-infer #{_do_rel_ee_inference._state_log_count}] "
                        f"obs_state_keys={obs_state_keys} state.shape={_qpos_t.shape} "
                        f"values={_state_vals.tolist()}",
                        flush=True,
                    )
                if policy_obj['type'] == 'PI05':
                    _lang = language_instruction if (language_instruction and str(language_instruction).strip()) else task.get('name', '')
                    _policy_input['language_instruction'] = _lang
                for _sensor in sensors:
                    _img = obs_snap['images'][f'sensor_{_sensor["id"]}']
                    _sid = str(_sensor['id'])
                    _img = fetch_image_with_config(_img, {
                        'sensor_id': _sid,
                        'sam3': (task.get('sensor_sam3') or {}).get(_sid),
                        'resize': task['sensor_img_size'][_sid],
                        'cropped_area': task['sensor_cropped_area'][_sid],
                        'rotate': task['sensor_rotate'][_sid],
                    })
                    if hasattr(_img, 'ndim') and _img.ndim == 3 and _img.shape[2] == 3:
                        _img = _img[:, :, ::-1]
                        _img = np.ascontiguousarray(_img)
                    _pixel_range = '-11' if policy_obj['type'] == 'PI05' else '01'
                    _img = process_image(_img, vision_backbone, to_cuda=True, pixel_range=_pixel_range, image_resolution=image_resolution)
                    _policy_input[f'observation.images.sensor_{_sensor["id"]}'] = _img.unsqueeze(0)

                if preprocessor is not None:
                    if policy_obj['type'] == 'PI05' and 'task' not in _policy_input:
                        _lang = _policy_input.get('language_instruction') or task.get('name', '')
                        if isinstance(_lang, str):
                            _lang = [_lang]
                        _policy_input['task'] = _lang
                    _policy_input = preprocessor(_policy_input)

                policy.reset()
                if hasattr(policy, 'predict_action_chunk'):
                    _raw_actions = policy.predict_action_chunk(_policy_input).squeeze(0)
                else:
                    _raw_list = []
                    _first = policy.select_action(_policy_input).squeeze(0)
                    _raw_list.append(_first)
                    while len(policy._action_queue if hasattr(policy, '_action_queue') else policy._queues.get('action', [])) > 0:
                        _q = policy._action_queue if hasattr(policy, '_action_queue') else policy._queues['action']
                        _raw_list.append(_q.popleft().squeeze(0))
                    _raw_actions = torch.stack(_raw_list)

                if postprocessor is not None:
                    _raw_actions = postprocessor(_raw_actions)
                _raw_np = _raw_actions.cpu().numpy()

                # Current EE pose — image/state 가 캡처된 T1 시점의 eepos (obs_snap).
                # 학습은 (image_t, eepos_t) 가 같은 frame t 에서 페어링되어 모델 출력의
                # local-frame trajectory anchor 가 image 시점과 일치한다고 가정한다.
                # 여기서 fresh RPC (get_ee_position) 로 다시 읽으면 추론 소요시간 만큼
                # 로봇이 더 이동한 T2 의 eepos 가 잡혀 anchor 가 T1→T2 로 shift,
                # 결과적으로 chunk 전체가 그 변위만큼 밀려 "데이터 3 step → 추론 4 step"
                # 같은 off-by-Δt drift 가 생긴다. obs_snap['robot_states'][..]['eepos']
                # 는 record_step 이 image 와 한 RPC 로 묶어준 같은 T1 값.
                _curr_eepos_local = None
                for _a in env.agents:
                    if _a.role == 'tool' or _a.ik_solver is None:
                        continue
                    _st = obs_snap['robot_states'].get(_a.id) or obs_snap['robot_states'].get(str(_a.id))
                    _ee_dict = (_st or {}).get('eepos') or {}
                    _ee_name = _a.ee_names[0] if _a.ee_names else None
                    if _ee_name and _ee_name in _ee_dict:
                        _curr_eepos_local = np.array(_ee_dict[_ee_name][:6], dtype=np.float32)
                        break
                _deltas_local = relative_trajectory_to_delta(_raw_np, current_eepos=_curr_eepos_local)
                return _deltas_local

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

            # Background inference 결과 수거 — non-blocking. 완료된 future 가 있으면
            # rel_action_queue 를 새 chunk 로 교체.
            #
            # Chunk boundary smoothing (option 1: bridge anchor):
            # 단순 clear+append 하면 robot 이 old chunk 의 다음 waypoint 방향으로
            # 가속 중인데 target 이 new chunk 의 첫 waypoint 로 점프해서 미세 진동.
            # → 기존 큐의 첫 점(= 곧 pop 될 다음 waypoint) 을 anchor 로 prepend 해서
            #   첫 step 은 old motion 자연 연장, 두 번째 step 부터 new chunk 사용.
            #   velocity 점프 제거, trajectory 만 부드럽게 휨.
            if _inference_future is not None and _inference_future.done():
                try:
                    _new_deltas = _inference_future.result()
                    _bridge_anchor = None
                    if len(rel_action_queue) > 0:
                        _bridge_anchor = np.asarray(rel_action_queue[0]).copy()
                    rel_action_queue.clear()
                    if _bridge_anchor is not None and len(_new_deltas) > 0:
                        # 1 step 은 anchor (= old chunk 의 다음 점)
                        rel_action_queue.append(_bridge_anchor)
                        # 나머지는 new chunk 의 2번째 이후 (첫 점은 anchor 가 대체)
                        for _d in _new_deltas[1:]:
                            rel_action_queue.append(_d)
                        print(
                            f'[INFER thread] chunk ready (smoothed), '
                            f'queue refilled to {len(rel_action_queue)} '
                            f'(1 anchor + {len(_new_deltas)-1} new)',
                            flush=True,
                        )
                    else:
                        # 큐가 비어 있던 경우 — anchor 없이 그대로 채움 (cold start)
                        for _d in _new_deltas:
                            rel_action_queue.append(_d)
                        _t_chunk_ready = time.time()
                        try:
                            _first_delta_preview = (
                                np.asarray(_new_deltas[0]).flatten()[:6].tolist()
                                if len(_new_deltas) > 0 else None
                            )
                        except Exception:
                            _first_delta_preview = None
                        print(
                            f'[INFER thread] chunk ready (no anchor, cold start), '
                            f'queue refilled to {len(rel_action_queue)} '
                            f'at t={_t_chunk_ready:.3f} (start={start:.3f}, '
                            f'elapsed_since_start={_t_chunk_ready - start:.3f}s) '
                            f'first_delta[:6]={_first_delta_preview}',
                            flush=True,
                        )
                        # Cold start: 첫 chunk inference 가 warmup 등으로 수 초 걸리는
                        # 경우, 기존 ``start`` 는 while 진입 시각이라 이미 과거.
                        # 이대로 두면 _t_target_for_step = start + action_dt 가 과거가
                        # 되어 interpolator 가 첫 waypoint 로 점프 → homepose 직후 덜컹.
                        # → chunk 도착 시각을 새 cadence 기준으로 리셋.
                        start = _t_chunk_ready
                except Exception as _ex:
                    print(f'[INFER thread] error: {_ex}', flush=True)
                _inference_future = None

            # 시간 기반 inference 게이트.
            _t_now_for_gate = time.time()
            _inference_due = (_t_now_for_gate - last_inference_time) >= inference_dt

            # relative_ee_pos: 백그라운드 inference 사용. due + idle 면 새 task 제출,
            # 그 외에는 queue 에서 pop. queue 비고 inference 도 안 끝났으면 action skip.
            if action_key_norm == 'relative_ee_pos':
                if _inference_due and _inference_future is None:
                    last_inference_time = _t_now_for_gate
                    _inference_future = inference_executor.submit(_do_rel_ee_inference, obs_t)
                if len(rel_action_queue) > 0:
                    state_t = rel_action_queue.popleft()
                else:
                    # Queue 비어 있음 — cold start 또는 inference 가 inference_dt 안에
                    # 못 끝난 경우. 짧게 대기 후 다음 iter 에서 다시 체크.
                    time.sleep(min(action_dt, 0.02))
                    continue
            # PI05 + relative_joint_pos: 같은 chunk의 나머지 absolute joint targets
            # 사용 (chunk-inference 시점의 qpos로 이미 더해진 상태). 재추론 없이 pop.
            # (upstream 의 'relative_joint_pos' action_key 분기 + stashed 의 시간-게이트
            # 통합 — 둘 조건 모두 만족할 때만 큐 재사용.)
            elif (policy_obj['type'] == 'PI05'
                  and action_key_norm == 'relative_joint_pos'
                  and len(pi05_chunk_queue) > 0
                  and not _inference_due):
                state_t = pi05_chunk_queue.popleft()
            else:
                # 재추론 전에 잔여 큐 폐기 (새 chunk 가 곧 덮어씀).
                rel_action_queue.clear()
                pi05_chunk_queue.clear()
                last_inference_time = _t_now_for_gate
                with torch.no_grad():
                    # UMI relative trajectory: 매 step마다 현재 EE pose 기준으로 재예측.
                    # temporal ensemble이 다른 기준 frame의 waypoint를 섞지 않도록 policy를 reset.
                    if use_relative_trajectory and action_key_norm == 'ee_delta':
                        policy.reset()
                    # 학습 측 _build_obs_state 와 동일 — arm features 만 user-selected,
                    # tool pose 는 obs_state_keys 와 무관하게 마지막에 항상 추가.
                    _arms_sync = [a for a in env.agents if not (a.role == 'tool' and a.ik_solver is None)]
                    _tools_sync = [a for a in env.agents if a.role == 'tool' and a.ik_solver is None]
                    parts = []
                    for _key in obs_state_keys:
                        if _key == 'qpos':
                            _aq = []
                            for _a in _arms_sync:
                                _st = obs_t['robot_states'].get(_a.id) or obs_t['robot_states'].get(str(_a.id))
                                if _st is not None and _st.get('qpos') is not None:
                                    _aq.append(np.array(_st['qpos'], dtype=np.float32))
                            if _aq:
                                parts.append(np.concatenate(_aq))
                        elif _key == 'qvel':
                            _vs = [np.array(_a.get_joint_vel() or [], dtype=np.float32) for _a in _arms_sync]
                            if any(len(x) > 0 for x in _vs):
                                parts.append(np.concatenate(_vs))
                        elif _key == 'qeffort':
                            _es = [np.array(_a.get_joint_effort() or [], dtype=np.float32) for _a in _arms_sync]
                            if any(len(x) > 0 for x in _es):
                                parts.append(np.concatenate(_es))
                        elif _key == 'eepos':
                            ee_parts = []
                            for _a in _arms_sync:
                                if _a.ik_solver is None:
                                    continue
                                ee_dict = _a.get_ee_position() or {}
                                for ee_name in sorted(_a.ee_names or []):
                                    vals = ee_dict.get(ee_name)
                                    if vals:
                                        ee_parts.append(np.array(vals, dtype=np.float32))
                            if ee_parts:
                                parts.append(np.concatenate(ee_parts))
                    # 항상 tool pose append
                    for _a in _tools_sync:
                        _t_qpos = _a.get_joint_states()
                        if _t_qpos:
                            parts.append(np.array(_t_qpos, dtype=np.float32))
                    qpos_np = np.concatenate(parts) if parts else np.zeros(0, dtype=np.float32)
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
                        # Capture the post-process frame size — this is what the
                        # WebRTC stream shows in the browser, so the heatmap PNG
                        # must match this aspect (not the raw camera).
                        if hasattr(image, 'shape') and len(image.shape) >= 2:
                            _vm_orig_sizes[f'sensor_{sensor["id"]}'] = (
                                int(image.shape[0]), int(image.shape[1])
                            )
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
                        image = process_image(image, vision_backbone, to_cuda=True, pixel_range=_pixel_range, image_resolution=image_resolution)
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
                        #
                        # NOTE: image/state 가 캡처된 같은 snapshot (obs_t) 의 eepos 를
                        # anchor 로 사용 — 학습이 (image_t, eepos_t) 페어로 라벨을 만들기
                        # 때문에 fresh RPC 대신 obs_t['robot_states'] 의 eepos 를 읽어야
                        # 추론 소요시간(T2-T1) 만큼의 drift 가 사라진다. (background path
                        # 의 _do_rel_ee_inference 와 동일한 보정.)
                        _curr_eepos = None
                        for _agent in env.agents:
                            if _agent.role == 'tool' or _agent.ik_solver is None:
                                continue
                            _st = obs_t['robot_states'].get(_agent.id) or obs_t['robot_states'].get(str(_agent.id))
                            _ee_dict = (_st or {}).get('eepos') or {}
                            _ee_name = _agent.ee_names[0] if _agent.ee_names else None
                            if _ee_name and _ee_name in _ee_dict:
                                _curr_eepos = np.array(_ee_dict[_ee_name][:6], dtype=np.float32)
                                break
                        deltas = relative_trajectory_to_delta(raw_np, current_eepos=_curr_eepos)
                        # DIAG: 모델 출력 (relative trajectory in current EE local frame) 의
                        # rotation/translation 통계 + 역변환된 world deltas 통계 출력.
                        # 학습 데이터에 rotation 없는데 추론에서 발생하면 여기서 값이
                        # 들킴 — 모델이 rotation dim 에 노이즈를 출력하고 있다는 뜻.
                        _rt = raw_np[:, 3:6] if raw_np.shape[1] >= 6 else None
                        _dt = deltas[:, 3:6] if deltas.shape[1] >= 6 else None
                        _tt = raw_np[:, :3]
                        _ddt = deltas[:, :3]
                        print(
                            f'[REL_DIAG] curr_eepos={_curr_eepos.tolist() if _curr_eepos is not None else None}\n'
                            f'  raw rel_traj translation max_abs={np.abs(_tt).max():.4f} '
                            f'rotation max_abs={np.abs(_rt).max():.4f}\n'
                            f'  world deltas    translation max_abs={np.abs(_ddt).max():.4f} '
                            f'rotation max_abs={np.abs(_dt).max():.4f}\n'
                            f'  rel_traj[0]={raw_np[0].tolist()} ... [N-1]={raw_np[-1].tolist()}',
                            flush=True
                        )
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

            # === Inference-time Vision Map (frontend toggle via task_control) ===
            # Uses the same forward pass for 'attention' (persistent hooks) —
            # essentially free. 'gradcam' needs a separate backward → expensive.
            _vm_method = task_control.get('vision_map_method')
            if _vm_method and _vm_holder is not None and 'policy_input_t' in dir():
                try:
                    if _vm_method == 'attention':
                        from .vision_map import render_attention_heatmaps_from_holder
                        _vm_heatmaps = render_attention_heatmaps_from_holder(
                            _vm_holder, policy, _vm_ordered_sensors, _vm_orig_sizes,
                        )
                    elif _vm_method == 'gradcam':
                        from .vision_map import compute_vision_map_from_preprocessed_batch
                        _vm_heatmaps = compute_vision_map_from_preprocessed_batch(
                            policy=policy, batch=policy_input_t,
                            ordered_sensors=_vm_ordered_sensors,
                            target_hw_per_sensor=_vm_orig_sizes,
                            method='gradcam',
                        )
                    else:
                        _vm_heatmaps = {}
                    if _vm_heatmaps:
                        socketio_instance.emit('inference_vision_map', {
                            'step': step_num,
                            'method': _vm_method,
                            'heatmaps': _vm_heatmaps,
                        })
                except Exception as e:
                    print(f'[InferenceVisionMap] step={step_num} method={_vm_method} error: {e}')

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
            # relative_ee_pos / ee_delta 일 때 action 컬럼 layout 은 lerobot_io 의
            # eepos 컬럼과 동일: [arms_with_ik 순서대로, separate_tool_agents 순서대로].
            # agent.id 정렬로 가면 ID 가 작은 tool 이 arm 보다 앞서버려 슬라이싱이 어긋남.
            # → arm 먼저 처리하고 그 다음 tool 처리하는 두 패스로 분리.
            arm_then_tool = (
                action_key_norm in ('ee_delta', 'relative_ee_pos')
            )
            if arm_then_tool:
                ordered_agents = (
                    [a for a in env.agents if a.role != 'tool' and a.ik_solver is not None]
                    + [a for a in env.agents if not (a.role != 'tool' and a.ik_solver is not None)]
                )
            else:
                ordered_agents = list(env.agents)

            # Scheduled-waypoint target time = 이 step 이 도달해야 하는 시각.
            # 학습 시 frame 간격이 1/hz 라고 보고, 현재 iteration 시작(start) 으로
            # 부터 1/hz 후 도달이 학습 분포와 정합. inference 가 dt 보다 오래
            # 걸려도 robot 은 큐의 미래 waypoint 따라 계속 진행.
            # action_key_norm 이 relative_ee_pos / ee_delta 면 항상 scheduled 사용,
            # joint 모드는 _t_target_for_step = None 으로 즉시 명령 fallback.
            # Scheduled-waypoint target time.
            # 일반 케이스: start + action_dt (학습 분포와 정합).
            # 일반 보호: time.time() + action_dt (start 가 stale 한 경우 — 예: 큰
            # GC pause, 일시적 stall — 에 대비. t_target 이 과거가 되어 interpolator
            # 가 점프하는 것을 막음). cold start fix 와 별도로 안전망 역할.
            if use_scheduled_waypoints:
                _t_target_for_step = max(start + action_dt, time.time() + action_dt)
            else:
                _t_target_for_step = None

            # === FIRST-STEP DIAG === scheduled-waypoint 의 첫 publish 시
            # t_target 이 과거에 잡히는지 확인. diff < 0 이면 보간기가 점프.
            if first_step and use_scheduled_waypoints and _t_target_for_step is not None:
                _now_dbg = time.time()
                _diff_dbg = _t_target_for_step - _now_dbg
                try:
                    _action_preview = (
                        final_action[:6].tolist()
                        if hasattr(final_action, 'tolist') else list(final_action[:6])
                    )
                except Exception:
                    _action_preview = None
                print(
                    f'[FIRST CHUNK DIAG] start={start:.3f} now={_now_dbg:.3f} '
                    f'action_dt={action_dt:.3f} t_target={_t_target_for_step:.3f} '
                    f'diff(t_target-now)={_diff_dbg:.3f}s '
                    f'action[:6]={_action_preview}',
                    flush=True,
                )

            for agent in ordered_agents:
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
                    # 이 분기는 ee_delta/relative_ee_pos 만 → use_scheduled_waypoints 항상 True.
                    thread_pool.submit(
                        agent.move_ee_delta_step_at,
                        ee_delta_dict, _t_target_for_step, tool_positions,
                    )
                    start_action_id += total_dim
                else:
                    target_qpos = final_action[start_action_id : start_action_id + agent.joint_len]
                    if first_step and agent.role != 'tool':
                        # 첫 cycle 만 move_to (smoothstep) 로 부드럽게. tool(그리퍼) 은
                        # SDK 자체 velocity profile 이라 그대로 step.
                        target_list = target_qpos.tolist() if hasattr(target_qpos, 'tolist') else list(target_qpos)
                        agent.move_to(target_list, duration=first_step_duration)
                    elif use_scheduled_waypoints:
                        target_list = target_qpos.tolist() if hasattr(target_qpos, 'tolist') else list(target_qpos)
                        thread_pool.submit(agent.move_to_joints_at, target_list, _t_target_for_step)
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
                        image = process_image(image, vision_backbone, to_cuda=True, pixel_range=_pixel_range, image_resolution=image_resolution)
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

            time.sleep(max(0, action_dt - (time.time() - start)))  # Loop at action_hz
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
        # Background inference executor 도 함께 정리 — 미정리 시 호출 반복마다
        # thread leak + 미완료 future 의 GPU 텐서가 GC 안 됨.
        try:
            if 'inference_executor' in locals() and inference_executor is not None:
                inference_executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
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

