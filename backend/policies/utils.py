from einops import rearrange
import cv2
import numpy as np
import torch
import os
from torch.utils.data import TensorDataset, DataLoader
import json
from types import SimpleNamespace
from lerobot.configs.types import PolicyFeature, FeatureType
from torchvision import transforms
from transformers import AutoImageProcessor
from PIL import Image
from scipy.spatial.transform import Rotation
import pyarrow.parquet as pq
from ..utils.lerobot_io import (
    read_episode, list_episodes, get_dataset_info, get_norm_stats_from_dataset,
    _parse_image_value,
    PARQUET_PATH_TEMPLATE, IMAGE_PATH_TEMPLATE, DEFAULT_CHUNK_SIZE,
)


def _eepos_to_homogeneous(eepos6: np.ndarray) -> np.ndarray:
    """[x, y, z, rx, ry, rz] (axis-angle) → 4x4 homogeneous matrix."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_rotvec(eepos6[3:6]).as_matrix()
    T[:3, 3] = eepos6[:3]
    return T


def _homogeneous_to_eepos(T: np.ndarray) -> np.ndarray:
    """4x4 homogeneous → [x, y, z, rx, ry, rz]."""
    rotvec = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return np.concatenate([T[:3, 3], rotvec]).astype(np.float32)


def compute_relative_trajectory_in_local_frame(
    future_eepos: np.ndarray, current_eepos: np.ndarray
) -> np.ndarray:
    """DexUMI 스타일 — chunk_size 만큼의 future eepos 를 current eepos 의 local
    frame 으로 변환 (action target 으로 사용).

    `T_relative[i] = invert(T_current) @ T_future[i]` 후 6dof 로 직렬화.

    Args:
        future_eepos: [N, D] 미래 N 개 step 의 eepos. D = 6 또는 6+tool.
        current_eepos: [D] 현재 step 의 eepos.

    Returns:
        [N, D] relative trajectory. 앞 6 차원은 current local frame, tool 차원은
        absolute 값 그대로 pass-through.
    """
    future_eepos = np.asarray(future_eepos, dtype=np.float32)
    current_eepos = np.asarray(current_eepos, dtype=np.float32)
    if future_eepos.ndim == 1:
        future_eepos = future_eepos[None, :]
    N, D = future_eepos.shape

    T_curr = _eepos_to_homogeneous(current_eepos)
    T_curr_inv = np.eye(4, dtype=np.float64)
    R_curr = T_curr[:3, :3]
    t_curr = T_curr[:3, 3]
    T_curr_inv[:3, :3] = R_curr.T
    T_curr_inv[:3, 3] = -R_curr.T @ t_curr

    out = np.zeros_like(future_eepos)
    for i in range(N):
        T_i = _eepos_to_homogeneous(future_eepos[i])
        T_rel = T_curr_inv @ T_i
        out[i, :6] = _homogeneous_to_eepos(T_rel)
        if D > 6:
            out[i, 6:] = future_eepos[i, 6:]  # tool 차원: absolute 유지
    return out


def relative_trajectory_in_local_frame_to_world_deltas(
    relative_traj: np.ndarray, current_eepos: np.ndarray
) -> np.ndarray:
    """역변환 (inference). current EE local frame 의 trajectory → world frame
    sequential deltas (move_ee_delta_step 으로 적용 가능한 형태).

    각 waypoint 를 world 로 풀고, 연속 waypoint 간 차분 (회전은 합성) 으로 step
    별 delta 계산. 첫 delta 는 current → world[0], 이후는 world[i-1] → world[i].
    Tool 차원은 absolute 값 그대로 (한 step 내 절대 위치로 보냄).
    """
    relative_traj = np.asarray(relative_traj, dtype=np.float32)
    current_eepos = np.asarray(current_eepos, dtype=np.float32)
    if relative_traj.ndim == 1:
        relative_traj = relative_traj[None, :]
    N, D = relative_traj.shape

    T_curr = _eepos_to_homogeneous(current_eepos)

    # 모든 waypoint 를 world frame 으로 변환
    T_world_list = []
    for i in range(N):
        T_rel = _eepos_to_homogeneous(relative_traj[i, :6])
        T_world_list.append(T_curr @ T_rel)

    deltas = np.zeros_like(relative_traj)
    T_prev = T_curr  # 첫 delta 는 current → world[0]
    for i in range(N):
        T_i = T_world_list[i]
        # Position delta in world
        delta_t = T_i[:3, 3] - T_prev[:3, 3]
        # Rotation delta: R_delta = R_i @ R_prev^T
        R_delta = T_i[:3, :3] @ T_prev[:3, :3].T
        rotvec_delta = Rotation.from_matrix(R_delta).as_rotvec()
        deltas[i, :3] = delta_t
        deltas[i, 3:6] = rotvec_delta
        if D > 6:
            deltas[i, 6:] = relative_traj[i, 6:]  # tool absolute pass-through
        T_prev = T_i
    return deltas


def delta_to_relative_trajectory(deltas: np.ndarray) -> np.ndarray:
    """UMI 방식의 relative trajectory 라벨로 변환.

    deltas: [T, D] sequential ee_delta. 앞 6차원은 [dx, dy, dz, dax, day, daz],
            7차원 이후는 tool joint (absolute 값)으로 변환 없이 그대로 유지.
    반환값: [T, D] 각 row i = 현재 위치 기준 i+1 step 후의 누적 displacement + tool

    UMI 방식: action[i] = T_now→t+i+1 (현재 pose 기준 절대 상대 위치)
    모든 waypoint가 동일한 기준점(현재 EE)에서 독립적으로 계산되므로
    오차가 누적되지 않음.
    """
    T = len(deltas)
    relative = np.zeros_like(deltas)

    # Translation: 단순 누적합
    relative[:, :3] = np.cumsum(deltas[:, :3], axis=0)

    # Rotation (axis-angle): proper 회전 합성
    cumulative = Rotation.identity()
    for i in range(T):
        cumulative = cumulative * Rotation.from_rotvec(deltas[i, 3:6])
        relative[i, 3:6] = cumulative.as_rotvec()

    # Tool 차원 (6 이후): absolute 값이므로 변환 없이 그대로 복사
    if deltas.shape[1] > 6:
        relative[:, 6:] = deltas[:, 6:]

    return relative


def relative_trajectory_to_delta(waypoints: np.ndarray, current_eepos=None) -> np.ndarray:
    """relative trajectory → sequential delta 역변환 (inference 시 사용).

    current_eepos 가 주어지면 DexUMI 스타일 (current EE local frame trajectory → world
    sequential deltas). None 이면 legacy 동작 (waypoints 를 world cumsum 으로 간주,
    naive 차분). 옛 학습 모델 호환 위해 기본 None.

    Args:
        waypoints: [T, D] relative trajectory. 앞 6 차원이 EE pose, D > 6 일 때 6 이후는
                   tool joint (absolute).
        current_eepos: [D] 현재 EE pose. 주어지면 local-frame waypoints 를 world 로
                       풀고 sequential delta 계산.

    Returns:
        [T, D] sequential deltas (+ tool absolute).
    """
    waypoints = np.asarray(waypoints, dtype=np.float32)
    if current_eepos is not None:
        # DexUMI 스타일 역변환: utils 의 helper 사용.
        return relative_trajectory_in_local_frame_to_world_deltas(waypoints, current_eepos)

    # Legacy 경로 (current_eepos 없을 때) — naive 누적합 모델용.
    T = len(waypoints)
    deltas = np.zeros_like(waypoints)

    # Translation: 연속 차분
    deltas[0, :3] = waypoints[0, :3]
    deltas[1:, :3] = np.diff(waypoints[:, :3], axis=0)

    # Rotation: 연속 회전 차분
    deltas[0, 3:6] = waypoints[0, 3:6]
    for i in range(1, T):
        r_prev = Rotation.from_rotvec(waypoints[i - 1, 3:6])
        r_curr = Rotation.from_rotvec(waypoints[i, 3:6])
        deltas[i, 3:6] = (r_prev.inv() * r_curr).as_rotvec()

    # Tool 차원: 그대로 복사
    if waypoints.shape[1] > 6:
        deltas[:, 6:] = waypoints[:, 6:]

    return deltas


# ─── action_key / obs_state_keys 통일 헬퍼 ───────────────────────────────
#
# 데이터 스키마 정합성:
#   action.joint     ← qaction (절대 joint position target).
#   action.ee_delta  ← ee_delta_action (t+1 shift, legacy 'action' 컬럼과 동일).
#   observation.qpos ← 현재 frame joint 위치 (= legacy observation.state).
#   observation.qvel/qeffort/eepos ← 그 외 관측치.
#
# action_key 값:
#   - 'joint' 또는 'qaction' → action.joint 컬럼 사용.
#   - 'ee_delta' 또는 'ee_delta_action' → action.ee_delta 컬럼 사용.
#   - 'relative_ee_pos' / 기타 → 기존 legacy 'action' 컬럼 (특수 전처리 분기 유지).
#
# obs_state_keys 값: ['qpos', 'qvel', 'qeffort', 'eepos'] 중 임의 부분집합.
# 학습/추론 모두 같은 순서로 concat 해서 observation state 구성.

_ACTION_KEY_ALIASES = {
    'qaction': 'joint',
    'joint': 'joint',
    'ee_delta_action': 'ee_delta',
    'ee_delta': 'ee_delta',
    'relative_ee_pos': 'relative_ee_pos',
    # relative_joint_pos: action target = chunk-anchored delta from observation.qpos.
    # Anchor 출처는 obs_state_keys와 독립적인 별도 채널 (parquet의 observation.qpos
    # 컬럼). relative_ee_pos가 observation.eepos를 anchor로 쓰는 것과 대칭.
    'relative_joint_pos': 'relative_joint_pos',
}


def _absolute_action_dims_from_features(action_names, has_succeed: bool):
    """action.joint feature names + has_succeed 로부터 absolute dim 인덱스 자동 추출.

    relative_joint_pos 모드에서 어떤 dim이 delta로 변환돼선 안 되는지 결정.
    - 이름에 'gripper' 또는 'tool' 포함 → absolute (개폐 명령 / 도구 joint)
    - has_succeed=True → action의 마지막 dim이 succeed/done flag → absolute

    relative_ee_pos가 EE pose의 처음 6 dim을 transform하고 나머지(tool)는 그대로
    두는 패턴과 같은 논리. user가 따로 설정할 필요 없음.

    Args:
        action_names: dataset features['action.joint']['names'] (list of str).
                       Empty/None이면 빈 list 반환 (호출자가 fallback).
        has_succeed: True면 action 마지막 dim이 succeed bit로 append됨.

    Returns:
        list of int — absolute로 유지할 action dim 인덱스.
    """
    if not action_names:
        return []
    abs_dims = []
    for i, name in enumerate(action_names):
        nl = (name or '').lower()
        if 'gripper' in nl or 'tool' in nl:
            abs_dims.append(i)
    if has_succeed:
        abs_dims.append(len(action_names))
    return abs_dims


def _normalize_action_key(action_key):
    """레거시 / 신규 값 모두 받아 신규 표준명으로 변환."""
    if action_key is None:
        return 'joint'
    return _ACTION_KEY_ALIASES.get(action_key, action_key)


def _select_action_column(df, action_key):
    """action_key → 데이터프레임에서 읽을 컬럼명. 컬럼 없으면 legacy 'action' fallback.

    NOTE: ee_delta / relative_ee_pos 는 컬럼이 아닌 derive (eepos 차분) 가 우선되므로
    호출자가 직접 컬럼명을 읽기 보다 `_get_action_data` 를 쓰는 게 일반적이다.
    """
    norm = _normalize_action_key(action_key)
    if norm == 'joint' and 'action.joint' in df.columns:
        return 'action.joint'
    if norm == 'ee_delta' and 'action.ee_delta' in df.columns:
        return 'action.ee_delta'
    return 'action'


def _get_action_data(df, action_key, eepos_tool_qpos_indices=None):
    """action_key → 학습/통계용 action 배열 (T, D) 반환.

    action_key 분기:
      - 'joint'(/'qaction'): action.joint 컬럼 우선, 없으면 legacy 'action'.
      - 'ee_delta'(/'ee_delta_action'): observation.eepos 시간 차분 (t+1 - t,
        마지막 0 padding) 으로 derive. eepos 없으면 legacy action.ee_delta 또는
        'action' fallback.
      - 'relative_ee_pos': ee_delta 와 동일하게 derive (호출자가 추후
        delta_to_relative_trajectory 적용).

    eepos_tool_qpos_indices: backward-compat. 옛 dataset 의 eepos 에 별도 그리퍼
    joint 가 빠진 경우, qpos 의 해당 인덱스를 eepos 끝에 append 한 뒤 diff 계산.

    eepos diff 로 derive 하는 이유: action.ee_delta 컬럼은 record_episode 의
    tele-type 별 분기에 의존해 텔레옵별로 값이 달라지거나 0 으로 떨어졌음.
    eepos 는 실제 robot motion 의 절대 EE 좌표라 시간 차분이 신뢰할 수 있는
    "다음 step delta" 가 됨. 회전(rxryrz axis-angle) 은 per-step 변화 작아
    naive 차분으로 충분.
    """
    norm = _normalize_action_key(action_key)

    if norm in ('joint', 'relative_joint_pos'):
        # relative_joint_pos: raw joint absolute을 그대로 반환. 호출자(get_norm_stats /
        # EpisodicDataset)가 observation.qpos를 anchor로 받아 chunk-anchored delta로
        # 후처리. relative_ee_pos 패턴과 대칭.
        col = 'action.joint' if 'action.joint' in df.columns else 'action'
        return np.array(df[col].tolist(), dtype=np.float32)

    if norm in ('ee_delta', 'relative_ee_pos'):
        if 'observation.eepos' in df.columns:
            eepos = np.array(df['observation.eepos'].tolist(), dtype=np.float32)
            if eepos_tool_qpos_indices:
                _qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else 'observation.state'
                if _qpos_col in df.columns:
                    _qpos_arr = np.array(df[_qpos_col].tolist(), dtype=np.float32)
                    eepos = np.concatenate([eepos, _qpos_arr[:, eepos_tool_qpos_indices]], axis=1)
            delta = np.zeros_like(eepos)
            if len(eepos) > 1:
                delta[:-1] = eepos[1:] - eepos[:-1]
            return delta
        # 옛 dataset 호환: 저장된 action.ee_delta 또는 legacy 'action' 사용.
        if 'action.ee_delta' in df.columns:
            return np.array(df['action.ee_delta'].tolist(), dtype=np.float32)
        if 'action' in df.columns:
            return np.array(df['action'].tolist(), dtype=np.float32)
        # 마지막 fallback: action.joint (옛 ee_delta 가 없으면 joint 라도 반환).
        return np.array(df['action.joint'].tolist(), dtype=np.float32)

    # 기타 / 미지의 action_key: 새/옛 schema 모두 대응.
    col = 'action' if 'action' in df.columns else 'action.joint'
    return np.array(df[col].tolist(), dtype=np.float32)


def _compute_eepos_tool_augment_indices(dataset_info):
    """옛 dataset 호환: observation.eepos 에 별도 tool agent (그리퍼) joint 가 빠진
    경우, qpos 의 어느 인덱스를 끝에 append 해야 하는지 계산.

    판별 로직: features 의 column names 파싱.
      - eepos.names 의 'robot_<id>_...' prefix 들 → eepos 에 이미 표현된 robot id 집합
        (arm EE 또는 'tool_' 표기 포함)
      - qpos.names 중 그 집합에 없는 robot id 는 'separate tool agent' (그리퍼 등)
      - 해당 robot id 의 qpos 인덱스가 augment 대상

    Returns:
        list[int] — qpos 의 추가 인덱스. 비어 있으면 augment 불필요
        (eepos 가 이미 tool 포함하거나, tool agent 가 없는 경우).
    """
    features = dataset_info.get("features", {}) if dataset_info else {}
    qpos_names = features.get("observation.qpos", {}).get("names") or []
    eepos_names = features.get("observation.eepos", {}).get("names") or []
    if not qpos_names or not eepos_names:
        return []

    # robot ID 추출. joint 이름에 underscore 가 있을 수 있어 ([^_]+) 로 limit.
    # 예: 'robot_1_knuckle_joint' → '1' (greedy \w+_ 쓰면 '1_knuckle' 로 잘못 잡힘)
    import re as _re
    _robot_re = _re.compile(r'^robot_([^_]+)_')
    def _rid(name):
        m = _robot_re.match(name)
        return m.group(1) if m else None

    eepos_robot_ids = set()
    for name in eepos_names:
        rid = _rid(name)
        if rid is not None:
            eepos_robot_ids.add(rid)

    tool_indices = []
    for i, name in enumerate(qpos_names):
        rid = _rid(name)
        if rid is not None and rid not in eepos_robot_ids:
            tool_indices.append(i)
    return tool_indices


def _build_obs_state(df, obs_state_keys):
    """obs_state_keys 순서대로 컬럼을 concat 해 (T, total_dim) state 행렬 반환.

    누락된 컬럼 (예: 옛 dataset 에 observation.qpos 가 없으면 observation.state 사용)
    은 가능한 한 fallback 으로 메우고, 정말 없으면 skip.
    """
    if not obs_state_keys:
        obs_state_keys = ['qpos']

    parts = []
    for key in obs_state_keys:
        # qpos: 신규 컬럼 observation.qpos 우선, 옛 dataset 은 observation.state 사용.
        if key == 'qpos':
            col = 'observation.qpos' if 'observation.qpos' in df.columns else 'observation.state'
        else:
            col = f'observation.{key}'
        if col in df.columns:
            arr = np.array(df[col].tolist(), dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(-1, 1)
            parts.append(arr)
    if not parts:
        # 최후의 fallback — observation.state 가 있으면 그것만.
        if 'observation.state' in df.columns:
            arr = np.array(df['observation.state'].tolist(), dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(-1, 1)
            return arr
        raise ValueError("No observation column found in dataset")
    return np.concatenate(parts, axis=1)


class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone='resnet18', n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, augment=False, wrist_sensor_ids=None):
        super(EpisodicDataset).__init__()
        self.episode_ids = episode_ids
        self.dataset_dir = dataset_dir
        self.sensor_ids = sensor_ids
        self.norm_stats = norm_stats
        self.chunk_size = chunk_size
        self.n_obs_steps = n_obs_steps
        self.policy_type = policy_type
        self.vision_backbone = vision_backbone
        self.action_key = action_key
        self.use_relative_trajectory = use_relative_trajectory
        self.obs_state_keys = obs_state_keys if obs_state_keys is not None else ['qpos']
        # Image augmentation: PI0.5 paper Appendix E recipe (RandomCrop 95% + Resize
        # + Rotate ±5° + ColorJitter). Applied on PIL images before process_image
        # so it interacts correctly with any downstream normalization. Train-only;
        # val/eval datasets instantiate with augment=False.
        # Wrist cameras: openpi skips spatial transforms on wrist views because
        # crop/rotate breaks the gripper↔observation geometric coupling. Only
        # ColorJitter is applied. wrist_sensor_ids defaults to empty (all sensors
        # get the full aug pipeline — backward compatible). User must configure
        # which sensor IDs are wrist-mounted to opt into the differentiated path.
        self.augment = augment
        self.wrist_sensor_ids = set(int(x) for x in (wrist_sensor_ids or []))
        if augment:
            self._image_augment_full = transforms.Compose([
                transforms.RandomResizedCrop(size=(224, 224), scale=(0.9025, 1.0), ratio=(0.95, 1.05)),
                transforms.RandomRotation(degrees=5),
                transforms.ColorJitter(brightness=0.3, contrast=0.4, saturation=0.5),
            ])
            self._image_augment_color_only = transforms.ColorJitter(
                brightness=0.3, contrast=0.4, saturation=0.5
            )
        else:
            self._image_augment_full = None
            self._image_augment_color_only = None
        self.info = None

        # Pre-load episode metadata (lengths) for efficient sampling
        self._ep_cache = {}
        self._dataset_info = get_dataset_info(dataset_dir)
        # Backward-compat: 옛 dataset 의 eepos 에 별도 tool agent joint 가 빠져 있으면
        # qpos 의 해당 인덱스를 매 episode 로드 시 eepos 뒤에 append.
        self._eepos_tool_qpos_indices = _compute_eepos_tool_augment_indices(self._dataset_info)
        if self._eepos_tool_qpos_indices:
            print(
                f"[EpisodicDataset] Augmenting eepos with qpos indices "
                f"{self._eepos_tool_qpos_indices} (backward-compat for separate tool agents).",
                flush=True,
            )

        # Pre-load episode → tasks mapping from episodes.jsonl (lerobot standard).
        # Each line: {"episode_index": N, "length": L, "tasks": ["pick up the cup", ...]}
        # PI05 uses the first entry of `tasks` per episode as its language instruction.
        # NOTE: origin/integration_2 moved lerobot_io.py from `..api.process.` to `..utils.`,
        # so we adopt the new import path. We keep our `_episode_tasks` lookup (per-episode
        # list) instead of origin's `_task_map` (task_index→task) because downstream
        # `_load_episode_parquet` uses _episode_tasks for PI05 language conditioning.
        from ..utils.lerobot_io import _read_jsonl, EPISODES_PATH
        episodes_meta = _read_jsonl(os.path.join(dataset_dir, EPISODES_PATH))
        self._episode_tasks = {
            int(e.get("episode_index")): (e.get("tasks") or [""])
            for e in episodes_meta
            if e.get("episode_index") is not None
        }

        self.__getitem__(0) # initialize self.info

    def __len__(self):
        return len(self.episode_ids)

    def _load_episode_parquet(self, episode_id):
        """Load parquet data for an episode, with caching."""
        if episode_id in self._ep_cache:
            return self._ep_cache[episode_id]

        chunk = episode_id // self._dataset_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
        parquet_path = os.path.join(
            self.dataset_dir,
            PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_id),
        )
        table = pq.read_table(parquet_path)
        df = table.to_pandas()

        # obs_state_keys 와 action_key 에 따라 state/action 구성. action_key 가
        # ee_delta/relative_ee_pos 면 _get_action_data 가 observation.eepos 시간
        # 차분으로 derive — action.ee_delta 컬럼 저장 안 함.
        state_data = _build_obs_state(df, self.obs_state_keys)
        action_data = _get_action_data(df, self.action_key, self._eepos_tool_qpos_indices)
        # eepos sequence — relative_ee_pos 학습에서 __getitem__ 이 chunk 별
        # current EE local frame trajectory 를 계산하는 데 사용. 없으면 None.
        eepos_data = (
            np.array(df['observation.eepos'].tolist(), dtype=np.float32)
            if 'observation.eepos' in df.columns else None
        )
        # qpos sequence — relative_joint_pos 학습에서 __getitem__ 이 chunk-anchored
        # delta 계산에 직접 사용. obs_state_keys와 독립적인 별도 채널이라
        # state_data와 별개로 보관. 옛 dataset 호환을 위해 observation.state도 fallback.
        _qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else (
            'observation.state' if 'observation.state' in df.columns else None
        )
        qpos_anchor_data = (
            np.array(df[_qpos_col].tolist(), dtype=np.float32)
            if _qpos_col else None
        )
        if qpos_anchor_data is not None and qpos_anchor_data.ndim == 1:
            qpos_anchor_data = qpos_anchor_data.reshape(-1, 1)
        # Backward-compat: 옛 dataset 의 eepos 에 별도 그리퍼 joint 가 없으면 qpos
        # 에서 뽑아 append (absolute pass-through 형태).
        if eepos_data is not None and self._eepos_tool_qpos_indices:
            _qpos_col2 = 'observation.qpos' if 'observation.qpos' in df.columns else 'observation.state'
            if _qpos_col2 in df.columns:
                _qpos_arr = np.array(df[_qpos_col2].tolist(), dtype=np.float32)
                _tool_cols = _qpos_arr[:, self._eepos_tool_qpos_indices]
                eepos_data = np.concatenate([eepos_data, _tool_cols], axis=1)

        # Language instruction from episodes.jsonl (per-episode `tasks` list).
        # Use the first entry; most datasets have a single task per episode.
        ep_tasks = self._episode_tasks.get(int(episode_id), [""])
        language_instruction = ep_tasks[0] if ep_tasks else ""

        # Get image data per sensor — from parquet (legacy), video files, or PNG dir.
        # CRITICAL: silent fallback to np.zeros((224,224,3)) was masking missing images
        # and causing the model to train on all-black frames. Each sensor MUST resolve
        # to actual images here; an assertion below catches the failure mode loudly.
        image_data = {}
        image_cols = [col for col in df.columns if col.startswith("observation.images.")]
        if image_cols:
            # Legacy image mode: images embedded in parquet
            for col in image_cols:
                sensor_name = col.replace("observation.images.", "")
                image_data[sensor_name] = list(df[col])
        else:
            # Video / PNG mode: prefer pre-decoded .npy (mmap'd), then mp4, then PNG dir.
            features = self._dataset_info.get("features", {})
            for feat_key, feat in features.items():
                if not feat_key.startswith("observation.images."):
                    continue
                sensor_name = feat_key.replace("observation.images.", "")
                video_path = os.path.join(
                    self.dataset_dir, "videos", f"chunk-{chunk:03d}", feat_key,
                    f"episode_{episode_id:06d}.mp4"
                )
                npy_path = video_path[:-4] + ".npy"
                # PNG fallback: images/{feat_key}/episode_{id:06d}/frame_{idx:06d}.png
                # lerobot_io.py writes frames here when video encoding isn't used.
                png_dir = os.path.join(
                    self.dataset_dir, "images", feat_key, f"episode_{episode_id:06d}"
                )
                if os.path.exists(npy_path):
                    image_data[sensor_name] = np.load(npy_path, mmap_mode="r")
                elif os.path.exists(video_path):
                    from ..utils.lerobot_io import _decode_video_frames
                    image_data[sensor_name] = _decode_video_frames(video_path)
                elif os.path.isdir(png_dir):
                    # Build a list of frame_XXXXXX.png paths in order. _parse_image_value
                    # accepts string paths and decodes via PIL.
                    n_frames = len(state_data)
                    image_data[sensor_name] = [
                        os.path.join(png_dir, f"frame_{i:06d}.png") for i in range(n_frames)
                    ]
                else:
                    image_data[sensor_name] = []

        # Loud assertion: if any sensor came back empty for an episode that has frames,
        # the dataset is silently feeding all-black images. Fail early instead of
        # training on garbage.
        for sensor_name, vals in image_data.items():
            if len(vals) == 0 and len(state_data) > 0:
                raise RuntimeError(
                    f"[EpisodicDataset] No image source found for sensor '{sensor_name}' "
                    f"in episode {episode_id} (dataset_dir={self.dataset_dir}). "
                    f"Checked: parquet column, .npy, .mp4, PNG dir. "
                    f"Training would silently use all-zero frames."
                )

        succeed = np.array(df["succeed"].tolist(), dtype=np.float32) if "succeed" in df.columns else None

        result = {
            "state_data": state_data,
            "action_data": action_data,
            "eepos_data": eepos_data,
            "qpos_anchor_data": qpos_anchor_data,
            "language_instruction": language_instruction,
            "image_data": image_data,
            "succeed": succeed,
            "episode_len": len(df),
        }
        self._ep_cache[episode_id] = result
        return result

    def __getitem__(self, index):
        episode_id = self.episode_ids[index]
        ep = self._load_episode_parquet(episode_id)

        episode_len = ep["episode_len"]
        state_data = ep["state_data"]   # (T, state_dim)
        action_data = ep["action_data"]  # (T, action_dim)
        language_instruction = ep["language_instruction"]

        action_dim = action_data.shape[1]

        # succeed 처리
        expected_action_dim = self.norm_stats['action']['mean'].shape[-1]
        any_has_succeed = (expected_action_dim > action_dim)
        if any_has_succeed:
            action_dim = expected_action_dim

        original_action_shape = (self.chunk_size, action_dim)

        if episode_len <= self.chunk_size + self.n_obs_steps - 1:
            start_ts = self.n_obs_steps - 1
        else:
            start_ts = np.random.choice(np.arange(self.n_obs_steps - 1, episode_len - self.chunk_size))
        end_ts = start_ts + self.chunk_size

        obs_step_start = start_ts - self.n_obs_steps + 1

        # Observation states
        qpos = []
        for i in range(self.n_obs_steps):
            idx = max(0, min(obs_step_start + i, episode_len - 1))
            qpos.append(state_data[idx])

        # Images
        image_dict = {}
        for sensor_id in self.sensor_ids:
            key = f"sensor_{sensor_id}"
            image_dict[key] = []
            img_vals = ep["image_data"].get(key, [])
            for i in range(self.n_obs_steps):
                idx = max(0, min(obs_step_start + i, episode_len - 1))
                if idx < len(img_vals):
                    img_array = _parse_image_value(img_vals[idx], self.dataset_dir)
                else:
                    img_array = np.zeros((224, 224, 3), dtype=np.uint8)
                image_dict[key].append(img_array)

        # Actions
        actual_end = min(episode_len, end_ts)
        action = action_data[start_ts:actual_end]

        if any_has_succeed:
            if ep["succeed"] is not None:
                succeed = ep["succeed"][start_ts:actual_end].reshape(-1, 1)
            else:
                succeed = np.zeros((action.shape[0], 1), dtype=np.float32)
            action = np.concatenate([action, succeed], axis=1)

        action_len = min(self.chunk_size, episode_len - start_ts)

        padded_action = np.zeros(original_action_shape, dtype=np.float32)
        if _normalize_action_key(self.action_key) == 'relative_ee_pos' or (self.use_relative_trajectory and _normalize_action_key(self.action_key) == 'ee_delta'):
            # DexUMI style: chunk 의 미래 N 개 eepos 를 현재 eepos local frame 으로
            # 변환. eepos 가 없으면 (옛 dataset) legacy naive cumsum 으로 fallback.
            eepos_data = ep.get("eepos_data")
            if eepos_data is not None and len(eepos_data) > start_ts:
                future_idx = np.arange(start_ts + 1, start_ts + self.chunk_size + 1)
                future_idx = np.clip(future_idx, 0, episode_len - 1)
                future_eepos = eepos_data[future_idx]
                current_eepos = eepos_data[start_ts]
                rel_traj = compute_relative_trajectory_in_local_frame(future_eepos, current_eepos)
                padded_action[:action_len, :rel_traj.shape[1]] = rel_traj[:action_len]
                # any_has_succeed 면 action 의 마지막 컬럼이 succeed flag — rel_traj
                # 는 eepos 차원만 다루므로 succeed slot 을 따로 복사.
                if any_has_succeed and action.shape[1] > rel_traj.shape[1]:
                    padded_action[:action_len, rel_traj.shape[1]:] = action[:action_len, rel_traj.shape[1]:]
            else:
                padded_action[:action_len] = delta_to_relative_trajectory(action[:action_len])
        elif _normalize_action_key(self.action_key) == 'relative_joint_pos':
            # chunk-anchored joint delta: action[t in chunk] - qpos[chunk_start].
            # anchor를 observation.qpos에서 직접 읽음 (obs_state_keys와 독립).
            # absolute dims (gripper/tool/done)는 dataset features에서 자동 추출.
            qpos_anchor_data = ep.get("qpos_anchor_data")
            chunk_slice = action[:action_len]
            if qpos_anchor_data is not None and len(qpos_anchor_data) > start_ts:
                action_dim = chunk_slice.shape[1]
                anchor_dim = qpos_anchor_data.shape[1]
                _features = (self._dataset_info or {}).get('features', {})
                _action_names = _features.get('action.joint', {}).get('names', [])
                _has_succeed = action_dim > len(_action_names) and len(_action_names) > 0
                _abs_dims = _absolute_action_dims_from_features(_action_names, _has_succeed)
                if _abs_dims:
                    effective_mask = [True] * action_dim
                    for idx in _abs_dims:
                        if 0 <= int(idx) < action_dim:
                            effective_mask[int(idx)] = False
                else:
                    shared = min(anchor_dim, action_dim)
                    effective_mask = [True] * shared + [False] * (action_dim - shared)
                anchor_at_start = qpos_anchor_data[start_ts]
                delta_chunk = chunk_slice.copy()
                for i, is_delta in enumerate(effective_mask):
                    if is_delta and i < anchor_dim:
                        delta_chunk[:, i] = delta_chunk[:, i] - anchor_at_start[i]
                padded_action[:action_len] = delta_chunk
            else:
                padded_action[:action_len] = chunk_slice
        else:
            padded_action[:action_len] = action
        is_pad = np.zeros(self.chunk_size)
        is_pad[action_len:] = 1

        item = dict()
        item['language_instruction'] = language_instruction
        for sensor_id in self.sensor_ids:
            processed_images = []
            # Pick aug based on whether this sensor is configured as wrist-mounted.
            # Wrist cams: ColorJitter only (preserve spatial geometry).
            # Other cams: full aug (crop+rotate+jitter).
            if int(sensor_id) in getattr(self, 'wrist_sensor_ids', set()):
                aug = getattr(self, '_image_augment_color_only', None)
            else:
                aug = getattr(self, '_image_augment_full', None)
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                if aug is not None:
                    image = aug(image)
                # PI05's SigLIP vision tower expects [-1, 1] (PaliGemma / openpi convention).
                # ToTensor alone yields [0, 1] and breaks pretrained visual features.
                _pixel_range = '-11' if self.policy_type == 'PI05' else '01'
                image = process_image(image, self.vision_backbone, pixel_range=_pixel_range)
                processed_images.append(image)

            # New lerobot expects [C, H, W] per sample (collate → [batch, C, H, W])
            stacked = torch.stack(processed_images)  # [n_obs_steps, C, H, W]
            if self.n_obs_steps == 1:
                item[f"observation.images.sensor_{sensor_id}"] = stacked.squeeze(0)  # [C, H, W]
            else:
                item[f"observation.images.sensor_{sensor_id}"] = stacked

        if self.n_obs_steps == 1:
            item["observation.state"] = torch.from_numpy(qpos[0]).float()
        elif self.policy_type in ['PI0', 'PI05']:
            item["observation.state"] = torch.from_numpy(np.concatenate(qpos)).float()
        else:
            item["observation.state"] = torch.from_numpy(np.array(qpos)).float()

        item["action"] = torch.from_numpy(padded_action).float()
        item["action_is_pad"] = torch.from_numpy(is_pad).bool()
        item['next.done'] = torch.from_numpy(np.zeros(1, dtype=np.bool_)).bool()

        if self.info is None:
            self.info = dict()
            for key, val in item.items():
                if key.startswith("observation.images"):
                    self.info[key] = PolicyFeature(FeatureType.VISUAL, shape=val.shape)
                if key == "observation.state":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=val.shape)
                if key == "action":
                    self.info[key] = PolicyFeature(FeatureType.ACTION, shape=val[0].shape)
                if key == "language_instruction":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=(1,))

        return item



def process_image(image, vision_backbone='resnet18', to_cuda=False, pixel_range='01'):
    """Preprocess an image into a model-ready tensor.

    pixel_range: '01' → standard torchvision ToTensor output in [0, 1].
                 '-11' → scaled to [-1, 1] (mandatory for PI05 / PaliGemma / SigLIP —
                         openpi preprocess_observation explicitly does `img / 255 * 2 - 1`).
                         Feeding [0, 1] into pi05_base shifts the vision feature
                         distribution and wrecks pretrained visual grounding; LoRA
                         can't fully recover it, which matches the "moves but misses"
                         symptom during inference.
    """
    if not isinstance(image, Image.Image):
        image = Image.fromarray(np.array(image))
    if vision_backbone not in VISION_BACKBONE_MAP:
        image_transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
        ])
        image = image_transform(image)
        if pixel_range == '-11':
            image = image * 2.0 - 1.0
    else:
        image_processor = AutoImageProcessor.from_pretrained(VISION_BACKBONE_MAP[vision_backbone])
        image = image_processor(image)['pixel_values'][0]  # backbone's own normalization

    return image.cuda() if to_cuda else image


def get_norm_stats(dataset_dir, num_episodes, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, chunk_size=50):
    """Compute normalization stats from LeRobot dataset parquet files.

    For ``action_key='relative_joint_pos'`` the action is overwritten with the
    chunk-anchored delta from observation.qpos before stats are accumulated —
    same idea as relative_ee_pos but in joint space. Absolute dims (gripper /
    tool / done) are auto-detected from action feature names + has_succeed via
    ``_absolute_action_dims_from_features``.
    """
    if obs_state_keys is None:
        obs_state_keys = ['qpos']

    dataset_info = get_dataset_info(dataset_dir)
    # Backward-compat: 옛 dataset 에 별도 그리퍼 joint 가 eepos 에 없으면 qpos 에서
    # 보강. dataset 클래스와 동일한 logic 으로 stats 도 7-dim 으로 계산해야 학습/추론
    # dim 이 일치.
    eepos_tool_qpos_indices = _compute_eepos_tool_augment_indices(dataset_info)
    all_qpos_data = []
    all_action_data = []
    observation_image_keys = []
    cnt = 0
    skipped_episodes = []

    # Check if any episode has succeed
    any_has_succeed = False
    expected_action_dim = None

    for episode_idx in range(num_episodes):
        chunk = episode_idx // dataset_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
        parquet_path = os.path.join(dataset_dir, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_idx))
        if not os.path.exists(parquet_path):
            skipped_episodes.append(episode_idx)
            continue

        table = pq.read_table(parquet_path)
        df = table.to_pandas()

        # obs_state_keys / action_key 에 따라 state/action 구성. ee_delta 등은
        # observation.eepos 차분으로 derive.
        state_data = _build_obs_state(df, obs_state_keys)
        action_data = _get_action_data(df, action_key, eepos_tool_qpos_indices)

        # Check succeed
        if "succeed" in df.columns:
            any_has_succeed = True

    # Second pass: collect data
    for episode_idx in range(num_episodes):
        if episode_idx in skipped_episodes:
            continue
        chunk = episode_idx // dataset_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
        parquet_path = os.path.join(dataset_dir, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_idx))
        table = pq.read_table(parquet_path)
        df = table.to_pandas()

        qpos = _build_obs_state(df, obs_state_keys)
        action = _get_action_data(df, action_key, eepos_tool_qpos_indices)

        if qpos.ndim == 1:
            qpos = qpos.reshape(1, -1)
        if action.ndim == 1:
            action = action.reshape(1, -1)

        # succeed 플래그
        if any_has_succeed:
            if "succeed" in df.columns:
                succeed = np.array(df["succeed"].tolist(), dtype=np.float32).reshape(-1, 1)
            else:
                succeed = np.zeros((action.shape[0], 1), dtype=np.float32)
            action = np.concatenate([action, succeed], axis=1)

        if expected_action_dim is None:
            expected_action_dim = action.shape[1]
        elif action.shape[1] != expected_action_dim:
            print(f'[WARN] episode_{episode_idx} action dim {action.shape[1]} != expected {expected_action_dim}, skipping.')
            skipped_episodes.append(episode_idx)
            continue

        if _normalize_action_key(action_key) == 'relative_ee_pos' or (use_relative_trajectory and _normalize_action_key(action_key) == 'ee_delta'):
            # eepos 있으면 DexUMI 스타일 — 매 frame 을 chunk 시작점으로 보고 그
            # 시점부터 chunk_size step 의 trajectory in local frame 으로 stats 누적.
            if 'observation.eepos' in df.columns:
                eepos_arr = np.array(df['observation.eepos'].tolist(), dtype=np.float32)
                # Backward-compat: dataset class __getitem__ 과 동일 차원이 되도록
                # eepos 끝에 tool qpos append.
                if eepos_tool_qpos_indices:
                    _qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else 'observation.state'
                    if _qpos_col in df.columns:
                        _qpos_arr = np.array(df[_qpos_col].tolist(), dtype=np.float32)
                        eepos_arr = np.concatenate(
                            [eepos_arr, _qpos_arr[:, eepos_tool_qpos_indices]], axis=1
                        )
                T_total = len(eepos_arr)
                if T_total >= 2 and chunk_size > 0:
                    rel_chunks = []
                    succeed_chunks = []
                    succeed_arr = None
                    if any_has_succeed and 'succeed' in df.columns:
                        succeed_arr = np.array(df['succeed'].tolist(), dtype=np.float32).reshape(-1, 1)
                    for start in range(T_total - 1):
                        future_idx = np.clip(
                            np.arange(start + 1, start + chunk_size + 1), 0, T_total - 1
                        )
                        rel_chunks.append(compute_relative_trajectory_in_local_frame(
                            eepos_arr[future_idx], eepos_arr[start]
                        ))
                        if succeed_arr is not None:
                            succeed_chunks.append(succeed_arr[future_idx])
                    action = np.concatenate(rel_chunks, axis=0)
                    # 옛 코드는 여기서 action 을 rel_chunks 로 통째 overwrite 해서
                    # 앞에서 붙여둔 succeed dim 이 사라졌다 → 모델이 succeed 차원을
                    # 학습 못 하고, 추론 시 inference_succeed 이벤트가 안 뜸.
                    if succeed_chunks:
                        succeed_concat = np.concatenate(succeed_chunks, axis=0)
                        action = np.concatenate([action, succeed_concat], axis=1)
                else:
                    action = delta_to_relative_trajectory(action)
            else:
                action = delta_to_relative_trajectory(action)

        # relative_joint_pos: obs_state_keys와 독립. observation.qpos 컬럼을 직접
        # anchor로 사용. relative_ee_pos가 observation.eepos를 직접 anchor로 쓰는
        # 패턴과 대칭. action target = chunk-anchored delta from qpos[chunk_start].
        if _normalize_action_key(action_key) == 'relative_joint_pos':
            _qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else 'observation.state'
            if _qpos_col in df.columns:
                anchor_arr = np.array(df[_qpos_col].tolist(), dtype=np.float32)
                if anchor_arr.ndim == 1:
                    anchor_arr = anchor_arr.reshape(-1, 1)
                T_total = action.shape[0]
                action_dim = action.shape[1]
                anchor_dim = anchor_arr.shape[1]
                # absolute_action_dims는 dataset features의 action.joint names + has_succeed
                # 로부터 자동 계산. gripper / tool / done dim은 절대 유지.
                _action_names = (dataset_info or {}).get('features', {}).get('action.joint', {}).get('names', [])
                _abs_dims = _absolute_action_dims_from_features(_action_names, any_has_succeed)
                if _abs_dims:
                    effective_mask = [True] * action_dim
                    for idx in _abs_dims:
                        if 0 <= int(idx) < action_dim:
                            effective_mask[int(idx)] = False
                else:
                    shared = min(anchor_dim, action_dim)
                    effective_mask = [True] * shared + [False] * (action_dim - shared)

                rel_chunks = []
                cs = max(1, int(chunk_size))
                for start in range(T_total):
                    end = min(start + cs, T_total)
                    _chunk = action[start:end].copy()
                    anchor_at_start = anchor_arr[start]
                    for i, is_delta in enumerate(effective_mask):
                        if is_delta and i < anchor_dim:
                            _chunk[:, i] = _chunk[:, i] - anchor_at_start[i]
                    rel_chunks.append(_chunk)
                action = np.concatenate(rel_chunks, axis=0) if rel_chunks else action

        # Collect image keys from parquet columns
        for col in df.columns:
            if col.startswith("observation.images."):
                sensor_name = col.replace("observation.images.", "")
                if sensor_name not in observation_image_keys:
                    observation_image_keys.append(sensor_name)

        cnt += qpos.shape[0]
        all_qpos_data.append(torch.from_numpy(qpos))
        all_action_data.append(torch.from_numpy(action))

    # Also collect image keys from features (video mode: not in parquet columns)
    if dataset_info:
        for feat_key, feat in dataset_info.get("features", {}).items():
            if feat_key.startswith("observation.images.") and feat.get("dtype") in ("video", "image"):
                sensor_name = feat_key.replace("observation.images.", "")
                if sensor_name not in observation_image_keys:
                    observation_image_keys.append(sensor_name)

    if skipped_episodes:
        print(f'[WARN] Skipped incompatible episodes: {skipped_episodes}')

    all_qpos_data = torch.cat(all_qpos_data, dim=0)
    all_action_data = torch.cat(all_action_data, dim=0)

    action_min = all_action_data.min(dim=0)[0]
    action_max = all_action_data.max(dim=0)[0]
    action_mean = all_action_data.mean(dim=0)
    action_std = all_action_data.std(dim=0)
    action_std = torch.clip(action_std, 1e-2, np.inf)
    # q01/q99 for QUANTILES normalization (Pi0.5). Using torch.quantile.
    action_q01 = torch.quantile(all_action_data.float(), 0.01, dim=0).to(all_action_data.dtype)
    action_q99 = torch.quantile(all_action_data.float(), 0.99, dim=0).to(all_action_data.dtype)
    # Guard against near-constant columns (e.g. a rarely-triggered done flag whose
    # q01 == q99 == 0). Without this the normalizer falls back to eps=1e-8 denominator
    # and the single non-zero value becomes ~1e8 in normalized space — destroys training.
    _range = action_q99 - action_q01
    _min_range = torch.clamp(_range, min=1e-2)
    action_q99 = action_q01 + _min_range

    qpos_min = all_qpos_data.min(dim=0)[0]
    qpos_max = all_qpos_data.max(dim=0)[0]
    qpos_mean = all_qpos_data.mean(dim=0)
    qpos_std = all_qpos_data.std(dim=0)
    qpos_std = torch.clip(qpos_std, 1e-2, np.inf)
    qpos_q01 = torch.quantile(all_qpos_data.float(), 0.01, dim=0).to(all_qpos_data.dtype)
    qpos_q99 = torch.quantile(all_qpos_data.float(), 0.99, dim=0).to(all_qpos_data.dtype)
    _qrange = qpos_q99 - qpos_q01
    _qmin_range = torch.clamp(_qrange, min=1e-2)
    qpos_q99 = qpos_q01 + _qmin_range

    stats = {
        "action": {
            "min": action_min.numpy(),
            "max": action_max.numpy(),
            "mean": action_mean.numpy().squeeze(),
            "std": action_std.numpy().squeeze(),
            "q01": action_q01.numpy(),
            "q99": action_q99.numpy(),
            "count": np.array([cnt]),
        },
        "observation.state": {
            "min": qpos_min.numpy(),
            "max": qpos_max.numpy(),
            "mean": qpos_mean.numpy().squeeze(),
            "std": qpos_std.numpy().squeeze(),
            "q01": qpos_q01.numpy(),
            "q99": qpos_q99.numpy(),
            "count": np.array([cnt]),
        },
    }

    for key in observation_image_keys:
        stats[f"observation.images.{key}"] = {
            "min": np.array([[[0.0]], [[0.0]], [[0.0]]]),
            "max": np.array([[[1.0]], [[1.0]], [[1.0]]]),
            "mean": np.array([[[0.5]], [[0.5]], [[0.5]]]),
            "std": np.array([[[0.25]], [[0.25]], [[0.25]]]),
            "count": np.array([cnt]),
        }

    return stats, skipped_episodes


class FullScanDataset(EpisodicDataset):
    """OOD feature 수집용: 모든 에피소드의 모든 스텝을 순차적으로 반환."""

    def __init__(self, *args, **kwargs):
        self._index_map = None
        super().__init__(*args, **kwargs)
        self._build_index_map()

    def _build_index_map(self):
        """(global_index) -> (episode_id, start_ts) 매핑 테이블 구축."""
        index_map = []
        for ep_idx, ep_id in enumerate(self.episode_ids):
            ep = self._load_episode_parquet(ep_id)
            episode_len = ep["episode_len"]

            stride = 57
            offset = (ep_idx * 17) % stride
            start = self.n_obs_steps - 1 + offset
            for ts in range(start, max(self.n_obs_steps, episode_len - self.chunk_size), stride):
                index_map.append((ep_id, ts))
        self._index_map = index_map
        print(f'[FullScanDataset] Total samples: {len(self._index_map)}')

    def __len__(self):
        if self._index_map is None:
            return len(self.episode_ids)
        return len(self._index_map)

    def __getitem__(self, index):
        if self._index_map is None:
            return super().__getitem__(index)

        ep_id, start_ts = self._index_map[index]
        ep = self._load_episode_parquet(ep_id)

        episode_len = ep["episode_len"]
        state_data = ep["state_data"]
        action_data = ep["action_data"]
        language_instruction = ep["language_instruction"]

        action_dim = action_data.shape[1]
        expected_action_dim = self.norm_stats['action']['mean'].shape[-1]
        any_has_succeed = (expected_action_dim > action_dim)
        if any_has_succeed:
            action_dim = expected_action_dim

        original_action_shape = (self.chunk_size, action_dim)
        end_ts = start_ts + self.chunk_size
        obs_step_start = start_ts - self.n_obs_steps + 1

        qpos = []
        for i in range(self.n_obs_steps):
            idx = max(0, min(obs_step_start + i, episode_len - 1))
            qpos.append(state_data[idx])

        image_dict = {}
        for sensor_id in self.sensor_ids:
            key = f"sensor_{sensor_id}"
            image_dict[key] = []
            img_vals = ep["image_data"].get(key, [])
            for i in range(self.n_obs_steps):
                idx = max(0, min(obs_step_start + i, episode_len - 1))
                if idx < len(img_vals):
                    img_array = _parse_image_value(img_vals[idx], self.dataset_dir)
                else:
                    img_array = np.zeros((224, 224, 3), dtype=np.uint8)
                image_dict[key].append(img_array)

        actual_end = min(episode_len, end_ts)
        action = action_data[start_ts:actual_end]

        if any_has_succeed:
            if ep["succeed"] is not None:
                succeed = ep["succeed"][start_ts:actual_end].reshape(-1, 1)
            else:
                succeed = np.zeros((action.shape[0], 1), dtype=np.float32)
            action = np.concatenate([action, succeed], axis=1)

        action_len = min(self.chunk_size, episode_len - start_ts)

        padded_action = np.zeros(original_action_shape, dtype=np.float32)
        if _normalize_action_key(self.action_key) == 'relative_ee_pos' or (self.use_relative_trajectory and _normalize_action_key(self.action_key) == 'ee_delta'):
            # DexUMI style: chunk 의 미래 N 개 eepos 를 현재 eepos local frame 으로
            # 변환. eepos 가 없으면 (옛 dataset) legacy naive cumsum 으로 fallback.
            eepos_data = ep.get("eepos_data")
            if eepos_data is not None and len(eepos_data) > start_ts:
                future_idx = np.arange(start_ts + 1, start_ts + self.chunk_size + 1)
                future_idx = np.clip(future_idx, 0, episode_len - 1)
                future_eepos = eepos_data[future_idx]
                current_eepos = eepos_data[start_ts]
                rel_traj = compute_relative_trajectory_in_local_frame(future_eepos, current_eepos)
                padded_action[:action_len, :rel_traj.shape[1]] = rel_traj[:action_len]
                # any_has_succeed 면 action 의 마지막 컬럼이 succeed flag — rel_traj
                # 는 eepos 차원만 다루므로 succeed slot 을 따로 복사.
                if any_has_succeed and action.shape[1] > rel_traj.shape[1]:
                    padded_action[:action_len, rel_traj.shape[1]:] = action[:action_len, rel_traj.shape[1]:]
            else:
                padded_action[:action_len] = delta_to_relative_trajectory(action[:action_len])
        elif _normalize_action_key(self.action_key) == 'relative_joint_pos':
            # chunk-anchored joint delta: action[t in chunk] - qpos[chunk_start].
            # anchor를 observation.qpos에서 직접 읽음 (obs_state_keys와 독립).
            # absolute dims (gripper/tool/done)는 dataset features에서 자동 추출.
            qpos_anchor_data = ep.get("qpos_anchor_data")
            chunk_slice = action[:action_len]
            if qpos_anchor_data is not None and len(qpos_anchor_data) > start_ts:
                action_dim = chunk_slice.shape[1]
                anchor_dim = qpos_anchor_data.shape[1]
                _features = (self._dataset_info or {}).get('features', {})
                _action_names = _features.get('action.joint', {}).get('names', [])
                _has_succeed = action_dim > len(_action_names) and len(_action_names) > 0
                _abs_dims = _absolute_action_dims_from_features(_action_names, _has_succeed)
                if _abs_dims:
                    effective_mask = [True] * action_dim
                    for idx in _abs_dims:
                        if 0 <= int(idx) < action_dim:
                            effective_mask[int(idx)] = False
                else:
                    shared = min(anchor_dim, action_dim)
                    effective_mask = [True] * shared + [False] * (action_dim - shared)
                anchor_at_start = qpos_anchor_data[start_ts]
                delta_chunk = chunk_slice.copy()
                for i, is_delta in enumerate(effective_mask):
                    if is_delta and i < anchor_dim:
                        delta_chunk[:, i] = delta_chunk[:, i] - anchor_at_start[i]
                padded_action[:action_len] = delta_chunk
            else:
                padded_action[:action_len] = chunk_slice
        else:
            padded_action[:action_len] = action
        is_pad = np.zeros(self.chunk_size)
        is_pad[action_len:] = 1

        item = dict()
        item['language_instruction'] = language_instruction
        for sensor_id in self.sensor_ids:
            processed_images = []
            # Pick aug based on whether this sensor is configured as wrist-mounted.
            # Wrist cams: ColorJitter only (preserve spatial geometry).
            # Other cams: full aug (crop+rotate+jitter).
            if int(sensor_id) in getattr(self, 'wrist_sensor_ids', set()):
                aug = getattr(self, '_image_augment_color_only', None)
            else:
                aug = getattr(self, '_image_augment_full', None)
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                if aug is not None:
                    image = aug(image)
                # PI05's SigLIP vision tower expects [-1, 1] (PaliGemma / openpi convention).
                # ToTensor alone yields [0, 1] and breaks pretrained visual features.
                _pixel_range = '-11' if self.policy_type == 'PI05' else '01'
                image = process_image(image, self.vision_backbone, pixel_range=_pixel_range)
                processed_images.append(image)

            # New lerobot expects [C, H, W] per sample (collate → [batch, C, H, W])
            stacked = torch.stack(processed_images)  # [n_obs_steps, C, H, W]
            if self.n_obs_steps == 1:
                item[f"observation.images.sensor_{sensor_id}"] = stacked.squeeze(0)  # [C, H, W]
            else:
                item[f"observation.images.sensor_{sensor_id}"] = stacked

        if self.n_obs_steps == 1:
            item["observation.state"] = torch.from_numpy(qpos[0]).float()
        elif self.policy_type in ['PI0', 'PI05']:
            item["observation.state"] = torch.from_numpy(np.concatenate(qpos)).float()
        else:
            item["observation.state"] = torch.from_numpy(np.array(qpos)).float()

        item["action"] = torch.from_numpy(padded_action).float()
        item["action_is_pad"] = torch.from_numpy(is_pad).bool()
        item['next.done'] = torch.from_numpy(np.zeros(1, dtype=np.bool_)).bool()

        if self.info is None:
            self.info = dict()
            for key, val in item.items():
                if key.startswith("observation.images"):
                    self.info[key] = PolicyFeature(FeatureType.VISUAL, shape=val.shape)
                if key == "observation.state":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=val.shape)
                if key == "action":
                    self.info[key] = PolicyFeature(FeatureType.ACTION, shape=val[0].shape)
                if key == "language_instruction":
                    self.info[key] = PolicyFeature(FeatureType.STATE, shape=(1,))

        return item


def load_data(dataset_dir, policy_type, num_episodes, sensor_ids, batch_size_train, batch_size_val, chunk_size, vision_backbone='resnet18', num_workers=1, n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, wrist_sensor_ids=None):
    if obs_state_keys is None:
        obs_state_keys = ['qpos']
    print(f'\nData from: {dataset_dir}\n')
    # obtain train test split
    train_ratio = 0.8
    shuffled_indices = np.random.permutation(num_episodes)
    split = max(1, int(train_ratio * num_episodes))
    train_indices = shuffled_indices[:split]
    # 에피소드가 적으면 train 데이터를 val에도 재사용
    val_indices = shuffled_indices[split:] if split < num_episodes else shuffled_indices

    # obtain normalization stats for qpos and action
    norm_stats, skipped_episodes = get_norm_stats(dataset_dir, num_episodes, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, chunk_size=chunk_size)

    # 호환 불가능한 에피소드 제외
    if skipped_episodes:
        valid_mask = ~np.isin(shuffled_indices, skipped_episodes)
        valid_indices = shuffled_indices[valid_mask]
        split = max(1, int(train_ratio * len(valid_indices)))
        train_indices = valid_indices[:split]
        val_indices = valid_indices[split:] if split < len(valid_indices) else valid_indices

    # construct dataset and dataloader. Image augmentation (paper Appendix E:
    # RandomCrop 95% + Rotate ±5° + ColorJitter) is applied only to PI05 training
    # splits — ACT/Diffusion have their own expected input distributions and val
    # needs deterministic data for honest loss comparison.
    train_augment = (policy_type == 'PI05')
    # wrist_sensor_ids: openpi skips spatial aug (crop+rotate) on wrist cams since
    # they encode end-effector geometry — random crop destroys gripper↔scene spatial
    # prior and breaks pretrained feature extraction. EasyTrainer's previous
    # augmentation infrastructure (added in audit-doc bug #24) was never wired through
    # load_data — that's now fixed by passing it from train.py via the train config.
    train_dataset = EpisodicDataset(train_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, augment=train_augment, wrist_sensor_ids=wrist_sensor_ids)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, augment=False, wrist_sensor_ids=wrist_sensor_ids)
    # Keep workers alive across epochs so EpisodicDataset._ep_cache (and OS page
    # cache backing the mmap'd frame .npy files) survives between epochs.
    loader_kwargs = dict(
        shuffle=True,
        pin_memory=True,
        num_workers=num_workers,
    )
    if num_workers > 0:
        loader_kwargs["persistent_workers"] = True
        loader_kwargs["prefetch_factor"] = 4
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size_train, **loader_kwargs)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size_val, **loader_kwargs)

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


_pi05_tokenizer = None

def _get_pi05_tokenizer():
    """Lazy-load PaliGemma tokenizer for PI05."""
    global _pi05_tokenizer
    if _pi05_tokenizer is None:
        from transformers import AutoTokenizer
        _pi05_tokenizer = AutoTokenizer.from_pretrained("google/paligemma-3b-pt-224")
    return _pi05_tokenizer


def prepare_pi05_language_tokens(batch, config, norm_stats=None):
    """Convert language_instruction + state → tokenized language tokens for PI05.

    Replicates the logic of Pi05PrepareStateTokenizerProcessorStep + TokenizerProcessorStep.
    """
    tokenizer = _get_pi05_tokenizer()
    max_length = getattr(config, 'tokenizer_max_length', 200)
    max_state_dim = getattr(config, 'max_state_dim', 32)

    # Get language instruction (string or list of strings)
    lang = batch.get('language_instruction', '')
    if isinstance(lang, str):
        lang = [lang]

    # Get state for discretization
    state = batch.get('observation.state')
    prompts = []
    for i, task_text in enumerate(lang):
        cleaned = task_text.strip().replace("_", " ").replace("\n", " ") if task_text else ""
        state_str = ""
        if state is not None:
            s = state[i] if state.dim() > 1 else state
            s_np = s.cpu().numpy().flatten()
            # Normalize to [-1, 1] using min-max if norm_stats available
            if norm_stats and 'observation.state' in norm_stats:
                s_min = norm_stats['observation.state']['min']
                s_max = norm_stats['observation.state']['max']
                s_range = s_max - s_min
                s_range[s_range < 1e-6] = 1.0
                s_np = 2.0 * (s_np - s_min) / s_range - 1.0
                s_np = np.clip(s_np, -1.0, 1.0)
            # Pad to max_state_dim
            if len(s_np) < max_state_dim:
                s_np = np.concatenate([s_np, np.zeros(max_state_dim - len(s_np))])
            # Discretize into 256 bins
            bins = np.linspace(-1, 1, 257)[:-1]
            discretized = np.digitize(s_np, bins) - 1
            state_str = " ".join(map(str, discretized.astype(int)))
        prompt = f"Task: {cleaned}, State: {state_str};\nAction: "
        prompts.append(prompt)

    tokenized = tokenizer(
        prompts,
        max_length=max_length,
        truncation=True,
        padding="max_length",
        return_tensors="pt",
    )

    device = state.device if state is not None else torch.device('cuda')
    batch['observation.language.tokens'] = tokenized['input_ids'].to(device)
    batch['observation.language.attention_mask'] = tokenized['attention_mask'].to(dtype=torch.bool, device=device)

    return batch


def forward_pass(batch, policy, norm_stats=None, preprocessor=None):
    data = {k: (v.cuda() if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
    # PI05 preprocessor pipeline expects complementary_data['task']. EasyTrainer
    # datasets store the same field under 'language_instruction', so bridge the
    # two so Pi05PrepareStateTokenizerProcessorStep can find the prompt.
    if 'task' not in data:
        if 'language_instruction' in data:
            data['task'] = data['language_instruction']
        elif hasattr(policy, 'config') and hasattr(policy.config, 'tokenizer_max_length'):
            # Determine batch size to build a placeholder list of the right length.
            bsz = None
            for v in data.values():
                if isinstance(v, torch.Tensor) and v.dim() >= 1:
                    bsz = v.shape[0]
                    break
            data['task'] = [''] * (bsz or 1)
    # Apply input preprocessor (Normalize state/action/images per cfg.normalization_mapping)
    # so the model trains on normalized targets and the gradient is balanced across joints.
    # The PI05 preprocessor pipeline includes Pi05PrepareStateTokenizerProcessorStep +
    # TokenizerProcessorStep which write `observation.language.tokens` correctly using
    # QUANTILE-normalized state and native state_dim (audit-doc bug #23 fix).
    if preprocessor is not None:
        data = preprocessor(data)
    elif hasattr(policy, 'config') and hasattr(policy.config, 'tokenizer_max_length'):
        # Fallback for the rare case where preprocessor failed to load (e.g. corrupted
        # checkpoint json). Use the legacy `prepare_pi05_language_tokens` to produce
        # *some* language tokens — the result is divergent from training (min-max norm,
        # padded state) and the model will perform poorly, but at least it won't crash.
        # Normal training/inference always has preprocessor and skips this branch.
        data = prepare_pi05_language_tokens(data, policy.config, norm_stats=norm_stats)
    return policy.forward(data)


def make_easytrainer_processors(policy_type, cfg, dataset_stats=None, pretrained_path=None):
    """Per-policy dispatch for building / loading the LeRobot Normalize+Unnormalize pipeline.

    Bypasses ``lerobot.policies.factory.make_pre_post_processors`` because that module
    eagerly imports ``lerobot.envs.configs`` → robots → motors, none of which we install
    in the EasyTrainer container. We just call the per-policy ``make_*_pre_post_processors``
    builders directly so the dependency surface stays minimal.

    Args:
        policy_type: 'ACT' | 'Diffusion' | 'PI05'
        cfg: the policy config (ACTConfig / DiffusionConfig / PI05Config)
        dataset_stats: numpy stats dict from get_norm_stats() — used when building from scratch.
        pretrained_path: when set, load saved processor json/safetensors from this dir
            instead of building. ``dataset_stats`` is ignored in this branch.

    Returns:
        (preprocessor, postprocessor) PolicyProcessorPipeline tuple, or (None, None)
        on a load failure when pretrained_path is set (caller falls back to raw I/O).
    """
    from lerobot.processor.pipeline import PolicyProcessorPipeline
    from lerobot.processor.converters import (
        batch_to_transition,
        transition_to_batch,
        policy_action_to_transition,
        transition_to_policy_action,
    )
    from lerobot.utils.constants import (
        POLICY_PREPROCESSOR_DEFAULT_NAME,
        POLICY_POSTPROCESSOR_DEFAULT_NAME,
    )

    if pretrained_path is not None:
        # Force-import the policy-specific processor modules before deserializing the
        # saved pipeline JSON. ProcessorStepRegistry is populated by @register decorators
        # at import time — without this, loading a PI05 checkpoint fails with
        # "Processor step 'pi05_prepare_state_tokenizer_processor_step' not found in
        # registry" and the code silently falls back to raw I/O (no normalization).
        # That skipped normalization is what makes the robot behave erratically at
        # inference even though training looked fine.
        if policy_type == 'PI05':
            import lerobot.policies.pi05.processor_pi05  # noqa: F401
        elif policy_type == 'ACT':
            import lerobot.policies.act.processor_act  # noqa: F401
        elif policy_type == 'Diffusion':
            import lerobot.policies.diffusion.processor_diffusion  # noqa: F401
        try:
            preprocessor = PolicyProcessorPipeline.from_pretrained(
                pretrained_model_name_or_path=pretrained_path,
                config_filename=f"{POLICY_PREPROCESSOR_DEFAULT_NAME}.json",
                to_transition=batch_to_transition,
                to_output=transition_to_batch,
            )
            postprocessor = PolicyProcessorPipeline.from_pretrained(
                pretrained_model_name_or_path=pretrained_path,
                config_filename=f"{POLICY_POSTPROCESSOR_DEFAULT_NAME}.json",
                to_transition=policy_action_to_transition,
                to_output=transition_to_policy_action,
            )
            return preprocessor, postprocessor
        except Exception as e:
            print(f'[WARN] make_easytrainer_processors: failed to load processors from '
                  f'{pretrained_path} ({type(e).__name__}: {e}). Returning (None, None).')
            return None, None

    # Build from scratch using per-policy factories
    if policy_type == 'ACT':
        from lerobot.policies.act.processor_act import make_act_pre_post_processors
        return make_act_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    if policy_type == 'Diffusion':
        from lerobot.policies.diffusion.processor_diffusion import make_diffusion_pre_post_processors
        return make_diffusion_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    if policy_type == 'PI05':
        from lerobot.policies.pi05.processor_pi05 import make_pi05_pre_post_processors
        return make_pi05_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    raise ValueError(f'make_easytrainer_processors: unsupported policy_type {policy_type!r}')


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
    

VISION_BACKBONE_MAP = {
    'dinov2': 'facebook/dinov2-base',
    'dinov3': 'facebook/dinov3-vitb16-pretrain-lvd1689m',
}