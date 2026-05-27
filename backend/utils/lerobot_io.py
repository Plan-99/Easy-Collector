"""
LeRobot dataset I/O helper module.

Provides functions to create, append, read, list, and delete episodes
in the LeRobot v2.1 dataset format (parquet + MP4 videos + JSON metadata).

This is a standalone module that does NOT depend on the full LeRobotDataset class.
It directly reads/writes the file structure:

    dataset_dir/
    ├── data/
    │   └── chunk-000/
    │       ├── episode_000000.parquet
    │       └── ...
    ├── videos/
    │   └── chunk-000/
    │       └── observation.images.sensor_{id}/
    │           ├── episode_000000.mp4
    │           └── ...
    └── meta/
        ├── info.json
        ├── episodes.jsonl
        ├── tasks.jsonl
        └── episodes_stats.jsonl
"""

import json
import os
import re
import shutil

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import datasets as hf_datasets
from PIL import Image

import sys as _sys
_sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'lerobot', 'src'))
from lerobot.datasets.feature_utils import get_hf_features_from_features, create_empty_dataset_info
from lerobot.datasets.io_utils import embed_images
from lerobot.datasets.utils import DEFAULT_IMAGE_PATH, DEFAULT_FEATURES
LEROBOT_CODEBASE_VERSION = "v2.1"

try:
    import av
except ImportError:
    av = None

# ─── Constants ───────────────────────────────────────────────────────────────

DEFAULT_CHUNK_SIZE = 1000
CODEBASE_VERSION = "v2.1"

PARQUET_PATH_TEMPLATE = "data/chunk-{chunk:03d}/episode_{ep:06d}.parquet"
IMAGE_PATH_TEMPLATE = "images/sensor_{sid}/episode_{ep:06d}/frame_{frame:06d}.png"

INFO_PATH = "meta/info.json"
EPISODES_PATH = "meta/episodes.jsonl"
TASKS_PATH = "meta/tasks.jsonl"
EPISODES_STATS_PATH = "meta/episodes_stats.jsonl"


# ─── Low-level JSON/JSONL helpers ────────────────────────────────────────────

def _write_json(data, fpath):
    os.makedirs(os.path.dirname(fpath), exist_ok=True)
    with open(fpath, "w") as f:
        json.dump(data, f, indent=2, default=_json_default)


def _read_json(fpath):
    with open(fpath) as f:
        return json.load(f)


def _append_jsonl(data, fpath):
    os.makedirs(os.path.dirname(fpath), exist_ok=True)
    with open(fpath, "a") as f:
        f.write(json.dumps(data, default=_json_default) + "\n")


def _read_jsonl(fpath):
    if not os.path.exists(fpath):
        return []
    with open(fpath) as f:
        return [json.loads(line) for line in f if line.strip()]


def _write_jsonl(data_list, fpath):
    os.makedirs(os.path.dirname(fpath), exist_ok=True)
    with open(fpath, "w") as f:
        for item in data_list:
            f.write(json.dumps(item, default=_json_default) + "\n")


def _json_default(obj):
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    raise TypeError(f"Object of type {type(obj)} is not JSON serializable")


# ─── Dataset creation ────────────────────────────────────────────────────────

def create_dataset(dataset_dir, agents, sensors, task, fps=20, action_key="joint"):
    """Create an empty LeRobot dataset directory structure.

    Args:
        dataset_dir: Path to the dataset root.
        agents: List of agent dicts (sorted by id).
        sensors: List of sensor dicts.
        task: Task dict containing sensor_img_size etc.
        fps: Recording frequency.
        action_key: 'qaction' or 'ee_delta_action'.

    Returns:
        The features dict that was written to info.json.
    """
    features = build_features(agents, sensors, task, action_key)
    # Merge with DEFAULT_FEATURES (timestamp, frame_index, episode_index, index, task_index)
    features = {**features, **DEFAULT_FEATURES}

    info = create_empty_dataset_info(
        LEROBOT_CODEBASE_VERSION, fps, features, use_videos=True, robot_type="easytrainer"
    )
    # Add custom field
    info["action_key"] = action_key
    # Agent metadata — observation.qpos 의 global 인덱스 중 tool joint 위치.
    # 학습/추론 측이 arm/tool 분리에 직접 사용. 기존 휴리스틱 (eepos.names 파싱) 보다
    # 정확 — tool_inner 케이스 (그리퍼 joint 가 arm URDF 안) 도 정확히 잡힘.
    info["tool_qpos_indices"] = _compute_tool_qpos_indices(agents)

    os.makedirs(dataset_dir, exist_ok=True)
    _write_json(info, os.path.join(dataset_dir, INFO_PATH))

    # Create empty jsonl files
    for path in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
        fpath = os.path.join(dataset_dir, path)
        os.makedirs(os.path.dirname(fpath), exist_ok=True)
        if not os.path.exists(fpath):
            open(fpath, "w").close()

    return features


def _compute_tool_qpos_indices(agents):
    """Robot DB 의 role / tool_inner / tool_index 정보로 global qpos 의 tool joint
    인덱스 계산. observation.qpos 컬럼은 agents_sorted (id 순) 의 joint_names 를
    순서대로 concat 한 것이므로, 같은 순회로 누적 offset 계산.

    판별 규칙 (agent 별로 어느 joint 가 tool 인지):
      - role == 'tool':                전체 joint 가 tool (그리퍼 같은 분리형 agent)
      - tool_inner=True + tool_index:  arm 안의 tool_index 만 tool (URDF 임베디드)
      - 외:                            tool joint 없음 (순수 arm)
    """
    agents_sorted = sorted(agents, key=lambda a: a['id'] if isinstance(a, dict) else a.id)
    tool_indices = []
    offset = 0
    for agent in agents_sorted:
        a = agent if isinstance(agent, dict) else agent.__dict__
        joint_names = a.get('joint_names', []) or []
        role = a.get('role') if isinstance(agent, dict) else getattr(agent, 'role', None)
        tool_inner = a.get('tool_inner', False) if isinstance(agent, dict) else getattr(agent, 'tool_inner', False)
        tool_index_local = a.get('tool_index') or [] if isinstance(agent, dict) else (getattr(agent, 'tool_index', None) or [])
        n = len(joint_names)
        if role == 'tool':
            # 전체 joint 가 tool
            tool_indices.extend(range(offset, offset + n))
        elif tool_inner and tool_index_local:
            for ti in tool_index_local:
                if 0 <= int(ti) < n:
                    tool_indices.append(offset + int(ti))
        offset += n
    return tool_indices


def build_features(agents, sensors, task, action_key="joint"):
    """Build the LeRobot features dict from agent/sensor configuration.

    The observation.state and action are concatenated across all agents (sorted by id).
    Additional fields: qvel, qeffort, eepos, ee_delta, action.ee_delta, succeed.
    """
    agents_sorted = sorted(agents, key=lambda a: a['id'] if isinstance(a, dict) else a.id)

    # Build motor names and dimensions
    state_names = []
    action_names = []
    state_dim = 0
    action_dim = 0

    # Build ee names and dimensions
    ee_names_list = []  # list of (robot_id, ee_name, per_ee_dim)
    ee_dim = 0

    for agent in agents_sorted:
        a = agent if isinstance(agent, dict) else agent.__dict__
        a_id = a.get('id', a.get('agent_id'))
        joint_names = a.get('joint_names', [])
        prefix_names = [f"robot_{a_id}_{jn}" for jn in joint_names]
        state_names.extend(prefix_names)
        action_names.extend(prefix_names)
        state_dim += len(joint_names)
        action_dim += len(joint_names)

        # EE features: only if agent has ik_solver.
        # observation.eepos 는 **순수 EE pose (x,y,z,rx,ry,rz) 6 dim** 만 담는다.
        # tool 은 tool_inner 든 separate tool agent 든 전부 qpos 에 있으며,
        # 학습/추론 시 tool_qpos_indices 로 붙인다 — eepos 규격을 6×ee 로 통일
        # (옛 tool_inner +1 / separate tool tail-append 분기를 모두 제거).
        has_ik = a.get('ik_solver') is not None if isinstance(agent, dict) else getattr(agent, 'ik_solver', None) is not None
        if has_ik:
            agent_ee_names = a.get('ee_names', []) if isinstance(agent, dict) else getattr(agent, 'ee_names', [])
            for ee_name in sorted(agent_ee_names):
                components = ["x", "y", "z", "rx", "ry", "rz"]
                ee_names_list.extend([f"robot_{a_id}_{ee_name}_{c}" for c in components])
                ee_dim += 6

    features = {
        # observation.qpos: 절대 joint position. 학습/추론 모두 _build_obs_state 헬퍼가
        # 이 컬럼을 우선으로 읽음 (옛 dataset 호환으로 observation.state fallback 도 지원).
        "observation.qpos": {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": state_names,
        },
        # action.joint: 절대 joint position target (qaction). 항상 저장. ee_delta /
        # relative_ee_pos action 은 학습 시 _get_action_data 가 observation.eepos
        # 차분으로 derive. 따라서 dataset 에는 action 의 source 한 가지 (joint) 만
        # 필요하고 'action' 이라는 redundant alias 컬럼은 더 이상 만들지 않음.
        "action.joint": {
            "dtype": "float32",
            "shape": (action_dim,),
            "names": action_names,
        },
        "observation.qvel": {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": state_names,
        },
        "observation.qeffort": {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": state_names,
        },
        "succeed": {
            "dtype": "float32",
            "shape": (1,),
            "names": None,
        },
    }

    # EE features (only if any agent has ik_solver). observation.eepos 만 저장하고
    # ee_delta / relative_ee_pos action 은 학습/추론 시 eepos 시간 차분 (eepos[t+1] -
    # eepos[t]) 으로 derive — 별도 컬럼 필요 없음. observation.ee_delta 도 같은
    # 이유로 미저장.
    if ee_dim > 0:
        features["observation.eepos"] = {
            "dtype": "float32",
            "shape": (ee_dim,),
            "names": ee_names_list,
        }

    # Video features (stored as MP4, not in parquet).
    # Multi-view: ``sensors`` 는 task['sensor_ids'] 순서대로 들어오고 같은
    # 물리 sensor_id 가 여러 번 등장할 수 있다. 각 view 는 자기 view_key 로
    # 별도 feature entry 를 갖는다 (예: observation.images.sensor_5,
    # observation.images.sensor_5_2). single-view 워크스페이스는 vkey 가
    # str(sensor_id) 와 같아 기존 dataset 와 bit-for-bit 동일.
    # 정렬: sensor_id stable sort 로 같은 물리 sensor 의 view 들이 인접 →
    # features dict 의 entry 순서, monitoring viewport 순서, 추론
    # image_features 순서가 모두 (sensor_id, occurrence) 오름차순으로 통일.
    from .sensor_view import view_key as _view_key
    _sorted_sensors = sorted(
        sensors,
        key=lambda _s: int((_s if isinstance(_s, dict) else _s.__dict__).get('id')),
    )
    _seen_occ: dict = {}
    sensor_img_size = task.get('sensor_img_size') or {}
    for sensor in _sorted_sensors:
        s = sensor if isinstance(sensor, dict) else sensor.__dict__
        s_id = int(s.get('id'))
        occ = _seen_occ.get(s_id, 0)
        _seen_occ[s_id] = occ + 1
        vkey = _view_key(s_id, occ)
        # Get image resolution from task config. Falls back to a per-sensor default
        # when the task config is missing or partially None — this avoids the
        # "'NoneType' object is not subscriptable" crash when task.sensor_img_size
        # is None at the dict level (e.g., new tasks where the user hasn't set it).
        # Lookup priority: view_key (new schema) → sensor_id str (old schema fallback,
        # only meaningful for first occurrence where vkey == str(s_id) anyway).
        img_size = None
        if isinstance(sensor_img_size, dict):
            img_size = sensor_img_size.get(vkey) or sensor_img_size.get(str(s_id))
        if not img_size:
            # Try sensor's own settings (sensor.settings.resolution = [w, h])
            sensor_settings = s.get('settings') or {}
            if isinstance(sensor_settings, str):
                try:
                    import json as _json
                    sensor_settings = _json.loads(sensor_settings)
                except Exception:
                    sensor_settings = {}
            img_size = sensor_settings.get('resolution') if isinstance(sensor_settings, dict) else None
        if not img_size:
            img_size = [640, 480]  # final fallback
        h, w = int(img_size[1]), int(img_size[0])  # [width, height] -> (H, W)
        features[f"observation.images.sensor_{vkey}"] = {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }

    return features


# ─── Episode append ──────────────────────────────────────────────────────────

def append_episode(dataset_dir, timesteps, agents, sensors, task,
                   language_instruction="", action_key="joint",
                   succeed_flags=None, fetch_image_fn=None, fps=20):
    """Append an episode to a LeRobot dataset.

    Saves frames as temporary PNGs, encodes to MP4 video, then cleans up PNGs.
    Stores all observation fields: qpos, qvel, qeffort, eepos, ee_delta, ee_delta_action, succeed.

    Args:
        dataset_dir: Path to the dataset root.
        timesteps: List of timestep objects (from Env).
        agents: List of agent objects (sorted by id).
        sensors: List of sensor dicts.
        task: Task dict.
        language_instruction: Language instruction string.
        action_key: 'qaction' or 'ee_delta_action'.
        succeed_flags: Optional list of succeed flags per timestep.
        fetch_image_fn: Optional function(img, config) -> processed_img.
        fps: Recording frequency (used only for initial dataset creation).

    Returns:
        episode_index of the saved episode.
    """
    info_path = os.path.join(dataset_dir, INFO_PATH)
    if not os.path.exists(info_path):
        create_dataset(dataset_dir, agents, sensors, task, fps=fps, action_key=action_key)

    info = _read_json(info_path)
    episode_index = info["total_episodes"]
    chunk = episode_index // info["chunks_size"]
    num_frames = len(timesteps)
    features = info.get("features", {})

    agents_sorted = sorted(agents, key=lambda a: a.id)

    # ── Build per-frame data ─────────────────────────────────────────────
    states = []
    actions_joint = []  # action.joint = qaction (always)
    qvels = []
    qefforts = []
    eepos_list = []
    succeed_list = []
    frame_indices = []
    timestamps_list = []
    fps = info["fps"]

    # Determine ee dimension from features (for zero-fill when agent has no ik_solver)
    ee_dim = features.get("observation.eepos", {}).get("shape", [0])
    ee_dim = ee_dim[0] if isinstance(ee_dim, (list, tuple)) else ee_dim

    for t in range(num_frames):
        ts = timesteps[t]
        frame_indices.append(t)
        timestamps_list.append(t / fps)

        qpos_parts = []
        qvel_parts = []
        qeffort_parts = []
        eepos_parts = []

        for agent in agents_sorted:
            robot_state = ts.observation['robot_states'][agent.id]

            # observation.state (qpos)
            qpos = robot_state.get('qpos')
            if qpos is not None:
                qpos_parts.append(np.array(qpos, dtype=np.float32))

            # observation.qvel
            qvel = robot_state.get('qvel')
            if qvel is not None:
                qvel_parts.append(np.array(qvel, dtype=np.float32))
            elif qpos is not None:
                qvel_parts.append(np.zeros(len(qpos), dtype=np.float32))

            # observation.qeffort
            qeffort = robot_state.get('qeffort')
            if qeffort is not None:
                qeffort_parts.append(np.array(qeffort, dtype=np.float32))
            elif qpos is not None:
                qeffort_parts.append(np.zeros(len(qpos), dtype=np.float32))

            # observation.eepos — flatten dict {ee_name: [x,y,z,rx,ry,rz(,tool)]}.
            # 순수 EE pose 6 dim 만 저장 — agent 가 tool_inner tool 을 7번째로
            # 넣어줘도 [:6] 으로 잘라낸다. tool 은 qpos 에만 존재.
            eepos = robot_state.get('eepos')
            if eepos is not None and isinstance(eepos, dict):
                for ee_name in sorted(eepos.keys()):
                    eepos_parts.append(np.array(eepos[ee_name][:6], dtype=np.float32))

            # action.ee_delta 는 더 이상 robot_state['ee_delta_action'] 을 직접 쓰지 않고,
            # 루프 끝난 뒤 eepos 시간 차분으로 derive (모든 tele-type 통일).

        # action.joint: 절대 joint position target (qaction). 항상 모든 agent
        # 에 대해 채움. action_key 와 무관.
        action_joint_parts = []
        for agent in agents_sorted:
            robot_state = ts.observation['robot_states'][agent.id]
            qact = robot_state.get('qaction')
            if qact is not None:
                action_joint_parts.append(np.array(qact, dtype=np.float32))

        # action 컬럼은 더 이상 만들지 않음. action.joint 가 single source of truth.
        # ee_delta / relative_ee_pos action 은 학습 시 _get_action_data 가
        # observation.eepos 시간 차분으로 derive.

        states.append(np.concatenate(qpos_parts) if qpos_parts else np.array([], dtype=np.float32))
        actions_joint.append(np.concatenate(action_joint_parts) if action_joint_parts else np.array([], dtype=np.float32))
        qvels.append(np.concatenate(qvel_parts) if qvel_parts else np.array([], dtype=np.float32))
        qefforts.append(np.concatenate(qeffort_parts) if qeffort_parts else np.array([], dtype=np.float32))

        if eepos_parts:
            eepos_list.append(np.concatenate(eepos_parts))
        elif ee_dim > 0:
            eepos_list.append(np.zeros(ee_dim, dtype=np.float32))
        # ee_delta_action_parts 는 더 이상 사용 안 함 — 아래에서 eepos diff 로 derive.

        # succeed flag
        if succeed_flags is not None and t < len(succeed_flags):
            succeed_list.append(float(succeed_flags[t]))
        else:
            succeed_list.append(0.0)

    states = np.array(states, dtype=np.float32)
    actions_joint_arr = np.array(actions_joint, dtype=np.float32)
    qvels_arr = np.array(qvels, dtype=np.float32)
    qefforts_arr = np.array(qefforts, dtype=np.float32)
    succeed_arr = np.array(succeed_list, dtype=np.float32)

    # action.ee_delta / relative_ee_pos 는 더 이상 dataset 컬럼으로 저장하지 않음.
    # 학습/추론 시 observation.eepos 시간 차분 (eepos[t+1] - eepos[t]) 으로 derive.
    # policies/utils.py 의 _get_action_data 가 action_key 별 분기 처리.

    # ── Save frames as temporary PNGs, then encode to MP4 ────────────────
    num_videos = 0
    # SAM3 tracker lifecycle: initialize once per sensor on the first frame,
    # propagate masks for the rest of the episode, then tear down. No-op when
    # the extension isn't installed.
    from .sam3_helper import start_episode as _sam3_start, end_episode as _sam3_end
    from .sensor_view import view_key as _view_key
    # Multi-view: 같은 물리 sensor 가 여러 view 로 등장하면 각 view 마다
    # 별도 비디오를 저장한다. raw frame (ts.observation['images']['sensor_5'])
    # 은 공유하고, view 별 crop/rotate/resize/sam3 만 다르게 적용.
    # Sensor_id stable sort → 같은 sensor 의 view 들 인접 + build_features 와
    # 동일 ordering (features 와 mp4 디렉터리 순서가 정합).
    _sorted_sensors_ep = sorted(
        sensors,
        key=lambda _s: int(_s['id']),
    )
    _view_occ: dict = {}
    for sensor in _sorted_sensors_ep:
        s_id_int = int(sensor['id'])
        occ = _view_occ.get(s_id_int, 0)
        _view_occ[s_id_int] = occ + 1
        vkey = _view_key(s_id_int, occ)
        s_id = str(s_id_int)  # physical id (for raw frame lookup)
        feature_key = f"observation.images.sensor_{vkey}"

        # Save frames as PNGs (encode_video_frames expects frame_NNNNNN.png)
        imgs_dir = os.path.join(
            dataset_dir, "images", feature_key, f"episode_{episode_index:06d}"
        )
        os.makedirs(imgs_dir, exist_ok=True)

        # Per-view config lookup: new schema 는 vkey 로, 기존 single-view 는
        # vkey == s_id 라 같은 dict 접근. Multi-view 환경에서 vkey 가
        # task['sensor_sam3'] 등에 없으면 (예: 마이그레이션 도중 부분 설정)
        # physical s_id 로 fallback — view 들끼리 같은 sam3/rotate/crop 공유.
        sam3_cfg = (task.get('sensor_sam3') or {}).get(vkey)
        if sam3_cfg is None:
            sam3_cfg = (task.get('sensor_sam3') or {}).get(s_id)
        if num_frames > 0 and sam3_cfg and sam3_cfg.get('enabled'):
            first_img = timesteps[0].observation['images'][f'sensor_{s_id}']
            # SAM3 tracker key 는 view_key 단위 (같은 카메라의 다른 view 가
            # 서로 다른 mask 를 가질 수 있게).
            _sam3_start(vkey, first_img, sam3_cfg)

        try:
            for t in range(num_frames):
                ts = timesteps[t]
                # Raw frame 은 물리 센서 단위로 obs dict 에 저장됨.
                img = ts.observation['images'][f'sensor_{s_id}']

                if fetch_image_fn is not None:
                    img = fetch_image_fn(img, {
                        'sensor_id': vkey,  # SAM3 tracker key 일치
                        'sam3': sam3_cfg,
                        'resize': (task.get('sensor_img_size') or {}).get(vkey)
                                  or (task.get('sensor_img_size') or {}).get(s_id),
                        'cropped_area': (task.get('sensor_cropped_area') or {}).get(vkey)
                                  or (task.get('sensor_cropped_area') or {}).get(s_id, {}),
                        'rotate': (task.get('sensor_rotate') or {}).get(vkey)
                                  or (task.get('sensor_rotate') or {}).get(s_id, 0),
                    })

                if isinstance(img, np.ndarray):
                    # yuv420p / libx264 는 width/height 가 *짝수* 여야 함. crop
                    # 영역이나 resize 가 홀수 dim 을 만들면 avcodec_open2 실패
                    # ("Generic error in an external library"). multi-view 환경에서
                    # view 별로 다른 crop 을 잡으면 한쪽이 홀수가 되는 케이스가 흔함.
                    # 마지막 row/col 1px 만 잘라서 짝수로 normalize — view 의 의미
                    # 가 깨질 정도가 아니라 안전.
                    if img.ndim == 3 and img.shape[2] == 3:
                        h, w = img.shape[:2]
                        if h % 2 or w % 2:
                            img = img[: h - (h % 2), : w - (w % 2), :]
                        # OpenCV uses BGR, convert to RGB for PIL/video encoding
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img_pil = Image.fromarray(img)
                else:
                    img_pil = img

                # NOTE: lerobot's encode_video_frames glob is "frame-NNNNNN.png" (hyphen).
                # If we use "frame_NNNNNN.png" (underscore) the encoder finds zero matches,
                # raises FileNotFoundError, the except branch warns silently and keeps the
                # PNGs without writing an mp4. The training dataset loader then falls back
                # to all-zero images and the model trains on a black screen.
                frame_path = os.path.join(imgs_dir, f"frame-{t:06d}.png")
                img_pil.save(frame_path)
        finally:
            if sam3_cfg and sam3_cfg.get('enabled'):
                _sam3_end(vkey)

        # Encode PNGs to MP4
        video_path = os.path.join(
            dataset_dir, "videos", f"chunk-{chunk:03d}", feature_key,
            f"episode_{episode_index:06d}.mp4"
        )
        # Hard fail on encoding errors. Previously this swallowed the exception and
        # kept the PNGs, which made the dataset loader silently fall back to
        # all-zero frames during training (the model never saw any image).
        from lerobot.datasets.video_utils import encode_video_frames
        encode_video_frames(
            imgs_dir=imgs_dir,
            video_path=video_path,
            fps=fps,
            vcodec="h264",
            pix_fmt="yuv420p",
            g=2,
            crf=30,
            overwrite=True,
        )
        if not os.path.exists(video_path) or os.path.getsize(video_path) == 0:
            raise RuntimeError(
                f"encode_video_frames produced no output for {feature_key} "
                f"(imgs_dir={imgs_dir}, video_path={video_path})"
            )
        num_videos += 1

        # Clean up temporary PNGs
        shutil.rmtree(imgs_dir)

    # Clean up empty images directory
    images_root = os.path.join(dataset_dir, "images")
    if os.path.exists(images_root):
        try:
            # Remove only if empty
            os.removedirs(images_root)
        except OSError:
            pass

    # ── Build task index ─────────────────────────────────────────────────
    tasks = _read_jsonl(os.path.join(dataset_dir, TASKS_PATH))
    task_str = language_instruction or ""
    task_index = None
    for t_entry in tasks:
        if t_entry.get("task") == task_str:
            task_index = t_entry["task_index"]
            break
    if task_index is None:
        task_index = len(tasks)
        _append_jsonl({"task_index": task_index, "task": task_str},
                      os.path.join(dataset_dir, TASKS_PATH))

    # ── Build parquet (no image data — videos are separate files) ────────
    global_start = info["total_frames"]
    episode_dict = {
        "index": np.arange(global_start, global_start + num_frames, dtype=np.int64),
        "episode_index": np.full(num_frames, episode_index, dtype=np.int64),
        "frame_index": np.array(frame_indices, dtype=np.int64),
        "timestamp": np.array(timestamps_list, dtype=np.float32),
        "task_index": np.full(num_frames, task_index, dtype=np.int64),
    }
    # action 은 더 이상 컬럼으로 저장 안 함 (action.joint 가 single source of truth).

    # Add new fields only if features define them (backward compat with old datasets).
    # observation.qpos 가 새 표준 — 옛 dataset 의 observation.state 와 동일 의미.
    if "observation.qpos" in features:
        episode_dict["observation.qpos"] = np.stack(states)
    elif "observation.state" in features:
        # 옛 schema 호환 (이전 코드로 만든 dataset 에 append 할 때).
        episode_dict["observation.state"] = np.stack(states)
    if "action.joint" in features and len(actions_joint_arr) > 0:
        episode_dict["action.joint"] = np.stack(actions_joint_arr)
    if "observation.qvel" in features:
        episode_dict["observation.qvel"] = np.stack(qvels_arr)
    if "observation.qeffort" in features:
        episode_dict["observation.qeffort"] = np.stack(qefforts_arr)
    if "succeed" in features:
        episode_dict["succeed"] = succeed_arr
    if "observation.eepos" in features and eepos_list:
        episode_dict["observation.eepos"] = np.stack(eepos_list)
    # action.ee_delta / observation.ee_delta 컬럼은 더 이상 저장 안 함. ee_delta /
    # relative_ee_pos action 은 학습/추론 시 _get_action_data 가 observation.eepos
    # 시간 차분으로 derive 한다 (모든 tele-type 통일).

    # Build parquet directly with pyarrow (avoids HF datasets encode_nested_example
    # compatibility issues with numpy scalars across different HF datasets versions)
    parquet_features = {k: v for k, v in features.items() if v.get("dtype") not in ("video", "image")}
    pa_arrays = {}
    for col_key in parquet_features:
        if col_key not in episode_dict:
            continue
        val = episode_dict[col_key]
        if isinstance(val, np.ndarray):
            pa_arrays[col_key] = pa.array(val.tolist())
        elif isinstance(val, list):
            pa_arrays[col_key] = pa.array(val)
        else:
            pa_arrays[col_key] = pa.array(val)
    table = pa.table(pa_arrays)

    parquet_path = os.path.join(
        dataset_dir,
        PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_index),
    )
    os.makedirs(os.path.dirname(parquet_path), exist_ok=True)
    pq.write_table(table, parquet_path)

    # ── Compute episode stats ────────────────────────────────────────────
    ep_stats = {}
    stat_pairs = [
        ("observation.qpos", states),
        ("action.joint", actions_joint_arr),
        ("observation.qvel", qvels_arr),
        ("observation.qeffort", qefforts_arr),
    ]
    if eepos_list:
        stat_pairs.append(("observation.eepos", np.stack(eepos_list)))

    for key, arr in stat_pairs:
        if arr.size > 0:
            ep_stats[key] = {
                "min": np.min(arr, axis=0).tolist(),
                "max": np.max(arr, axis=0).tolist(),
                "mean": np.mean(arr, axis=0).tolist(),
                "std": np.std(arr, axis=0).tolist(),
                "count": int(num_frames),
            }

    # ── Update metadata ──────────────────────────────────────────────────
    info["total_episodes"] = episode_index + 1
    info["total_frames"] = global_start + num_frames
    info["total_chunks"] = chunk + 1
    info["total_tasks"] = task_index + 1
    info["total_videos"] = info.get("total_videos", 0) + num_videos
    info["splits"] = {"train": f"0:{episode_index + 1}"}
    _write_json(info, info_path)

    _append_jsonl(
        {"episode_index": episode_index, "length": num_frames, "tasks": [task_str]},
        os.path.join(dataset_dir, EPISODES_PATH),
    )

    _append_jsonl(
        {"episode_index": episode_index, "stats": ep_stats},
        os.path.join(dataset_dir, EPISODES_STATS_PATH),
    )

    print(f"[LeRobot] Saved episode {episode_index} ({num_frames} frames, {num_videos} videos) to {dataset_dir}")
    return episode_index


# ─── Episode reading ─────────────────────────────────────────────────────────

def read_episode(dataset_dir, episode_index):
    """Read a single episode from a LeRobot dataset.

    Returns:
        dict with keys:
            - images: {sensor_name: list of np.ndarray}
            - states: {robot_name: list of np.ndarray}
            - actions: {robot_name: list of np.ndarray}
            - language_instruction: str
            - num_frames: int
            - action_data: np.ndarray (T, action_dim)
            - state_data: np.ndarray (T, state_dim)
    """
    info = _read_json(os.path.join(dataset_dir, INFO_PATH))
    chunk = episode_index // info["chunks_size"]

    parquet_path = os.path.join(
        dataset_dir,
        PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_index),
    )
    table = pq.read_table(parquet_path)
    df = table.to_pandas()

    num_frames = len(df)

    # 새 schema 는 observation.qpos / action.joint, 옛 schema 는 observation.state /
    # action. 두 가지 모두 지원.
    features = info["features"]
    qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else 'observation.state'
    action_col = 'action.joint' if 'action.joint' in df.columns else 'action'

    state_data = np.array(df[qpos_col].tolist(), dtype=np.float32)
    action_data = np.array(df[action_col].tolist(), dtype=np.float32)

    # Parse robot-specific data from concatenated state/action
    state_names = features.get(qpos_col, {}).get("names") or features.get("observation.state", {}).get("names", [])
    action_names = features.get(action_col, {}).get("names") or features.get("action", {}).get("names", [])

    # Group by robot
    states_by_robot = {}
    actions_by_robot = {}

    # Parse robot names from feature names
    # EasyTrainer format: "robot_{id}_{joint_name}"
    # convert_hdf5 format: "joint_1", "joint_2", ... (no robot prefix)
    current_robot = None
    current_start = 0

    for i, name in enumerate(state_names):
        m = re.match(r'^(robot_\d+)_', name)
        if m:
            robot_name = m.group(1)
        else:
            # No robot_ prefix: treat all as single robot
            robot_name = "robot_0"

        if robot_name != current_robot:
            if current_robot is not None:
                states_by_robot[current_robot] = state_data[:, current_start:i]
                actions_by_robot[current_robot] = action_data[:, current_start:i]
            current_robot = robot_name
            current_start = i
    if current_robot is not None:
        states_by_robot[current_robot] = state_data[:, current_start:]
        actions_by_robot[current_robot] = action_data[:, current_start:]

    # Read images from parquet (HF embedded) or video files
    images = {}
    image_cols = [col for col in df.columns if col.startswith("observation.images.")]
    if image_cols:
        # Image mode: HF datasets embed images as {'bytes': b'...', 'path': '...'}
        for col in image_cols:
            sensor_name = col.replace("observation.images.", "")
            img_list = []
            for val in df[col]:
                img_list.append(_parse_image_value(val, dataset_dir))
            images[sensor_name] = img_list
    else:
        # Video mode: decode frames from mp4 files
        for key, feat in features.items():
            if not key.startswith("observation.images."):
                continue
            sensor_name = key.replace("observation.images.", "")
            video_path = os.path.join(
                dataset_dir, "videos", f"chunk-{chunk:03d}", key,
                f"episode_{episode_index:06d}.mp4"
            )
            if os.path.exists(video_path):
                images[sensor_name] = _decode_video_frames(video_path)
            else:
                images[sensor_name] = [np.zeros((224, 224, 3), dtype=np.uint8)] * num_frames

    # Get language instruction from tasks
    task_index = int(df["task_index"].iloc[0])
    tasks = _read_jsonl(os.path.join(dataset_dir, TASKS_PATH))
    language_instruction = ""
    for t_entry in tasks:
        if t_entry.get("task_index") == task_index:
            language_instruction = t_entry.get("task", "")
            break

    return {
        "images": images,
        "states": states_by_robot,
        "actions": actions_by_robot,
        "language_instruction": language_instruction,
        "num_frames": num_frames,
        "state_data": state_data,
        "action_data": action_data,
        "succeed": np.array(df["succeed"].tolist(), dtype=np.float32) if "succeed" in df.columns else None,
    }


def read_episode_frame(dataset_dir, episode_index, frame_index):
    """Read a single frame (images only) from a LeRobot dataset. Efficient for replay."""
    info = _read_json(os.path.join(dataset_dir, INFO_PATH))
    features = info["features"]

    images = {}
    for key, feat in features.items():
        if feat.get("dtype") == "image":
            sensor_name = key.replace("observation.images.", "")
            s_id = sensor_name.replace("sensor_", "")
            rel_path = IMAGE_PATH_TEMPLATE.format(sid=s_id, ep=episode_index, frame=frame_index)
            abs_path = os.path.join(dataset_dir, rel_path)
            if os.path.exists(abs_path):
                images[sensor_name] = np.array(Image.open(abs_path))

    return images


# ─── Episode listing ─────────────────────────────────────────────────────────

def list_episodes(dataset_dir):
    """List all episodes in a dataset.

    Returns:
        List of dicts with 'episode_index', 'length', 'tasks'.
    """
    episodes_path = os.path.join(dataset_dir, EPISODES_PATH)
    return _read_jsonl(episodes_path)


def get_episode_count(dataset_dir):
    """Get total number of episodes."""
    info_path = os.path.join(dataset_dir, INFO_PATH)
    if not os.path.exists(info_path):
        return 0
    info = _read_json(info_path)
    return info.get("total_episodes", 0)


# ─── Dataset metadata ────────────────────────────────────────────────────────

def get_dataset_info(dataset_dir):
    """Read dataset info.json."""
    info_path = os.path.join(dataset_dir, INFO_PATH)
    if not os.path.exists(info_path):
        return None
    return _read_json(info_path)


def get_dataset_metadata(dataset_dir):
    """Get sensor/robot names from dataset features."""
    info = get_dataset_info(dataset_dir)
    if info is None:
        return {"sensors": [], "robots": []}

    features = info.get("features", {})
    sensors = []
    robots = set()

    # observation.qpos (신규 표준) 우선 검사, 없으면 observation.state (옛 dataset)
    # 로 fallback. 둘 다 names 형식이 동일: "robot_{id}_{joint_name}" 또는 prefix
    # 없으면 single-robot dataset 으로 간주.
    qpos_feat = features.get("observation.qpos") or features.get("observation.state")

    for key, feat in features.items():
        if key.startswith("observation.images."):
            sensors.append(key.replace("observation.images.", ""))

    if qpos_feat is not None:
        has_robot_prefix = False
        for name in qpos_feat.get("names", []):
            m = re.match(r'^(robot_\d+)_', name)
            if m:
                robots.add(m.group(1))
                has_robot_prefix = True
        # If no robot_ prefix found, treat as single-robot dataset.
        if not has_robot_prefix and qpos_feat.get("names"):
            robots.add("robot_0")

    return {"sensors": sorted(sensors), "robots": sorted(robots)}


def get_norm_stats_from_dataset(dataset_dir):
    """Compute normalization stats from episode stats."""
    stats_entries = _read_jsonl(os.path.join(dataset_dir, EPISODES_STATS_PATH))
    if not stats_entries:
        return None

    aggregated = {}
    for entry in stats_entries:
        ep_stats = entry.get("stats", {})
        for key, s in ep_stats.items():
            count = s["count"]
            if key not in aggregated:
                aggregated[key] = {
                    "min": np.array(s["min"]),
                    "max": np.array(s["max"]),
                    "sum": np.array(s["mean"]) * count,
                    "sum_sq": (np.array(s["std"]) ** 2 + np.array(s["mean"]) ** 2) * count,
                    "count": count,
                }
            else:
                agg = aggregated[key]
                agg["min"] = np.minimum(agg["min"], np.array(s["min"]))
                agg["max"] = np.maximum(agg["max"], np.array(s["max"]))
                agg["sum"] += np.array(s["mean"]) * count
                agg["sum_sq"] += (np.array(s["std"]) ** 2 + np.array(s["mean"]) ** 2) * count
                agg["count"] += count

    result = {}
    for key, agg in aggregated.items():
        mean = agg["sum"] / agg["count"]
        var = agg["sum_sq"] / agg["count"] - mean ** 2
        std = np.sqrt(np.maximum(var, 0))
        result[key] = {
            "min": agg["min"],
            "max": agg["max"],
            "mean": mean,
            "std": np.clip(std, 1e-2, None),
            "count": np.array([agg["count"]]),
        }

    return result


# ─── Episode deletion ────────────────────────────────────────────────────────

def delete_episode(dataset_dir, episode_index):
    """Delete a single episode from the dataset.

    Removes parquet file, image directory, and updates metadata.
    Note: Does NOT re-index remaining episodes.
    """
    info = _read_json(os.path.join(dataset_dir, INFO_PATH))
    chunk = episode_index // info["chunks_size"]

    # Remove parquet
    parquet_path = os.path.join(
        dataset_dir,
        PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_index),
    )
    if os.path.exists(parquet_path):
        os.remove(parquet_path)

    # Remove video files and image directories
    features = info.get("features", {})
    for key, feat in features.items():
        if feat.get("dtype") == "video":
            video_path = os.path.join(
                dataset_dir, "videos", f"chunk-{chunk:03d}", key,
                f"episode_{episode_index:06d}.mp4"
            )
            if os.path.exists(video_path):
                os.remove(video_path)
        elif feat.get("dtype") == "image":
            sensor_name = key.replace("observation.images.", "")
            s_id = sensor_name.replace("sensor_", "")
            img_dir = os.path.join(dataset_dir, "images", f"sensor_{s_id}", f"episode_{episode_index:06d}")
            if os.path.exists(img_dir):
                shutil.rmtree(img_dir)

    # Rebuild episodes.jsonl without the deleted episode
    episodes = _read_jsonl(os.path.join(dataset_dir, EPISODES_PATH))
    deleted_length = 0
    remaining = []
    for ep in episodes:
        if ep["episode_index"] != episode_index:
            remaining.append(ep)
        else:
            deleted_length = ep.get("length", 0)
    _write_jsonl(remaining, os.path.join(dataset_dir, EPISODES_PATH))

    # Rebuild stats
    stats = _read_jsonl(os.path.join(dataset_dir, EPISODES_STATS_PATH))
    remaining_stats = [s for s in stats if s["episode_index"] != episode_index]
    _write_jsonl(remaining_stats, os.path.join(dataset_dir, EPISODES_STATS_PATH))

    # Update info
    info["total_episodes"] = len(remaining)
    info["total_frames"] -= deleted_length
    _write_json(info, os.path.join(dataset_dir, INFO_PATH))


# ─── Dataset-level operations ────────────────────────────────────────────────

def get_episodes_as_file_list(dataset_dir):
    """Return episodes in a format compatible with the old HDF5 file listing.

    Returns list of dicts: [{'name': 'episode_000000', 'index': 0, 'length': 100}, ...]
    """
    episodes = list_episodes(dataset_dir)
    result = []
    for ep in episodes:
        idx = ep["episode_index"]
        result.append({
            "name": f"episode_{idx:06d}",
            "index": idx,
            "length": ep.get("length", 0),
        })
    return result


# ─── Video decoding ──────────────────────────────────────────────────────────

def _parse_image_value(val, dataset_dir=None):
    """Parse an image value from a parquet row.

    Handles multiple formats:
        - dict with 'bytes' key (HF embed_images format): decode bytes to numpy
        - dict with 'path' key only: load from file path
        - str (raw file path): load from file
        - PIL.Image: convert to numpy
        - np.ndarray: return as-is
    """
    if isinstance(val, np.ndarray):
        return val
    if isinstance(val, Image.Image):
        return np.array(val)
    if isinstance(val, dict):
        # HF embedded format: {'bytes': b'...', 'path': '...'}
        if val.get('bytes') is not None:
            import io
            return np.array(Image.open(io.BytesIO(val['bytes'])))
        elif val.get('path'):
            path = val['path']
            if not os.path.isabs(path) and dataset_dir:
                path = os.path.join(dataset_dir, path)
            if os.path.exists(path):
                return np.array(Image.open(path))
    if isinstance(val, str):
        path = val
        if not os.path.isabs(path) and dataset_dir:
            path = os.path.join(dataset_dir, path)
        if os.path.exists(path):
            return np.array(Image.open(path))
    return np.zeros((224, 224, 3), dtype=np.uint8)


def _decode_video_frames(video_path):
    """Decode all frames from an mp4 video file using PyAV.

    Returns list of numpy arrays (RGB, HWC).
    """
    if av is None:
        print("[WARN] PyAV not installed, cannot decode video")
        return []

    frames = []
    try:
        with av.open(str(video_path)) as container:
            stream = container.streams.video[0]
            for frame in container.decode(stream):
                img = frame.to_ndarray(format="rgb24")
                frames.append(img)
    except Exception as e:
        print(f"[ERROR] Failed to decode video {video_path}: {e}")
    return frames


# ─── Episode mutation: language / trim / copy ───────────────────────────────

def _ensure_task_index(dataset_dir, language_instruction):
    """Lookup or append a task entry; returns task_index."""
    tasks_path = os.path.join(dataset_dir, TASKS_PATH)
    tasks = _read_jsonl(tasks_path)
    task_str = language_instruction or ""
    for t_entry in tasks:
        if t_entry.get("task") == task_str:
            return t_entry["task_index"]
    new_index = len(tasks)
    _append_jsonl({"task_index": new_index, "task": task_str}, tasks_path)
    info_path = os.path.join(dataset_dir, INFO_PATH)
    info = _read_json(info_path)
    info["total_tasks"] = max(info.get("total_tasks", 0), new_index + 1)
    _write_json(info, info_path)
    return new_index


def _video_path(dataset_dir, feature_key, chunk, episode_index):
    return os.path.join(
        dataset_dir, "videos", f"chunk-{chunk:03d}", feature_key,
        f"episode_{episode_index:06d}.mp4",
    )


def _parquet_path(dataset_dir, chunk, episode_index):
    return os.path.join(
        dataset_dir,
        PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_index),
    )


def set_episode_language(dataset_dir, episode_index, language_instruction):
    """Update the language_instruction (task_index) of an existing episode."""
    info = _read_json(os.path.join(dataset_dir, INFO_PATH))
    chunk = episode_index // info["chunks_size"]
    parquet_path = _parquet_path(dataset_dir, chunk, episode_index)
    if not os.path.exists(parquet_path):
        raise FileNotFoundError(f"Episode parquet not found: {parquet_path}")

    new_task_index = _ensure_task_index(dataset_dir, language_instruction)

    table = pq.read_table(parquet_path)
    df = table.to_pandas()
    df["task_index"] = np.full(len(df), new_task_index, dtype=np.int64)
    pq.write_table(pa.Table.from_pandas(df), parquet_path)

    # Update episodes.jsonl tasks list
    episodes = _read_jsonl(os.path.join(dataset_dir, EPISODES_PATH))
    for ep in episodes:
        if ep.get("episode_index") == episode_index:
            ep["tasks"] = [language_instruction or ""]
            break
    _write_jsonl(episodes, os.path.join(dataset_dir, EPISODES_PATH))


def trim_episode(dataset_dir, episode_index, start, end):
    """Trim an episode in place to the [start, end) frame range.

    start/end are inclusive/exclusive frame indices. Updates parquet, mp4 videos,
    episode stats, and dataset-level totals. start must be >= 0 and < end <= length.
    """
    info_path = os.path.join(dataset_dir, INFO_PATH)
    info = _read_json(info_path)
    chunk = episode_index // info["chunks_size"]
    parquet_path = _parquet_path(dataset_dir, chunk, episode_index)
    if not os.path.exists(parquet_path):
        raise FileNotFoundError(f"Episode parquet not found: {parquet_path}")

    table = pq.read_table(parquet_path)
    df = table.to_pandas()
    n = len(df)
    start = max(0, int(start))
    end = min(n, int(end))
    if start >= end:
        raise ValueError(f"Invalid trim range: start={start}, end={end}, length={n}")

    new_n = end - start
    df = df.iloc[start:end].reset_index(drop=True)
    df["frame_index"] = np.arange(new_n, dtype=np.int64)
    fps = info.get("fps", 20)
    df["timestamp"] = (np.arange(new_n) / float(fps)).astype(np.float32)
    pq.write_table(pa.Table.from_pandas(df), parquet_path)

    # Re-encode trimmed videos
    features = info.get("features", {})
    video_keys = [k for k, f in features.items() if f.get("dtype") == "video"]
    for feature_key in video_keys:
        video_path = _video_path(dataset_dir, feature_key, chunk, episode_index)
        if not os.path.exists(video_path):
            continue
        frames = _decode_video_frames(video_path)
        trimmed = frames[start:end]
        if not trimmed:
            continue
        # Write PNGs and re-encode
        tmp_dir = os.path.join(dataset_dir, "_tmp_trim", feature_key, f"episode_{episode_index:06d}")
        if os.path.exists(tmp_dir):
            shutil.rmtree(tmp_dir)
        os.makedirs(tmp_dir, exist_ok=True)
        for i, fr in enumerate(trimmed):
            Image.fromarray(fr).save(os.path.join(tmp_dir, f"frame-{i:06d}.png"))
        from lerobot.datasets.video_utils import encode_video_frames
        os.remove(video_path)
        encode_video_frames(
            imgs_dir=tmp_dir,
            video_path=video_path,
            fps=fps,
            vcodec="h264",
            pix_fmt="yuv420p",
            g=2,
            crf=30,
            overwrite=True,
        )
        shutil.rmtree(tmp_dir, ignore_errors=True)
    tmp_root = os.path.join(dataset_dir, "_tmp_trim")
    if os.path.isdir(tmp_root):
        shutil.rmtree(tmp_root, ignore_errors=True)

    # Update episodes.jsonl length
    episodes = _read_jsonl(os.path.join(dataset_dir, EPISODES_PATH))
    old_length = 0
    for ep in episodes:
        if ep.get("episode_index") == episode_index:
            old_length = ep.get("length", 0)
            ep["length"] = new_n
            break
    _write_jsonl(episodes, os.path.join(dataset_dir, EPISODES_PATH))

    # Recompute episode stats
    stats_entries = _read_jsonl(os.path.join(dataset_dir, EPISODES_STATS_PATH))
    new_stats = _compute_episode_stats(df, features, new_n)
    for entry in stats_entries:
        if entry.get("episode_index") == episode_index:
            entry["stats"] = new_stats
            break
    _write_jsonl(stats_entries, os.path.join(dataset_dir, EPISODES_STATS_PATH))

    # Update info totals
    info["total_frames"] = info.get("total_frames", 0) - (old_length - new_n)
    _write_json(info, info_path)


def downsample_episode_range(dataset_dir, episode_index, start, end, factor):
    """Downsample frames inside [start, end) by ``factor`` (keep every Nth).
    Frames outside the range are preserved. Re-encodes parquet + videos and
    updates per-episode/dataset metadata, mirroring ``trim_episode``.

    factor must be an integer >= 2. start/end are inclusive/exclusive.
    """
    factor = int(factor)
    if factor < 2:
        raise ValueError(f"factor must be >= 2 (got {factor})")

    info_path = os.path.join(dataset_dir, INFO_PATH)
    info = _read_json(info_path)
    chunk = episode_index // info["chunks_size"]
    parquet_path = _parquet_path(dataset_dir, chunk, episode_index)
    if not os.path.exists(parquet_path):
        raise FileNotFoundError(f"Episode parquet not found: {parquet_path}")

    table = pq.read_table(parquet_path)
    df = table.to_pandas()
    n = len(df)
    start = max(0, int(start))
    end = min(n, int(end))
    if start >= end:
        raise ValueError(f"Invalid range: start={start}, end={end}, length={n}")

    # Build keep-list: untouched before/after the range, every Nth inside.
    keep = list(range(0, start)) + list(range(start, end, factor)) + list(range(end, n))
    new_n = len(keep)
    if new_n == n:
        # factor=1 would no-op; we already rejected that.
        raise ValueError("Downsample would not change frame count")

    df = df.iloc[keep].reset_index(drop=True)
    df["frame_index"] = np.arange(new_n, dtype=np.int64)
    fps = info.get("fps", 20)
    df["timestamp"] = (np.arange(new_n) / float(fps)).astype(np.float32)
    pq.write_table(pa.Table.from_pandas(df), parquet_path)

    # Re-encode videos using the same kept frame indices so video stays in sync
    # with the parquet timeline (will visibly play faster in the speed-up range).
    features = info.get("features", {})
    video_keys = [k for k, f in features.items() if f.get("dtype") == "video"]
    for feature_key in video_keys:
        video_path = _video_path(dataset_dir, feature_key, chunk, episode_index)
        if not os.path.exists(video_path):
            continue
        frames = _decode_video_frames(video_path)
        sampled = [frames[i] for i in keep if i < len(frames)]
        if not sampled:
            continue
        tmp_dir = os.path.join(dataset_dir, "_tmp_downsample", feature_key, f"episode_{episode_index:06d}")
        if os.path.exists(tmp_dir):
            shutil.rmtree(tmp_dir)
        os.makedirs(tmp_dir, exist_ok=True)
        for i, fr in enumerate(sampled):
            Image.fromarray(fr).save(os.path.join(tmp_dir, f"frame-{i:06d}.png"))
        from lerobot.datasets.video_utils import encode_video_frames
        os.remove(video_path)
        encode_video_frames(
            imgs_dir=tmp_dir,
            video_path=video_path,
            fps=fps,
            vcodec="h264",
            pix_fmt="yuv420p",
            g=2,
            crf=30,
            overwrite=True,
        )
        shutil.rmtree(tmp_dir, ignore_errors=True)
    tmp_root = os.path.join(dataset_dir, "_tmp_downsample")
    if os.path.isdir(tmp_root):
        shutil.rmtree(tmp_root, ignore_errors=True)

    # Update episodes.jsonl length
    episodes = _read_jsonl(os.path.join(dataset_dir, EPISODES_PATH))
    old_length = 0
    for ep in episodes:
        if ep.get("episode_index") == episode_index:
            old_length = ep.get("length", 0)
            ep["length"] = new_n
            break
    _write_jsonl(episodes, os.path.join(dataset_dir, EPISODES_PATH))

    # Recompute episode stats
    stats_entries = _read_jsonl(os.path.join(dataset_dir, EPISODES_STATS_PATH))
    new_stats = _compute_episode_stats(df, features, new_n)
    for entry in stats_entries:
        if entry.get("episode_index") == episode_index:
            entry["stats"] = new_stats
            break
    _write_jsonl(stats_entries, os.path.join(dataset_dir, EPISODES_STATS_PATH))

    # Update info totals
    info["total_frames"] = info.get("total_frames", 0) - (old_length - new_n)
    _write_json(info, info_path)


def _compute_episode_stats(df, features, num_frames):
    """Recompute per-episode stats over the parquet df."""
    stats = {}
    numeric_keys = [
        "observation.qpos", "observation.state",  # 옛 dataset 호환
        "action", "action.joint",
        "observation.qvel", "observation.qeffort",
        "observation.eepos",
        "action.ee_delta",  # 옛 dataset 호환 (신규는 미저장)
    ]
    for key in numeric_keys:
        if key not in features or key not in df.columns:
            continue
        arr = np.array(df[key].tolist(), dtype=np.float32)
        if arr.size == 0:
            continue
        stats[key] = {
            "min": np.min(arr, axis=0).tolist(),
            "max": np.max(arr, axis=0).tolist(),
            "mean": np.mean(arr, axis=0).tolist(),
            "std": np.std(arr, axis=0).tolist(),
            "count": int(num_frames),
        }
    return stats


def copy_episode_to(src_dir, src_index, dst_dir, language_override=None):
    """Copy a single episode from src_dir to dst_dir, returning the new episode_index.

    Source and destination must share the same feature schema (same robot/sensor IDs).
    If they differ, the function raises ValueError. The destination dataset is created
    on the fly if missing — but only when the source has at least one feature schema.
    """
    src_info = _read_json(os.path.join(src_dir, INFO_PATH))
    src_chunk = src_index // src_info["chunks_size"]
    src_parquet = _parquet_path(src_dir, src_chunk, src_index)
    if not os.path.exists(src_parquet):
        raise FileNotFoundError(f"Source episode not found: {src_parquet}")

    dst_info_path = os.path.join(dst_dir, INFO_PATH)
    if not os.path.exists(dst_info_path):
        # Bootstrap dst with src's info (features, fps)
        os.makedirs(dst_dir, exist_ok=True)
        bootstrap = dict(src_info)
        bootstrap.update({
            "total_episodes": 0, "total_frames": 0, "total_chunks": 0,
            "total_videos": 0, "total_tasks": 0, "splits": {"train": "0:0"},
        })
        _write_json(bootstrap, dst_info_path)
        for p in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
            fpath = os.path.join(dst_dir, p)
            os.makedirs(os.path.dirname(fpath), exist_ok=True)
            if not os.path.exists(fpath):
                open(fpath, "w").close()

    dst_info = _read_json(dst_info_path)

    src_features = src_info.get("features", {})
    dst_features = dst_info.get("features", {})
    if dst_features and set(src_features.keys()) != set(dst_features.keys()):
        raise ValueError(
            "Cannot copy episode — feature schemas differ between source and target dataset"
        )

    new_index = dst_info["total_episodes"]
    new_chunk = new_index // dst_info["chunks_size"]
    new_parquet = _parquet_path(dst_dir, new_chunk, new_index)
    os.makedirs(os.path.dirname(new_parquet), exist_ok=True)

    # Read source parquet, rewrite indices
    table = pq.read_table(src_parquet)
    df = table.to_pandas()
    num_frames = len(df)

    # Resolve language instruction (use override or look up from source tasks)
    if language_override is None:
        src_tasks = _read_jsonl(os.path.join(src_dir, TASKS_PATH))
        src_task_index = int(df["task_index"].iloc[0]) if num_frames else 0
        language_override = ""
        for t in src_tasks:
            if t.get("task_index") == src_task_index:
                language_override = t.get("task", "")
                break

    new_task_index = _ensure_task_index(dst_dir, language_override)
    new_global_start = dst_info.get("total_frames", 0)

    df["episode_index"] = np.full(num_frames, new_index, dtype=np.int64)
    df["frame_index"] = np.arange(num_frames, dtype=np.int64)
    df["index"] = np.arange(new_global_start, new_global_start + num_frames, dtype=np.int64)
    df["task_index"] = np.full(num_frames, new_task_index, dtype=np.int64)
    pq.write_table(pa.Table.from_pandas(df), new_parquet)

    # Copy mp4 video files
    num_videos = 0
    for key, feat in src_features.items():
        if feat.get("dtype") != "video":
            continue
        src_video = _video_path(src_dir, key, src_chunk, src_index)
        if not os.path.exists(src_video):
            continue
        dst_video = _video_path(dst_dir, key, new_chunk, new_index)
        os.makedirs(os.path.dirname(dst_video), exist_ok=True)
        shutil.copy2(src_video, dst_video)
        num_videos += 1

    # Recompute stats and append
    stats = _compute_episode_stats(df, src_features, num_frames)
    _append_jsonl(
        {"episode_index": new_index, "stats": stats},
        os.path.join(dst_dir, EPISODES_STATS_PATH),
    )
    _append_jsonl(
        {"episode_index": new_index, "length": num_frames, "tasks": [language_override or ""]},
        os.path.join(dst_dir, EPISODES_PATH),
    )

    # If destination didn't have features yet, persist them now (bootstrap path)
    if not dst_features:
        dst_info["features"] = src_features
        if "action_key" in src_info and "action_key" not in dst_info:
            dst_info["action_key"] = src_info["action_key"]
        if "fps" not in dst_info:
            dst_info["fps"] = src_info.get("fps", 20)

    dst_info["total_episodes"] = new_index + 1
    dst_info["total_frames"] = new_global_start + num_frames
    dst_info["total_chunks"] = new_chunk + 1
    dst_info["total_videos"] = dst_info.get("total_videos", 0) + num_videos
    dst_info["splits"] = {"train": f"0:{new_index + 1}"}
    _write_json(dst_info, dst_info_path)
    return new_index
