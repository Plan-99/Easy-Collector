from einops import rearrange
import cv2
import numpy as np
import torch
import os
import re
import shutil
from torch.utils.data import TensorDataset, DataLoader, WeightedRandomSampler
import json
from types import SimpleNamespace
from lerobot.configs.types import PolicyFeature, FeatureType
from torchvision import transforms
from transformers import AutoImageProcessor
from PIL import Image
from scipy.spatial.transform import Rotation
import pyarrow as pa
import pyarrow.parquet as pq
import datasets as hf_datasets

import sys as _sys
_sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lerobot', 'src'))
from lerobot.datasets.feature_utils import get_hf_features_from_features, create_empty_dataset_info
from lerobot.datasets.io_utils import embed_images
from lerobot.datasets.utils import DEFAULT_IMAGE_PATH, DEFAULT_FEATURES

try:
    import av
except ImportError:
    av = None

# ─── LeRobot I/O constants & functions (inlined from the former utils/lerobot_io.py) ─────

LEROBOT_CODEBASE_VERSION = "v2.1"
CODEBASE_VERSION = "v2.1"
DEFAULT_CHUNK_SIZE = 1000

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

def create_dataset(dataset_dir, agents, sensors, task, fps=20, action_key="qaction"):
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

    os.makedirs(dataset_dir, exist_ok=True)
    _write_json(info, os.path.join(dataset_dir, INFO_PATH))

    # Create empty jsonl files
    for path in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
        fpath = os.path.join(dataset_dir, path)
        os.makedirs(os.path.dirname(fpath), exist_ok=True)
        if not os.path.exists(fpath):
            open(fpath, "w").close()

    return features


def build_features(agents, sensors, task, action_key="qaction"):
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

        # EE features: only if agent has ik_solver
        has_ik = a.get('ik_solver') is not None if isinstance(agent, dict) else getattr(agent, 'ik_solver', None) is not None
        if has_ik:
            agent_ee_names = a.get('ee_names', []) if isinstance(agent, dict) else getattr(agent, 'ee_names', [])
            tool_inner = a.get('tool_inner', False) if isinstance(agent, dict) else getattr(agent, 'tool_inner', False)
            per_ee_dim = 6 + (1 if tool_inner else 0)
            for ee_name in sorted(agent_ee_names):
                components = ["x", "y", "z", "rx", "ry", "rz"]
                if tool_inner:
                    components.append("tool")
                ee_names_list.extend([f"robot_{a_id}_{ee_name}_{c}" for c in components])
                ee_dim += per_ee_dim

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": state_names,
        },
        "action": {
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

    # EE features (only if any agent has ik_solver)
    if ee_dim > 0:
        features["observation.eepos"] = {
            "dtype": "float32",
            "shape": (ee_dim,),
            "names": ee_names_list,
        }
        features["observation.ee_delta"] = {
            "dtype": "float32",
            "shape": (ee_dim,),
            "names": ee_names_list,
        }
        features["action.ee_delta"] = {
            "dtype": "float32",
            "shape": (ee_dim,),
            "names": ee_names_list,
        }

    # Video features (stored as MP4, not in parquet)
    for sensor in sensors:
        s = sensor if isinstance(sensor, dict) else sensor.__dict__
        s_id = str(s.get('id'))
        # Get image resolution from task config
        img_size = task.get('sensor_img_size', {}).get(s_id, [640, 480])
        h, w = img_size[1], img_size[0]  # [width, height] -> (H, W)
        features[f"observation.images.sensor_{s_id}"] = {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }

    return features


# ─── Episode append ──────────────────────────────────────────────────────────

def append_episode(dataset_dir, timesteps, agents, sensors, task,
                   language_instruction="", action_key="qaction",
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
    actions = []
    qvels = []
    qefforts = []
    eepos_list = []
    ee_delta_list = []
    ee_delta_action_list = []
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
        ee_delta_parts = []
        ee_delta_action_parts = []

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

            # observation.eepos — flatten dict {ee_name: [6+tool]}
            eepos = robot_state.get('eepos')
            if eepos is not None and isinstance(eepos, dict):
                for ee_name in sorted(eepos.keys()):
                    eepos_parts.append(np.array(eepos[ee_name], dtype=np.float32))

            # observation.ee_delta — flatten dict {ee_name: [6+tool]}
            ee_delta = robot_state.get('ee_delta')
            if ee_delta is not None and isinstance(ee_delta, dict):
                for ee_name in sorted(ee_delta.keys()):
                    ee_delta_parts.append(np.array(ee_delta[ee_name], dtype=np.float32))

            # action.ee_delta (ee_delta_action) — flatten dict {ee_name: [6+tool]}
            ee_delta_act = robot_state.get('ee_delta_action')
            if ee_delta_act is not None and isinstance(ee_delta_act, dict):
                for ee_name in sorted(ee_delta_act.keys()):
                    ee_delta_action_parts.append(np.array(ee_delta_act[ee_name], dtype=np.float32))

        # action: concatenate qaction across agents
        action_parts = []
        for agent in agents_sorted:
            robot_state = ts.observation['robot_states'][agent.id]
            if action_key == 'ee_delta_action':
                ee_delta_a = robot_state.get('ee_delta_action')
                if ee_delta_a is not None:
                    if isinstance(ee_delta_a, dict):
                        for ee_name in sorted(ee_delta_a.keys()):
                            if t + 1 < num_frames:
                                next_val = timesteps[t + 1].observation['robot_states'][agent.id]['ee_delta_action'][ee_name]
                            else:
                                next_val = [0.0] * len(ee_delta_a[ee_name])
                            action_parts.append(np.array(next_val, dtype=np.float32))
                    else:
                        if t + 1 < num_frames:
                            next_val = timesteps[t + 1].observation['robot_states'][agent.id]['ee_delta_action']
                        else:
                            next_val = [0.0] * len(ee_delta_a)
                        action_parts.append(np.array(next_val, dtype=np.float32))
                else:
                    qact = robot_state.get('qaction')
                    if qact is not None:
                        action_parts.append(np.array(qact, dtype=np.float32))
            else:
                qact = robot_state.get('qaction')
                if qact is not None:
                    action_parts.append(np.array(qact, dtype=np.float32))

        states.append(np.concatenate(qpos_parts) if qpos_parts else np.array([], dtype=np.float32))
        actions.append(np.concatenate(action_parts) if action_parts else np.array([], dtype=np.float32))
        qvels.append(np.concatenate(qvel_parts) if qvel_parts else np.array([], dtype=np.float32))
        qefforts.append(np.concatenate(qeffort_parts) if qeffort_parts else np.array([], dtype=np.float32))

        if eepos_parts:
            eepos_list.append(np.concatenate(eepos_parts))
        elif ee_dim > 0:
            eepos_list.append(np.zeros(ee_dim, dtype=np.float32))

        if ee_delta_parts:
            ee_delta_list.append(np.concatenate(ee_delta_parts))
        elif ee_dim > 0:
            ee_delta_list.append(np.zeros(ee_dim, dtype=np.float32))

        if ee_delta_action_parts:
            ee_delta_action_list.append(np.concatenate(ee_delta_action_parts))
        elif ee_dim > 0:
            ee_delta_action_list.append(np.zeros(ee_dim, dtype=np.float32))

        # succeed flag
        if succeed_flags is not None and t < len(succeed_flags):
            succeed_list.append(float(succeed_flags[t]))
        else:
            succeed_list.append(0.0)

    states = np.array(states, dtype=np.float32)
    actions = np.array(actions, dtype=np.float32)
    qvels_arr = np.array(qvels, dtype=np.float32)
    qefforts_arr = np.array(qefforts, dtype=np.float32)
    succeed_arr = np.array(succeed_list, dtype=np.float32)

    # ── Save frames as temporary PNGs, then encode to MP4 ────────────────
    num_videos = 0
    for sensor in sensors:
        s_id = str(sensor['id'])
        feature_key = f"observation.images.sensor_{s_id}"

        # Save frames as PNGs (encode_video_frames expects frame_NNNNNN.png)
        imgs_dir = os.path.join(
            dataset_dir, "images", feature_key, f"episode_{episode_index:06d}"
        )
        os.makedirs(imgs_dir, exist_ok=True)

        for t in range(num_frames):
            ts = timesteps[t]
            img = ts.observation['images'][f'sensor_{s_id}']

            if fetch_image_fn is not None:
                img = fetch_image_fn(img, {
                    'resize': task.get('sensor_img_size', {}).get(s_id),
                    'cropped_area': task.get('sensor_cropped_area', {}).get(s_id, {}),
                    'rotate': task.get('sensor_rotate', {}).get(s_id, 0),
                })

            if isinstance(img, np.ndarray):
                # OpenCV uses BGR, convert to RGB for PIL/video encoding
                if img.ndim == 3 and img.shape[2] == 3:
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
        "observation.state": np.stack(states),
        "action": np.stack(actions),
    }

    # Add new fields only if features define them (backward compat with old datasets)
    if "observation.qvel" in features:
        episode_dict["observation.qvel"] = np.stack(qvels_arr)
    if "observation.qeffort" in features:
        episode_dict["observation.qeffort"] = np.stack(qefforts_arr)
    if "succeed" in features:
        episode_dict["succeed"] = succeed_arr
    if "observation.eepos" in features and eepos_list:
        episode_dict["observation.eepos"] = np.stack(eepos_list)
    if "observation.ee_delta" in features and ee_delta_list:
        episode_dict["observation.ee_delta"] = np.stack(ee_delta_list)
    if "action.ee_delta" in features and ee_delta_action_list:
        episode_dict["action.ee_delta"] = np.stack(ee_delta_action_list)

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
        ("observation.state", states),
        ("action", actions),
        ("observation.qvel", qvels_arr),
        ("observation.qeffort", qefforts_arr),
    ]
    if eepos_list:
        stat_pairs.append(("observation.eepos", np.stack(eepos_list)))
    if ee_delta_list:
        stat_pairs.append(("observation.ee_delta", np.stack(ee_delta_list)))
    if ee_delta_action_list:
        stat_pairs.append(("action.ee_delta", np.stack(ee_delta_action_list)))

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

    # Read states and actions from parquet. 새 schema (observation.qpos / action.joint)
    # 와 옛 schema (observation.state / action) 모두 지원.
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
    sequential deltas). None 이면 legacy 동작 (naive 차분).
    """
    waypoints = np.asarray(waypoints, dtype=np.float32)
    if current_eepos is not None:
        return relative_trajectory_in_local_frame_to_world_deltas(waypoints, current_eepos)

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


# ─── DexUMI relative trajectory 헬퍼 (current EE local frame 변환) ───────

def _eepos_to_homogeneous(eepos6: np.ndarray) -> np.ndarray:
    """[x, y, z, rx, ry, rz] (axis-angle) → 4x4 homogeneous matrix."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_rotvec(eepos6[3:6]).as_matrix()
    T[:3, 3] = eepos6[:3]
    return T


def _homogeneous_to_eepos(T: np.ndarray) -> np.ndarray:
    rotvec = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return np.concatenate([T[:3, 3], rotvec]).astype(np.float32)


def compute_relative_trajectory_in_local_frame(
    future_eepos: np.ndarray, current_eepos: np.ndarray
) -> np.ndarray:
    """DexUMI 스타일 — future eepos chunk 를 current eepos local frame 으로.
    `T_relative[i] = invert(T_current) @ T_future[i]`.
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
            out[i, 6:] = future_eepos[i, 6:]
    return out


def relative_trajectory_in_local_frame_to_world_deltas(
    relative_traj: np.ndarray, current_eepos: np.ndarray
) -> np.ndarray:
    """역변환 (inference)."""
    relative_traj = np.asarray(relative_traj, dtype=np.float32)
    current_eepos = np.asarray(current_eepos, dtype=np.float32)
    if relative_traj.ndim == 1:
        relative_traj = relative_traj[None, :]
    N, D = relative_traj.shape

    T_curr = _eepos_to_homogeneous(current_eepos)
    T_world_list = []
    for i in range(N):
        T_rel = _eepos_to_homogeneous(relative_traj[i, :6])
        T_world_list.append(T_curr @ T_rel)

    deltas = np.zeros_like(relative_traj)
    T_prev = T_curr
    for i in range(N):
        T_i = T_world_list[i]
        delta_t = T_i[:3, 3] - T_prev[:3, 3]
        R_delta = T_i[:3, :3] @ T_prev[:3, :3].T
        rotvec_delta = Rotation.from_matrix(R_delta).as_rotvec()
        deltas[i, :3] = delta_t
        deltas[i, 3:6] = rotvec_delta
        if D > 6:
            deltas[i, 6:] = relative_traj[i, 6:]
        T_prev = T_i
    return deltas


# ─── action_key / obs_state_keys 통일 헬퍼 ───────────────────────────────

_ACTION_KEY_ALIASES = {
    'qaction': 'joint',
    'joint': 'joint',
    'ee_delta_action': 'ee_delta',
    'ee_delta': 'ee_delta',
    'relative_ee_pos': 'relative_ee_pos',
}


def _normalize_action_key(action_key):
    if action_key is None:
        return 'joint'
    return _ACTION_KEY_ALIASES.get(action_key, action_key)


def _get_action_data(df, action_key, eepos_ee_dim=0, tool_qpos_indices=None):
    """action_key 에 따라 dataset 에서 action 배열 (T, D) 반환.

    - 'joint'(/'qaction'): action.joint 컬럼 우선, 없으면 legacy 'action'.
    - 'ee_delta' / 'relative_ee_pos': eepos(순수 EE 앞 eepos_ee_dim) + qpos[tool]
      을 _assemble_action_eepos 로 조립한 뒤 시간 차분. (relative_ee_pos 의 진짜
      학습 target 은 chunk relative trajectory 라 __getitem__ 이 eepos 로부터
      직접 계산 — 여기 반환은 norm_stats 폴백/통계용 placeholder.)
    """
    norm = _normalize_action_key(action_key)
    if norm == 'joint':
        col = 'action.joint' if 'action.joint' in df.columns else 'action'
        return np.array(df[col].tolist(), dtype=np.float32)
    if norm in ('ee_delta', 'relative_ee_pos'):
        eepos = _assemble_action_eepos(df, eepos_ee_dim, tool_qpos_indices)
        if eepos is not None:
            delta = np.zeros_like(eepos)
            if len(eepos) > 1:
                delta[:-1] = eepos[1:] - eepos[:-1]
            return delta
        if 'action.ee_delta' in df.columns:
            return np.array(df['action.ee_delta'].tolist(), dtype=np.float32)
        if 'action' in df.columns:
            return np.array(df['action'].tolist(), dtype=np.float32)
        return np.array(df['action.joint'].tolist(), dtype=np.float32)
    col = 'action' if 'action' in df.columns else 'action.joint'
    return np.array(df[col].tolist(), dtype=np.float32)


def _compute_tool_qpos_indices(dataset_info):
    """qpos 의 어느 인덱스가 tool joint 인지 계산 (**전체 tool 집합**).

    state 의 tool_pose 추출 / arm joint 분리에 쓰인다. (eepos 증강용이 아니라
    "qpos 안에서 tool 이 어디냐"를 답한다.)

    우선순위:
      1. info.json 의 ``tool_qpos_indices`` 필드 (lerobot_io 가 dataset 생성 시
         robot DB role/tool_inner/tool_index 기반으로 정확히 계산해 저장).
      2. fallback 휴리스틱: qpos joint 중 eepos 에 robot_id 가 없는 것 = tool.
         (구 dataset 은 eepos 에 tool 이 없으므로 이 휴리스틱이 전체 tool 을 찾음.)

    backend/policies/utils.py 와 동일 로직.
    """
    if dataset_info and isinstance(dataset_info, dict):
        explicit = dataset_info.get('tool_qpos_indices')
        if isinstance(explicit, list):
            try:
                return [int(x) for x in explicit]
            except Exception:
                pass

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


def _compute_eepos_ee_dim(dataset_info):
    """observation.eepos 컬럼에서 순수 EE pose 가 차지하는 앞쪽 dim 수 (= 6 × ee수).

    eepos 규격 통일: action 은 항상 'raw eepos 앞 ee_dim (순수 EE) + qpos[tool]'.
    구 dataset 은 eepos 뒤에 tool 이 baked 돼 있을 수 있어 이 dim 까지만 잘라 쓴다.
    EE 이름은 `_x` 로 끝나는 컴포넌트가 ee 당 정확히 1개라 그 수 × 6.

    backend/policies/utils.py 와 동일 로직.
    """
    features = dataset_info.get("features", {}) if dataset_info else {}
    eepos_names = features.get("observation.eepos", {}).get("names") or []
    if not eepos_names:
        return 0
    num_ee = sum(1 for n in eepos_names if str(n).endswith('_x'))
    return 6 * num_ee


def _assemble_action_eepos(df, eepos_ee_dim, tool_qpos_indices):
    """relative_ee_pos / ee_delta action 용 eepos 조립 — 구/신 dataset 통일 경로.

    raw observation.eepos 를 순수 EE (앞 eepos_ee_dim) 로 자르고, tool 은 항상
    observation.qpos[tool_qpos_indices] 에서 가져와 뒤에 붙인다.
      - 신규 dataset: eepos 가 이미 순수 EE → 슬라이스 no-op.
      - 구 dataset: eepos 에 tool 이 baked → 앞 eepos_ee_dim 만 취해 버림.
    어느 쪽이든 tool 은 qpos 가 single source. eepos 컬럼 없으면 None.

    backend/policies/utils.py 와 동일 로직.
    """
    if 'observation.eepos' not in df.columns:
        return None
    eepos = np.array(df['observation.eepos'].tolist(), dtype=np.float32)
    if eepos.ndim == 1:
        eepos = eepos.reshape(-1, 1)
    if eepos_ee_dim and 0 < eepos_ee_dim < eepos.shape[1]:
        eepos = eepos[:, :eepos_ee_dim]
    if tool_qpos_indices:
        qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else (
            'observation.state' if 'observation.state' in df.columns else None)
        if qpos_col is not None:
            qpos_arr = np.array(df[qpos_col].tolist(), dtype=np.float32)
            if qpos_arr.ndim == 1:
                qpos_arr = qpos_arr.reshape(-1, 1)
            eepos = np.concatenate([eepos, qpos_arr[:, tool_qpos_indices]], axis=1)
    return eepos


def _build_obs_state(df, obs_state_keys, tool_qpos_indices=None, eepos_ee_dim=0):
    """obs_state_keys 순서대로 컬럼 concat. 옛 dataset 도 호환 (qpos→state fallback).

    Semantic (NEW):
      - 사용자 선택 obs_state_keys 의 qpos/qvel/qeffort 는 **arm 부분만** 추출
        (tool joint 인덱스 제외).
      - eepos 는 끝에 augment 된 tool dim 제거.
      - 마지막에 **항상 qpos[tool_qpos_indices] (tool_pose) 1+ dim 추가**.
      - obs_state_keys == [] + tool 있음 → state = [tool_pose] 만.
      - obs_state_keys == [] + tool 없음 → 빈 state (0 dim).
    """
    if obs_state_keys is None:
        obs_state_keys = ['qpos']

    tool_idx = list(tool_qpos_indices) if tool_qpos_indices else []
    has_tool = len(tool_idx) > 0

    qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns \
        else ('observation.state' if 'observation.state' in df.columns else None)
    qpos_arr = None
    if qpos_col is not None:
        qpos_arr = np.array(df[qpos_col].tolist(), dtype=np.float32)
        if qpos_arr.ndim == 1:
            qpos_arr = qpos_arr.reshape(-1, 1)

    arm_idx = None
    if qpos_arr is not None and has_tool:
        n_joints = qpos_arr.shape[1]
        arm_idx = [i for i in range(n_joints) if i not in tool_idx]

    parts = []
    for key in obs_state_keys:
        if key == 'qpos':
            if qpos_arr is None:
                continue
            parts.append(qpos_arr[:, arm_idx] if (arm_idx is not None) else qpos_arr)
            continue
        col = f'observation.{key}'
        if col not in df.columns:
            continue
        arr = np.array(df[col].tolist(), dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape(-1, 1)
        if key in ('qvel', 'qeffort') and arm_idx is not None and arr.shape[1] >= len(arm_idx) + len(tool_idx):
            arr = arr[:, arm_idx]
        elif key == 'eepos' and eepos_ee_dim and 0 < eepos_ee_dim < arr.shape[1]:
            # raw eepos 를 순수 EE 앞부분(eepos_ee_dim)만 — tool 은 아래에서
            # qpos 로부터 한 번만 append. 새 규격 eepos 는 이미 순수 EE → no-op.
            arr = arr[:, :eepos_ee_dim]
        parts.append(arr)

    if has_tool and qpos_arr is not None:
        parts.append(qpos_arr[:, tool_idx])

    if not parts:
        return np.zeros((len(df), 0), dtype=np.float32)

    return np.concatenate(parts, axis=1)


class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone='resnet18', n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, augment=False, wrist_sensor_ids=None, image_resolution=None):
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
        # Unified resize target — lets users mix datasets recorded with different camera
        # resolutions in a single training run. process_image() resizes every frame to
        # this size before stacking, so the batch tensor is well-defined regardless of
        # source dimensions. Default (224, 224) matches the previous hard-coded behavior.
        if image_resolution is None:
            image_resolution = (224, 224)
        if isinstance(image_resolution, int):
            image_resolution = (image_resolution, image_resolution)
        else:
            image_resolution = (int(image_resolution[0]), int(image_resolution[1]))
        self.image_resolution = image_resolution
        # Image augmentation: PI0.5 paper Appendix E recipe (RandomResizedCrop 95%
        # + Rotate ±5° + ColorJitter). Applied on PIL images before process_image.
        # Train-only; val/eval datasets pass augment=False.
        # Wrist cameras: openpi skips spatial transforms because crop/rotate breaks
        # the gripper↔observation geometric coupling (audit-doc bug #31). User
        # configures wrist_sensor_ids to opt into ColorJitter-only path.
        self.augment = augment
        self.wrist_sensor_ids = set(int(x) for x in (wrist_sensor_ids or []))
        if augment:
            self._image_augment_full = transforms.Compose([
                transforms.RandomResizedCrop(size=self.image_resolution, scale=(0.9025, 1.0), ratio=(0.95, 1.05)),
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
        # eepos 규격 통일: action 은 항상 'eepos 앞 _eepos_ee_dim (순수 EE) +
        # qpos[_tool_qpos_indices]'. 구/신 dataset 무관 단일 경로.
        #  - _tool_qpos_indices: qpos 안 tool 위치 (state tool_pose / arm 분리 /
        #    action 의 tool append 에 모두 사용 — single source).
        #  - _eepos_ee_dim: raw eepos 에서 순수 EE 앞부분 dim (구 dataset 의
        #    baked tool 을 잘라내기 위함; 신규 dataset 은 eepos 가 이미 순수 EE).
        self._tool_qpos_indices = _compute_tool_qpos_indices(self._dataset_info)
        self._eepos_ee_dim = _compute_eepos_ee_dim(self._dataset_info)

        # Pre-load tasks once (avoid re-reading jsonl every __getitem__)
        # _read_jsonl / TASKS_PATH는 본 파일 상단(line 31~)에 inline돼 있어 import 불필요.
        tasks = _read_jsonl(os.path.join(dataset_dir, TASKS_PATH))
        self._task_map = {t.get("task_index"): t.get("task", "") for t in tasks}

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

        # obs_state_keys + action_key 에 따라 state/action 구성. ee_delta /
        # relative_ee_pos 면 _get_action_data 가 eepos 차분으로 derive (relative_ee_pos
        # 의 chunk 변환은 __getitem__ 에서 eepos_data 로 직접 처리).
        state_data = _build_obs_state(df, self.obs_state_keys, self._tool_qpos_indices,
                                      eepos_ee_dim=self._eepos_ee_dim)
        action_data = _get_action_data(df, self.action_key,
                                       self._eepos_ee_dim, self._tool_qpos_indices)
        # eepos_data — relative_ee_pos chunk 변환용. 통일 경로: 순수 EE +
        # qpos[tool]. 구/신 dataset 무관 동일 결과 (_assemble_action_eepos).
        eepos_data = _assemble_action_eepos(df, self._eepos_ee_dim, self._tool_qpos_indices)

        # Get language instruction from pre-loaded task map
        task_index = int(df["task_index"].iloc[0])
        language_instruction = self._task_map.get(task_index, "")

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
                    # _decode_video_frames는 본 파일 line 932에 정의되어 있음.
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
            # DexUMI: chunk 의 미래 N 개 eepos 를 현재 eepos local frame 으로 변환.
            # eepos 없으면 (옛 dataset) legacy naive cumsum fallback.
            eepos_data = ep.get("eepos_data")
            if eepos_data is not None and len(eepos_data) > start_ts:
                future_idx = np.arange(start_ts + 1, start_ts + self.chunk_size + 1)
                future_idx = np.clip(future_idx, 0, episode_len - 1)
                future_eepos = eepos_data[future_idx]
                current_eepos = eepos_data[start_ts]
                rel_traj = compute_relative_trajectory_in_local_frame(future_eepos, current_eepos)
                padded_action[:action_len, :rel_traj.shape[1]] = rel_traj[:action_len]
                # any_has_succeed 면 action 의 마지막 컬럼이 succeed flag — rel_traj
                # 는 eepos 차원만 다루므로 succeed slot 을 따로 복사해야 모델이 그
                # dim 을 학습할 수 있다. (get_norm_stats 와 동일 layout).
                if any_has_succeed and action.shape[1] > rel_traj.shape[1]:
                    padded_action[:action_len, rel_traj.shape[1]:] = action[:action_len, rel_traj.shape[1]:]
            else:
                padded_action[:action_len] = delta_to_relative_trajectory(action[:action_len])
        elif _normalize_action_key(self.action_key) == 'relative_joint_pos':
            # chunk-anchored joint delta: action[t in chunk] - qpos[chunk_start].
            # anchor를 ep["state_data"]에서 직접 읽음 (training_server에서 state_data
            # 가 곧 qpos이므로 anchor source로 직접 사용). obs_state_keys와 별개 채널.
            # absolute dims (gripper/tool/done)는 dataset features에서 자동 추출.
            qpos_anchor_data = ep.get("state_data")
            chunk_slice = action[:action_len]
            if qpos_anchor_data is not None and len(qpos_anchor_data) > start_ts:
                if qpos_anchor_data.ndim == 1:
                    qpos_anchor_data = qpos_anchor_data.reshape(-1, 1)
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
        # PI05/PaliGemma pretrained weights expect [-1, 1] (audit-doc bug #1).
        # Feeding [0, 1] into pi05_base shifts the SigLIP feature distribution and
        # wrecks pretrained visual grounding; LoRA can't fully recover.
        _pixel_range = '-11' if self.policy_type == 'PI05' else '01'
        for sensor_id in self.sensor_ids:
            processed_images = []
            # Multi-view: sensor_id 가 view_key ("5" or "5_2") 일 수 있음. wrist
            # 판정은 항상 물리 sensor_id 기준.
            # Wrist cams: ColorJitter only (preserve spatial geometry).
            # Other cams: full aug (RandomResizedCrop + RandomRotation + ColorJitter).
            _vk = str(sensor_id)
            _phys_sid_str = _vk.split('_', 1)[0]
            try:
                _phys_sid = int(_phys_sid_str)
            except ValueError:
                _phys_sid = None
            if _phys_sid is not None and _phys_sid in getattr(self, 'wrist_sensor_ids', set()):
                aug = getattr(self, '_image_augment_color_only', None)
            else:
                aug = getattr(self, '_image_augment_full', None)
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                if aug is not None:
                    image = aug(image)
                image = process_image(image, self.vision_backbone, pixel_range=_pixel_range, image_resolution=self.image_resolution)
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


def _absolute_action_dims_from_features(action_names, has_succeed: bool):
    """action.joint feature names + has_succeed 로부터 absolute dim 인덱스 자동 추출.

    relative_joint_pos 모드에서 어떤 dim이 delta로 변환돼선 안 되는지 결정.
    - 이름에 'gripper' 또는 'tool' 포함 → absolute
    - has_succeed=True → action 마지막 dim이 succeed bit → absolute
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


def process_image(image, vision_backbone='resnet18', to_cuda=False, pixel_range='01', image_resolution=None):
    """Preprocess an image into a model-ready tensor.

    pixel_range: '01' → standard torchvision ToTensor output in [0, 1].
                 '-11' → scaled to [-1, 1] (mandatory for PI05 / PaliGemma / SigLIP —
                         openpi preprocess_observation explicitly does `img / 255 * 2 - 1`).
                         Feeding [0, 1] into pi05_base shifts the vision feature
                         distribution and wrecks pretrained visual grounding (audit-doc
                         bug #1). NOTE: modeling_pi05._preprocess_images expects [-1,1]
                         and does NOT re-scale (audit-doc bug #28 fix).

    image_resolution: (H, W) target resize used to UNIFY heterogeneous source resolutions
                      so that datasets recorded with different camera sizes can be mixed
                      in a single training run. Defaults to (224, 224) — backwards compatible
                      with the previous hard-coded resize. For DINO backbones, the HF
                      AutoImageProcessor's internal {height, width} is overridden so the
                      same target is honored end-to-end.
    """
    if image_resolution is None:
        image_resolution = (224, 224)
    # Normalize tuple/list/int → (H, W) ints.
    if isinstance(image_resolution, int):
        image_resolution = (image_resolution, image_resolution)
    elif isinstance(image_resolution, (list, tuple)) and len(image_resolution) == 2:
        image_resolution = (int(image_resolution[0]), int(image_resolution[1]))
    else:
        raise ValueError(f"image_resolution must be int or (H, W); got {image_resolution!r}")

    if not isinstance(image, Image.Image):
        image = Image.fromarray(np.array(image))
    if vision_backbone not in VISION_BACKBONE_MAP:
        image_transform = transforms.Compose([
            transforms.Resize(image_resolution),
            transforms.ToTensor(),
        ])
        image = image_transform(image)
        if pixel_range == '-11':
            image = image * 2.0 - 1.0
    else:
        image_processor = AutoImageProcessor.from_pretrained(VISION_BACKBONE_MAP[vision_backbone])
        # Override DINO's default crop/resize so the requested target is honored.
        h, w = image_resolution
        image = image_processor(
            image,
            do_resize=True, do_center_crop=False,
            size={"height": h, "width": w},
        )['pixel_values'][0]  # backbone's own normalization

    return image.cuda() if to_cuda else image


def get_norm_stats(dataset_dir, num_episodes, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, chunk_size=50):
    """Compute normalization stats from LeRobot dataset parquet files.

    For ``action_key='relative_joint_pos'`` the action is overwritten with the
    chunk-anchored delta from observation.qpos before stats are accumulated.
    Absolute dims (gripper / tool / done) are auto-detected from action feature
    names + has_succeed via ``_absolute_action_dims_from_features``.
    """
    if obs_state_keys is None:
        obs_state_keys = ['qpos']

    dataset_info = get_dataset_info(dataset_dir)
    # eepos 규격 통일: EpisodicDataset 과 동일하게 _tool_qpos_indices(qpos 안
    # tool 위치) + _eepos_ee_dim(eepos 의 순수 EE 앞부분) 으로 처리해야 norm
    # stats 차원이 dataset class 와 일치한다.
    tool_qpos_indices = _compute_tool_qpos_indices(dataset_info)
    eepos_ee_dim = _compute_eepos_ee_dim(dataset_info)
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

        # obs_state_keys / action_key 에 따른 컬럼 선택
        state_data = _build_obs_state(df, obs_state_keys, tool_qpos_indices,
                                      eepos_ee_dim=eepos_ee_dim)
        action_data = _get_action_data(df, action_key, eepos_ee_dim, tool_qpos_indices)

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

        qpos = _build_obs_state(df, obs_state_keys, tool_qpos_indices,
                                eepos_ee_dim=eepos_ee_dim)
        action = _get_action_data(df, action_key, eepos_ee_dim, tool_qpos_indices)

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
            # DexUMI 스타일 — 매 frame 을 chunk 시작점으로 보고 chunk_size step 의
            # local-frame trajectory 누적해서 stats 계산. 옛 dataset (eepos 없음) 만
            # legacy naive cumsum fallback.
            # dataset class __getitem__ 과 동일하게 통일 경로로 eepos 조립
            # (순수 EE 앞 eepos_ee_dim + qpos[tool]). 없으면 None.
            eepos_arr = _assemble_action_eepos(df, eepos_ee_dim, tool_qpos_indices)
            if eepos_arr is not None:
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
                    # rel_chunks 차원 뒤에 succeed dim 도 같이 붙여 복원.
                    if succeed_chunks:
                        succeed_concat = np.concatenate(succeed_chunks, axis=0)
                        action = np.concatenate([action, succeed_concat], axis=1)
                else:
                    action = delta_to_relative_trajectory(action)
            else:
                action = delta_to_relative_trajectory(action)

        # relative_joint_pos: obs_state_keys와 독립. observation.qpos 컬럼을 직접
        # anchor로 사용. relative_ee_pos가 observation.eepos를 anchor로 쓰는 패턴
        # 과 대칭. action target = chunk-anchored delta from qpos[chunk_start].
        if action_key == 'relative_joint_pos':
            _qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else 'observation.state'
            if _qpos_col in df.columns:
                anchor_arr = np.array(df[_qpos_col].tolist(), dtype=np.float32)
                if anchor_arr.ndim == 1:
                    anchor_arr = anchor_arr.reshape(-1, 1)
                T_total = action.shape[0]
                action_dim = action.shape[1]
                anchor_dim = anchor_arr.shape[1]
                # absolute_action_dims는 features의 action.joint names + has_succeed
                # 로부터 자동 계산. gripper/tool/done은 절대 유지.
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
    # q01/q99 (1st & 99th percentiles) — required by PI05 QUANTILES normalization.
    # Computed unconditionally so the same stats dict can drive any policy type.
    action_q01 = torch.quantile(all_action_data.float(), 0.01, dim=0)
    action_q99 = torch.quantile(all_action_data.float(), 0.99, dim=0)
    # Guard against near-constant columns (e.g. done flag with q01==q99==0).
    # Without this the normalizer falls back to eps=1e-8 denominator and the
    # single non-zero value becomes ~1e8 in normalized space (audit-doc bug #13).
    _arange = action_q99 - action_q01
    action_q99 = action_q01 + torch.clamp(_arange, min=1e-2)

    qpos_min = all_qpos_data.min(dim=0)[0]
    qpos_max = all_qpos_data.max(dim=0)[0]
    qpos_mean = all_qpos_data.mean(dim=0)
    qpos_std = all_qpos_data.std(dim=0)
    qpos_std = torch.clip(qpos_std, 1e-2, np.inf)
    qpos_q01 = torch.quantile(all_qpos_data.float(), 0.01, dim=0)
    qpos_q99 = torch.quantile(all_qpos_data.float(), 0.99, dim=0)
    _qrange = qpos_q99 - qpos_q01
    qpos_q99 = qpos_q01 + torch.clamp(_qrange, min=1e-2)

    stats = {
        "action": {
            "min": action_min.numpy(),
            "max": action_max.numpy(),
            "mean": action_mean.numpy().squeeze(),
            "std": action_std.numpy().squeeze(),
            "q01": action_q01.numpy().squeeze(),
            "q99": action_q99.numpy().squeeze(),
            "count": np.array([cnt]),
        },
        "observation.state": {
            "min": qpos_min.numpy(),
            "max": qpos_max.numpy(),
            "mean": qpos_mean.numpy().squeeze(),
            "std": qpos_std.numpy().squeeze(),
            "q01": qpos_q01.numpy().squeeze(),
            "q99": qpos_q99.numpy().squeeze(),
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
            # DexUMI: chunk 의 미래 N 개 eepos 를 현재 eepos local frame 으로 변환.
            # eepos 없으면 (옛 dataset) legacy naive cumsum fallback.
            eepos_data = ep.get("eepos_data")
            if eepos_data is not None and len(eepos_data) > start_ts:
                future_idx = np.arange(start_ts + 1, start_ts + self.chunk_size + 1)
                future_idx = np.clip(future_idx, 0, episode_len - 1)
                future_eepos = eepos_data[future_idx]
                current_eepos = eepos_data[start_ts]
                rel_traj = compute_relative_trajectory_in_local_frame(future_eepos, current_eepos)
                padded_action[:action_len, :rel_traj.shape[1]] = rel_traj[:action_len]
                # any_has_succeed 면 action 의 마지막 컬럼이 succeed flag — rel_traj
                # 는 eepos 차원만 다루므로 succeed slot 을 따로 복사해야 모델이 그
                # dim 을 학습할 수 있다. (get_norm_stats 와 동일 layout).
                if any_has_succeed and action.shape[1] > rel_traj.shape[1]:
                    padded_action[:action_len, rel_traj.shape[1]:] = action[:action_len, rel_traj.shape[1]:]
            else:
                padded_action[:action_len] = delta_to_relative_trajectory(action[:action_len])
        elif _normalize_action_key(self.action_key) == 'relative_joint_pos':
            # chunk-anchored joint delta: action[t in chunk] - qpos[chunk_start].
            # anchor를 ep["state_data"]에서 직접 읽음 (training_server에서 state_data
            # 가 곧 qpos이므로 anchor source로 직접 사용). obs_state_keys와 별개 채널.
            # absolute dims (gripper/tool/done)는 dataset features에서 자동 추출.
            qpos_anchor_data = ep.get("state_data")
            chunk_slice = action[:action_len]
            if qpos_anchor_data is not None and len(qpos_anchor_data) > start_ts:
                if qpos_anchor_data.ndim == 1:
                    qpos_anchor_data = qpos_anchor_data.reshape(-1, 1)
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
        # PI05/PaliGemma pretrained weights expect [-1, 1] (audit-doc bug #1).
        # Feeding [0, 1] into pi05_base shifts the SigLIP feature distribution and
        # wrecks pretrained visual grounding; LoRA can't fully recover.
        _pixel_range = '-11' if self.policy_type == 'PI05' else '01'
        for sensor_id in self.sensor_ids:
            processed_images = []
            # Multi-view: sensor_id 가 view_key ("5" or "5_2") 일 수 있음. wrist
            # 판정은 항상 물리 sensor_id 기준.
            # Wrist cams: ColorJitter only (preserve spatial geometry).
            # Other cams: full aug (RandomResizedCrop + RandomRotation + ColorJitter).
            _vk = str(sensor_id)
            _phys_sid_str = _vk.split('_', 1)[0]
            try:
                _phys_sid = int(_phys_sid_str)
            except ValueError:
                _phys_sid = None
            if _phys_sid is not None and _phys_sid in getattr(self, 'wrist_sensor_ids', set()):
                aug = getattr(self, '_image_augment_color_only', None)
            else:
                aug = getattr(self, '_image_augment_full', None)
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                if aug is not None:
                    image = aug(image)
                image = process_image(image, self.vision_backbone, pixel_range=_pixel_range, image_resolution=self.image_resolution)
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


def load_data(dataset_dir, policy_type, num_episodes, sensor_ids, batch_size_train, batch_size_val, chunk_size, vision_backbone='resnet18', num_workers=1, n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None, wrist_sensor_ids=None, image_resolution=None):
    if obs_state_keys is None:
        obs_state_keys = ['qpos']
    print(f'\nData from: {dataset_dir}\n')

    # Surface wrist_sensor_ids decision early so users can verify wire-through
    # (audit-doc §7 checklist #1). Empty set = all cams get full spatial aug.
    _wrist_set = set(int(x) for x in (wrist_sensor_ids or []))
    if _wrist_set:
        print(f'[CONFIG] wrist_sensor_ids: {sorted(_wrist_set)} (skip spatial aug)', flush=True)
    else:
        print('[CONFIG] wrist_sensor_ids: [] (all cams get full crop+rotate aug)', flush=True)

    # obtain train test split
    train_ratio = 0.8
    shuffled_indices = np.random.permutation(num_episodes)
    split = max(1, int(train_ratio * num_episodes))
    train_indices = shuffled_indices[:split]
    # 에피소드가 적으면 train 데이터를 val에도 재사용
    val_indices = shuffled_indices[split:] if split < num_episodes else shuffled_indices

    # obtain normalization stats for qpos and action.
    # action_key='relative_joint_pos'면 raw action이 아니라 chunk-anchored delta
    # 분포로 q01/q99/mean/std를 잡음. absolute dims는 features에서 자동 검출.
    norm_stats, skipped_episodes = get_norm_stats(
        dataset_dir, num_episodes,
        action_key=action_key,
        use_relative_trajectory=use_relative_trajectory,
        obs_state_keys=obs_state_keys,
        chunk_size=chunk_size,
    )

    # 호환 불가능한 에피소드 제외
    if skipped_episodes:
        valid_mask = ~np.isin(shuffled_indices, skipped_episodes)
        valid_indices = shuffled_indices[valid_mask]
        split = max(1, int(train_ratio * len(valid_indices)))
        train_indices = valid_indices[:split]
        val_indices = valid_indices[split:] if split < len(valid_indices) else valid_indices

    # construct dataset and dataloader.
    # train: augment=True (RandomResizedCrop+Rotate+ColorJitter), wrist cams get
    # ColorJitter-only (audit-doc bug #31). val/eval: augment=False.
    train_dataset = EpisodicDataset(train_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, augment=True, wrist_sensor_ids=_wrist_set, image_resolution=image_resolution)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys, augment=False, wrist_sensor_ids=_wrist_set, image_resolution=image_resolution)

    # Per-episode sampling weights sidecar (written by the backend when
    # multiple datasets with non-uniform weights are merged). Absent file ⇒
    # uniform sampling (legacy path).
    train_sampler = None
    weights_path = os.path.join(dataset_dir, 'meta', 'episode_sample_weights.json')
    if os.path.exists(weights_path):
        try:
            with open(weights_path, 'r') as f:
                ep_weights_all = json.load(f)
            # Align with train_indices (post split + skipped_episodes filter).
            # Each train_indices[i] is the original episode index in the merged
            # dataset, which matches the position in ep_weights_all.
            per_sample = [float(ep_weights_all[int(ei)]) for ei in train_indices]
            if any(w <= 0 for w in per_sample):
                raise ValueError('non-positive weight found')
            train_sampler = WeightedRandomSampler(
                weights=per_sample,
                num_samples=len(per_sample),
                replacement=True,
            )
            uniq = sorted(set(round(w, 4) for w in per_sample))
            print(f'[CONFIG] WeightedRandomSampler enabled — {len(per_sample)} train episodes, '
                  f'unique weights={uniq}', flush=True)
        except Exception as e:
            print(f'[CONFIG][WARN] failed to load episode_sample_weights.json '
                  f'({weights_path}): {e} — falling back to uniform sampling.', flush=True)
            train_sampler = None

    # Keep workers alive across epochs so EpisodicDataset._ep_cache (and OS page
    # cache backing the mmap'd frame .npy files) survives between epochs.
    common_loader_kwargs = dict(
        pin_memory=True,
        num_workers=num_workers,
    )
    if num_workers > 0:
        common_loader_kwargs["persistent_workers"] = True
        common_loader_kwargs["prefetch_factor"] = 4
    if train_sampler is not None:
        # sampler is mutually exclusive with shuffle in DataLoader.
        train_dataloader = DataLoader(train_dataset, batch_size=batch_size_train,
                                      sampler=train_sampler, **common_loader_kwargs)
    else:
        train_dataloader = DataLoader(train_dataset, batch_size=batch_size_train,
                                      shuffle=True, **common_loader_kwargs)
    # Validation always samples uniformly so the reported loss reflects the
    # natural episode distribution — weights only bias training optimization.
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size_val,
                                shuffle=True, **common_loader_kwargs)

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


def make_policy(ckpt_path, seed, learning_rate, lr_backbone, policy_obj, task, robot, sensors, gripper=None):
    args_override = policy_obj['settings']
    if policy_obj['type'] == 'ACT':
        args_override['ckpt_dir'] = ckpt_path
        args_override['policy_class'] = policy_obj['type']
        args_override['task_name'] = task['name']
        args_override['seed'] = seed
        args_override['state_dim'] = robot['joint_dim']
        if gripper is not None:
            args_override['state_dim'] += 1 # gripper state dim
        args_override['num_queries'] = int(policy_obj['settings']['chunk_size'])
        
        args_override['learning_rate'] = learning_rate
        args_override['lr_backbone'] = lr_backbone
        # args_override[''] = int(policy_obj['settings']['lr_backbone'])
        
        
        sensor_names = [sensor['name'] for sensor in sensors]
        args_override['camera_names'] = sensor_names

        from .policies import ACTPolicy
        policy = ACTPolicy(args_override)
    else:
        raise NotImplementedError
    return policy


def make_optimizer(policy_class, policy):
    if policy_class == 'ACT':
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer

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
    # Bridge language_instruction → task for PI05 (audit-doc bug #8). The PI05
    # preprocessor pipeline expects `complementary_data['task']` (a list of strings).
    # Datasets / dataloaders may carry it as `language_instruction` instead.
    is_pi05 = hasattr(policy, 'config') and hasattr(policy.config, 'tokenizer_max_length')
    if is_pi05 and 'task' not in data:
        lang = data.get('language_instruction', '')
        if isinstance(lang, str):
            # Determine batch size to build a placeholder list of the right length.
            bsz = None
            for v in data.values():
                if isinstance(v, torch.Tensor) and v.dim() >= 1:
                    bsz = v.shape[0]
                    break
            data['task'] = [lang] * (bsz or 1)
        elif isinstance(lang, (list, tuple)):
            data['task'] = list(lang)
        else:
            data['task'] = ['']
    # Apply input preprocessor (Normalize state/action/images per cfg.normalization_mapping)
    # so the model trains on normalized targets and the gradient is balanced across joints.
    # The PI05 preprocessor pipeline includes Pi05PrepareStateTokenizerProcessorStep +
    # TokenizerProcessorStep which write `observation.language.tokens` correctly using
    # QUANTILE-normalized state and native state_dim (audit-doc bug #23 fix).
    if preprocessor is not None:
        data = preprocessor(data)
    elif is_pi05:
        # Fallback for the rare case where preprocessor failed to load (e.g. corrupted
        # checkpoint json). Use the legacy `prepare_pi05_language_tokens` to produce
        # *some* language tokens — divergent from training (min-max norm, padded state)
        # but keeps the model from crashing. Normal flow always has preprocessor.
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