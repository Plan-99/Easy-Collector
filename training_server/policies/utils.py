from einops import rearrange
import cv2
import numpy as np
import torch
import os
import re
import shutil
from torch.utils.data import TensorDataset, DataLoader
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

    # Read states and actions from parquet
    state_data = np.array(df["observation.state"].tolist(), dtype=np.float32)
    action_data = np.array(df["action"].tolist(), dtype=np.float32)

    # Parse robot-specific data from concatenated state/action
    features = info["features"]
    state_names = features["observation.state"]["names"]
    action_names = features["action"]["names"]

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

    for key, feat in features.items():
        if key.startswith("observation.images."):
            sensors.append(key.replace("observation.images.", ""))
        elif key == "observation.state":
            # names can be:
            #   - "robot_{id}_{joint_name}" (EasyTrainer format)
            #   - "joint_1", "joint_2", ... (convert_hdf5_package format, no robot prefix)
            has_robot_prefix = False
            for name in feat.get("names", []):
                m = re.match(r'^(robot_\d+)_', name)
                if m:
                    robots.add(m.group(1))
                    has_robot_prefix = True
            # If no robot_ prefix found, treat as single-robot dataset
            # Robot name is unknown, just report dimension info
            if not has_robot_prefix and feat.get("names"):
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


def relative_trajectory_to_delta(waypoints: np.ndarray) -> np.ndarray:
    """relative trajectory → sequential delta 역변환 (inference 시 사용).

    waypoints: [T, D] relative trajectory. 앞 6차원은 (T_now→t+i),
               7차원 이후는 tool joint (absolute 값)으로 변환 없이 그대로 유지.
    반환값: [T, D] sequential deltas + tool
    """
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




class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone='resnet18', n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None):
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
        self.info = None

        # Pre-load episode metadata (lengths) for efficient sampling
        self._ep_cache = {}
        self._dataset_info = get_dataset_info(dataset_dir)

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

        state_data = np.array(df["observation.state"].tolist(), dtype=np.float32)
        action_data = np.array(df["action"].tolist(), dtype=np.float32)

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
        if self.action_key == 'relative_ee_pos' or (self.use_relative_trajectory and self.action_key == 'ee_delta_action'):
            padded_action[:action_len] = delta_to_relative_trajectory(action[:action_len])
        else:
            padded_action[:action_len] = action
        is_pad = np.zeros(self.chunk_size)
        is_pad[action_len:] = 1

        item = dict()
        item['language_instruction'] = language_instruction
        for sensor_id in self.sensor_ids:
            processed_images = []
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                image = process_image(image)
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



def process_image(image, vision_backbone='resnet18', to_cuda=False):
    if not isinstance(image, Image.Image):
        image = Image.fromarray(np.array(image))
    if vision_backbone not in VISION_BACKBONE_MAP:
        image_transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
        ])
        image = image_transform(image)
    else:
        image_processor = AutoImageProcessor.from_pretrained(VISION_BACKBONE_MAP[vision_backbone])
        image = image_processor(image)['pixel_values'][0]  # Assuming the image is a PIL Image or numpy array

    return image.cuda() if to_cuda else image  # Add batch dimension


def get_norm_stats(dataset_dir, num_episodes, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None):
    """Compute normalization stats from LeRobot dataset parquet files."""
    if obs_state_keys is None:
        obs_state_keys = ['qpos']

    dataset_info = get_dataset_info(dataset_dir)
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

        state_data = np.array(df["observation.state"].tolist(), dtype=np.float32)
        action_data = np.array(df["action"].tolist(), dtype=np.float32)

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

        qpos = np.array(df["observation.state"].tolist(), dtype=np.float32)
        action = np.array(df["action"].tolist(), dtype=np.float32)

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

        if action_key == 'relative_ee_pos' or (use_relative_trajectory and action_key == 'ee_delta_action'):
            action = delta_to_relative_trajectory(action)

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

    qpos_min = all_qpos_data.min(dim=0)[0]
    qpos_max = all_qpos_data.max(dim=0)[0]
    qpos_mean = all_qpos_data.mean(dim=0)
    qpos_std = all_qpos_data.std(dim=0)
    qpos_std = torch.clip(qpos_std, 1e-2, np.inf)

    stats = {
        "action": {
            "min": action_min.numpy(),
            "max": action_max.numpy(),
            "mean": action_mean.numpy().squeeze(),
            "std": action_std.numpy().squeeze(),
            "count": np.array([cnt]),
        },
        "observation.state": {
            "min": qpos_min.numpy(),
            "max": qpos_max.numpy(),
            "mean": qpos_mean.numpy().squeeze(),
            "std": qpos_std.numpy().squeeze(),
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
        if self.action_key == 'relative_ee_pos' or (self.use_relative_trajectory and self.action_key == 'ee_delta_action'):
            padded_action[:action_len] = delta_to_relative_trajectory(action[:action_len])
        else:
            padded_action[:action_len] = action
        is_pad = np.zeros(self.chunk_size)
        is_pad[action_len:] = 1

        item = dict()
        item['language_instruction'] = language_instruction
        for sensor_id in self.sensor_ids:
            processed_images = []
            for image in image_dict[f"sensor_{sensor_id}"]:
                image = Image.fromarray(np.array(image))
                image = process_image(image)
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


def load_data(dataset_dir, policy_type, num_episodes, sensor_ids, batch_size_train, batch_size_val, chunk_size, vision_backbone='resnet18', num_workers=1, n_obs_steps=1, action_key='qaction', use_relative_trajectory=False, obs_state_keys=None):
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
    norm_stats, skipped_episodes = get_norm_stats(dataset_dir, num_episodes, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys)

    # 호환 불가능한 에피소드 제외
    if skipped_episodes:
        valid_mask = ~np.isin(shuffled_indices, skipped_episodes)
        valid_indices = shuffled_indices[valid_mask]
        split = max(1, int(train_ratio * len(valid_indices)))
        train_indices = valid_indices[:split]
        val_indices = valid_indices[split:] if split < len(valid_indices) else valid_indices

    # construct dataset and dataloader
    train_dataset = EpisodicDataset(train_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, sensor_ids, norm_stats, chunk_size, policy_type, vision_backbone, n_obs_steps=n_obs_steps, action_key=action_key, use_relative_trajectory=use_relative_trajectory, obs_state_keys=obs_state_keys)
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
    # PI05 needs tokenized language inputs
    if hasattr(policy, 'config') and hasattr(policy.config, 'tokenizer_max_length'):
        data = prepare_pi05_language_tokens(data, policy.config, norm_stats=norm_stats)
    # Apply input preprocessor (Normalize state/action/images per cfg.normalization_mapping)
    # so the model trains on normalized targets and the gradient is balanced across joints.
    if preprocessor is not None:
        data = preprocessor(data)
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