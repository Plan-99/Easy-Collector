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
