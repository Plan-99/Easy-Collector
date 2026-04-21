"""
기존 HDF5 에피소드 데이터를 LeRobot v2.1 포맷으로 변환하는 마이그레이션 스크립트.

Usage:
    python -m src.backend.scripts.migrate_hdf5_to_lerobot --dataset-dir /root/backend/datasets

또는 특정 데이터셋만:
    python -m src.backend.scripts.migrate_hdf5_to_lerobot --dataset-dir /root/backend/datasets --dataset-ids 1 2 3
"""

import argparse
import os
import sys
import shutil
import h5py
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent.parent))

from backend.api.process.lerobot_io import (
    create_dataset, _write_json, _append_jsonl, _read_json,
    PARQUET_PATH_TEMPLATE, IMAGE_PATH_TEMPLATE, INFO_PATH,
    EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH, DEFAULT_CHUNK_SIZE,
)
from backend.configs.global_configs import DATASET_DIR
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image


def migrate_single_dataset(dataset_dir, backup=True):
    """Migrate a single HDF5 dataset directory to LeRobot format."""
    hdf5_files = sorted(
        [f for f in os.listdir(dataset_dir) if f.startswith('episode_') and f.endswith('.hdf5')],
        key=lambda x: int(x.replace('episode_', '').replace('.hdf5', ''))
    )

    if not hdf5_files:
        print(f"  No HDF5 files found in {dataset_dir}, skipping.")
        return False

    print(f"  Found {len(hdf5_files)} HDF5 episodes to migrate.")

    # Read first episode to determine features
    first_ep_path = os.path.join(dataset_dir, hdf5_files[0])
    with h5py.File(first_ep_path, 'r') as f:
        sensor_names = list(f["observations/images"].keys())
        robot_names = list(f["observations/qpos"].keys())

        # Compute dimensions
        state_dim = 0
        action_dim = 0
        state_names = []
        action_names = []
        for robot_name in robot_names:
            qpos = f[f"observations/qpos/{robot_name}"]
            robot_dim = qpos.shape[1]
            state_dim += robot_dim
            action_dim += robot_dim
            for j in range(robot_dim):
                state_names.append(f"{robot_name}_joint_{j}")
                action_names.append(f"{robot_name}_joint_{j}")

        # Image shapes
        image_shapes = {}
        for sname in sensor_names:
            img_data = f[f"observations/images/{sname}"]
            image_shapes[sname] = img_data.shape[1:]  # (H, W, C)

    # Build features
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
    }
    for sname in sensor_names:
        features[f"observation.images.{sname}"] = {
            "dtype": "image",
            "shape": list(image_shapes[sname]),
            "names": ["height", "width", "channels"],
        }

    # Create LeRobot dataset structure in a temp directory
    temp_dir = dataset_dir + "_lerobot_tmp"
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
    os.makedirs(temp_dir)

    info = {
        "codebase_version": "v2.1",
        "robot_type": "easytrainer",
        "total_episodes": 0,
        "total_frames": 0,
        "total_tasks": 0,
        "total_chunks": 0,
        "total_videos": 0,
        "chunks_size": DEFAULT_CHUNK_SIZE,
        "fps": 20,
        "splits": {},
        "data_path": PARQUET_PATH_TEMPLATE.replace("{chunk", "{episode_chunk").replace("{ep", "{episode_index"),
        "image_path": "images/{image_key}/episode_{episode_index:06d}/frame_{frame_index:06d}.png",
        "features": features,
        "action_key": "qaction",
    }
    _write_json(info, os.path.join(temp_dir, INFO_PATH))
    for p in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
        fp = os.path.join(temp_dir, p)
        os.makedirs(os.path.dirname(fp), exist_ok=True)
        open(fp, "w").close()

    # Migrate each episode
    for ep_idx, hdf5_file in enumerate(hdf5_files):
        hdf5_path = os.path.join(dataset_dir, hdf5_file)
        print(f"  Migrating {hdf5_file} -> episode_{ep_idx:06d}")

        with h5py.File(hdf5_path, 'r') as f:
            # Read data
            states_list = []
            actions_list = []
            for robot_name in robot_names:
                states_list.append(f[f"observations/qpos/{robot_name}"][:].astype(np.float32))
                actions_list.append(f[f"qaction/{robot_name}"][:].astype(np.float32))

            states = np.concatenate(states_list, axis=1)
            actions = np.concatenate(actions_list, axis=1)
            num_frames = states.shape[0]

            # Language instruction
            if "language_instruction" in f:
                lang = f["language_instruction"][()]
                if isinstance(lang, bytes):
                    lang = lang.decode('utf-8')
            else:
                lang = ""

            # Succeed
            succeed = None
            if "succeed" in f:
                succeed = f["succeed"][:].astype(np.float32)

            # Save images
            image_paths_dict = {}
            for sname in sensor_names:
                imgs = f[f"observations/images/{sname}"][:]
                paths = []
                for frame_idx in range(num_frames):
                    s_id = sname.replace("sensor_", "")
                    rel_path = IMAGE_PATH_TEMPLATE.format(sid=s_id, ep=ep_idx, frame=frame_idx)
                    abs_path = os.path.join(temp_dir, rel_path)
                    os.makedirs(os.path.dirname(abs_path), exist_ok=True)
                    Image.fromarray(imgs[frame_idx]).save(abs_path)
                    paths.append(rel_path)
                image_paths_dict[f"observation.images.{sname}"] = paths

        # Build task
        task_str = lang or ""
        tasks = []
        tasks_path = os.path.join(temp_dir, TASKS_PATH)
        with open(tasks_path) as tf:
            import json
            for line in tf:
                if line.strip():
                    tasks.append(json.loads(line))

        task_index = None
        for t in tasks:
            if t.get("task") == task_str:
                task_index = t["task_index"]
                break
        if task_index is None:
            task_index = len(tasks)
            _append_jsonl({"task_index": task_index, "task": task_str}, tasks_path)

        # Build parquet
        chunk = ep_idx // DEFAULT_CHUNK_SIZE
        info = _read_json(os.path.join(temp_dir, INFO_PATH))
        global_start = info["total_frames"]

        data = {
            "index": np.arange(global_start, global_start + num_frames, dtype=np.int64),
            "episode_index": np.full(num_frames, ep_idx, dtype=np.int64),
            "frame_index": np.arange(num_frames, dtype=np.int64),
            "timestamp": (np.arange(num_frames, dtype=np.float32) / 20.0),
            "task_index": np.full(num_frames, task_index, dtype=np.int64),
            "observation.state": states.tolist(),
            "action": actions.tolist(),
        }
        for img_key, paths in image_paths_dict.items():
            data[img_key] = paths
        if succeed is not None:
            data["succeed"] = succeed.tolist()

        table = pa.table(data)
        parquet_path = os.path.join(temp_dir, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=ep_idx))
        os.makedirs(os.path.dirname(parquet_path), exist_ok=True)
        pq.write_table(table, parquet_path)

        # Episode stats
        ep_stats = {}
        for key, arr in [("observation.state", states), ("action", actions)]:
            if arr.size > 0:
                ep_stats[key] = {
                    "min": np.min(arr, axis=0).tolist(),
                    "max": np.max(arr, axis=0).tolist(),
                    "mean": np.mean(arr, axis=0).tolist(),
                    "std": np.std(arr, axis=0).tolist(),
                    "count": int(num_frames),
                }

        # Update metadata
        info["total_episodes"] = ep_idx + 1
        info["total_frames"] = global_start + num_frames
        info["total_chunks"] = chunk + 1
        info["total_tasks"] = task_index + 1
        info["splits"] = {"train": f"0:{ep_idx + 1}"}
        _write_json(info, os.path.join(temp_dir, INFO_PATH))

        _append_jsonl(
            {"episode_index": ep_idx, "length": num_frames, "tasks": [task_str]},
            os.path.join(temp_dir, EPISODES_PATH),
        )
        _append_jsonl(
            {"episode_index": ep_idx, "stats": ep_stats},
            os.path.join(temp_dir, EPISODES_STATS_PATH),
        )

    # Swap directories
    if backup:
        backup_dir = dataset_dir + "_hdf5_backup"
        print(f"  Backing up original to {backup_dir}")
        if os.path.exists(backup_dir):
            shutil.rmtree(backup_dir)
        shutil.move(dataset_dir, backup_dir)
    else:
        shutil.rmtree(dataset_dir)

    shutil.move(temp_dir, dataset_dir)
    print(f"  Migration complete: {len(hdf5_files)} episodes converted.")
    return True


def main():
    parser = argparse.ArgumentParser(description="Migrate HDF5 episodes to LeRobot format")
    parser.add_argument("--dataset-dir", default=DATASET_DIR,
                        help="Root directory containing dataset folders")
    parser.add_argument("--dataset-ids", nargs="*", type=str, default=None,
                        help="Specific dataset IDs to migrate (default: all)")
    parser.add_argument("--no-backup", action="store_true",
                        help="Delete original HDF5 files instead of backing up")
    args = parser.parse_args()

    root_dir = args.dataset_dir
    if not os.path.exists(root_dir):
        print(f"Dataset directory not found: {root_dir}")
        sys.exit(1)

    # Find dataset directories
    if args.dataset_ids:
        ds_dirs = [os.path.join(root_dir, ds_id) for ds_id in args.dataset_ids]
    else:
        ds_dirs = [
            os.path.join(root_dir, d)
            for d in os.listdir(root_dir)
            if os.path.isdir(os.path.join(root_dir, d)) and not d.startswith('tmp')
        ]

    print(f"Found {len(ds_dirs)} dataset directories to check.")

    migrated = 0
    for ds_dir in sorted(ds_dirs):
        ds_name = os.path.basename(ds_dir)

        # Check if already migrated (has meta/info.json)
        if os.path.exists(os.path.join(ds_dir, "meta", "info.json")):
            print(f"[{ds_name}] Already in LeRobot format, skipping.")
            continue

        # Check if has HDF5 files
        has_hdf5 = any(f.endswith('.hdf5') for f in os.listdir(ds_dir))
        if not has_hdf5:
            print(f"[{ds_name}] No HDF5 files found, skipping.")
            continue

        print(f"[{ds_name}] Migrating...")
        try:
            if migrate_single_dataset(ds_dir, backup=not args.no_backup):
                migrated += 1
        except Exception as e:
            import traceback
            print(f"[{ds_name}] ERROR: {traceback.format_exc()}")

    print(f"\nDone. Migrated {migrated} datasets.")


if __name__ == "__main__":
    main()
