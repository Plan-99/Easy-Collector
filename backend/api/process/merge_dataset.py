import os
import shutil
from ...configs.global_configs import DATASET_DIR
from .lerobot_io import (
    list_episodes, read_episode, get_dataset_info, create_dataset,
    _read_json, _write_json, _read_jsonl, _write_jsonl, _append_jsonl,
    PARQUET_PATH_TEMPLATE, IMAGE_PATH_TEMPLATE, INFO_PATH,
    EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH, DEFAULT_CHUNK_SIZE,
)
from lerobot.datasets.utils import DEFAULT_IMAGE_PATH
import numpy as np
import pyarrow.parquet as pq


def merge_dataset(source_dataset, target_datasets):
    """Merges episodes from target datasets into a source dataset (LeRobot format)."""
    source_path = os.path.join(DATASET_DIR, str(source_dataset.id))
    if not os.path.isdir(source_path):
        os.makedirs(source_path)

    print(f"Merging datasets into source dataset: {source_dataset.name} (ID: {source_dataset.id})")

    source_info = get_dataset_info(source_path)
    if source_info is None:
        print(f"[ERROR] Source dataset has no info.json: {source_path}")
        return

    for target_dataset in target_datasets:
        if target_dataset.id == source_dataset.id:
            continue

        target_path = os.path.join(DATASET_DIR, str(target_dataset.id))
        print(f"Processing target dataset: {target_dataset.name} (ID: {target_dataset.id})")

        if not os.path.isdir(target_path):
            print(f"  - Warning: Target directory {target_path} not found. Skipping.")
            continue

        target_info = get_dataset_info(target_path)
        if target_info is None:
            print(f"  - Warning: No info.json in {target_path}. Skipping.")
            continue

        target_episodes = list_episodes(target_path)
        print(f"  - Found {len(target_episodes)} episodes to merge.")

        # Copy tasks from target to source
        src_tasks = _read_jsonl(os.path.join(source_path, TASKS_PATH))
        tgt_tasks = _read_jsonl(os.path.join(target_path, TASKS_PATH))
        existing_task_strs = {t["task"] for t in src_tasks}
        for tgt_task in tgt_tasks:
            if tgt_task["task"] not in existing_task_strs:
                new_idx = len(src_tasks)
                src_tasks.append({"task_index": new_idx, "task": tgt_task["task"]})
                existing_task_strs.add(tgt_task["task"])
        _write_jsonl(src_tasks, os.path.join(source_path, TASKS_PATH))

        # Build task index mapping (target -> source)
        tgt_task_map = {}
        for tgt_task in tgt_tasks:
            for src_task in src_tasks:
                if src_task["task"] == tgt_task["task"]:
                    tgt_task_map[tgt_task["task_index"]] = src_task["task_index"]
                    break

        for ep_entry in target_episodes:
            ep_idx = ep_entry["episode_index"]
            tgt_chunk = ep_idx // target_info.get("chunks_size", DEFAULT_CHUNK_SIZE)

            # Re-read source info for current counters
            source_info = _read_json(os.path.join(source_path, INFO_PATH))
            new_ep_idx = source_info["total_episodes"]
            new_chunk = new_ep_idx // source_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
            global_start = source_info["total_frames"]
            num_frames = ep_entry.get("length", 0)

            # Copy parquet file as-is (preserves HF embedded images + schema)
            tgt_parquet = os.path.join(target_path, PARQUET_PATH_TEMPLATE.format(chunk=tgt_chunk, ep=ep_idx))
            if not os.path.exists(tgt_parquet):
                print(f"    - Parquet not found for episode {ep_idx}, skipping.")
                continue
            new_parquet = os.path.join(source_path, PARQUET_PATH_TEMPLATE.format(chunk=new_chunk, ep=new_ep_idx))
            os.makedirs(os.path.dirname(new_parquet), exist_ok=True)
            shutil.copy2(tgt_parquet, new_parquet)

            # Copy image files on disk (for image-mode datasets)
            features = source_info.get("features", {})
            for key, feat in features.items():
                if feat.get("dtype") != "image" or not key.startswith("observation.images."):
                    continue
                old_img_dir = os.path.join(target_path, "images", key, f"episode_{ep_idx:06d}")
                new_img_dir = os.path.join(source_path, "images", key, f"episode_{new_ep_idx:06d}")
                if os.path.isdir(old_img_dir):
                    if os.path.isdir(new_img_dir):
                        shutil.rmtree(new_img_dir)
                    shutil.copytree(old_img_dir, new_img_dir)

            # Copy video files on disk (for video-mode datasets)
            for key, feat in features.items():
                if feat.get("dtype") != "video" or not key.startswith("observation.images."):
                    continue
                old_vid = os.path.join(target_path, "videos", f"chunk-{tgt_chunk:03d}", key, f"episode_{ep_idx:06d}.mp4")
                new_vid = os.path.join(source_path, "videos", f"chunk-{new_chunk:03d}", key, f"episode_{new_ep_idx:06d}.mp4")
                if os.path.isfile(old_vid):
                    os.makedirs(os.path.dirname(new_vid), exist_ok=True)
                    shutil.copy2(old_vid, new_vid)

            # Copy episode stats
            tgt_stats = _read_jsonl(os.path.join(target_path, EPISODES_STATS_PATH))
            ep_stats = {}
            for s in tgt_stats:
                if s.get("episode_index") == ep_idx:
                    ep_stats = s.get("stats", {})
                    break

            # Update source metadata
            source_info["total_episodes"] = new_ep_idx + 1
            source_info["total_frames"] = global_start + num_frames
            source_info["total_chunks"] = new_chunk + 1
            source_info["total_tasks"] = len(src_tasks)
            source_info["splits"] = {"train": f"0:{new_ep_idx + 1}"}
            _write_json(source_info, os.path.join(source_path, INFO_PATH))

            _append_jsonl(
                {"episode_index": new_ep_idx, "length": num_frames, "tasks": ep_entry.get("tasks", [""])},
                os.path.join(source_path, EPISODES_PATH),
            )
            _append_jsonl(
                {"episode_index": new_ep_idx, "stats": ep_stats},
                os.path.join(source_path, EPISODES_STATS_PATH),
            )

            print(f"    - Merged episode {ep_idx} -> {new_ep_idx}")

        # Delete target dataset
        print(f"  - Deleting dataset record for: {target_dataset.name} (ID: {target_dataset.id})")
        target_dataset.delete()

        if os.path.exists(target_path):
            shutil.rmtree(target_path)

    print("Dataset merge process completed.")
