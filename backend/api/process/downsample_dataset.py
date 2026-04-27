"""
Downsample a LeRobot dataset by keeping N out of every M steps per episode.

Creates a new dataset with the downsampled episodes.
"""

import os
import shutil

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image

from ...utils.lerobot_io import (
    read_episode, list_episodes, get_dataset_info,
    _read_json, _write_json, _read_jsonl, _write_jsonl, _append_jsonl,
    PARQUET_PATH_TEMPLATE, INFO_PATH,
    EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH, DEFAULT_CHUNK_SIZE,
)
from lerobot.datasets.video_utils import encode_video_frames
from ...configs.global_configs import DATASET_DIR


def downsample_dataset(dataset_id, new_dataset_id, keep, every,
                       socketio_instance, task_control):
    """Downsample dataset by keeping `keep` frames out of every `every` frames.

    For each episode, frames are grouped into blocks of `every` frames,
    and only the first `keep` frames of each block are retained.
    The result is written into new_dataset_id as a new dataset.
    """
    dataset_path = os.path.join(DATASET_DIR, str(dataset_id))
    new_dataset_path = os.path.join(DATASET_DIR, str(new_dataset_id))

    if not os.path.exists(dataset_path):
        print(f"[ERROR] Dataset path {dataset_path} does not exist.")
        return

    src_info = get_dataset_info(dataset_path)
    if src_info is None:
        print(f"[ERROR] No info.json found in {dataset_path}")
        return

    # Create new dataset directory with same structure
    if os.path.exists(new_dataset_path):
        shutil.rmtree(new_dataset_path)
    os.makedirs(new_dataset_path, exist_ok=True)

    # Copy info.json but reset counters
    new_info = dict(src_info)
    new_info["total_episodes"] = 0
    new_info["total_frames"] = 0
    new_info["total_tasks"] = 0
    new_info["total_chunks"] = 0
    new_info["total_videos"] = 0
    new_info["splits"] = {}
    _write_json(new_info, os.path.join(new_dataset_path, INFO_PATH))
    for path in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
        fpath = os.path.join(new_dataset_path, path)
        os.makedirs(os.path.dirname(fpath), exist_ok=True)
        open(fpath, "w").close()

    episodes = list_episodes(dataset_path)
    features = src_info.get("features", {})
    src_fps = src_info.get("fps", 20)

    # Keep original fps — downsampling reduces frame count, not playback speed
    new_fps = src_fps

    socketio_instance.emit('downsample_progress', {'progress': 0})

    total_videos = 0
    written_ep_count = 0

    for i, ep_entry in enumerate(episodes):
        if task_control.get('stop'):
            print("Stopping Downsample")
            return

        ep_idx = ep_entry["episode_index"]

        try:
            # Read source parquet
            chunk = ep_idx // src_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
            src_parquet = os.path.join(
                dataset_path, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=ep_idx)
            )
            table = pq.read_table(src_parquet)
            num_frames = table.num_rows

            # Build keep indices: for every block of `every` frames, keep first `keep`
            keep_indices = [idx for idx in range(num_frames) if (idx % every) < keep]
            new_num_frames = len(keep_indices)
            if new_num_frames == 0:
                print(f"[WARN] Episode {ep_idx} has 0 frames after downsample, skipping")
                continue

            # New episode index (sequential, no gaps)
            new_ep_idx = written_ep_count
            new_chunk = new_ep_idx // src_info.get("chunks_size", DEFAULT_CHUNK_SIZE)

            # Read video frames for this episode
            ep_data = read_episode(dataset_path, ep_idx)

            # Process video/image data
            sensor_names = sorted(ep_data["images"].keys())
            for cam_name in sensor_names:
                feature_key = f"observation.images.{cam_name}"

                # Save selected frames as temporary PNGs
                # NOTE: encode_video_frames expects "frame-NNNNNN.png" (hyphen, not underscore)
                imgs_dir = os.path.join(
                    new_dataset_path, "images", feature_key, f"episode_{new_ep_idx:06d}"
                )
                os.makedirs(imgs_dir, exist_ok=True)

                for new_frame_idx, orig_frame_idx in enumerate(keep_indices):
                    img_array = ep_data["images"][cam_name][orig_frame_idx]
                    img = Image.fromarray(img_array)
                    frame_path = os.path.join(imgs_dir, f"frame-{new_frame_idx:06d}.png")
                    img.save(frame_path)

                # Encode PNGs to MP4
                video_path = os.path.join(
                    new_dataset_path, "videos", f"chunk-{new_chunk:03d}", feature_key,
                    f"episode_{new_ep_idx:06d}.mp4"
                )
                try:
                    encode_video_frames(
                        imgs_dir=imgs_dir,
                        video_path=video_path,
                        fps=new_fps,
                        vcodec="h264",
                        pix_fmt="yuv420p",
                        g=2,
                        crf=30,
                        overwrite=True,
                    )
                    total_videos += 1
                except Exception as e:
                    print(f"[WARN] Video encoding failed for {feature_key}: {e}")

                # Clean up temporary PNGs
                shutil.rmtree(imgs_dir)

            # Clean up empty images directory tree
            images_root = os.path.join(new_dataset_path, "images")
            if os.path.exists(images_root):
                shutil.rmtree(images_root, ignore_errors=True)

            # ── Write parquet using pyarrow directly (avoids HF datasets type issues) ──
            new_info_current = _read_json(os.path.join(new_dataset_path, INFO_PATH))
            global_start = new_info_current["total_frames"]

            # Select rows from source table
            selected_table = table.take(keep_indices)
            df = selected_table.to_pandas()

            # Rebuild metadata columns with correct values
            arrays = {}
            for col in df.columns:
                if col == "index":
                    arrays[col] = pa.array(
                        np.arange(global_start, global_start + new_num_frames, dtype=np.int64)
                    )
                elif col == "episode_index":
                    arrays[col] = pa.array(
                        np.full(new_num_frames, new_ep_idx, dtype=np.int64)
                    )
                elif col == "frame_index":
                    arrays[col] = pa.array(
                        np.arange(new_num_frames, dtype=np.int64)
                    )
                elif col == "timestamp":
                    arrays[col] = pa.array(
                        (np.arange(new_num_frames, dtype=np.float32) / new_fps).tolist(),
                        type=pa.float32(),
                    )
                else:
                    # Keep original data (state, action, qvel, etc.) — convert via lists
                    vals = df[col].tolist()
                    if isinstance(vals[0], np.ndarray):
                        arrays[col] = pa.array([v.tolist() for v in vals])
                    else:
                        arrays[col] = pa.array(vals)

            new_table = pa.table(arrays)
            new_parquet = os.path.join(
                new_dataset_path,
                PARQUET_PATH_TEMPLATE.format(chunk=new_chunk, ep=new_ep_idx),
            )
            os.makedirs(os.path.dirname(new_parquet), exist_ok=True)
            pq.write_table(new_table, new_parquet)

            # ── Compute episode stats ──
            ep_stats = {}
            stat_columns = [
                "observation.state", "action",
                "observation.qvel", "observation.qeffort",
                "observation.eepos", "observation.ee_delta", "action.ee_delta",
            ]
            for col_name in stat_columns:
                if col_name not in df.columns:
                    continue
                arr = np.array(df[col_name].tolist(), dtype=np.float32)
                if arr.size > 0:
                    ep_stats[col_name] = {
                        "min": np.min(arr, axis=0).tolist(),
                        "max": np.max(arr, axis=0).tolist(),
                        "mean": np.mean(arr, axis=0).tolist(),
                        "std": np.std(arr, axis=0).tolist(),
                        "count": int(new_num_frames),
                    }

            # Copy tasks (once)
            src_tasks = _read_jsonl(os.path.join(dataset_path, TASKS_PATH))
            dst_tasks_path = os.path.join(new_dataset_path, TASKS_PATH)
            if not _read_jsonl(dst_tasks_path) and src_tasks:
                _write_jsonl(src_tasks, dst_tasks_path)

            # Update metadata
            new_info_current["total_episodes"] = new_ep_idx + 1
            new_info_current["total_frames"] = global_start + new_num_frames
            new_info_current["total_chunks"] = new_chunk + 1
            new_info_current["total_tasks"] = len(src_tasks) if src_tasks else 1
            new_info_current["total_videos"] = total_videos
            new_info_current["splits"] = {"train": f"0:{new_ep_idx + 1}"}
            new_info_current["fps"] = new_fps
            _write_json(new_info_current, os.path.join(new_dataset_path, INFO_PATH))

            _append_jsonl(
                {"episode_index": new_ep_idx, "length": new_num_frames,
                 "tasks": ep_entry.get("tasks", [""])},
                os.path.join(new_dataset_path, EPISODES_PATH),
            )
            _append_jsonl(
                {"episode_index": new_ep_idx, "stats": ep_stats},
                os.path.join(new_dataset_path, EPISODES_STATS_PATH),
            )

            written_ep_count += 1

            socketio_instance.emit('downsample_progress', {
                'progress': (i + 1) / len(episodes),
            })

        except Exception as e:
            import traceback
            print(f"[ERROR] Failed to downsample episode {ep_idx}: {e}")
            traceback.print_exc()
            continue

    socketio_instance.emit('downsample_complete', {
        'dataset_id': new_dataset_id,
    })
    print(f"[INFO] Downsample complete: {len(episodes)} episodes ({written_ep_count} written) -> {new_dataset_path}")
