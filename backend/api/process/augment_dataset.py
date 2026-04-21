import os
from PIL import Image, ImageDraw, ImageEnhance
import cv2
import random
import numpy as np
import shutil
from .lerobot_io import (
    read_episode, list_episodes, get_dataset_info, create_dataset,
    append_episode as lerobot_append_episode, _read_json, _write_json,
    _append_jsonl, _write_jsonl, _read_jsonl,
    PARQUET_PATH_TEMPLATE, IMAGE_PATH_TEMPLATE, INFO_PATH,
    EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH, DEFAULT_CHUNK_SIZE,
)
from lerobot.datasets.feature_utils import get_hf_features_from_features
from lerobot.datasets.io_utils import embed_images
from lerobot.datasets.utils import DEFAULT_IMAGE_PATH
from lerobot.datasets.video_utils import encode_video_frames
from ...configs.global_configs import DATASET_DIR
import datasets as hf_datasets
import pyarrow.parquet as pq

def adjust_lightness(image, lightness):
    if lightness != 0:
        enhancer = ImageEnhance.Brightness(image)
        return enhancer.enhance(1 + lightness / 100)
    return image

def draw_rectangles(image, rect_params):
    if rect_params:
        draw = ImageDraw.Draw(image)
        for rect, fill in rect_params:
            draw.rectangle(rect, fill=fill)
    return image

def generate_rect_params(rectangles_config, img_width, img_height):
    rect_params = []
    count = rectangles_config.get('count', 0)
    if count > 0:
        for _ in range(count):
            rect_width = random.randint(int(img_width * 0.1), int(img_width * 0.3))
            rect_height = random.randint(int(img_height * 0.1), int(img_height * 0.3))
            x0 = random.randint(0, img_width - rect_width)
            y0 = random.randint(0, img_height - rect_height)
            x1 = x0 + rect_width
            y1 = y0 + rect_height

            if rectangles_config.get('randomColor'):
                color_rgb = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            else:
                hex_color = rectangles_config.get('color', '#000000').lstrip('#')
                color_rgb = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

            color_bgr = (color_rgb[2], color_rgb[1], color_rgb[0])
            rect_params.append(((x0, y0, x1, y1), color_bgr))
    return rect_params

def add_salt_and_pepper_noise(image, amount):
    if amount > 0:
        output = np.copy(np.array(image))
        num_salt = np.ceil(amount * output.size * 0.5)
        coords = [np.random.randint(0, i - 1, int(num_salt)) for i in output.shape]
        output[coords[0], coords[1], :] = 255

        num_pepper = np.ceil(amount * output.size * 0.5)
        coords = [np.random.randint(0, i - 1, int(num_pepper)) for i in output.shape]
        output[coords[0], coords[1], :] = 0
        return Image.fromarray(output)
    return image

def add_gaussian_noise(image, mean, sigma):
    if sigma > 0:
        img_array = np.array(image)
        gaussian_noise = np.random.normal(mean, sigma, img_array.shape)
        noisy_image_array = img_array + gaussian_noise
        noisy_image_array = np.clip(noisy_image_array, 0, 255)
        return Image.fromarray(noisy_image_array.astype('uint8'))
    return image

def generate_prospective_transform(width, height, scale_factor=0, degrees=0, shear=0, perspective=0):
    C = np.eye(3)
    C[0, 2] = -width / 2
    C[1, 2] = -height / 2

    perspective = perspective * 0.0001

    P = np.eye(3)
    P[2, 0] = random.uniform(-perspective, perspective)
    P[2, 1] = random.uniform(-perspective, perspective)

    R = np.eye(3)
    a = random.uniform(-degrees, degrees)
    s = random.uniform(1-scale_factor*0.01, 1+scale_factor*0.01)
    R[:2] = cv2.getRotationMatrix2D(angle=a, center=(0, 0), scale=s)

    S = np.eye(3)
    S[0, 1] = np.tan(random.uniform(-shear, shear) * np.pi / 180)
    S[1, 0] = np.tan(random.uniform(-shear, shear) * np.pi / 180)

    T = np.eye(3)
    T[0, 2] = width / 2
    T[1, 2] = height / 2

    M = T @ S @ R @ P @ C
    return M

def prospective_transform(image, M):
    if M is None:
        return image

    width, height = image.size
    img_np = np.array(image)
    result_np = cv2.warpPerspective(img_np, M, dsize=(width, height), borderValue=(114, 114, 114))
    return Image.fromarray(result_np)

def apply_hsv(image, h_adj, s_adj, v_adj):
    img_np = np.array(image)
    hsv = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
    h, s, v_channel = cv2.split(hsv)

    h = h.astype(np.float32)
    h_new = (h + h_adj) % 180
    h_new = h_new.astype(np.uint8)

    s = s.astype(np.float32)
    s_new = np.clip(s * s_adj, 0, 255).astype(np.uint8)

    v_channel = v_channel.astype(np.float32)
    v_new = np.clip(v_channel * v_adj, 0, 255).astype(np.uint8)

    final_hsv = cv2.merge((h_new, s_new, v_new))
    img_rgb = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2RGB)

    return Image.fromarray(img_rgb)


def augment_dataset(dataset_id, aug_dataset_id, lightness, rectangles, salt_and_pepper, gaussian, prospective, hsv, socketio_instance, task_control):
    dataset_path = os.path.join(DATASET_DIR, str(dataset_id))
    aug_dataset_path = os.path.join(DATASET_DIR, str(aug_dataset_id))

    if not os.path.exists(dataset_path):
        print(f"[ERROR] Dataset path {dataset_path} does not exist.")
        return

    # Read source dataset info
    src_info = get_dataset_info(dataset_path)
    if src_info is None:
        print(f"[ERROR] No info.json found in {dataset_path}")
        return

    # Create augmented dataset with same features
    if os.path.exists(aug_dataset_path):
        shutil.rmtree(aug_dataset_path)
    os.makedirs(aug_dataset_path, exist_ok=True)

    # Copy info.json but reset counters
    aug_info = dict(src_info)
    aug_info["total_episodes"] = 0
    aug_info["total_frames"] = 0
    aug_info["total_tasks"] = 0
    aug_info["total_chunks"] = 0
    aug_info["total_videos"] = 0
    aug_info["splits"] = {}
    _write_json(aug_info, os.path.join(aug_dataset_path, INFO_PATH))
    for path in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
        fpath = os.path.join(aug_dataset_path, path)
        os.makedirs(os.path.dirname(fpath), exist_ok=True)
        open(fpath, "w").close()

    episodes = list_episodes(dataset_path)
    features = src_info.get("features", {})
    src_fps = src_info.get("fps", 20)

    socketio_instance.emit('augmentation_progress', {'progress': 0})

    h_gain = 0.5
    s_gain = 0.7
    v_gain = 0.4

    if hsv and not hsv.get('random'):
        fixed_h_adj = hsv.get('h', 0) * 180
        fixed_s_adj = 1 + hsv.get('s', 0)
        fixed_v_adj = 1 + hsv.get('v', 0)

    total_videos = 0

    for i, ep_entry in enumerate(episodes):
        if task_control.get('stop'):
            print("Stopping Data Augmentation")
            return

        ep_idx = ep_entry["episode_index"]

        try:
            ep_data = read_episode(dataset_path, ep_idx)

            # Read the parquet to copy non-image data
            chunk = ep_idx // src_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
            src_parquet = os.path.join(dataset_path, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=ep_idx))
            table = pq.read_table(src_parquet)
            df = table.to_pandas()

            # Determine rect_params from first image
            rect_params = []
            sensor_names = sorted(ep_data["images"].keys())
            if sensor_names and ep_data["images"][sensor_names[0]]:
                first_img = ep_data["images"][sensor_names[0]][0]
                img_height, img_width = first_img.shape[:2]
                rect_params = generate_rect_params(rectangles, img_width, img_height)

            # Augment images and build new episode data
            aug_ep_idx = i  # New sequential index
            aug_chunk = aug_ep_idx // src_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
            num_frames = ep_data["num_frames"]

            # Augment and save images as temp PNGs, then encode to MP4
            for cam_name in sensor_names:
                feature_key = f"observation.images.{cam_name}"

                # Save augmented frames as temporary PNGs
                imgs_dir = os.path.join(
                    aug_dataset_path, "images", feature_key, f"episode_{aug_ep_idx:06d}"
                )
                os.makedirs(imgs_dir, exist_ok=True)

                for frame_idx in range(num_frames):
                    img_array = ep_data["images"][cam_name][frame_idx]
                    img = Image.fromarray(img_array)

                    img = adjust_lightness(img, lightness)
                    img = draw_rectangles(img, rect_params)
                    img = add_salt_and_pepper_noise(img, salt_and_pepper.get('amount', 0))
                    img = add_gaussian_noise(img, gaussian.get('mean', 0), gaussian.get('sigma', 0))

                    if prospective:
                        transform_matrix = generate_prospective_transform(
                            img.width, img.height,
                            prospective.get('scale_factor', 0),
                            prospective.get('degrees', 0),
                            prospective.get('shear', 0),
                            prospective.get('perspective', 0)
                        )
                        img = prospective_transform(img, transform_matrix)

                    if hsv:
                        if hsv.get('random'):
                            rand_h_adj = (np.random.rand() * 2 - 1) * h_gain * 180
                            rand_s_adj = (np.random.rand() * 2 - 1) * s_gain + 1
                            rand_v_adj = (np.random.rand() * 2 - 1) * v_gain + 1
                            img = apply_hsv(img, rand_h_adj, rand_s_adj, rand_v_adj)
                        else:
                            img = apply_hsv(img, fixed_h_adj, fixed_s_adj, fixed_v_adj)

                    frame_path = os.path.join(imgs_dir, f"frame_{frame_idx:06d}.png")
                    img.save(frame_path)

                # Encode PNGs to MP4
                video_path = os.path.join(
                    aug_dataset_path, "videos", f"chunk-{aug_chunk:03d}", feature_key,
                    f"episode_{aug_ep_idx:06d}.mp4"
                )
                try:
                    encode_video_frames(
                        imgs_dir=imgs_dir,
                        video_path=video_path,
                        fps=src_fps,
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

            # Clean up empty images directory
            images_root = os.path.join(aug_dataset_path, "images")
            if os.path.exists(images_root):
                try:
                    os.removedirs(images_root)
                except OSError:
                    pass

            # Build new parquet using HF datasets
            aug_info_current = _read_json(os.path.join(aug_dataset_path, INFO_PATH))
            global_start = aug_info_current["total_frames"]

            # Read source non-image data from parquet
            state_data = np.array(df["observation.state"].tolist(), dtype=np.float32)
            action_data = np.array(df["action"].tolist(), dtype=np.float32)

            episode_dict = {
                "index": np.arange(global_start, global_start + num_frames, dtype=np.int64),
                "episode_index": np.full(num_frames, aug_ep_idx, dtype=np.int64),
                "frame_index": np.arange(num_frames, dtype=np.int64),
                "timestamp": np.array(df["timestamp"].tolist(), dtype=np.float32),
                "task_index": np.array(df["task_index"].tolist(), dtype=np.int64),
                "observation.state": state_data,
                "action": action_data,
            }

            # Copy all additional fields from source parquet
            for col_name in ["observation.qvel", "observation.qeffort", "observation.eepos",
                             "observation.ee_delta", "action.ee_delta"]:
                if col_name in df.columns:
                    episode_dict[col_name] = np.array(df[col_name].tolist(), dtype=np.float32)

            if "succeed" in df.columns:
                episode_dict["succeed"] = np.array(df["succeed"].tolist(), dtype=np.float32)

            parquet_features = {k: v for k, v in features.items() if v.get("dtype") not in ("video", "image")}
            hf_features = get_hf_features_from_features(parquet_features)
            ep_dataset = hf_datasets.Dataset.from_dict(episode_dict, features=hf_features, split="train")

            aug_parquet = os.path.join(aug_dataset_path, PARQUET_PATH_TEMPLATE.format(chunk=aug_chunk, ep=aug_ep_idx))
            os.makedirs(os.path.dirname(aug_parquet), exist_ok=True)
            ep_dataset.to_parquet(aug_parquet)

            # Compute episode stats for all numerical fields
            ep_stats = {}
            stat_pairs = [("observation.state", state_data), ("action", action_data)]
            for col_name in ["observation.qvel", "observation.qeffort", "observation.eepos",
                             "observation.ee_delta", "action.ee_delta"]:
                if col_name in episode_dict:
                    stat_pairs.append((col_name, episode_dict[col_name]))

            for key, arr in stat_pairs:
                if arr.size > 0:
                    ep_stats[key] = {
                        "min": np.min(arr, axis=0).tolist(),
                        "max": np.max(arr, axis=0).tolist(),
                        "mean": np.mean(arr, axis=0).tolist(),
                        "std": np.std(arr, axis=0).tolist(),
                        "count": int(num_frames),
                    }

            # Copy task info
            src_tasks = _read_jsonl(os.path.join(dataset_path, TASKS_PATH))
            aug_tasks_path = os.path.join(aug_dataset_path, TASKS_PATH)
            if not _read_jsonl(aug_tasks_path) and src_tasks:
                _write_jsonl(src_tasks, aug_tasks_path)

            # Update metadata
            aug_info_current["total_episodes"] = aug_ep_idx + 1
            aug_info_current["total_frames"] = global_start + num_frames
            aug_info_current["total_chunks"] = aug_chunk + 1
            aug_info_current["total_tasks"] = len(src_tasks) if src_tasks else 1
            aug_info_current["total_videos"] = total_videos
            aug_info_current["splits"] = {"train": f"0:{aug_ep_idx + 1}"}
            _write_json(aug_info_current, os.path.join(aug_dataset_path, INFO_PATH))

            _append_jsonl(
                {"episode_index": aug_ep_idx, "length": num_frames, "tasks": ep_entry.get("tasks", [""])},
                os.path.join(aug_dataset_path, EPISODES_PATH),
            )
            _append_jsonl(
                {"episode_index": aug_ep_idx, "stats": ep_stats},
                os.path.join(aug_dataset_path, EPISODES_STATS_PATH),
            )

            socketio_instance.emit('augmentation_progress', {
                'progress': (i + 1) / len(episodes),
            })
        except Exception as e:
            import traceback
            print(f"[ERROR] Error processing episode {ep_idx}: {traceback.format_exc()}")

    socketio_instance.emit('augmentation_complete', {'dataset_id': aug_dataset_id})

    task_control['stop'] = True
    return
