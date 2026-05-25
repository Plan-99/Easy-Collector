"""
Unified dataset transform pipeline.

Replaces the previously separate augment / downsample / (would-be) crop / rotate
endpoints. A single pass over each source episode applies, in order:

    augment (repeated `repeat` times) → downsample → crop → rotate

Each operation is optional — pass ``None`` to skip. The output is a single new
dataset with up to ``repeat × source_episodes`` episodes.

Operations schema (request body of POST /dataset/<id>/transform):
    {
      "name": "<output dataset name>",
      "task_id": <workspace id>,
      "operations": {
        "repeat": 1,                       # augment 반복 횟수 (default 1)
        "augmentation": {                  # null 이면 skip
          "lightness": 0,                  # -100..100
          "hsv": {"h":0,"s":0,"v":0,"random":false},
          "rectangles": {"count":0,"randomColor":false,"color":"#000000"},
          "saltAndPepper": {"amount":0},
          "gaussian": {"mean":0,"sigma":0},
          "prospective": {"scale_factor":0,"degrees":0,"shear":0,"perspective":0}
        },
        "downsample": {"keep":1,"every":2},    # null = skip
        "crop":   {"regions": {"<sensor>":[x1,y1,x2,y2], ...}}, # null = skip
        "rotate": {"angles":  {"<sensor>":90, ...}}              # null = skip; deg ∈ {0,90,180,270}
      }
    }

Socket.io events:
    transform_progress  {"progress": 0..1}
    transform_complete  {"dataset_id": <new id>}

Implementation strategy
-----------------------
Re-use the augmentation primitives from ``augment_dataset.py`` and the
gripper-transition detector from ``downsample_dataset.py``. The metadata
write-out (info.json / episodes.jsonl / episodes_stats.jsonl / tasks.jsonl)
is mostly identical to those modules, so we keep the same shape but apply
all per-frame transforms in a single inner loop.
"""

from __future__ import annotations

import os
import shutil
import traceback
from typing import Any

import cv2
import numpy as np
import pyarrow.parquet as pq
from PIL import Image

import datasets as hf_datasets
from lerobot.datasets.feature_utils import get_hf_features_from_features
from lerobot.datasets.video_utils import encode_video_frames

from ...configs.global_configs import DATASET_DIR
from ...utils.lerobot_io import (
    DEFAULT_CHUNK_SIZE,
    EPISODES_PATH,
    EPISODES_STATS_PATH,
    INFO_PATH,
    PARQUET_PATH_TEMPLATE,
    TASKS_PATH,
    _append_jsonl,
    _read_json,
    _read_jsonl,
    _write_json,
    _write_jsonl,
    get_dataset_info,
    list_episodes,
    read_episode,
)

# Reuse pixel-level helpers — they're stateless image -> image transforms.
from .augment_dataset import (
    add_gaussian_noise,
    add_salt_and_pepper_noise,
    adjust_lightness,
    apply_hsv,
    draw_rectangles,
    generate_prospective_transform,
    generate_rect_params,
    prospective_transform,
)
from .downsample_dataset import _detect_tool_transition_frames


# ───────────────────────────────────────────────────────────────────────────
# Per-frame transform stack
# ───────────────────────────────────────────────────────────────────────────

_VALID_ROTATIONS = (0, 90, 180, 270)
_CV2_ROTATIONS = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


# ───────────────────────────────────────────────────────────────────────────
# Range resolution
# ───────────────────────────────────────────────────────────────────────────
# Frontend now sends [min, max] for each numeric augmentation parameter so the
# applied value varies between repeats. Legacy scalars (from /augment wrapper
# or older clients) are still accepted and treated as [v, v].

def _resolve_range(value, default=0.0, integer=False):
    """Sample a value from a range. Accepts:
        - [min, max] or (min, max): uniform sample
        - scalar (number): use directly
        - None / other: ``default``
    """
    if value is None:
        v = default
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        lo, hi = float(value[0]), float(value[1])
        if hi < lo:
            lo, hi = hi, lo
        v = float(np.random.uniform(lo, hi)) if hi > lo else lo
    elif isinstance(value, (int, float)):
        v = float(value)
    else:
        v = default
    return int(round(v)) if integer else v


def _hsv_random_params() -> tuple[float, float, float]:
    """Match augment_dataset.py random gains so output stays consistent across
    the augmentation-only and full-transform paths."""
    h_gain, s_gain, v_gain = 0.5, 0.7, 0.4
    return (
        (np.random.rand() * 2 - 1) * h_gain * 180,
        (np.random.rand() * 2 - 1) * s_gain + 1,
        (np.random.rand() * 2 - 1) * v_gain + 1,
    )


# ───────────────────────────────────────────────────────────────────────────
# Per-sensor lookup + snapshot creation
# ───────────────────────────────────────────────────────────────────────────

def _get_sensor_aug(augmentation: dict | None, sensor_name: str) -> dict | None:
    """Return augmentation params that apply to ``sensor_name``.

    Layout precedence:
        augmentation.sensors[sensor_name] > augmentation.default > legacy flat

    Legacy flat form (single config for all sensors) is returned as-is when
    no ``sensors`` map is present — preserves the /augment wrapper contract.
    Returns None when nothing applies.
    """
    if not augmentation:
        return None
    sensors_map = augmentation.get('sensors')
    if isinstance(sensors_map, dict):
        spec = sensors_map.get(sensor_name) or augmentation.get('default')
        return spec or None
    # legacy: the augmentation dict itself is the per-sensor config (applied
    # to every camera). Strip the 'sensors'/'default' meta-keys to keep the
    # downstream snapshot maker focused on actual params.
    return {k: v for k, v in augmentation.items() if k not in ('sensors', 'default')}


def _make_aug_snapshot(sensor_aug: dict, img_w: int, img_h: int) -> dict:
    """Resolve all range params once for a (sensor, repeat) tuple. The returned
    snapshot is a flat dict of scalars + pre-computed rect_params and
    persp_matrix so every frame in this repeat reuses identical augmentation.

    HSV.random=True is the documented exception — frame-level resampling
    happens inside ``_apply_augment_snapshot`` so we don't precompute h/s/v
    here for that branch.
    """
    snap: dict = {}

    snap['lightness'] = _resolve_range(sensor_aug.get('lightness'), default=0)

    sap = sensor_aug.get('saltAndPepper') or {}
    snap['salt_amount'] = _resolve_range(sap.get('amount'), default=0)

    g = sensor_aug.get('gaussian') or {}
    snap['gaussian_mean'] = _resolve_range(g.get('mean'), default=0)
    snap['gaussian_sigma'] = _resolve_range(g.get('sigma'), default=0)

    # Rectangles — count is per-repeat (we use the upper bound for color
    # distribution); positions and individual colors are sampled inside
    # generate_rect_params(). We pass the user's count range through so
    # generate_rect_params sees a scalar.
    rect_cfg = sensor_aug.get('rectangles') or {}
    rect_count = _resolve_range(rect_cfg.get('count'), default=0, integer=True)
    snap['rect_params'] = []
    if rect_count > 0:
        snap['rect_params'] = generate_rect_params(
            {**rect_cfg, 'count': rect_count}, img_w, img_h,
        )

    # Perspective — sample each axis once
    p = sensor_aug.get('prospective') or {}
    scale_f = _resolve_range(p.get('scale_factor'), default=0)
    deg = _resolve_range(p.get('degrees'), default=0)
    shear = _resolve_range(p.get('shear'), default=0)
    persp = _resolve_range(p.get('perspective'), default=0)
    if any((scale_f, deg, shear, persp)):
        snap['persp_matrix'] = generate_prospective_transform(
            img_w, img_h, scale_f, deg, shear, persp,
        )
    else:
        snap['persp_matrix'] = None

    hsv = sensor_aug.get('hsv') or {}
    snap['hsv'] = {
        'random': bool(hsv.get('random')),
        'h_adj': _resolve_range(hsv.get('h'), default=0) * 180,
        's_adj': 1 + _resolve_range(hsv.get('s'), default=0),
        'v_adj': 1 + _resolve_range(hsv.get('v'), default=0),
        'enabled': bool(hsv),
    }
    return snap


def _apply_augment_snapshot(img: Image.Image, snap: dict) -> Image.Image:
    """Apply a pre-resolved augmentation snapshot. Order mirrors
    augment_dataset.py: lightness → rectangles → salt&pepper → gaussian →
    perspective → hsv."""
    img = adjust_lightness(img, snap['lightness'])
    img = draw_rectangles(img, snap['rect_params'])
    img = add_salt_and_pepper_noise(img, snap['salt_amount'])
    img = add_gaussian_noise(img, snap['gaussian_mean'], snap['gaussian_sigma'])
    if snap['persp_matrix'] is not None:
        img = prospective_transform(img, snap['persp_matrix'])
    hsv = snap['hsv']
    if hsv['enabled']:
        if hsv['random']:
            h_adj, s_adj, v_adj = _hsv_random_params()
        else:
            h_adj = hsv['h_adj']
            s_adj = hsv['s_adj']
            v_adj = hsv['v_adj']
        img = apply_hsv(img, h_adj, s_adj, v_adj)
    return img


def _normalize_crop_box(box, img_w: int, img_h: int):
    """Clamp a crop box and force its dimensions to be even.

    Video encoding via libx264 + yuv420p (the codec we use in
    encode_video_frames) requires width and height to be divisible by 2 —
    yuv420p chroma subsampling halves both axes and odd sizes break ffmpeg
    silently. Without this normalization the encode call returns success
    but produces no output → resulting dataset has 0 videos and the camera
    view goes blank in the viewer. We shrink the box by one pixel on the
    bottom/right edge when needed (cheaper than padding the image and
    preserves the user's intended crop within ±1 px).

    Returns ``(x1, y1, x2, y2)`` ints with even (x2-x1) and (y2-y1), or
    ``None`` if the input is invalid or degenerate (< 2 px on any axis).
    """
    if not box or len(box) != 4:
        return None
    x1, y1, x2, y2 = (int(round(v)) for v in box)
    x1 = max(0, min(x1, img_w)); x2 = max(0, min(x2, img_w))
    y1 = max(0, min(y1, img_h)); y2 = max(0, min(y2, img_h))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    if (x2 - x1) % 2 == 1:
        x2 -= 1  # shrink right edge; safe because (x2 - x1) >= 2 above
    if (y2 - y1) % 2 == 1:
        y2 -= 1  # shrink bottom edge
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    return (x1, y1, x2, y2)


def _apply_crop_rotate(img: Image.Image, sensor_name: str,
                      crop: dict | None, rotate: dict | None) -> Image.Image:
    """Crop by per-sensor box then rotate by per-sensor angle (0/90/180/270)."""
    if crop and crop.get('regions'):
        box = crop['regions'].get(sensor_name)
        w, h = img.size
        norm = _normalize_crop_box(box, w, h)
        if norm is not None:
            img = img.crop(norm)

    if rotate and rotate.get('angles'):
        ang = int(rotate['angles'].get(sensor_name, 0) or 0)
        if ang in _CV2_ROTATIONS:
            arr = cv2.rotate(np.array(img), _CV2_ROTATIONS[ang])
            img = Image.fromarray(arr)
        elif ang != 0 and ang not in _VALID_ROTATIONS:
            print(f"[transform] ignoring invalid rotation {ang} for {sensor_name}")
    return img


def _output_shape_after_crop_rotate(src_shape, sensor_name: str,
                                     crop: dict | None, rotate: dict | None):
    """Return (h, w, 3) after crop + rotate, matching the run-time pipeline.

    Uses the same crop normalization as ``_apply_crop_rotate`` so that the
    shape written to info.json exactly matches the dimensions ffmpeg actually
    encodes (otherwise lerobot's video reader rejects mismatched videos).
    """
    h, w = int(src_shape[0]), int(src_shape[1])
    if crop and crop.get('regions'):
        box = crop['regions'].get(sensor_name)
        norm = _normalize_crop_box(box, w, h)
        if norm is not None:
            x1, y1, x2, y2 = norm
            w, h = (x2 - x1), (y2 - y1)
    if rotate and rotate.get('angles'):
        ang = int(rotate['angles'].get(sensor_name, 0) or 0)
        if ang in (90, 270):
            h, w = w, h
    channels = src_shape[2] if len(src_shape) >= 3 else 3
    return [h, w, channels]


# ───────────────────────────────────────────────────────────────────────────
# Downsample selection
# ───────────────────────────────────────────────────────────────────────────

def _downsample_indices(num_frames: int, downsample: dict | None,
                        qpos_arr, tool_indices: list) -> np.ndarray:
    """Return the frame indices to keep. Without downsample, returns all frames."""
    if not downsample:
        return np.arange(num_frames)

    keep = int(downsample.get('keep', 1))
    every = int(downsample.get('every', 2))
    if keep <= 0 or every <= 0 or keep >= every:
        # invalid → no downsampling
        return np.arange(num_frames)

    # Same logic as downsample_dataset.py: keep first `keep` of every `every`
    base = np.arange(num_frames)
    pattern = (base % every) < keep
    selected = set(base[pattern].tolist())

    # preserve gripper transitions (with ±tool_pad padding)
    if downsample.get('preserve_tool_transitions', True) and tool_indices and qpos_arr is not None:
        try:
            transitions = _detect_tool_transition_frames(qpos_arr, tool_indices)
            pad = int(downsample.get('tool_pad', 1))
            for t in transitions:
                for k in range(-pad, pad + 1):
                    j = int(t) + k
                    if 0 <= j < num_frames:
                        selected.add(j)
        except Exception as e:
            print(f"[transform] tool transition detection failed: {e}")

    return np.array(sorted(selected), dtype=np.int64)


# ───────────────────────────────────────────────────────────────────────────
# Metadata helpers
# ───────────────────────────────────────────────────────────────────────────

def _init_output_dataset(src_info: dict, out_path: str,
                         crop: dict | None, rotate: dict | None) -> dict:
    """Create the empty output dataset and return its info dict."""
    if os.path.exists(out_path):
        shutil.rmtree(out_path)
    os.makedirs(out_path, exist_ok=True)

    out_info = dict(src_info)
    out_info["total_episodes"] = 0
    out_info["total_frames"] = 0
    out_info["total_tasks"] = 0
    out_info["total_chunks"] = 0
    out_info["total_videos"] = 0
    out_info["splits"] = {}

    # Update per-sensor shape if crop / rotate changes resolution. Both 'video'
    # and 'image' feature dtypes carry shape so we update both.
    features = dict(out_info.get("features", {}))
    for fkey, feat in list(features.items()):
        if not fkey.startswith("observation.images."):
            continue
        if feat.get("dtype") not in ("video", "image"):
            continue
        sensor_name = fkey[len("observation.images."):]
        src_shape = feat.get("shape") or [0, 0, 3]
        new_shape = _output_shape_after_crop_rotate(src_shape, sensor_name, crop, rotate)
        if new_shape != list(src_shape):
            new_feat = dict(feat)
            new_feat["shape"] = new_shape
            # video info.{width,height} if present (lerobot writes this).
            v_info = dict(new_feat.get("video_info") or {})
            if v_info:
                v_info["video.height"] = new_shape[0]
                v_info["video.width"] = new_shape[1]
                new_feat["video_info"] = v_info
            features[fkey] = new_feat
    out_info["features"] = features

    _write_json(out_info, os.path.join(out_path, INFO_PATH))
    for p in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
        fp = os.path.join(out_path, p)
        os.makedirs(os.path.dirname(fp), exist_ok=True)
        open(fp, "w").close()
    return out_info


def _as_seq(arr, dtype):
    """Match augment_dataset._as_seq — wrap 1-D into (-1, 1) for length-1 features."""
    a = np.asarray(arr, dtype=dtype)
    if a.ndim == 1:
        a = a.reshape(-1, 1)
    return a


def _write_episode_parquet_and_stats(out_path: str, out_info: dict,
                                      out_ep_idx: int, src_df,
                                      selected_idx: np.ndarray,
                                      features: dict, chunks_size: int) -> dict:
    """Write a downsampled parquet + return per-episode stats. Matches the column
    handling in augment_dataset.py and downsample_dataset.py (qpos/state, joint/action)."""
    out_chunk = out_ep_idx // chunks_size

    # subset rows
    df = src_df.iloc[selected_idx].reset_index(drop=True)
    num_frames = len(df)

    qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else 'observation.state'
    action_col = 'action.joint' if 'action.joint' in df.columns else 'action'
    state_data = np.array(df[qpos_col].tolist(), dtype=np.float32)
    action_data = np.array(df[action_col].tolist(), dtype=np.float32)

    global_start = out_info["total_frames"]
    episode_dict = {
        "index": _as_seq(np.arange(global_start, global_start + num_frames), np.int64),
        "episode_index": _as_seq(np.full(num_frames, out_ep_idx), np.int64),
        "frame_index": _as_seq(np.arange(num_frames), np.int64),
        "timestamp": _as_seq(df["timestamp"].tolist(), np.float32),
        "task_index": _as_seq(df["task_index"].tolist(), np.int64),
        ("observation.state" if "observation.state" in features and "observation.qpos" not in features
                              else "observation.qpos"): state_data,
    }
    if "action.joint" in features:
        episode_dict["action.joint"] = action_data
    else:
        episode_dict["action"] = action_data

    for col in ["observation.qvel", "observation.qeffort", "observation.eepos",
                "observation.ee_delta", "action.ee_delta"]:
        if col in df.columns:
            episode_dict[col] = np.array(df[col].tolist(), dtype=np.float32)
    if "succeed" in df.columns:
        episode_dict["succeed"] = _as_seq(df["succeed"].tolist(), np.float32)

    parquet_features = {k: v for k, v in features.items() if v.get("dtype") not in ("video", "image")}
    hf_features = get_hf_features_from_features(parquet_features)
    ep_ds = hf_datasets.Dataset.from_dict(episode_dict, features=hf_features, split="train")

    parquet_path = os.path.join(out_path, PARQUET_PATH_TEMPLATE.format(chunk=out_chunk, ep=out_ep_idx))
    os.makedirs(os.path.dirname(parquet_path), exist_ok=True)
    ep_ds.to_parquet(parquet_path)

    # stats
    ep_stats: dict[str, Any] = {}
    qpos_key = "observation.qpos" if "observation.qpos" in episode_dict else "observation.state"
    action_stat_key = "action.joint" if "action.joint" in features else "action"
    stat_pairs = [(qpos_key, state_data), (action_stat_key, action_data)]
    for col in ["observation.qvel", "observation.qeffort", "observation.eepos",
                "observation.ee_delta", "action.joint", "action.ee_delta"]:
        if col in episode_dict:
            stat_pairs.append((col, episode_dict[col]))
    for key, arr in stat_pairs:
        if arr.size > 0:
            ep_stats[key] = {
                "min": np.min(arr, axis=0).tolist(),
                "max": np.max(arr, axis=0).tolist(),
                "mean": np.mean(arr, axis=0).tolist(),
                "std": np.std(arr, axis=0).tolist(),
                "count": int(num_frames),
            }

    return {
        "num_frames": num_frames,
        "out_chunk": out_chunk,
        "ep_stats": ep_stats,
        "global_start": global_start,
    }


# ───────────────────────────────────────────────────────────────────────────
# Main entry point
# ───────────────────────────────────────────────────────────────────────────

def transform_dataset(dataset_id, new_dataset_id, operations,
                      socketio_instance, task_control):
    """Apply the configured transform pipeline. ``operations`` shape — see
    module docstring."""
    src_path = os.path.join(DATASET_DIR, str(dataset_id))
    out_path = os.path.join(DATASET_DIR, str(new_dataset_id))

    if not os.path.exists(src_path):
        print(f"[transform] source dataset not found: {src_path}")
        socketio_instance.emit('transform_complete', {'dataset_id': new_dataset_id, 'error': 'source_missing'})
        return

    src_info = get_dataset_info(src_path)
    if src_info is None:
        print(f"[transform] no info.json at {src_path}")
        socketio_instance.emit('transform_complete', {'dataset_id': new_dataset_id, 'error': 'info_missing'})
        return

    operations = operations or {}
    augmentation = operations.get('augmentation')
    downsample = operations.get('downsample')
    crop = operations.get('crop')
    rotate = operations.get('rotate')
    repeat = max(1, int(operations.get('repeat', 1) or 1))
    # repeat only makes sense with augmentation — otherwise we'd produce
    # identical copies. Clamp to 1 for pure crop/rotate/downsample.
    if augmentation is None and repeat > 1:
        repeat = 1

    out_info = _init_output_dataset(src_info, out_path, crop, rotate)
    chunks_size = src_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
    src_fps = src_info.get("fps", 20)
    features = src_info.get("features", {})
    tool_indices = src_info.get("tool_qpos_indices") or []

    episodes = list_episodes(src_path)
    total_steps = max(1, len(episodes) * repeat)
    step = 0
    out_ep_idx = 0
    total_videos = 0

    socketio_instance.emit('transform_progress', {'progress': 0})

    for ep_entry in episodes:
        if task_control.get('stop'):
            print("[transform] stop signal received")
            break
        src_ep_idx = ep_entry["episode_index"]

        try:
            ep_data = read_episode(src_path, src_ep_idx)
        except Exception:
            print(f"[transform] read_episode failed for {src_ep_idx}:\n{traceback.format_exc()}")
            continue

        # source parquet (for non-image columns)
        src_chunk = src_ep_idx // chunks_size
        src_parquet = os.path.join(src_path, PARQUET_PATH_TEMPLATE.format(chunk=src_chunk, ep=src_ep_idx))
        try:
            src_df = pq.read_table(src_parquet).to_pandas()
        except Exception:
            print(f"[transform] parquet read failed: {src_parquet}\n{traceback.format_exc()}")
            continue

        # downsample selection — independent of augment repeat
        qpos_col = 'observation.qpos' if 'observation.qpos' in src_df.columns else 'observation.state'
        qpos_arr = np.array(src_df[qpos_col].tolist(), dtype=np.float32) if qpos_col in src_df.columns else None
        selected_idx = _downsample_indices(ep_data["num_frames"], downsample, qpos_arr, tool_indices)
        if len(selected_idx) == 0:
            print(f"[transform] skipping episode {src_ep_idx}: downsample produced 0 frames")
            continue

        sensor_names = sorted(ep_data["images"].keys())

        # Pre-compute per-(sensor, repeat) augmentation snapshots so every
        # frame in this repeat reuses identical sampled values — preserving
        # temporal consistency within an episode.
        for rep in range(repeat):
            if task_control.get('stop'):
                break

            sensor_snaps: dict[str, dict] = {}
            if augmentation:
                for cam_name in sensor_names:
                    sensor_aug = _get_sensor_aug(augmentation, cam_name)
                    if not sensor_aug:
                        continue
                    frames = ep_data["images"].get(cam_name) or []
                    if not frames:
                        continue
                    h_img, w_img = frames[0].shape[:2]
                    sensor_snaps[cam_name] = _make_aug_snapshot(sensor_aug, w_img, h_img)

            # write images → encode video, per sensor
            for cam_name in sensor_names:
                feature_key = f"observation.images.{cam_name}"
                imgs_dir = os.path.join(out_path, "images", feature_key, f"episode_{out_ep_idx:06d}")
                os.makedirs(imgs_dir, exist_ok=True)
                snap = sensor_snaps.get(cam_name)

                for out_frame_idx, src_frame_idx in enumerate(selected_idx):
                    arr = ep_data["images"][cam_name][int(src_frame_idx)]
                    img = Image.fromarray(arr)
                    if snap is not None:
                        img = _apply_augment_snapshot(img, snap)
                    img = _apply_crop_rotate(img, cam_name, crop, rotate)
                    img.save(os.path.join(imgs_dir, f"frame-{out_frame_idx:06d}.png"))

                video_path = os.path.join(
                    out_path, "videos", f"chunk-{out_ep_idx // chunks_size:03d}",
                    feature_key, f"episode_{out_ep_idx:06d}.mp4",
                )
                try:
                    encode_video_frames(
                        imgs_dir=imgs_dir, video_path=video_path,
                        fps=src_fps, vcodec="h264", pix_fmt="yuv420p",
                        g=2, crf=30, overwrite=True,
                    )
                    total_videos += 1
                except Exception:
                    print(f"[transform] video encode failed for {feature_key}/ep{out_ep_idx}:\n{traceback.format_exc()}")
                shutil.rmtree(imgs_dir, ignore_errors=True)

            # rm empty 'images/' tree
            images_root = os.path.join(out_path, "images")
            if os.path.exists(images_root):
                try:
                    os.removedirs(images_root)
                except OSError:
                    pass

            # parquet + stats + meta
            written = _write_episode_parquet_and_stats(
                out_path, out_info, out_ep_idx, src_df, selected_idx, features, chunks_size,
            )

            src_tasks = _read_jsonl(os.path.join(src_path, TASKS_PATH))
            out_tasks_path = os.path.join(out_path, TASKS_PATH)
            if not _read_jsonl(out_tasks_path) and src_tasks:
                _write_jsonl(src_tasks, out_tasks_path)

            out_info["total_episodes"] = out_ep_idx + 1
            out_info["total_frames"] = written["global_start"] + written["num_frames"]
            out_info["total_chunks"] = written["out_chunk"] + 1
            out_info["total_tasks"] = len(src_tasks) if src_tasks else 1
            out_info["total_videos"] = total_videos
            out_info["splits"] = {"train": f"0:{out_ep_idx + 1}"}
            _write_json(out_info, os.path.join(out_path, INFO_PATH))

            _append_jsonl(
                {"episode_index": out_ep_idx, "length": written["num_frames"],
                 "tasks": ep_entry.get("tasks", [""])},
                os.path.join(out_path, EPISODES_PATH),
            )
            _append_jsonl(
                {"episode_index": out_ep_idx, "stats": written["ep_stats"]},
                os.path.join(out_path, EPISODES_STATS_PATH),
            )

            out_ep_idx += 1
            step += 1
            socketio_instance.emit('transform_progress', {'progress': step / total_steps})

    socketio_instance.emit('transform_complete', {'dataset_id': new_dataset_id})
    # Backward-compat: old front-end clients listen on 'augmentation_complete'.
    # Emitting both lets us migrate the UI without breaking older builds.
    socketio_instance.emit('augmentation_complete', {'dataset_id': new_dataset_id})
    task_control['stop'] = True
