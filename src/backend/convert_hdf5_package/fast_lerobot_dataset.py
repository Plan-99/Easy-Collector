"""FastLeRobotDataset — standalone extraction from giga_datasets.

Requires: lerobot==0.3.2, av, datasets, numpy, torch, Pillow
"""

import logging
from pathlib import Path
from typing import Callable

import av
import datasets
import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset as _LeRobotDataset
from lerobot.datasets.utils import (
    check_timestamps_sync,
    embed_images,
    get_episode_data_index,
    validate_episode_buffer,
    validate_frame,
)
from PIL import Image
from typing_extensions import override


class FastLeRobotDataset(_LeRobotDataset):
    """This class overrides the `LeRobotDataset`(lerobot version 0.3.2) class
    to accelerate the data conversion process.

    What it does is:
    - Doesn't store temporary image files to disk, instead, it's kept in memory until the whole episode is saved.
    - Only consider observation.state and action features to compute episode statistics.

    Beside, it's recommended to use video mode rather than image mode when converting large datasets. It's easy for data transfer and storage.
    """

    def __init__(
        self,
        repo_id: str,
        root: str | Path | None = None,
        episodes: list[int] | None = None,
        image_transforms: Callable | None = None,
        delta_timestamps: dict[list[float]] | None = None,
        tolerance_s: float = 1e-4,
        revision: str | None = None,
        force_cache_sync: bool = False,
        download_videos: bool = True,
        video_backend: str | None = 'pyav',
        skip_video_decoding: bool = False,
    ):
        super().__init__(
            repo_id=repo_id,
            root=root,
            episodes=episodes,
            image_transforms=image_transforms,
            delta_timestamps=delta_timestamps,
            tolerance_s=tolerance_s,
            revision=revision,
            force_cache_sync=force_cache_sync,
            download_videos=download_videos,
            video_backend=video_backend,
        )
        self.skip_video_decoding = skip_video_decoding

    @override
    def add_frame(self, frame: dict, task: str, timestamp: float | None = None) -> None:
        """This function only adds the frame to the episode_buffer and nothing
        is written to disk.

        To save those frames, the 'save_episode()' method then needs to be called.
        """
        # Convert torch tensors to numpy arrays for serialization/storage
        for name in frame:
            if isinstance(frame[name], torch.Tensor):
                frame[name] = frame[name].numpy()

        validate_frame(frame, self.features)

        if self.episode_buffer is None:
            self.episode_buffer = self.create_episode_buffer()

        # Automatically add frame_index and timestamp to episode buffer
        frame_index = self.episode_buffer['size']
        if timestamp is None:
            timestamp = frame_index / self.fps
        self.episode_buffer['frame_index'].append(frame_index)
        self.episode_buffer['timestamp'].append(timestamp)
        self.episode_buffer['task'].append(task)

        # Add frame features to episode_buffer
        for key in frame:
            if key not in self.features:
                raise ValueError(f"An element of the frame is not in the features. '{key}' not in '{self.features.keys()}'.")

            self.episode_buffer[key].append(frame[key])

        self.episode_buffer['size'] += 1

    @override
    def save_episode(self, episode_data: dict | None = None) -> None:
        """This will save to disk the current episode in self.episode_buffer.

        Args:
            episode_data (dict | None, optional): Dict containing the episode data to save. If None, this will
                save the current episode in self.episode_buffer, which is filled with 'add_frame'. Defaults to
                None.
        """
        if not episode_data:
            episode_buffer = self.episode_buffer

        validate_episode_buffer(episode_buffer, self.meta.total_episodes, self.features)

        # 'size' and 'task' are bookkeeping fields, omitted from parquet payload
        episode_length = episode_buffer.pop('size')
        tasks = episode_buffer.pop('task')
        episode_tasks = list(set(tasks))
        episode_index = episode_buffer['episode_index']

        episode_buffer['index'] = np.arange(self.meta.total_frames, self.meta.total_frames + episode_length)
        episode_buffer['episode_index'] = np.full((episode_length,), episode_index)

        # Register any new tasks encountered during this episode
        for task in episode_tasks:
            task_index = self.meta.get_task_index(task)
            if task_index is None:
                self.meta.add_task(task)

        # Map natural-language task names to task indices
        episode_buffer['task_index'] = np.array([self.meta.get_task_index(task) for task in tasks])

        for key, ft in self.features.items():
            # index, episode_index, task_index are already processed above, and image and video
            # are processed separately by storing image path and frame info as meta data
            if key in ['index', 'episode_index', 'task_index'] or ft['dtype'] in ['image', 'video']:
                continue
            episode_buffer[key] = np.stack(episode_buffer[key])

        self._save_episode_table(episode_buffer, episode_index)
        ep_stats = _compute_episode_stats(episode_buffer)

        if len(self.meta.video_keys) > 0:
            video_paths = self.encode_episode_videos(episode_buffer, episode_index)
            for key in self.meta.video_keys:
                episode_buffer[key] = video_paths[key]

        # Persist episode meta after encoding videos to include video metadata
        self.meta.save_episode(episode_index, episode_length, episode_tasks, ep_stats)

        ep_data_index = get_episode_data_index(self.meta.episodes, [episode_index])
        ep_data_index_np = {k: t.numpy() for k, t in ep_data_index.items()}
        check_timestamps_sync(
            episode_buffer['timestamp'],
            episode_buffer['episode_index'],
            ep_data_index_np,
            self.fps,
            self.tolerance_s,
        )

        video_files = list(self.root.rglob('*.mp4'))
        assert len(video_files) == self.num_episodes * len(self.meta.video_keys)

        parquet_files = list(self.root.rglob('*.parquet'))
        assert len(parquet_files) == self.num_episodes

        if not episode_data:  # Reset the buffer
            self.episode_buffer = self.create_episode_buffer()

    @override
    def _save_episode_table(self, episode_buffer: dict, episode_index: int) -> None:
        episode_dict = {key: episode_buffer[key] for key in self.hf_features}
        ep_dataset = datasets.Dataset.from_dict(episode_dict, features=self.hf_features, split='train')
        ep_dataset = embed_images(ep_dataset)
        ep_data_path = self.root / self.meta.get_data_file_path(ep_index=episode_index)
        ep_data_path.parent.mkdir(parents=True, exist_ok=True)
        ep_dataset.to_parquet(ep_data_path)

    @override
    def encode_episode_videos(self, episode_buffer: dict, episode_index: int) -> dict:
        """Use ffmpeg to convert frames stored as png into mp4 videos.

        Note: `encode_video_frames` is a blocking call. Making it asynchronous shouldn't speedup encoding,
        since video encoding with ffmpeg is already using multithreading.
        """
        video_paths = {}
        for key in self.meta.video_keys:
            video_path = self.root / self.meta.get_video_file_path(episode_index, key)
            video_paths[key] = str(video_path)
            if video_path.is_file():
                continue

            imgs = episode_buffer[key]

            _encode_video_frames(imgs, video_path, self.fps, overwrite=True)

        return video_paths

    @override
    def __getitem__(self, idx: int) -> dict:
        item = self.hf_dataset[idx]
        ep_idx = item['episode_index'].item()

        query_indices = None
        if self.delta_indices is not None:
            query_indices, padding = self._get_query_indices(idx, ep_idx)
            query_result = self._query_hf_dataset(query_indices)
            item = {**item, **padding}
            for key, val in query_result.items():
                item[key] = val

        # Optional: skip costly video decoding when only computing stats
        if not self.skip_video_decoding and len(self.meta.video_keys) > 0:
            current_ts = item['timestamp'].item()
            query_timestamps = self._get_query_timestamps(current_ts, query_indices)
            try:
                video_frames = self._query_videos(query_timestamps, ep_idx)
            except Exception as e:
                logging.warning(
                    f'Failed to decode video frames for episode {ep_idx} in timestamps: {query_timestamps}. Error: {e}. Falling back to zeros.'
                )
                video_frames = {}
                # Construct zero tensors matching expected shapes per key
                for vid_key, query_ts in query_timestamps.items():
                    num_queries = len(query_ts)
                    # Prefer shapes from metadata when available
                    ft_shape = self.meta.shapes.get(vid_key)

                    # Derive channel-first (C,H,W)
                    if isinstance(ft_shape, tuple) and len(ft_shape) == 3:
                        if ft_shape[0] in (1, 3, 4):  # likely CHW
                            c, h, w = ft_shape[0], ft_shape[1], ft_shape[2]
                        elif ft_shape[2] in (1, 3, 4):  # likely HWC
                            c, h, w = ft_shape[2], ft_shape[0], ft_shape[1]
                        else:
                            c, h, w = 3, ft_shape[0], ft_shape[1]
                    else:
                        # Conservative default
                        c, h, w = 3, 224, 224

                    if num_queries > 1:
                        zeros_shape = (num_queries, c, h, w)
                    else:
                        zeros_shape = (c, h, w)

                    video_frames[vid_key] = torch.zeros(zeros_shape, dtype=torch.float32)

            item = {**video_frames, **item}

        if self.image_transforms is not None:
            image_keys = self.meta.camera_keys
            for cam in image_keys:
                item[cam] = self.image_transforms(item[cam])

        # Add task as a string
        task_idx = item['task_index'].item()
        item['task'] = self.meta.tasks[task_idx]

        return item


def _encode_video_frames(
    imgs: list[np.ndarray],
    video_path: Path | str,
    fps: int,
    vcodec: str = 'libsvtav1',
    pix_fmt: str = 'yuv420p',
    g: int | None = 2,
    crf: int | None = 30,
    fast_decode: int = 0,
    log_level: int | None = av.logging.ERROR,
    overwrite: bool = False,
) -> None:
    """Encode a sequence of RGB frames into a video file using PyAV/ffmpeg."""
    if vcodec not in ['h264', 'hevc', 'libsvtav1']:
        raise ValueError(f'Unsupported video codec: {vcodec}. Supported codecs are: h264, hevc, libsvtav1.')

    video_path = Path(video_path)
    video_path.parent.mkdir(parents=True, exist_ok=overwrite)

    if (vcodec == 'libsvtav1' or vcodec == 'hevc') and pix_fmt == 'yuv444p':
        logging.warning(f"Incompatible pixel format 'yuv444p' for codec {vcodec}, auto-selecting format 'yuv420p'")
        pix_fmt = 'yuv420p'

    if len(imgs) == 0:
        raise FileNotFoundError('No images found.')
    dummy_image = Image.fromarray(imgs[0])
    width, height = dummy_image.size

    video_options = {}
    if g is not None:
        video_options['g'] = str(g)
    if crf is not None:
        video_options['crf'] = str(crf)
    if fast_decode:
        key = 'svtav1-params' if vcodec == 'libsvtav1' else 'tune'
        value = f'fast-decode={fast_decode}' if vcodec == 'libsvtav1' else 'fastdecode'
        video_options[key] = value

    if log_level is not None:
        logging.getLogger('libav').setLevel(log_level)

    with av.open(str(video_path), 'w') as output:
        output_stream = output.add_stream(vcodec, fps, options=video_options)
        output_stream.pix_fmt = pix_fmt
        output_stream.width = width
        output_stream.height = height

        for input_data in imgs:
            input_image = Image.fromarray(input_data).convert('RGB')
            input_frame = av.VideoFrame.from_image(input_image)
            packet = output_stream.encode(input_frame)
            if packet:
                output.mux(packet)

        packet = output_stream.encode()
        if packet:
            output.mux(packet)

    if log_level is not None:
        av.logging.restore_default_callback()

    if not video_path.exists():
        raise OSError(f'Video encoding did not work. File not found: {video_path}.')


def _get_feature_stats(array: np.ndarray, axis: tuple, keepdims: bool) -> dict[str, np.ndarray]:
    return {
        'min': np.min(array, axis=axis, keepdims=keepdims),
        'max': np.max(array, axis=axis, keepdims=keepdims),
        'mean': np.mean(array, axis=axis, keepdims=keepdims),
        'std': np.std(array, axis=axis, keepdims=keepdims),
        'count': np.array([len(array)]),
    }


def _compute_episode_stats(episode_data: dict[str, list[str] | np.ndarray]) -> dict:
    ep_stats = {}
    for key, data in episode_data.items():
        if key not in ['observation.state', 'action']:
            continue

        ep_ft_array = data
        axes_to_reduce = 0
        keepdims = data.ndim == 1

        ep_stats[key] = _get_feature_stats(ep_ft_array, axis=axes_to_reduce, keepdims=keepdims)

    return ep_stats
