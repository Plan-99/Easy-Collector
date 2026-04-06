"""
사용법(예시):
  python convert_from_hdf5_1piper.py \
    --data-path /path/to/hdf5_dir \
    --out-dir /path/to/output \
    --task "pick up the blue object"

기본값:
  --data-path  ./datasets   (스크립트 위치 기준)
  --out-dir    ./output     (스크립트 위치 기준)
  --task       "I miss old Kanye"
"""

"""Script to convert HDF5 data to the LeRobot dataset v2.1 format."""

import dataclasses
from pathlib import Path
from typing import Dict, List, Literal

import h5py
import numpy as np
import psutil
import torch
import tqdm
import tyro
from fast_lerobot_dataset import FastLeRobotDataset


# -------------------------
# Camera mapping (HDF5 -> LeRobot)
# -------------------------
HDF5_CAMERAS = ["sensor_9", "sensor_10", "sensor_8"]
LEROBOT_CAMERAS = ["cam_high", "cam_left_wrist", "cam_right_wrist"]

SENSOR_TO_CAM = {
    "sensor_9": "cam_high",
    "sensor_10": "cam_left_wrist",
    "sensor_8": "cam_right_wrist",
}


@dataclasses.dataclass(frozen=True)
class DatasetConfig:
    """Configuration for LeRobot dataset creation."""
    use_videos: bool = True
    tolerance_s: float = 0.0001
    image_writer_processes: int = 10
    image_writer_threads: int = 5
    video_backend: str | None = None


DEFAULT_DATASET_CONFIG = DatasetConfig()


def get_cpu_memory(unit: str = "GB") -> str:
    factors = {"GB": 1024**3, "MB": 1024**2, "KB": 1024, "B": 1}
    factor = factors[unit]
    mem_info = psutil.virtual_memory()
    mem_total = mem_info.total / factor
    mem_used = mem_info.used / factor
    return f"{mem_used:.2f}/{mem_total:.2f} {unit}"


def create_empty_dataset(
    out_dir: Path,
    repo_id: str,
    robot_type: str,
    mode: Literal["video", "image"] = "image",
    *,
    dataset_config: DatasetConfig = DEFAULT_DATASET_CONFIG,
) -> FastLeRobotDataset:
    motors = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
        "gripper",
    ]

    # IMPORTANT: Dataset schema uses cam_* (not sensor_*)
    cameras = LEROBOT_CAMERAS

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(motors),),
            "names": motors,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(motors),),
            "names": motors,
        },
    }

    for cam in cameras:
        features[f"observation.images.{cam}"] = {
            "dtype": mode,
            "shape": (150, 200, 3),
            "names": ["height", "width", "channels"],
        }

    lerobot_dataset = FastLeRobotDataset.create(
        root=out_dir,
        repo_id=repo_id,
        fps=10,
        robot_type=robot_type,
        features=features,
        use_videos=dataset_config.use_videos,
        tolerance_s=dataset_config.tolerance_s,
        image_writer_processes=dataset_config.image_writer_processes,
        image_writer_threads=dataset_config.image_writer_threads,
        video_backend=dataset_config.video_backend,
    )
    return lerobot_dataset


def get_cameras(hdf5_files: List[Path]) -> List[str]:
    with h5py.File(hdf5_files[0], "r") as ep:
        return [key for key in ep["/observations/images"].keys() if "depth" not in key]


def has_velocity(hdf5_files: List[Path]) -> bool:
    with h5py.File(hdf5_files[0], "r") as ep:
        return "/observations/qvel" in ep


def has_effort(hdf5_files: List[Path]) -> bool:
    with h5py.File(hdf5_files[0], "r") as ep:
        return "/observations/effort" in ep


def load_raw_images_per_camera(ep: h5py.File, cameras: List[str]) -> Dict[str, np.ndarray]:
    imgs_per_cam: Dict[str, np.ndarray] = {}
    for camera in cameras:
        uncompressed = ep[f"/observations/images/{camera}"].ndim == 4

        if uncompressed:
            imgs_array = ep[f"/observations/images/{camera}"][:]
        else:
            import cv2

            imgs_array = []
            for data in ep[f"/observations/images/{camera}"]:
                imgs_array.append(cv2.cvtColor(cv2.imdecode(data, 1), cv2.COLOR_BGR2RGB))
            imgs_array = np.array(imgs_array)

        imgs_per_cam[camera] = imgs_array
    return imgs_per_cam


def load_raw_episode_data(
    ep_path: Path,
) -> tuple[dict[str, np.ndarray], torch.Tensor, torch.Tensor, torch.Tensor | None, torch.Tensor | None]:
    with h5py.File(ep_path, "r") as f:
        # 1) qpos: robot_1 only -> (T, 7)
        qpos = f["observations/qpos/robot_4"][:].astype(np.float32)  # (T, 7)

        start_idx = 0
        end_idx = qpos.shape[0]

        state = torch.from_numpy(qpos[start_idx:end_idx])  # (T, 7)

        # 2) action: robot_1 only -> (T, 7)
        act = f["qaction/robot_4"][:].astype(np.float32)  # (T, 7)
        action = torch.from_numpy(act[start_idx:end_idx])  # (T, 7)

        velocity = None
        effort = None

        # 3) images: 3 cameras (sensor_1/2/3)
        imgs_per_cam = load_raw_images_per_camera(f, HDF5_CAMERAS)
        for cam in imgs_per_cam:
            imgs_per_cam[cam] = imgs_per_cam[cam][start_idx:end_idx]

    return imgs_per_cam, state, action, velocity, effort



def populate_dataset(
    dataset: FastLeRobotDataset,
    hdf5_files: list[Path],
    task: str,
    episodes: list[int] | None = None,
) -> FastLeRobotDataset:
    if episodes is None:
        episodes = list(range(len(hdf5_files)))

    for ep_idx in tqdm.tqdm(episodes):
        ep_path = hdf5_files[ep_idx]

        try:
            imgs_per_cam, state, action, velocity, effort = load_raw_episode_data(ep_path)
            num_frames = state.shape[0]

            for i in range(num_frames):
                frame = {
                    "observation.state": state[i],
                    "action": action[i],
                }

                for sensor_name, img_array in imgs_per_cam.items():
                    img = img_array[i]

                    # HWC RGB -> BGR (video writer expects BGR often)
                    if img.ndim == 3 and img.shape[-1] == 3:
                        img = img[..., ::-1].copy()

                    cam_name = SENSOR_TO_CAM[sensor_name]  # sensor_* -> cam_*
                    frame[f"observation.images.{cam_name}"] = img

                dataset.add_frame(frame, task)

            dataset.save_episode()
            print(f"{ep_idx} Done, get_cpu_memory: {get_cpu_memory()}")

        except Exception as e:
            print(f"{ep_idx} Error: {e}")
            continue

    return dataset


_SCRIPT_DIR = Path(__file__).resolve().parent


def convert_lerobot(
    task: str = "I miss old Kanye",
    data_path: Path = _SCRIPT_DIR / "datasets",
    out_dir: Path = _SCRIPT_DIR / "output",
) -> None:
    data_files = sorted(
        [p for p in data_path.glob("*.hdf5") if p.is_file()],
        key=lambda x: int(x.stem.split("_")[-1].split(".")[0]),
    )

    dataset = create_empty_dataset(
        out_dir,
        "giga-brain/agilex_example",
        robot_type="piper",
        mode="video",
    )

    populate_dataset(dataset, data_files, task=task)


if __name__ == "__main__":
    tyro.cli(convert_lerobot)