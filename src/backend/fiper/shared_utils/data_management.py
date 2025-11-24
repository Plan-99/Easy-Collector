"""This file contains shared utility functions for data management including saving/loading etc."""

import os
import pickle
import numpy as np
from typing import Optional, Union, Dict, Any, Tuple
import torch
from .utility_functions import ensure_list
import h5py
from backend.fiper.shared_utils.embedding_helper import EmbeddingHelper
from tqdm import tqdm

supported_data_types = ["npy", "pkl", "pt", "hdf5"]


class LazyTensorManager:
    """
    A class to manage lazy loading/saving of tensors.
    """

    def __init__(
        self,
        save_dir: str,
        key: str,
        batch_size: int = 1000,
        dtype=torch.float32,
        max_batches_in_cache: int = 1,
        max_bytes_per_batch: int = 2**30,
        element_size: int = None,
    ):
        if element_size is not None:
            batch_size = max_bytes_per_batch // element_size
        self.save_dir = save_dir
        self.key = key
        self.batch_size = batch_size
        self.max_batches_in_cache = max_batches_in_cache
        self.cache = {}
        if not os.path.exists(save_dir):
            os.makedirs(save_dir, exist_ok=True)
        if not os.path.exists(self._get_filepath(0)):
            self.num_batches = 0
        else:
            self.num_batches = len([f for f in os.listdir(save_dir) if f.startswith(f"{key}_part_")])

    def get_batch(self, batch_index):
        if batch_index in self.cache:
            return self.cache[batch_index]
        batch = torch.load(self._get_filepath(batch_index), weights_only=True)
        self.cache[batch_index] = batch
        if len(self.cache) > self.max_batches_in_cache:
            self.cache.pop(next(iter(self.cache)))
        return batch

    def __getitem__(self, index):
        batch_index = index // self.batch_size
        batch_offset = index % self.batch_size
        batch = self.get_batch(batch_index)
        return batch[batch_offset]

    def __len__(self):
        return self.num_batches * self.batch_size

    def save_tensor_batch(self, batch_index, tensor, overwrite=True):
        os.makedirs(self.save_dir, exist_ok=True)
        filepath = self._get_filepath(batch_index)
        if os.path.exists(filepath) and not overwrite:
            raise FileExistsError(f"File {filepath} already exists. Use overwrite=True to replace it.")
        torch.save(tensor, filepath)

    def save_full_tensor_in_batches(self, tensor, save_dir):
        os.makedirs(save_dir, exist_ok=True)
        num_batches = (tensor.shape[0] + self.batch_size - 1) // self.batch_size
        for i in range(num_batches):
            batch = tensor[i * self.batch_size : (i + 1) * self.batch_size]
            torch.save(batch, self._get_filepath(i))

    def _get_filepath(self, batch_index):
        return os.path.join(self.save_dir, f"{self.key}_part_{batch_index}.pt")


def load_data(
    load_dirs: Union[str, list[str]],
    keywords: Optional[Union[str, list[str]]] = None,
    data_types: Optional[Union[str, list[str]]] = None,
    weights_only: bool = True,
    return_filenames: bool = False,
    error_if_not_found: bool = True,
) -> Union[Any, Tuple[Any, list[str]]]:
    load_dirs, keywords, data_types = ensure_list(load_dirs, keywords, data_types)
    if data_types is not None:
        assert any(data_type in supported_data_types for data_type in data_types)
    not_found = 0
    data = []
    all_filenames = []
    for load_dir in load_dirs:
        if not os.path.exists(load_dir) or not os.path.isdir(load_dir):
            print(f"Directory {load_dir} not found or is not a directory. Skipping...")
            not_found += 1
            continue
        filenames = _get_filenames(load_dir, keywords=keywords, data_types=data_types)
        if return_filenames:
            all_filenames.extend(filenames)
        for filename in filenames:
            file_path = os.path.join(load_dir, filename)
            if filename.endswith(".npy"):
                data.append(np.load(file_path, allow_pickle=True))
            elif filename.endswith(".pkl"):
                with open(file_path, "rb") as f:
                    file = pickle.load(f)
                    data.append(file)
            elif filename.endswith(".pt"):
                data.append(torch.load(file_path, weights_only=weights_only))
    if not_found == len(load_dirs) and error_if_not_found:
        raise FileNotFoundError(f"Directories {load_dirs} not found or are not directories.")
    if len(data) == 0 and error_if_not_found:
        raise FileNotFoundError(f"No files found in {load_dirs} with keywords {keywords} and data types {data_types}.")
    if len(data) == 1:
        data = data[0]
    if return_filenames:
        return data, all_filenames
    return data


def save_data(
    save_dir: str,
    filenames: Union[str, list[str]],
    data: Union[Any, list[Any]],
    data_types: Optional[Union[str, list[str]]] = "pkl",
    overwrite: bool = False,
) -> None:
    if isinstance(filenames, str):
        filenames = [filenames]
    if isinstance(data_types, str):
        data_types = [data_types]
    if not isinstance(data, list):
        data = [data]
    assert len(filenames) == len(data), "Number of filenames must match number of data items."
    assert any(data_type in supported_data_types for data_type in data_types)
    if len(data_types) > 1:
        assert len(filenames) == len(data_types)
    os.makedirs(save_dir, exist_ok=True)
    for i in range(len(filenames)):
        if len(data_types) == 1:
            data_type = data_types[0]
        else:
            data_type = data_types[i]
        if not filenames[i].endswith("." + data_type):
            filenames[i] = filenames[i] + "." + data_type
        file_path = os.path.join(save_dir, filenames[i])
        if os.path.exists(file_path):
            if overwrite:
                os.remove(file_path)
            else:
                Warning(f"File {file_path} already exists. Use overwrite=True to replace it.")
                continue
        if file_path.endswith(".npy"):
            np.save(file_path, data[i], allow_pickle=True)
        elif file_path.endswith(".pkl"):
            with open(file_path, "wb") as f:
                pickle.dump(data[i], f)
        elif file_path.endswith(".pt"):
            torch.save(data[i], file_path)


def batch_ec_to_fiper_rollout_converter(
    batch_data: list[tuple[str, dict]], embedding_helper: EmbeddingHelper, is_calibration: bool = True
) -> list[dict]:
    """
    Converts a batch of Easy-Collector rollout data to FiPer format.
    """
    rollouts_ec = [item[1] for item in batch_data]

    # Assumes embedding_helper has a method `get_embedding_batch` for batch processing
    all_embeddings_batch = embedding_helper.get_embedding_batch(rollouts_ec)

    processed_rollouts = []
    index = 0
    for (filename, rollout_ec), all_action_preds in tqdm(zip(batch_data, all_embeddings_batch), total=len(batch_data), desc="Converting rollouts"):
        rollout_fiper = {}
        if is_calibration:
            rollout_type = "calibration"
            rollout_subtype = "ca"
            is_success = True
        else:
            rollout_type = "test"
            if index < 4:
                is_success = True
                rollout_subtype = "id"
            else:
                is_success = False
                rollout_subtype = "ood"
        rollout_fiper['metadata'] = {
            'task': "unknown_task", 'episode': filename, 'num_robots': len(rollout_ec['qaction'].keys()),
            'rollout_type': rollout_type, 'rollout_subtype': rollout_subtype,
            'action_prediction_horizon': 1, 'action_execution_horizon': 1, 'action_batch_size': 1,
            'has_encoder_feat': True, 'has_state_feat': True, 'action_mappings': {}, 'successful': is_success,
            'num_steps': len(rollout_ec['qaction'][next(iter(rollout_ec['qaction']))]),
        }
        rollout_fiper['rollout'] = []
        for step in range(len(rollout_ec['qaction'][next(iter(rollout_ec['qaction']))])):
            rgb = rollout_ec['observations']['images'][next(iter(rollout_ec['observations']['images']))][step]
            action = rollout_ec['qaction'][next(iter(rollout_ec['qaction']))][step]
            agent_pos = rollout_ec['observations']['qpos'][next(iter(rollout_ec['observations']['qpos']))][step]
            action_pred = all_action_preds[step]
            fiper_step = {
                'rgb': rgb, 'action': action, 'agent_pos': agent_pos, 'action_pred': action_pred,
                'obs_embedding': action_pred.flatten(), 'state_embedding': action_pred.flatten(),
                'timestamp': step, 'step': step,
            }
            rollout_fiper['rollout'].append(fiper_step)
        processed_rollouts.append(rollout_fiper)
        index += 1
    return processed_rollouts


def load_raw_rollouts(
    embedding_helper: EmbeddingHelper,
    load_dirs: Union[str, list[str]],
    data_types: Union[str, list[str]] = ["npy", "pkl", "hdf5"],
    keywords: Optional[Union[str, list[str]]] = ["episode", "eps", "rollout"],
    searched_filenames: Optional[Union[str, list[str]]] = [],
    with_filename: bool = False,
    raise_error_if_not_found: bool = True,
    is_calibration: bool = True,
) -> list[Dict[str, Any]]:
    """
    Load raw rollouts from specified directories. If HDF5 files are found, they
    are first converted to .pkl files in the same directory, overwriting existing
    files. Then, all .pkl and .npy files are loaded.
    """
    load_dirs, searched_filenames, data_types, keywords = ensure_list(
        load_dirs, searched_filenames, data_types, keywords
    )
    assert any(dt in supported_data_types for dt in data_types), f"data_types must be in {supported_data_types}"

    # Stage 1: Convert all HDF5 files to PKL format first.
    if "hdf5" in data_types:
        for load_dir in load_dirs:
            if not os.path.exists(load_dir) or not os.path.isdir(load_dir):
                continue

            hdf5_filenames = _get_filenames(
                load_dir, keywords=keywords, data_types=["hdf5"], searched_filenames=searched_filenames
            )

            if not hdf5_filenames:
                continue

            hdf5_batch_to_process = []

            for filename in hdf5_filenames:
                file_path = os.path.join(load_dir, filename)
                with h5py.File(file_path, "r") as f:
                    rollout_ec = hdf5_to_dict(f)
                hdf5_batch_to_process.append((filename, rollout_ec))

            if hdf5_batch_to_process:
                print(f"Converting {len(hdf5_batch_to_process)} HDF5 files in {load_dir} to PKL format...")
                processed_rollouts = batch_ec_to_fiper_rollout_converter(hdf5_batch_to_process, embedding_helper, is_calibration=is_calibration)

                new_filenames = [item[0].replace(".hdf5", ".pkl") for item in hdf5_batch_to_process]
                save_data(load_dir, new_filenames, processed_rollouts, data_types="pkl", overwrite=True)
                print(f"Finished converting and saving {len(new_filenames)} files.")

    # Stage 2: Load all PKL and NPY files.
    data = []
    not_found_dirs = 0
    load_data_types = [dt for dt in data_types if dt in ["pkl", "npy"]]

    for load_dir in load_dirs:
        if not os.path.exists(load_dir) or not os.path.isdir(load_dir):
            print(f"Directory {load_dir} not found or is not a directory. Skipping...")
            not_found_dirs += 1
            continue

        filenames = _get_filenames(
            load_dir, keywords=keywords, data_types=load_data_types, searched_filenames=searched_filenames
        )

        for filename in tqdm(filenames, desc=f"Loading rollouts from {os.path.basename(load_dir)}"):
            file_path = os.path.join(load_dir, filename)
            rollout = None
            if filename.endswith(".npy"):
                rollout = np.load(file_path, allow_pickle=True)
            elif filename.endswith(".pkl"):
                with open(file_path, "rb") as f:
                    rollout = pickle.load(f)
                    # print(rollout.keys())
                    # print(rollout['metadata']["successful"])
                    # print(rollout['rollout'][0].keys())

            if rollout is not None:
                if with_filename:
                    data.append({"fileinfo": filename, "rollout": rollout})
                else:
                    data.append(rollout)

    if not_found_dirs == len(load_dirs) and raise_error_if_not_found:
        raise FileNotFoundError(f"Directories {load_dirs} not found or are not directories.")
    if len(data) == 0 and raise_error_if_not_found:
        raise FileNotFoundError(
            f"No files found in {load_dirs} with keywords {keywords} or filenames {searched_filenames} and data types {load_data_types}."
        )
    return data


def hdf5_to_dict(h5_obj):
    """
    Recursively explore HDF5 objects (Group/Dataset) and
    convert them to Python Dictionary and NumPy Array.
    """
    data = {}
    for key, item in h5_obj.items():
        if isinstance(item, h5py.Group):
            data[key] = hdf5_to_dict(item)
        elif isinstance(item, h5py.Dataset):
            val = item[()]
            data[key] = val
    return data


def _get_filenames(
    load_dir: str,
    keywords: Optional[Union[str, list[str]]] = None,
    data_types: Union[str, list[str]] = ["pkl", "npy", "pt", "hdf5"],
    searched_filenames: Optional[Union[str, list[str]]] = None,
) -> list[str]:
    """
    Filters files in a directory based on a combination of criteria, applying
    filters for specific filenames, data types, and keywords sequentially.
    """
    candidate_files = os.listdir(load_dir)

    # Filter by a specific list of filenames if provided
    if searched_filenames is not None:
        searched_set = set(ensure_list(searched_filenames))
        candidate_files = [f for f in candidate_files if f in searched_set]

    # Filter by data types (file extensions)
    data_types_list = ensure_list(data_types)
    if data_types_list:
        candidate_files = [f for f in candidate_files if any(f.endswith(dt) for dt in data_types_list)]

    # Filter by keywords
    keywords_list = ensure_list(keywords)
    if keywords_list:
        candidate_files = [f for f in candidate_files if any(kw in f for kw in keywords_list)]

    return sorted(candidate_files)