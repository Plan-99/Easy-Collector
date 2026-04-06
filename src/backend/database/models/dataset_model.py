from orator import Model, SoftDeletes
from orator.orm import belongs_to
from .task_model import Task
from orator.orm import accessor
import os
from ...configs.global_configs import DATASET_DIR
from ...api.process.lerobot_io import get_episodes_as_file_list, get_dataset_metadata, get_dataset_info

class Dataset(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'task_id',
    ]

    __appends__ = [
        'robot_ids',
        'sensor_ids',
        'episode_len',
        'episodes',
        'dataset_metadata',
    ]

    __casts__ = {
        'sensor_ids': 'json',
        'dataset_metadata': 'json',
    }

    __timestamps__ = True


    @belongs_to
    def task(self):
        return Task

    @accessor
    def robot_ids(self):
        return self.task.robot_ids if self.task else None

    @accessor
    def sensor_ids(self):
        return self.task.sensor_ids if self.task else None

    @accessor
    def episode_len(self):
        return self.task.episode_len if self.task else None

    @accessor
    def episodes(self):
        folder_path = os.path.join(DATASET_DIR, str(self.id))
        if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
            return []

        episodes = get_episodes_as_file_list(folder_path)
        return episodes

    @accessor
    def dataset_metadata(self):
        """Returns sensor/robot names from LeRobot dataset metadata."""
        folder_path = os.path.join(DATASET_DIR, str(self.id))
        if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
            return {'sensors': [], 'robots': []}

        return get_dataset_metadata(folder_path)
