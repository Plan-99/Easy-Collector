from peewee import CharField, IntegerField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import os
import datetime
from ...configs.global_configs import DATASET_DIR
from ...utils.lerobot_io import get_episodes_as_file_list, get_dataset_metadata


class Dataset(SoftDeleteModel):
    class Meta:
        table_name = 'datasets'

    __casts__ = {
        'sensor_ids': 'json',
        'dataset_metadata': 'json',
    }

    __appends__ = [
        'robot_ids',
        'sensor_ids',
        'episode_len',
        'episodes',
        'dataset_metadata',
    ]

    name = CharField(null=True)
    task_id = IntegerField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        return super().save(*args, **kwargs)

    @property
    def task(self):
        from .task_model import Task
        return Task.find(self.task_id) if self.task_id else None

    @property
    def robot_ids(self):
        task = self.task
        return task._sensor_ids if task else None  # Note: original was task.robot_ids

    @property
    def sensor_ids(self):
        task = self.task
        return task._sensor_ids if task else None

    @property
    def episode_len(self):
        task = self.task
        return task.episode_len if task else None

    @property
    def episodes(self):
        folder_path = os.path.join(DATASET_DIR, str(self.id))
        if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
            return []
        return get_episodes_as_file_list(folder_path)

    @property
    def dataset_metadata(self):
        folder_path = os.path.join(DATASET_DIR, str(self.id))
        if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
            return {'sensors': [], 'robots': []}
        return get_dataset_metadata(folder_path)
