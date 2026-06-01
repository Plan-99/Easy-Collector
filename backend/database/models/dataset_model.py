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
    # ── Curriculum 소유 데이터셋 표식 ────────────────────────────────────────
    # origin == 'curriculum' 인 데이터셋은 워크스페이스 UI에서 편집/병합/삭제 차단.
    # 커리큘럼 초기화(reset) 또는 실패 데이터 버리기(discard failure)로만 변경.
    # docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
    origin = CharField(null=True, default='manual')  # 'manual' | 'curriculum'
    stage_id = IntegerField(null=True)
    checkpoint_group_id = IntegerField(null=True)
    checkpoint_id = IntegerField(null=True)
    role = CharField(null=True)  # 'success' | 'failure' | 'dagger'
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
