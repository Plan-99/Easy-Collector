from peewee import IntegerField, BooleanField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class RolloutResult(SoftDeleteModel):
    """(rollout × checkpoint_group) 단위 결과.

    success: 그룹 성공 여부(그룹 내 전 체크포인트 성공 시 True).
    episodes: 이 결과로 저장된 에피소드 참조 — [{checkpoint_id, dataset_id, episode, role, steps}, ...]
    docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
    """

    class Meta:
        table_name = 'rollout_results'

    __casts__ = {
        'episodes': 'json',
    }

    rollout_id = IntegerField(null=True)
    checkpoint_group_id = IntegerField(null=True)
    stage_id = IntegerField(null=True)
    success = BooleanField(null=True)
    episodes = TextField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        val = getattr(self, 'episodes', None)
        if val is not None and not isinstance(val, str):
            self.episodes = json.dumps(val)
        return super().save(*args, **kwargs)

    @classmethod
    def create(cls, **kwargs):
        if isinstance(kwargs.get('episodes'), (list, dict)):
            kwargs['episodes'] = json.dumps(kwargs['episodes'])
        return super().create(**kwargs)
