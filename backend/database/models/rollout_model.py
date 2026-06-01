from peewee import CharField, IntegerField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class Rollout(SoftDeleteModel):
    """커리큘럼에서 플래너 1회 실행.

    타겟 체크포인트 그룹(여러 개 가능)을 지정해 성공/실패를 판정한다. 그룹별 결과는
    RolloutResult 로 분리 기록.
    docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
    """

    class Meta:
        table_name = 'rollouts'

    STATUS_RUNNING = 'running'
    STATUS_FINISHED = 'finished'
    STATUS_STOPPED = 'stopped'
    STATUS_ERROR = 'error'

    __casts__ = {
        'target_group_ids': 'json',
        'noise_snapshot': 'json',
    }

    curriculum_id = IntegerField(null=True)
    target_group_ids = TextField(null=True)
    noise_snapshot = TextField(null=True)
    status = CharField(null=True, default=STATUS_RUNNING)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        for field_name in self.__casts__:
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    @classmethod
    def create(cls, **kwargs):
        for field_name in ('target_group_ids', 'noise_snapshot'):
            if field_name in kwargs and isinstance(kwargs[field_name], (list, dict)):
                kwargs[field_name] = json.dumps(kwargs[field_name])
        return super().create(**kwargs)

    @property
    def results(self):
        from .rollout_result_model import RolloutResult
        return list(RolloutResult.all_active().where(RolloutResult.rollout_id == self.id))

    def to_dict(self):
        data = super().to_dict()
        data['results'] = [r.to_dict() for r in self.results]
        return data
