from peewee import CharField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class Planner(SoftDeleteModel):
    class Meta:
        table_name = 'planners'

    __casts__ = {
        'task_ids': 'json',
        'plan': 'json',
        'plans': 'json',
        'blocks': 'json',
    }

    name = CharField(null=True)
    task_ids = TextField(null=True)
    # Legacy: 단일 flat block 리스트. 새 버전에서는 plans(그룹별 분기)로 대체.
    # 마이그레이션 호환을 위해 컬럼은 유지.
    plan = TextField(null=True)
    # plans: [{ id, workspace_ids: [...], blocks: [...] }, ...]
    # 그룹은 워크스페이스의 로봇 겹침에 따라 connected component로 결정됨.
    plans = TextField(null=True)
    blocks = TextField(null=True)
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
        for field_name in ('task_ids', 'plan', 'plans', 'blocks'):
            if field_name in kwargs and isinstance(kwargs[field_name], (list, dict)):
                kwargs[field_name] = json.dumps(kwargs[field_name])
        return super().create(**kwargs)
