from peewee import CharField, IntegerField, TextField, DateTimeField, FloatField
from ..config.database import SoftDeleteModel
import json
import datetime


class Checkpoint(SoftDeleteModel):
    class Meta:
        table_name = 'checkpoints'

    __casts__ = {
        'dataset_info': 'json',
        'train_settings': 'json',
    }

    # ── 학습 큐 상태 머신 ──────────────────────────────────────────────────
    # status 가 가질 수 있는 명시적 값들 (열거 전용; DB는 단순 CharField).
    # 큐 라이프사이클: queued → running → finished | failed | canceled
    # 'training' 은 legacy 별칭 — startup migration이 'running'으로 변환한다.
    STATUS_QUEUED = 'queued'
    STATUS_RUNNING = 'running'
    STATUS_FINISHED = 'finished'
    STATUS_FAILED = 'failed'
    STATUS_CANCELED = 'canceled'
    ACTIVE_STATUSES = (STATUS_QUEUED, STATUS_RUNNING)

    name = CharField(null=True)
    dir = CharField(null=True)
    file_name = CharField(null=True)
    task_id = IntegerField(null=True)
    policy_id = IntegerField(null=True)
    dataset_info = TextField(null=True)
    status = CharField(null=True)
    train_settings = TextField(null=True)
    load_model_id = IntegerField(null=True)
    loss = FloatField(null=True)
    best_epoch = IntegerField(null=True)
    # Lifecycle timestamps — TrainingScheduler가 전이마다 채운다.
    # queued_at: 큐 진입, started_at: scheduler가 picking, finished_at: 종료(어떤 결과든).
    queued_at = DateTimeField(null=True)
    started_at = DateTimeField(null=True)
    finished_at = DateTimeField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        for field_name in ('dataset_info', 'train_settings'):
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    @classmethod
    def create(cls, **kwargs):
        for field_name in ('dataset_info', 'train_settings'):
            if field_name in kwargs and isinstance(kwargs[field_name], (dict, list)):
                kwargs[field_name] = json.dumps(kwargs[field_name])
        return super().create(**kwargs)

    @property
    def task(self):
        from .task_model import Task
        return Task.find(self.task_id) if self.task_id else None

    @property
    def policy(self):
        from .policy_model import Policy
        return Policy.find(self.policy_id) if self.policy_id else None

    @property
    def load_model(self):
        return Checkpoint.find(self.load_model_id) if self.load_model_id else None

    @property
    def _dataset_info(self):
        """Get dataset_info with enriched dataset names."""
        from .dataset_model import Dataset
        val = self.dataset_info
        if isinstance(val, str):
            try:
                val = json.loads(val)
            except (json.JSONDecodeError, TypeError):
                return val

        if isinstance(val, dict):
            for key in val:
                dataset = Dataset.find(key)
                val[key]['name'] = dataset.name if dataset else 'Unknown'

        return val

    def _to_dict_shallow(self):
        """관계(task/policy/load_model)를 재귀 확장하지 않고 자기 컬럼만 반환.

        load_model(부모 체크포인트) 임베드 전용. fine-tune 계보가 깊으면
        to_dict() 가 load_model.to_dict() 로 계보 전체를 재귀 확장하고, 매
        단계마다 task.to_dict() 의 N+1 쿼리가 다시 터져 응답이 수십 초~분
        단위로 늘어진다. 부모는 name/id 등 컬럼 값만 있으면 충분(프론트는
        load_model.name 만 사용)하므로 shallow 로 끊는다."""
        return super().to_dict()

    def to_dict(self):
        data = super().to_dict()
        # Enrich dataset_info
        data['dataset_info'] = self._dataset_info
        # Include relationships
        task = self.task
        data['task'] = task.to_dict() if task else None
        policy = self.policy
        data['policy'] = policy.to_dict() if policy else None
        load_model = self.load_model
        # load_model 은 shallow — 깊은 계보 재귀 + task N+1 폭발 방지.
        data['load_model'] = load_model._to_dict_shallow() if load_model else None
        return data
