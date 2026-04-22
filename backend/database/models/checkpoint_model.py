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
        data['load_model'] = load_model.to_dict() if load_model else None
        return data
