from peewee import CharField, IntegerField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class Teleoperator(SoftDeleteModel):
    class Meta:
        table_name = 'teleoperators'

    __casts__ = {
        'settings': 'json',
        'robot_ids': 'json',
    }

    name = CharField(null=True)
    type = CharField(null=True)
    settings = TextField(null=True)
    assembly_id = IntegerField(null=True)
    robot_ids = TextField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        for field_name in ('settings', 'robot_ids'):
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    @classmethod
    def first_or_create(cls, **kwargs):
        """Find first matching record or create a new one."""
        instance = cls.get_or_none(**{k: v for k, v in kwargs.items()})
        if instance:
            return instance
        return cls.create(**kwargs)

    @property
    def _settings(self):
        val = self.settings
        if isinstance(val, str):
            try:
                return json.loads(val)
            except (json.JSONDecodeError, TypeError):
                return {}
        return val or {}
