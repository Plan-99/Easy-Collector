from peewee import (
    CharField, IntegerField, TextField, DateTimeField
)
from ..config.database import SoftDeleteModel
import json
import datetime


class LeaderRobotPreset(SoftDeleteModel):
    class Meta:
        table_name = 'leader_robot_presets'

    __casts__ = {
        'origin': 'json',
        'gripper_dxl_range': 'json',
        'dxl_ids': 'json',
        'sign_corrector': 'json',
        'gripper_dxl_ids': 'json',
    }

    name = CharField(null=True)
    robot_id = IntegerField(null=True)
    gripper_id = IntegerField(null=True)
    origin = TextField(null=True)
    gripper_dxl_range = TextField(null=True)
    dxl_ids = TextField(null=True)
    sign_corrector = TextField(null=True)
    port_name = CharField(null=True)
    gripper_dxl_ids = TextField(null=True)
    ema = CharField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        # Serialize JSON fields
        for field_name in self.__casts__:
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    @classmethod
    def first_or_create(cls, **kwargs):
        """Find first matching record or create a new one."""
        try:
            return cls.get_or_none(**kwargs) or cls.create(**kwargs)
        except Exception:
            return cls.create(**kwargs)
