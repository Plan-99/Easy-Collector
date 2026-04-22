from peewee import CharField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


GRIPPER_CONFIGS = {
    'robotiq': {
        'usb_port': '/dev/ttyUSB0'
    }
}


class Gripper(SoftDeleteModel):
    class Meta:
        table_name = 'grippers'

    __casts__ = {
        'settings': 'json',
    }

    name = CharField(null=True)
    type = CharField(null=True)
    read_topic = CharField(null=True)
    write_topic = CharField(null=True)
    settings = TextField(null=True)
    image = CharField(null=True)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        for field_name in ('settings',):
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    @classmethod
    def create(cls, **kwargs):
        # Apply observer logic
        if 'settings' not in kwargs or not kwargs['settings']:
            gripper_type = kwargs.get('type', '')
            if gripper_type in GRIPPER_CONFIGS:
                kwargs['settings'] = GRIPPER_CONFIGS[gripper_type]
            else:
                kwargs['settings'] = {}
        if isinstance(kwargs.get('settings'), (dict, list)):
            kwargs['settings'] = json.dumps(kwargs['settings'])
        return super().create(**kwargs)
