import datetime
import json
from peewee import CharField, TextField, IntegerField, DateTimeField, BooleanField, ForeignKeyField
from ..config.database import BaseModel
from .robot_model import Robot


class RobotPose(BaseModel):
    class Meta:
        table_name = 'robot_poses'

    robot = ForeignKeyField(Robot, backref='poses', on_delete='CASCADE')
    name = CharField(default='home')
    pose = TextField(null=True)  # JSON array of joint values
    color = CharField(default='')  # hex color e.g. '#4FC3F7'
    is_default = BooleanField(default=False)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)

    __casts__ = {
        'pose': 'json',
    }

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        val = getattr(self, 'pose', None)
        if val is not None and not isinstance(val, str):
            self.pose = json.dumps(val)
        return super().save(*args, **kwargs)

    def to_dict(self):
        data = {
            'id': self.id,
            'robot_id': self.robot_id,
            'name': self.name,
            'pose': self._get_pose(),
            'color': self.color or '',
            'is_default': self.is_default,
            'created_at': self.created_at.isoformat() if self.created_at else None,
            'updated_at': self.updated_at.isoformat() if self.updated_at else None,
        }
        return data

    def _get_pose(self):
        val = self.pose
        if isinstance(val, str):
            try:
                return json.loads(val)
            except (json.JSONDecodeError, TypeError):
                return val
        return val or []
