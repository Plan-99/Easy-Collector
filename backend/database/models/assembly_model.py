from peewee import CharField, IntegerField, BooleanField, DateTimeField
from ..config.database import SoftDeleteModel
import datetime


class Assembly(SoftDeleteModel):
    class Meta:
        table_name = 'assemblies'

    __appends__ = [
        'robots'
    ]

    name = CharField(null=True)
    left_arm_id = IntegerField(null=True)
    right_arm_id = IntegerField(null=True)
    left_tool_id = IntegerField(null=True)
    right_tool_id = IntegerField(null=True)
    mobile_base_id = IntegerField(null=True)
    hide = BooleanField(default=False)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        return super().save(*args, **kwargs)

    @property
    def teleoperators(self):
        from .teleoperator_model import Teleoperator
        return list(Teleoperator.select().where(
            Teleoperator.assembly_id == self.id,
            Teleoperator.deleted_at.is_null()
        ))

    @property
    def left_arm(self):
        from .robot_model import Robot
        return Robot.find(self.left_arm_id) if self.left_arm_id else None

    @property
    def right_arm(self):
        from .robot_model import Robot
        return Robot.find(self.right_arm_id) if self.right_arm_id else None

    @property
    def left_tool(self):
        from .robot_model import Robot
        return Robot.find(self.left_tool_id) if self.left_tool_id else None

    @property
    def right_tool(self):
        from .robot_model import Robot
        return Robot.find(self.right_tool_id) if self.right_tool_id else None

    @property
    def mobile_base(self):
        from .robot_model import Robot
        return Robot.find(self.mobile_base_id) if self.mobile_base_id else None

    @property
    def robots(self):
        from .robot_model import Robot
        robots = []
        robot_ids = set([
            self.left_arm_id,
            self.right_arm_id,
            self.left_tool_id,
            self.right_tool_id,
            self.mobile_base_id,
        ])
        for robot_id in robot_ids:
            if robot_id is not None:
                robot = Robot.find(robot_id)
                if robot:
                    robots.append(robot.to_dict())
        return robots

    def to_dict(self):
        data = super().to_dict()
        # Include teleoperators
        data['teleoperators'] = [t.to_dict() for t in self.teleoperators]
        # Include related robot models
        for rel in ('left_arm', 'right_arm', 'left_tool', 'right_tool', 'mobile_base'):
            obj = getattr(self, rel)
            data[rel] = obj.to_dict() if obj else None
        return data
