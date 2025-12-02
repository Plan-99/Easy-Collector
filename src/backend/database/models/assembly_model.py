from orator import Model, accessor, SoftDeletes
from orator.orm import has_many, belongs_to
import json
from ..models.teleoperator_model import Teleoperator as TeleoperatorModel
from .robot_model import Robot as RobotModel


class Assembly(Model, SoftDeletes):
    __fillable__ = [
        'name',
        'left_arm_id',
        'right_arm_id',
        'left_tool_id',
        'right_tool_id',
        'moblie_base_id',
    ]

    __timestamps__ = True

    @has_many
    def teleoperators(self):
        return TeleoperatorModel
    
    @belongs_to('left_arm_id')
    def left_arm(self):
        return RobotModel
    
    @belongs_to('right_arm_id')
    def right_arm(self):
        return RobotModel
    
    @belongs_to('left_tool_id')
    def left_tool(self):
        return RobotModel
    
    @belongs_to('right_tool_id')
    def right_tool(self):
        return RobotModel
    
    @belongs_to('moblie_base_id')
    def mobile_base(self):
        return RobotModel
    
