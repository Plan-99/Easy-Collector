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
        'mobile_base_id',
    ]

    __timestamps__ = True

    __appends__ = [
        'robots'
    ]

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
    
    @belongs_to('mobile_base_id')
    def mobile_base(self):
        return RobotModel
    
    @accessor
    def robots(self):
        robots = []
        # 중복된 로봇은 한 번만 추가
        robot_ids = set([
            self.left_arm_id,
            self.right_arm_id,
            self.left_tool_id,
            self.right_tool_id,
            self.mobile_base_id,
        ])
        for robot_id in robot_ids:
            if robot_id is not None:
                robot = RobotModel.find(robot_id).to_dict()
                if robot:
                    robots.append(robot)
        return robots
