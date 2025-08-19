from orator import Model, accessor, SoftDeletes
from orator.orm import has_one
import json
from .leader_robot_preset_model import LeaderRobotPreset


ROBOT_CONFIGS = {
    'ur5e': {},
    'piper': {}
}


class RobotObserver:
    def creating(self, robot):
        if not getattr(robot, 'settings', None):
            robot_type = robot.type
            if robot_type in ROBOT_CONFIGS:
                robot.settings = ROBOT_CONFIGS[robot_type]
            else:
                robot.settings = {}


class Robot(Model, SoftDeletes):
    __fillable__ = [
        'name',
        'type',
        'settings',
    ]

    __casts__ = {
        'joint_names': 'json',
        'settings': 'json',
    }

    __appends__ = [
        'joint_upper_bounds',
        'joint_lower_bounds',
        'gripper_range',
        'joint_dim',
        'joint_names',
        'read_topic',
        'read_topic_msg',
        'write_topic',
        'write_topic_msg',
        'move_action',
        'yml_path',
    ]

    @has_one
    def leader_robot_preset(self):
        return LeaderRobotPreset
    
    
    @accessor
    def joint_dim(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return 7
        if type == 'ur5e':
            return 6

        return self.settings['joint_names'].__len__()
    
    @accessor
    def joint_names(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        if type == 'ur5e':
            return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        return self.settings['joint_names']
    
    @accessor
    def read_topic(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return f'/ec_robot_{self.id}/joint_states_single'
        if type == 'ur5e':
            return f'/ec_robot_{self.id}/joint_states'

        return self.settings['read_topic']
    
    @accessor
    def read_topic_msg(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return 'sensor_msgs/JointState'
        if type == 'ur5e':
            return 'sensor_msgs/JointState'
        
        return self.settings['read_topic_msg']
    
    @accessor
    def write_topic(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return f'/ec_robot_{self.id}/joint_states'
        if type == 'ur5e':
            return f'/ec_robot_{self.id}/joint_states'

        return self.settings['write_topic']
    
    @accessor
    def write_topic_msg(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return 'sensor_msgs/JointState'
        if type == 'ur5e':
            return 'sensor_msgs/JointState'
        
        return self.settings['write_topic_msg']
    
    @accessor
    def move_action(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return ''
        if type == 'ur5e':
            return 'move_to_joint_position'

        return ''
    
    @accessor
    def yml_path(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return 'piper.yml'
        if type == 'ur5e':
            return 'ur5e.yml'

        return ''
    
    @accessor
    def joint_upper_bounds(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return [2.618, 2.618, 0, 1.745, 1.22, 2.094, 0.0726]
        if type == 'ur5e': 
            return [2.618, 2.618, 2.618, 1.745, 1.22, 2.094, 0]

        return self.settings['joint_upper_bounds']
    
    @accessor
    def joint_lower_bounds(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return [-2.618, 0, -2.618, -1.745, -1.22, -2.094, 0]
        if type == 'ur5e':
            return [-2.618, -2.618, -2.618, -1.745, -1.22, -2.094, 0]

        return self.settings['joint_lower_bounds']
    
    @accessor
    def gripper_range(self):
        type = self.get_raw_attribute('type')
        if type == 'piper':
            return [0, 0.0726]
        if type == 'ur5e':
            return [0, 0.08]

        return []
    
    @staticmethod
    def boot():
        Robot.observe(RobotObserver())
    