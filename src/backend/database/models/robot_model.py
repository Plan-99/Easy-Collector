from orator import Model, accessor, SoftDeletes
from orator.orm import has_one
from .leader_robot_preset_model import LeaderRobotPreset
from ...configs.global_configs import SUPPORT_ROBOTS


# ROBOT_CONFIGS = {
#     'ur5e': {},
#     'piper': {}
# }


# class RobotObserver:
#     def creating(self, robot):
#         if not getattr(robot, 'settings', None):
#             robot_type = robot.type
#             if robot_type in ROBOT_CONFIGS:
#                 robot.settings = ROBOT_CONFIGS[robot_type]
#             else:
#                 robot.settings = {}


class Robot(Model, SoftDeletes):
    __fillable__ = [
        'name',
        'type',
        'role',
        'settings',
        'hide'
    ]

    __casts__ = {
        'joint_names': 'json',
        'gripper_names': 'json',
        'settings': 'json',
        'homepose': 'json',
    }

    __appends__ = [
        'tools',
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
        'can_port',
        'role',
        'company',
        'tool_inner',
        'ip_address',
    ]

    def get_robot_type_info(self):
        return next(
            (robot for robot in SUPPORT_ROBOTS if robot.get('name') == self.type), 
            None
        )
    

    @has_one
    def leader_robot_preset(self):
        return LeaderRobotPreset
    
    @accessor
    def company(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('company', '')
        return 'custom'
    
    @accessor
    def tools(self):
        tool_ids = self.settings.get('tool_ids', [])
        if len(tool_ids) == 0:
            return []
        
        return Robot.where_in('id', tool_ids).get()

    @accessor
    def joint_dim(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('joint_dim', 6)
        return self.settings['joint_names'].__len__()
    
    @accessor
    def joint_names(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('joint_names', [])

        return self.settings['joint_names']
    
    @accessor
    def read_topic(self):
        if self.type != 'custom':
            return f'/ec_robot_{self.id}' + self.get_robot_type_info().get('read_topic', '')

        return self.settings['read_topic']
    
    @accessor
    def read_topic_msg(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('read_topic_msg', '')
        return self.settings['read_topic_msg']
    
    @accessor
    def write_topic(self):
        if self.type != 'custom':
            return f'/ec_robot_{self.id}' + self.get_robot_type_info().get('write_topic', '')

        return self.settings['write_topic']
    
    @accessor
    def write_topic_msg(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('write_topic_msg', '')
        
        return self.settings['write_topic_msg']
    
    @accessor
    def move_action(self):
        return ''
    
    @accessor
    def yml_path(self):
        return ''
    
    @accessor
    def joint_upper_bounds(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('joint_upper_bounds', [])
        return self.settings['joint_upper_bounds']
    
    @accessor
    def joint_lower_bounds(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('joint_lower_bounds', [])
        return self.settings['joint_lower_bounds']
    
    @accessor
    def gripper_range(self):
        return self.settings['gripper_range'] if 'gripper_range' in self.settings else [0, 1]
    
    @accessor
    def can_port(self):
        if self.type != 'custom':
            network_interface = self.get_robot_type_info().get('network_interface', None)
            if network_interface == 'can':
                return self.settings.get('can_port', 'can_0')
        return None
    
    @accessor
    def ip_address(self):
        if self.type != 'custom':
            network_interface = self.get_robot_type_info().get('network_interface', None)
            if network_interface == 'ip':
                return self.settings.get('ip_address', '')
        return None
    
    @accessor
    def role(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('role', 'single_arm')
            
        return 'custom'
    
    @accessor
    def tool_inner(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('tool_inner', False)
        return self.settings.get('tool_inner', False)
    
    # @staticmethod
    # def boot():
    #     Robot.observe(RobotObserver())