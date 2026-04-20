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
        'joint_dim',
        'joint_names',
        'read_topic',
        'read_topic_msg',
        'write_type',
        'write_topic',
        'write_topic_msg',
        'move_action',
        'yml_path',
        'can_port',
        'role',
        'company',
        'tool_inner',
        'tool_index',
        'ip_address',
        'port',
        'changer_address',
        'serial_port',
        'ik_available',
        'is_sim',
        'interpolation',
    ]

    def get_robot_type_info(self):
        return next(
            (robot for robot in SUPPORT_ROBOTS if robot.get('name') == self.type), 
            {}
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
    def write_type(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('write_type', 'topic')
        return self.settings.get('write_type', 'topic')
    

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
    def interpolation(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('interpolation', False)
        return self.settings.get('interpolation', False)

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
    def can_port(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'can_port' in custom_fields:
                port = self.settings.get('can_port', 'can0')
                if port.startswith('can_'):
                    port = 'can' + port[4:]
                return port
        return None
    
    @accessor
    def ip_address(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'ip_address' in custom_fields:
                return self.settings.get('ip_address', '')
        return None
    
    @accessor
    def port(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'port' in custom_fields:
                return self.settings.get('port', '')
        return None
    
    @accessor
    def changer_address(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'changer_address' in custom_fields:
                return self.settings.get('changer_address', '')
        return None
    
    @accessor
    def serial_port(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'serial_port' in custom_fields:
                return self.settings.get('serial_port', '')
        return None

    
    @accessor
    def role(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('role', 'single_arm')
            
        return self.settings.get('role', '')
    
    @accessor
    def tool_inner(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('tool_inner', False)
        return self.settings.get('tool_inner', False)
    
    @accessor
    def tool_index(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('tool_index', [])
        return self.settings.get('tool_index', [])
    
    @accessor
    def ik_available(self):
        if self.type != 'custom':
            robot_info = self.get_robot_type_info()
            return 'ik_setting' in robot_info
        return 'ik_setting' in (self.settings or {})

    @accessor
    def is_sim(self):
        return self.settings.get('is_sim', False)
    
    # @staticmethod
    # def boot():
    #     Robot.observe(RobotObserver())
