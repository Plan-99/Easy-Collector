from peewee import (
    CharField, TextField, BooleanField, DateTimeField, IntegerField
)
from ..config.database import SoftDeleteModel
from ...configs.global_configs import SUPPORT_ROBOTS
import json
import datetime


class Robot(SoftDeleteModel):
    class Meta:
        table_name = 'robots'

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
        'sdk_control',
        'sdk_type',
    ]

    name = CharField(null=True)
    type = CharField(null=True)
    # NOTE: SQL 컬럼명은 'role'이지만 클래스 어트리뷰트는 property/setter로 노출하기 위해
    # 다른 이름(role_db)으로 둔다. 같은 이름의 property가 있으면 peewee가 INSERT 시
    # getattr(cls, 'role')로 property를 집어 들어 'get_sort_key' 에러가 난다.
    role_db = CharField(null=True, column_name='role')
    settings = TextField(null=True)
    homepose = TextField(null=True)
    hide = BooleanField(default=False)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        for field_name in ('settings', 'homepose'):
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    @property
    def _settings(self):
        """Parsed settings dict."""
        val = self.settings
        if isinstance(val, str):
            try:
                return json.loads(val)
            except (json.JSONDecodeError, TypeError):
                return {}
        return val or {}

    def get_robot_type_info(self):
        return next(
            (robot for robot in SUPPORT_ROBOTS if robot.get('name') == self.type),
            {}
        )

    @property
    def leader_robot_preset(self):
        from .leader_robot_preset_model import LeaderRobotPreset
        return LeaderRobotPreset.get_or_none(LeaderRobotPreset.robot_id == self.id)

    @property
    def company(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('company', '')
        return 'custom'

    @property
    def tools(self):
        tool_ids = self._settings.get('tool_ids', [])
        if len(tool_ids) == 0:
            return []
        return list(Robot.select().where(Robot.id.in_(tool_ids)))

    @property
    def joint_dim(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('joint_dim', 6)
        return len(self._settings.get('joint_names', []))

    @property
    def joint_names(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('joint_names', [])
        return self._settings.get('joint_names', [])

    @property
    def read_topic(self):
        if self.type != 'custom':
            info = self.get_robot_type_info()
            if info.get('sdk_control'):
                return f'/ec_robot_{self.id}/interpolated_joint_cmd'
            return f'/ec_robot_{self.id}' + info.get('read_topic', '')
        return self._settings.get('read_topic', '')

    @property
    def read_topic_msg(self):
        if self.type != 'custom':
            info = self.get_robot_type_info()
            if info.get('sdk_control'):
                return 'sensor_msgs/JointState'
            return info.get('read_topic_msg', '')
        return self._settings.get('read_topic_msg', '')

    @property
    def write_type(self):
        if self.type != 'custom':
            info = self.get_robot_type_info()
            if info.get('sdk_control'):
                return 'sdk'
            return info.get('write_type', 'topic')
        return self._settings.get('write_type', 'topic')

    @property
    def write_topic(self):
        if self.type != 'custom':
            info = self.get_robot_type_info()
            if info.get('sdk_control'):
                return ''
            return f'/ec_robot_{self.id}' + info.get('write_topic', '')
        return self._settings.get('write_topic', '')

    @property
    def write_topic_msg(self):
        if self.type != 'custom':
            info = self.get_robot_type_info()
            if info.get('sdk_control'):
                return ''
            return info.get('write_topic_msg', '')
        return self._settings.get('write_topic_msg', '')

    @property
    def interpolation(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('interpolation', False)
        return self._settings.get('interpolation', False)

    @property
    def sdk_control(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('sdk_control', False)
        return self._settings.get('sdk_control', False)

    @property
    def sdk_type(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('sdk_type', '')
        return self._settings.get('sdk_type', '')

    @property
    def move_action(self):
        return ''

    @property
    def yml_path(self):
        return ''

    @property
    def joint_upper_bounds(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('joint_upper_bounds', [])
        return self._settings.get('joint_upper_bounds', [])

    @property
    def joint_lower_bounds(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('joint_lower_bounds', [])
        return self._settings.get('joint_lower_bounds', [])

    @property
    def can_port(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'can_port' in custom_fields:
                port = self._settings.get('can_port', 'can0')
                if port.startswith('can_'):
                    port = 'can' + port[4:]
                return port
        return None

    @property
    def ip_address(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'ip_address' in custom_fields:
                return self._settings.get('ip_address', '')
        return None

    @property
    def port(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'port' in custom_fields:
                return self._settings.get('port', '')
        return None

    @property
    def changer_address(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'changer_address' in custom_fields:
                return self._settings.get('changer_address', '')
        return None

    @property
    def serial_port(self):
        if self.type != 'custom':
            custom_fields = self.get_robot_type_info().get('custom_fields', [])
            if 'serial_port' in custom_fields:
                return self._settings.get('serial_port', '')
        return None

    @property
    def role(self):
        # DB 값이 있으면 사용, 없으면 global_configs에서 가져옴
        db_role = self.role_db or ''
        if db_role:
            return db_role
        if self.type != 'custom':
            return self.get_robot_type_info().get('role', 'single_arm')
        return self._settings.get('role', '')

    @role.setter
    def role(self, value):
        self.role_db = value

    @property
    def tool_inner(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('tool_inner', False)
        return self._settings.get('tool_inner', False)

    @property
    def tool_index(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('tool_index', [])
        return self._settings.get('tool_index', [])

    @property
    def ik_available(self):
        if self.type != 'custom':
            return self.get_robot_type_info().get('ik_available', False)
        return 'ik_setting' in self._settings

    @property
    def is_sim(self):
        return self._settings.get('is_sim', False)

    def to_dict(self):
        data = super().to_dict()
        # Include leader_robot_preset
        preset = self.leader_robot_preset
        if preset:
            data['leader_robot_preset'] = preset.to_dict()
        else:
            data['leader_robot_preset'] = None
        # Include poses (hasMany)
        from .robot_pose_model import RobotPose
        poses = RobotPose.select().where(RobotPose.robot_id == self.id)
        data['poses'] = [p.to_dict() for p in poses]
        # homepose: default pose 또는 기존 homepose 필드 (하위호환)
        default_pose = next((p for p in poses if p.is_default), None)
        if default_pose:
            data['homepose'] = default_pose._get_pose()
        # Default ee_definitions (frontend General teleop tab 표시용)
        from ...configs.robot_ik_defaults import get_default_ee_definitions
        data['default_ee_definitions'] = get_default_ee_definitions(self.type)
        return data
