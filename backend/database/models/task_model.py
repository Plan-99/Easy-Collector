from peewee import CharField, IntegerField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class Task(SoftDeleteModel):
    class Meta:
        table_name = 'tasks'

    __casts__ = {
        'home_pose': 'json',
        'end_pose': 'json',
        'settings': 'json',
        'sensor_ids': 'json',
        'sensor_img_size': 'json',
        'sensor_cropped_area': 'json',
        'sensor_rotate': 'json',
    }

    __appends__ = [
        'joint_dim',
        'sensors',
        'sensor_img_size',
        'sensor_cropped_area',
        'sensor_rotate',
        'sensor_sam3'
    ]

    name = CharField(null=True)
    home_pose = TextField(null=True)
    end_pose = TextField(null=True)
    image = CharField(null=True)
    episode_len = IntegerField(null=True)
    settings = TextField(null=True)
    assembly_id = IntegerField(null=True)
    sensor_ids = TextField(null=True)
    sensor_img_size = TextField(null=True)
    sensor_cropped_area = TextField(null=True)
    sensor_rotate = TextField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        for field_name in self.__casts__:
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    def _get_json(self, field_name):
        val = getattr(self, field_name, None)
        if isinstance(val, str):
            try:
                return json.loads(val)
            except (json.JSONDecodeError, TypeError):
                return val
        return val

    @property
    def _settings(self):
        return self._get_json('settings') or {}

    @property
    def _sensor_ids(self):
        return self._get_json('sensor_ids') or []

    @property
    def assembly(self):
        from .assembly_model import Assembly
        return Assembly.find(self.assembly_id) if self.assembly_id else None

    @property
    def home_pose_computed(self):
        from .robot_model import Robot
        home_pose = {}
        assembly = self.assembly
        if not assembly:
            return home_pose
        for robot_dict in assembly.robots:
            robot = Robot.find(robot_dict['id'])
            if str(robot.id) not in self._settings.get('robots', {}):
                home_pose[str(robot.id)] = [0.0] * len(robot.joint_names)
            else:
                home_pose[str(robot.id)] = self._settings['robots'][str(robot.id)].get('home_pose', [0.0] * len(robot.joint_names))
        return home_pose

    @property
    def joint_dim(self):
        from .robot_model import Robot
        joint_dim = 0
        assembly = self.assembly
        if not assembly:
            return joint_dim
        for robot_dict in assembly.robots:
            robot = Robot.find(robot_dict['id'])
            joint_dim += robot.joint_dim
        return joint_dim

    @property
    def sensors(self):
        from .sensor_model import Sensor
        sensors = []
        for sensor_id in self._sensor_ids:
            sensor = Sensor.find(sensor_id)
            if sensor:
                sensors.append(sensor.to_dict())
        return sensors

    @property
    def sensor_img_size_computed(self):
        img_size = {}
        for sensor_id in self._sensor_ids:
            if str(sensor_id) not in self._settings.get('sensors', {}):
                img_size[str(sensor_id)] = [640, 480]
            else:
                img_size[str(sensor_id)] = self._settings['sensors'][str(sensor_id)].get('img_size', [640, 480])
        return img_size

    @property
    def sensor_cropped_area_computed(self):
        cropped_area = {}
        for sensor_id in self._sensor_ids:
            if str(sensor_id) not in self._settings.get('sensors', {}):
                cropped_area[str(sensor_id)] = [0, 0, 640, 480]
            else:
                cropped_area[str(sensor_id)] = self._settings['sensors'][str(sensor_id)].get('cropped_area', [0, 0, 640, 480])
        return cropped_area

    @property
    def sensor_rotate_computed(self):
        rotation = {}
        for sensor_id in self._sensor_ids:
            if str(sensor_id) not in self._settings.get('sensors', {}):
                rotation[str(sensor_id)] = 0
            else:
                rotation[str(sensor_id)] = self._settings['sensors'][str(sensor_id)].get('rotate', 0)
        return rotation

    @property
    def sensor_sam3_computed(self):
        # SAM3 per-sensor segmentation config. Default = disabled / no-op so
        # the rest of the pipeline can read this dict unconditionally.
        default = {
            'enabled': False,
            'text_prompts': [],
            'boxes': [],
            'mode': 'background',  # 'off' | 'background' | 'object'
            'color': [0, 0, 0],
        }
        out = {}
        for sensor_id in self._sensor_ids:
            sid = str(sensor_id)
            stored = self._settings.get('sensors', {}).get(sid, {}).get('sam3') or {}
            out[sid] = {**default, **stored}
        return out

    def to_dict(self):
        data = super().to_dict()
        # Override appended properties
        data['home_pose'] = self.home_pose_computed
        data['joint_dim'] = self.joint_dim
        data['sensors'] = self.sensors
        data['sensor_img_size'] = self.sensor_img_size_computed
        data['sensor_cropped_area'] = self.sensor_cropped_area_computed
        data['sensor_rotate'] = self.sensor_rotate_computed
        data['sensor_sam3'] = self.sensor_sam3_computed
        data['sensor_ids'] = self._sensor_ids
        data['settings'] = self._settings
        # Include assembly
        assembly = self.assembly
        data['assembly'] = assembly.to_dict() if assembly else None
        return data
