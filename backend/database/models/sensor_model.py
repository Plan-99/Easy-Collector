from peewee import CharField, TextField, BooleanField, DateTimeField
from ..config.database import SoftDeleteModel
from ...configs.global_configs import SUPPORT_SENSORS, get_sensor_by_name
import json
import datetime


class Sensor(SoftDeleteModel):
    class Meta:
        table_name = 'sensors'

    __casts__ = {
        'settings': 'json',
        'resolution': 'json',
    }

    __appends__ = [
        'company',
        'serial_no',
        'ip_address',
        'device_index',
        'read_topic',
        'read_topic_msg',
        'resolution',
        'process_id'
    ]

    name = CharField(null=True)
    type = CharField(null=True)
    settings = TextField(null=True)
    hide = BooleanField(default=False)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        val = getattr(self, 'settings', None)
        if val is not None and not isinstance(val, str):
            self.settings = json.dumps(val)
        return super().save(*args, **kwargs)

    @classmethod
    def create(cls, **kwargs):
        # Apply observer logic
        if 'settings' not in kwargs or not kwargs['settings']:
            sensor_type = kwargs.get('type', '')
            info = get_sensor_by_name(sensor_type)
            if info is not None:
                kwargs['settings'] = info
            else:
                kwargs['settings'] = {}
        if isinstance(kwargs.get('settings'), (dict, list)):
            kwargs['settings'] = json.dumps(kwargs['settings'])
        return super().create(**kwargs)

    @property
    def _settings(self):
        val = self.settings
        if isinstance(val, str):
            try:
                return json.loads(val)
            except (json.JSONDecodeError, TypeError):
                return {}
        return val or {}

    def get_sensor_type_info(self):
        return next(
            (sensor for sensor in SUPPORT_SENSORS if sensor.get('name') == self.type),
            {}
        )

    @property
    def company(self):
        sensor_info = self.get_sensor_type_info()
        return sensor_info.get('company', 'Unknown')

    @property
    def read_topic(self):
        if self.type != 'custom':
            return f'/ec_sensor_{self.id}' + self.get_sensor_type_info().get('read_topic', '')
        return self._settings.get('read_topic', '')

    @property
    def read_topic_msg(self):
        if self.type != 'custom':
            return self.get_sensor_type_info().get('read_topic_msg', '')
        return self._settings.get('read_topic_msg', '')

    @property
    def serial_no(self):
        return self._settings.get('serial_number', None)

    @property
    def resolution(self):
        if self.type == 'custom':
            return self._settings.get('resolution', [640, 480])
        return self.get_sensor_type_info().get('resolution', [640, 480])

    @property
    def ip_address(self):
        return self._settings.get('ip_address', None)

    @property
    def device_index(self):
        return self._settings.get('device_index', None)

    @property
    def process_id(self):
        return f'sensor_{self.id}'
