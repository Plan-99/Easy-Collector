from orator import Model, accessor, SoftDeletes
from ...configs.global_configs import SUPPORT_SENSORS, get_sensor_by_name
import json

# SENSOR_CONFIGS = {
#     'realsense_camera': {
#         'serial_number': '00000000'
#     },
#     'custom_sensor': {
#         'process_cmd': 'rosrun my_package my_sensor_node',
#         'topic_name': 'custom_sensor_topic'
#     }
# }


class SensorObserver:
    def creating(self, sensor):
        if not getattr(sensor, 'settings', None):
            sensor_type = sensor.type
            if get_sensor_by_name(sensor_type) is not None:
                sensor.settings = get_sensor_by_name(sensor_type)
            else:
                sensor.settings = {}
                

class Sensor(Model, SoftDeletes):
    __fillable__ = [
        'name',
        'type',
        'settings',
        'hide'
    ]

    __casts__ = {
        'settings': 'json',
        'resolution': 'json',
    }

    __appends__ = [
        'company',
        'serial_no', 
        'ip_address',
        'read_topic',
        'read_topic_msg',
        'resolution',
    ]

    def get_sensor_type_info(self):
        return next(
            (sensor for sensor in SUPPORT_SENSORS if sensor.get('name') == self.type), 
            {}
        )
    
    @accessor
    def company(self):
        sensor_info = self.get_sensor_type_info()
        return sensor_info.get('company', 'Unknown')
    
    @accessor
    def read_topic(self):
        if self.type != 'custom':
            return f'/ec_sensor_{self.id}' + self.get_sensor_type_info().get('read_topic', '')
        
        return self.settings['read_topic']

    @accessor
    def read_topic_msg(self):
        if self.type != 'custom':
            return self.get_sensor_type_info().get('read_topic_msg', '')
        
        return self.settings['read_topic_msg']
    
    @accessor
    def serial_no(self):
        settings = json.loads(self.get_raw_attribute('settings'))
        if 'serial_number' in settings:
            return settings['serial_number']
        return None
    
    @accessor
    def resolution(self):
        if self.type == 'custom':
            settings = json.loads(self.get_raw_attribute('settings'))
            if 'resolution' in settings:
                return settings['resolution']
            return [640, 480]

        return self.get_sensor_type_info().get('resolution', [640, 480])
    
    @accessor
    def ip_address(self):
        settings = json.loads(self.get_raw_attribute('settings'))
        if 'ip_address' in settings:
            return settings['ip_address']
        return None
    
    

    @staticmethod
    def boot():
        Sensor.observe(SensorObserver())