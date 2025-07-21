from orator import Model, accessor, SoftDeletes
import json

SENSOR_CONFIGS = {
    'realsense_camera': {
        'serial_number': '00000000'
    },
    'custom_sensor': {
        'process_cmd': 'rosrun my_package my_sensor_node',
        'topic_name': 'custom_sensor_topic'
    }
}


class SensorObserver:
    def creating(self, sensor):
        if not getattr(sensor, 'settings', None):
            sensor_type = sensor.type
            if sensor_type in SENSOR_CONFIGS:
                sensor.settings = SENSOR_CONFIGS[sensor_type]
            else:
                sensor.settings = {}
                

class Sensor(Model, SoftDeletes):
    __fillable__ = [
        'name',
        'type',
        'settings',
    ]

    __casts__ = {
        'settings': 'json',
    }

    __appends__ = ['serial_no', 'topic']

    
    @accessor
    def serial_no(self):
        settings = json.loads(self.get_raw_attribute('settings'))
        if 'serial_number' in settings:
            return settings['serial_number']
        
        return None
    

    @accessor
    def topic(self):
        if self.type == 'realsense_camera':
            return f'/ec_sensor_{self.id}/color/image_raw/compressed'
        
        return None
    

    @staticmethod
    def boot():
        Sensor.observe(SensorObserver())