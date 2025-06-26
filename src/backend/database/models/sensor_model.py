# /root/src/backend/database/models/robot_model.py
from .model import DBModel

class SensorModel(DBModel):
    TABLE_NAME = "sensors"
    
    COLUMNS = {
        'name': {'default': 'sensor_1'},
        'type': {'default': 'realsense_camera'},
        'settings': {'default': '{}'},
        'image': {'default': 'default_sensor_image.png'},
    }

    NEW_COLS = ['serial_no', 'topic']
    
    def __init__(self, **kwargs):
        super().__init__(table_name=self.TABLE_NAME, **kwargs)
        
        for key, value in kwargs.items():
            setattr(self, key, value)

    def set_data(self):
        if self.type == 'realsense_camera':
            self.serial_no = self.settings['serial_number']
            self.topic = f'/{self.name}/color/image_raw/compressed'