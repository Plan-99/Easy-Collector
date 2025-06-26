# /root/src/backend/database/models/robot_model.py
from .model import DBModel

class GripperModel(DBModel):
    TABLE_NAME = "grippers"
    
    COLUMNS = {
        'name': {'default': 'gripper_1'},
        'type': {'default': 'robotiq'},
        'read_topic': {'default': 'gripper/cmd'},
        'write_topic': {'default': 'gripper/cmd'},
        'settings': {'default': '{}'},
        'image': {'default': 'default_robot_image.png'},
    }
    
    def __init__(self, **kwargs):
        super().__init__(table_name=self.TABLE_NAME, **kwargs)
        
        for key, value in kwargs.items():
            setattr(self, key, value)