# /root/src/backend/database/models/robot_model.py
from .model import DBModel

class LeaderRobotPresetModel(DBModel):
    TABLE_NAME = "leader_robot_presets"
    
    COLUMNS = {
        'name': {'default': 'preset_1'},
        'origin': {'default': '[968, 3055, 64461, 1030, 2007, 61410, 233]'},
        'dxl_ids': {'default': '[2, 1, 0, 3, 4, 5, 6]'}
    }
    
    def __init__(self, **kwargs):
        super().__init__(table_name=self.TABLE_NAME, **kwargs)
        
        for key, value in kwargs.items():
            setattr(self, key, value)