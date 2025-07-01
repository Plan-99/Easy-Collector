# /root/src/backend/database/models/robot_model.py
from .model import DBModel

class LeaderRobotPresetModel(DBModel):
    TABLE_NAME = "leader_robot_presets"
    
    COLUMNS = {
        'name': {'default': 'preset_1'},
        'robot_id': {'default': 2},
        'origin': {'default': '[12, 3039, 2867, 3049, 2938, 2961]'},
        'dxl_ids': {'default': '[0, 1, 2, 3, 4, 5, 6]'},
        'sign_corrector': {'default': '[1, -1, 1, 1, -1, 1, 1]'}
    }
    
    def __init__(self, **kwargs):
        super().__init__(table_name=self.TABLE_NAME, **kwargs)
        
        for key, value in kwargs.items():
            setattr(self, key, value)