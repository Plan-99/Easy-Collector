# /root/src/backend/database/models/robot_model.py
from .model import DBModel
import json

class TaskModel(DBModel):
    TABLE_NAME = "tasks"
    
    # 'COLUMNS'를 딕셔너리로 변경하여 컬럼별 속성(예: default)을 정의합니다.
    COLUMNS = {
        'name': {'default': 'task_1'},
        'robot_id': {'default': 1},
        'sensor_ids': {'default': [1, 2]},
        'home_pose': {'default': ['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0']},
        'end_pose': {'default': ['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0']},
        'prompt':{'default': 'Pick and place the object'},
        'episode_len': {'default': 100},
        'dataset_dir': {'default': '/root/src/backend/datasets'},
    }
    
    def __init__(self, **kwargs):
        # 이 부분은 변경할 필요 없습니다. 부모 클래스가 모든 것을 처리합니다.
        super().__init__(table_name=self.TABLE_NAME, **kwargs)
        
        for key, value in kwargs.items():
            setattr(self, key, value)