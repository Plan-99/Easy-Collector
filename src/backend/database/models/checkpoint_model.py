# /root/src/backend/database/models/robot_model.py
from .model import DBModel
import json

class CheckpointModel(DBModel):
    TABLE_NAME = "checkpoints"
    
    # 'COLUMNS'를 딕셔너리로 변경하여 컬럼별 속성(예: default)을 정의합니다.
    COLUMNS = {
        'name': {'default': 'ckpt_1'},
        'dir': {'default': '/root/src/backend/policies/act/checkpoints/pick_can_ckpt'},
        'file_name':{'default': 'policy_best.ckpt'},
        'task_id': {'default': 1},
        'policy_id': {'default': 1},
        'settings': {'default': '{}'},
    }
    
    def __init__(self, **kwargs):
        # 이 부분은 변경할 필요 없습니다. 부모 클래스가 모든 것을 처리합니다.
        super().__init__(table_name=self.TABLE_NAME, **kwargs)
        
        for key, value in kwargs.items():
            setattr(self, key, value)