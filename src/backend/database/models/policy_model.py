import json
import sqlite3
from .model import DBModel

from ...configs.policy_config import POLICY_CONFIGS

class PolicyModel(DBModel):
    TABLE_NAME = "policies"
    COLUMNS = {
        'name': {'default': 'act_1'},
        'type': {'default': 'ACT'},
        'batch_size': {'default': 20},
        'num_epochs': {'default': 2000},
        'settings': {'default': '{}'},
        'is_deleted': {'default': False}
    }
    
    def __init__(self, **kwargs):
        
        super().__init__(table_name=self.TABLE_NAME, **kwargs)
        
        for key, value in kwargs.items():
            setattr(self, key, value)
             