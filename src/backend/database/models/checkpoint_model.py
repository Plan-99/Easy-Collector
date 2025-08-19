from orator import Model, SoftDeletes
from orator.orm import belongs_to, accessor
from .task_model import Task
from .policy_model import Policy
from .dataset_model import Dataset
import json

class Checkpoint(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'dir',
        'file_name',
        'task_id',
        'policy_id',
        'dataset_info',
        'is_training',
        'train_settings',
        'load_model_id',
        'loss',
        'best_epoch',
    ]
    
    __casts__ = {
        'dataset_info': 'json',
        'is_training': 'boolean',
        'train_settings': 'json',
    }
    
    __timestamps__ = True

    @belongs_to
    def task(self):
        return Task
    
    @belongs_to
    def policy(self):
        return Policy
    
    @belongs_to
    def load_model(self):
        return Checkpoint
    
    @accessor
    def dataset_info(self):
        dataset_info = self.get_raw_attribute('dataset_info')
        if isinstance(dataset_info, str):
            dataset_info = json.loads(dataset_info)

        for key, value in dataset_info.items():
            dataset_info[key]['name'] = Dataset.find(key).name if Dataset.find(key) else 'Unknown'

        return dataset_info