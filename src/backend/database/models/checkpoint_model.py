from orator import Model, SoftDeletes
from orator.orm import belongs_to
from .task_model import Task
from .policy_model import Policy

class Checkpoint(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'dir',
        'file_name',
        'task_id',
        'policy_id',
        'dataset_info',
    ]
    
    __casts__ = {
        'dataset_info': 'json',
    }
    
    __timestamps__ = True

    @belongs_to
    def task(self):
        return Task
    
    @belongs_to
    def policy(self):
        return Policy