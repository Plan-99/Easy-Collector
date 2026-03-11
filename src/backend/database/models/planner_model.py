from orator import Model, SoftDeletes
from orator.orm import belongs_to

class Planner(Model, SoftDeletes):
    
    __fillable__ = [
        'name',
        'task_ids',
        'plan',
    ]

    __casts__ = {
        'task_ids': 'json',
        'plan': 'json',
    }
    
    __timestamps__ = True