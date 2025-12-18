from orator import Model, accessor, SoftDeletes
from orator.orm import has_one
import json


class Teleoperator(Model, SoftDeletes):
    __fillable__ = [
        'name',
        'type',
        'settings',
        'assembly_id',
    ]

    __casts__ = {
        'settings': 'json',
        'robot_ids': 'json',
    }

    __timestamps__ = True