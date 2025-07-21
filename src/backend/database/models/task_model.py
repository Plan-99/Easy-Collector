from orator import Model, SoftDeletes
from orator.orm import belongs_to
from .robot_model import Robot
from .sensor_model import Sensor

class Task(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'robot_ids',
        'sensor_ids',
        'home_pose',
        'end_pose',
        'image',
        'episode_len',
        'sensor_img_size',
    ]

    __casts__ = {
        'sensor_ids': 'json',
        'robot_ids': 'json',
        'home_pose': 'json',
        'end_pose': 'json',
        'sensor_img_size': 'json',
    }
    
    __timestamps__ = True
