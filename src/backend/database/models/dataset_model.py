from orator import Model, SoftDeletes
from orator.orm import belongs_to
from .task_model import Task
from orator.orm import accessor
import os

class Dataset(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'task_id',
    ]
    
    __appends__ = [
        'robot_ids',
        'sensor_ids',
        'sensor_img_size',
        'episode_len',
    ]

    __casts__ = {
        'sensor_ids': 'json',
    }
    
    __timestamps__ = True


    @belongs_to
    def task(self):
        return Task
    
    @accessor
    def robot_ids(self):
        return self.task.robot_ids if self.task else None
    
    @accessor
    def sensor_ids(self):
        return self.task.sensor_ids if self.task else None
    
    @accessor
    def sensor_img_size(self):
        return self.task.sensor_img_size if self.task else None
    
    @accessor
    def episode_len(self):
        return self.task.episode_len if self.task else None
    