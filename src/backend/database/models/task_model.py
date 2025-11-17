from orator import Model, SoftDeletes
from orator.orm import belongs_to, accessor
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
        'settings',
    ]

    __casts__ = {
        'sensor_ids': 'json',
        'robot_ids': 'json',
        'home_pose': 'json',
        'end_pose': 'json',
        'sensor_img_size': 'json',
        'settings': 'json',
    }
    
    
    __timestamps__ = True


    @accessor
    def home_pose(self):
        home_pose = {}
        for id in self.robot_ids:
            robot = Robot.find(id)
            if str(id) not in self.settings.get('robots', {}):
                print('a', id)
                home_pose[str(id)] = [0.0] * len(robot.joint_names)
            else:
                home_pose[str(id)] = self.settings['robots'][str(id)].get('home_pose', [0.0] * len(robot.joint_names))
                print(home_pose, self.id)
        return home_pose
