from orator import Model, SoftDeletes
from orator.orm import belongs_to, accessor
from .robot_model import Robot
from .sensor_model import Sensor
from .assembly_model import Assembly

class Task(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'sensor_ids',
        'home_pose',
        'end_pose',
        'image',
        'episode_len',
        'sensor_settings',
        'settings',
        'assembly_id',
    ]

    __casts__ = {
        'sensor_ids': 'json',
        'home_pose': 'json',
        'end_pose': 'json',
        'sensor_settings': 'json',
        'settings': 'json',
    }

    __appends__ = [
        'joint_dim'
    ]

    __timestamps__ = True


    @accessor
    def home_pose(self):
        home_pose = {}
        if not self.assembly:
            return home_pose
        for robot in self.assembly.robots:
            robot = Robot.find(robot['id'])
            if str(robot.id) not in self.settings.get('robots', {}):
                home_pose[str(robot.id)] = [0.0] * len(robot.joint_names)
            else:
                home_pose[str(robot.id)] = self.settings['robots'][str(robot.id)].get('home_pose', [0.0] * len(robot.joint_names))
        return home_pose
    
    @accessor
    def joint_dim(self):
        joint_dim = 0
        if not self.assembly:
            return joint_dim
        for robot in self.assembly.robots:
            robot = Robot.find(robot['id'])
            joint_dim += robot.joint_dim
        return joint_dim
    
    @belongs_to('assembly_id')
    def assembly(self):
        return Assembly
        
