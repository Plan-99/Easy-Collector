from orator import Model, SoftDeletes
from orator.orm import belongs_to, accessor
from .robot_model import Robot
from .sensor_model import Sensor
from .assembly_model import Assembly

class Task(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'home_pose',
        'end_pose',
        'image',
        'episode_len',
        'settings',
        'assembly_id',
        'sensor_ids',
        'sensor_img_size',
        'sensor_cropped_area',
        'sensor_rotate',
    ]

    __casts__ = {
        'home_pose': 'json',
        'end_pose': 'json',
        'settings': 'json',
        'sensor_ids': 'json',
        'sensor_img_size': 'json',
        'sensor_cropped_area': 'json',
        'sensor_rotate': 'json',
    }

    __appends__ = [
        'joint_dim',
        'sensors',
        'sensor_img_size',
        'sensor_cropped_area',
        'sensor_rotate'
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
    
    @accessor
    def sensors(self):
        sensors = []
        for sensor_id in self.sensor_ids:
            sensor = Sensor.find(sensor_id)
            if sensor:
                sensors.append(sensor.to_dict())
        return sensors
    
    @accessor
    def sensor_img_size(self):
        img_size = {}
        for sensor_id in self.sensor_ids:
            if str(sensor_id) not in self.settings.get('sensors', {}):
                img_size[str(sensor_id)] = [640, 480]
            else:
                img_size[str(sensor_id)] = self.settings['sensors'][str(sensor_id)].get('img_size', [640, 480])
        return img_size

    @accessor
    def sensor_cropped_area(self):
        cropped_area = {}
        for sensor_id in self.sensor_ids:
            if str(sensor_id) not in self.settings.get('sensors', {}):
                cropped_area[str(sensor_id)] = [0, 0, 640, 480]
            else:
                cropped_area[str(sensor_id)] = self.settings['sensors'][str(sensor_id)].get('cropped_area', [0, 0, 640, 480])
        return cropped_area
    
    @accessor
    def sensor_rotate(self):
        rotation = {}
        for sensor_id in self.sensor_ids:
            if str(sensor_id) not in self.settings.get('sensors', {}):
                rotation[str(sensor_id)] = 0
            else:
                rotation[str(sensor_id)] = self.settings['sensors'][str(sensor_id)].get('rotate', 0)
        return rotation
    
    @belongs_to('assembly_id')
    def assembly(self):
        return Assembly
        
