# /root/src/backend/database/models/robot_model.py
from .model import DBModel

class RobotModel(DBModel):
    TABLE_NAME = "robots"
    
    COLUMNS = {
        'name': {'default': 'robot_1'},
        'type': {'default': 'ur5e'},
        'joint_dim': {'default': 6},
        'joint_names': {'default': '["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]'},
        'read_topic': {'default': '/ur5e/joint_states'},
        'read_topic_msg': {'default': 'sensor_msgs/JointState'},
        'write_topic': {'default': '/ur5e/ur5e_scaled_pos_joint_traj_controller/command'},
        'write_topic_msg': {'default': 'trajectory_msgs/JointTrajectory'},
        'move_action': {'default': '/ur5e/ur5e_scaled_pos_joint_traj_controller/follow_joint_trajectory'},
        'yml_path': {'default': 'ur5e.yml'},
        'launch_script': {'default': None},
        'settings': {'default': '{}'},
        'image': {'default': 'default_robot_image.png'},
    }

    NEW_COLS = []
    
    def __init__(self, **kwargs):
        super().__init__(table_name=self.TABLE_NAME, **kwargs)
        
        for key, value in kwargs.items():
            setattr(self, key, value)

    def set_data(self):
        if self.type == 'ur5e':
            