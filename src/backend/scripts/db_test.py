import sqlite3
import os
from ..database.db_init import get_db_connection
from ..database.models.robot_model import RobotModel
from ..database.models.policy_model import PolicyModel
from ..database.models.task_model import TaskModel
from ..database.models.gripper_model import GripperModel
from ..database.models.sensor_model import SensorModel
from ..database.models.checkpoint_model import CheckpointModel
from ..database.models.leader_robot_preset_model import LeaderRobotPresetModel

conn = get_db_connection()
cursor = conn.cursor()


# piper_robot = RobotModel(name='piper')
# piper_robot.create()

robot = RobotModel.find_one({ 'id': 7 })
robot.type = 'piper'
robot.origin = [12, 3039, 2867, 3049, 2938, 2961]
robot.read_topic = 'piper/joint_states_single'
robot.write_topic = 'piper/joint_states'
robot.update()

preset = LeaderRobotPresetModel.find_one({ 'id': 1 })
preset.robot_id = 7
preset.origin = [12, 3039, 2867, 3049, 2938, 2961, 2050]
preset.update()


# sensor1 = SensorModel(name='camera1')
# sensor1.create()

# sensor2 = SensorModel(name='camera2')
# sensor2.create()

# task = TaskModel.find_one({ 'id': 1 })
# task.sensor_ids = [5, 6]
# task.update()




# 연결 종료
conn.close()

