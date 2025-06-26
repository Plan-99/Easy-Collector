import sqlite3
import os
from ..database.db_init import get_db_connection
from ..database.models.robot_model import RobotModel
from ..database.models.policy_model import PolicyModel
from ..database.models.task_model import TaskModel
from ..database.models.gripper_model import GripperModel
from ..database.models.sensor_model import SensorModel
from ..database.models.checkpoint_model import CheckpointModel

conn = get_db_connection()
cursor = conn.cursor()


# sensor1 = SensorModel(name='camera1')
# sensor1.create()

# sensor2 = SensorModel(name='camera2')
# sensor2.create()

task = TaskModel.find_one({ 'id': 1 })
task.sensor_ids = [5, 6]
task.update()




# 연결 종료
conn.close()

