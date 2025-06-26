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

# 데이터베이스 테이블 생성 쿼리
create_robot_table_query = RobotModel.generate_create_table_sql()
create_policy_table_query = PolicyModel.generate_create_table_sql()
create_task_table_query = TaskModel.generate_create_table_sql()
create_gripper_table_query = GripperModel.generate_create_table_sql()
create_sensor_table_query = SensorModel.generate_create_table_sql()
create_checkpoint_table_query = CheckpointModel.generate_create_table_sql()

try:
    cursor.execute(create_robot_table_query)
    cursor.execute(create_policy_table_query)
    cursor.execute(create_task_table_query)
    cursor.execute(create_gripper_table_query)
    cursor.execute(create_sensor_table_query)
    cursor.execute(create_checkpoint_table_query)

except sqlite3.Error as e:
    print(f"데이터베이스 오류: {e}")

finally:
    if conn:
        conn.close()
        
        
# 데이터 추가
try:
    default_policy = PolicyModel()
    default_policy.create()
    
    default_task = TaskModel()
    default_task.create()
    
    default_robot = RobotModel()
    default_robot.create()
    
    default_sensor = SensorModel(
        name='camera1'
    )
    default_sensor.create()
    
    custom_sensor = SensorModel(
        name='camera2'
    )
    custom_sensor.create()
    
    default_gripper = GripperModel()
    default_gripper.create()
    
    default_checkpoint = CheckpointModel()
    default_checkpoint.create()
    
    # sensor = SensorModel.find_one({ 'id': 1 })
    # sensor.settings['serial_number'] = 'aaaaa'
    # sensor.update()
    
    # policy = PolicyModel.find_one(name='my_act_policy_1')
    # policy.settings['learning_rate'] = 12
    # policy.update()

    print("데이터를 성공적으로 추가했습니다.")
except sqlite3.IntegrityError:
    print("데이터가 이미 존재하거나 다른 오류가 발생했습니다.")

# 연결 종료
conn.close()