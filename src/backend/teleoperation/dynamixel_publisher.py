# #!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64
import dynamixel_sdk as dxl  # 다이나믹셀 SDK 사용
import sys
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from robotiq_85_msgs.msg import GripperCmd
# from sensor_msgs.msg import JointState
# from gello_controller.srv import MoveRobot, MoveRobotRequest
# from collections import deque
# from std_srvs.srv import Trigger, TriggerResponse
import argparse
import threading
# import numpy as np
import time
import math

# Import database and data models
import os
from ..database.models.robot_model import Robot as RobotModel
from ..database.models.gripper_model import Gripper as GripperModel
from ..database.models.leader_robot_preset_model import LeaderRobotPreset as LeaderRobotPresetModel

from ..env.agent import Agent
from ..env.dxl_controller import DxlController

class Leader(Node):
    def __init__(self, robot, leader_robot_preset, gripper=None) -> None:
        # ROS 노드 초기화ur5e/ur5e_scaled_pos_joint_traj_controller/command
        super().__init__('dynamixel_publisher_2')
        
        
        self.origin = leader_robot_preset.origin  # 다이나믹셀의 원점 위치
        self.gripper_dxl_range = leader_robot_preset.gripper_dxl_range  # 다이나믹셀의 원점 위치
        self.sign_corrector = leader_robot_preset.sign_corrector
        self.dxl_ids = leader_robot_preset.dxl_ids  # 다이나믹셀 ID
        
        self.address = 132  # 다이나믹셀의 현재 위치 주소 (주소 132번은 현재 위치)
        self.rate = self.create_rate(10)  # 10Hz로 퍼블리시
        self.p_gain = 1
        self.is_synced = False
        
        self.gripper_config = gripper
        
        self.agent = Agent(robot)
        
        time.sleep(0.1)

        self.dxl_controller = DxlController('/dev/ttyUSB0', self.dxl_ids)

        self.sync_leader_robot()
        self.position_pub()
        

    def get_rad_pos(self, position, dxl_id):
        pos = (position - self.origin[dxl_id]) % 4096
        pos = pos / 4096 * 360
        if pos > 180:
            pos -= 360
        elif pos < -180:
            pos += 360
        return pos / 360 * 2 * math.pi * self.sign_corrector[dxl_id]
    
    
    def rad_to_tick(self, rad, dxl_id):
        pos_deg = rad * 180.0 / math.pi * self.sign_corrector[dxl_id]
        
        if pos_deg < 0:
            pos_deg += 360
            
        pos_steps = pos_deg / 360.0 * 4096.0

        position = (pos_steps + self.origin[dxl_id]) % 4096
        
        return int(round(position))

    
    def sync_leader_robot(self):
        follower_pos = self.agent.joint_states
        pos = []
        
        for index, dxl_id in enumerate(self.dxl_ids[:-1]):
            pos.append(self.rad_to_tick(follower_pos[index], dxl_id))

        moved = self.dxl_controller.move_controller(pos)
        
        if moved:
            print('Will you start teleoperation? Close Gripper to Start!')
            
            gripper_pos = self.dxl_controller.read_dynamixel(self.dxl_ids[-1])  # 그리퍼의 현재 위치를 읽어오기
            while gripper_pos > self.gripper_dxl_range[1]:
                gripper_pos = self.dxl_controller.read_dynamixel(self.dxl_ids[-1])  # 그리퍼의 현재 위치를 읽어오기

            self.is_synced = True
            self.dxl_controller.remove_torque()
        else:
            raise Exception
        
    def get_gripper_pos(self):
        if self.gripper_config is None:
            gripper_tick_pos = self.dxl_controller.read_dynamixel(self.dxl_ids[-1])
            gripper_pos_low = self.agent.gripper_range[0]
            gripper_pos_high = self.agent.gripper_range[-1]
            gripper_pos = (gripper_tick_pos - self.gripper_dxl_range[1]) / (self.gripper_dxl_range[0] - self.gripper_dxl_range[1]) * (gripper_pos_high - gripper_pos_low) + gripper_pos_low
            return gripper_pos
        else:
            return 0.0


    def position_pub(self):
        target_pos = [0] * 7
        while rclpy.ok():
            if not self.is_synced:
                continue
            for index, dxl_id in enumerate(self.dxl_ids[:-1]):
                # 다이나믹셀 값 읽어오기
                position = self.dxl_controller.read_dynamixel(dxl_id)
                    
                target_pos[index] = self.get_rad_pos(position, dxl_id)

                
            # 그리퍼 매핑
            target_pos[-1] = self.get_gripper_pos()
                
            self.agent.move_step(target_pos)
            self.rate.sleep()

        
        self.dxl_controller.portHandler.closePort()
        
        
def main(args):
    from orator import DatabaseManager, Model

    BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    DB_DIR = os.path.join(BASE_DIR, 'backend/database')
    DB_PATH = os.path.join(DB_DIR, 'main.db')

    config = {
        'mysql': {
            'driver': 'sqlite',
            'database': DB_PATH,
        }
    }

    db = DatabaseManager(config)
    Model.set_connection_resolver(db)

    leader_robot_preset = LeaderRobotPresetModel.find(args['leader_robot_preset_id'])
    robot = RobotModel.find(leader_robot_preset.robot_id).to_dict()
    gripper = GripperModel.args['gripper_id'] if 'gripper_id' in args else None

    try:
        rclpy.init(args=None)
        leader = Leader(robot, leader_robot_preset , gripper)
        rclpy.spin(leader)

    except KeyboardInterrupt:
        pass
    finally:
        leader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--leader_robot_preset_id', default=None, required=True)
    
    main(vars(parser.parse_args()))
    sys.exit(0)
