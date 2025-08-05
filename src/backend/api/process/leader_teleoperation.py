# #!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math
# import numpy as np
import time
import math

from ...env.dxl_controller import DxlController

class Leader():
    def __init__(self, agent, socketio_instance, leader_robot_preset, log_emit_id='leader_teleoperation', port='/dev/ttyUSB0', gripper=None) -> None:
        # ROS 노드 초기화ur5e/ur5e_scaled_pos_joint_traj_controller/command
        self.socketio_instance = socketio_instance
        self.log_emit_id = log_emit_id
        self.origin = leader_robot_preset['origin']  # 다이나믹셀의 원점 위치
        self.gripper_dxl_range = leader_robot_preset['gripper_dxl_range']  # 다이나믹셀의 원점 위치
        self.sign_corrector = leader_robot_preset['sign_corrector']
        self.dxl_ids = leader_robot_preset['dxl_ids']  # 다이나믹셀 ID
        
        self.address = 132  # 다이나믹셀의 현재 위치 주소 (주소 132번은 현재 위치)
        self.p_gain = 1
        self.is_synced = False
        
        self.gripper_config = gripper
        
        self.agent = agent
        self.dxl_controller = DxlController(port, self.dxl_ids)
        

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

        self.socketio_instance.emit(self.log_emit_id, {
            'log': f'Syncing Leader Robot',
            'type': 'stdout'
        })

        moved = self.dxl_controller.move_controller(pos)
        
        if moved:
            self.socketio_instance.emit(self.log_emit_id, {
                'log': 'Will you start teleoperation? Close Gripper to Start!',
                'type': 'stdout '
            })
            
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


    def position_pub(self, task_control):
        target_pos = [0] * 7
        while not task_control['stop']:
            for index, dxl_id in enumerate(self.dxl_ids[:-1]):
                # 다이나믹셀 값 읽어오기
                position = self.dxl_controller.read_dynamixel(dxl_id)
                    
                target_pos[index] = self.get_rad_pos(position, dxl_id)

            # 그리퍼 매핑
            target_pos[-1] = self.get_gripper_pos()
                
            self.agent.move_step(target_pos)
        
        self.dxl_controller.portHandler.closePort()
