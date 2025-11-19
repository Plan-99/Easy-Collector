# #!/usr/bin/env python
from concurrent.futures import thread
import rclpy
from rclpy.node import Node
import math
# import numpy as np
import time
import math
import threading
from ...env.dxl_controller import DxlController


class Leader():
    def __init__(self, agent, socketio_instance, leader_robot_preset, log_emit_id='leader_teleoperation', port='/dev/ttyUSB0') -> None:
        # ROS 노드 초기화ur5e/ur5e_scaled_pos_joint_traj_controller/command
        self.socketio_instance = socketio_instance
        self.log_emit_id = log_emit_id
        self.origin = leader_robot_preset['origin']  # 다이나믹셀의 원점 위치
        self.gripper_dxl_range = leader_robot_preset['gripper_dxl_range']  # 다이나믹셀의 원점 위치
        self.sign_corrector = leader_robot_preset['sign_corrector']
        self.dxl_ids = leader_robot_preset['dxl_ids']  # 다이나믹셀 ID
        self.gripper_dxl_ids = leader_robot_preset.get('gripper_dxl_ids', [])
        self.ema = float(leader_robot_preset.get('ema', 0.0)) # EMA 필터 값

        # 트리거 그리퍼 설정 (기본값: 첫 번째 그리퍼)
        self.trigger_gripper_index = 0
        self.trigger_dxl_id = self.gripper_dxl_ids[self.trigger_gripper_index]

        self.is_paused = False


        self.address = 132  # 다이나믹셀의 현재 위치 주소 (주소 132번은 현재 위치)
        self.p_gain = 1
        self.is_synced = False
        self.agent = agent
        self.dxl_controller = DxlController(port, self.dxl_ids, self.gripper_dxl_ids)
        self.target_pos = [0] * self.agent.joint_len  # 목표 위치 초기화 (7개의 관절)
        self.target_tool_pos = [0] * len(self.agent.tools)  # 도구 그리퍼 목표 위치 초기화

    def get_rad_pos(self, position, dxl_id):
        # pos = math.fmod((position - self.origin[dxl_id]), 4096)
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
        time.sleep(1.0)
        follower_pos = self.agent.joint_states
        pos = []
        for index, dxl_id in enumerate(self.dxl_ids):
            if dxl_id in self.gripper_dxl_ids:
                pos.append(follower_pos[index])
            else:
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
            trigger_index = 0
            gripper_pos = self.dxl_controller.read_dynamixel(self.gripper_dxl_ids[trigger_index])  # 그리퍼의 현재 위치를 읽어오기
            if self.gripper_dxl_range[trigger_index][0] < self.gripper_dxl_range[trigger_index][1]:
                while gripper_pos < self.gripper_dxl_range[trigger_index][1]:
                    gripper_pos = self.dxl_controller.read_dynamixel(self.gripper_dxl_ids[trigger_index])  # 그리퍼의 현재 위치를 읽어오기
            else:
                while gripper_pos > self.gripper_dxl_range[trigger_index][1]:
                    gripper_pos = self.dxl_controller.read_dynamixel(self.gripper_dxl_ids[trigger_index])  # 그리퍼의 현재 위치를 읽어오기
            self.is_synced = True
            self.dxl_controller.remove_torque()
        else:
            raise Exception
    

    def get_gripper_pos(self, dxl_pos, dxl_id):
        gripper_index = self.gripper_dxl_ids.index(dxl_id)
        gripper_pos_low = self.agent.gripper_range[0]
        gripper_pos_high = self.agent.gripper_range[-1]
        gripper_pos = (dxl_pos - self.gripper_dxl_range[gripper_index][1]) / (self.gripper_dxl_range[gripper_index][0] - self.gripper_dxl_range[gripper_index][1]) * (gripper_pos_high - gripper_pos_low) + gripper_pos_low
        if gripper_pos < gripper_pos_low:
            return gripper_pos_low
        elif gripper_pos > gripper_pos_high:
            return gripper_pos_high
        return gripper_pos

        

    def position_pub(self, task_control):

        first_flag = True
        prev_pos = [0] * self.agent.joint_len

        while not task_control['stop']:

            start = time.time()

            # 타겟 포지션 가져오기 ------------------
            positions = self.dxl_controller.read_all_dynamixel()
            tool_index = 0
            for index, dxl_id in enumerate(self.dxl_ids):
                try:
                    position = positions[index]
                    if dxl_id in self.gripper_dxl_ids:
                        if len(self.agent.tools) == 0:
                            self.target_pos[index] = self.get_gripper_pos(position, dxl_id)
                            tool_index += 1
                        else:
                            self.target_tool_pos[tool_index] = self.get_gripper_pos(position, dxl_id)
                    else:
                        self.target_pos[index] = self.get_rad_pos(position, dxl_id)

                        if first_flag:  # 첫 번째 루프에서 이전 위치 초기화
                            prev_pos[index] = self.target_pos[index]

                        # EMA 필터 적용
                        self.target_pos[index] = self.ema * prev_pos[index] + (1 - self.ema) * self.target_pos[index]
                        prev_pos[index] = self.target_pos[index]

                except Exception as e:
                    self.socketio_instance.emit(self.log_emit_id, {
                        'log': f'Error reading from Dynamixel ID {dxl_id}: {str(e)}',
                        'type': 'stderr'
                    })
                    print(f"Error reading from Dynamixel ID {dxl_id}: {str(e)}")
                    continue

            first_flag = False

            # self.target_pos[-1] = self.get_gripper_pos()
            #-----------------------------------------



            # 그리퍼를 벌리면 정지하도록 하는 코드--------------
            try:
                gripper_pos = positions[self.dxl_ids.index(self.trigger_dxl_id)]
                gripper_range = self.gripper_dxl_range[self.trigger_gripper_index]
            except IndexError:
                print(f"Error: DXL ID {self.trigger_dxl_id} not found in self.dxl_ids.")
                continue
    
            if gripper_range[0] < gripper_range[1]:
                should_pause = gripper_pos < gripper_range[0] - 300
                should_resume = gripper_pos > gripper_range[0] - 100
            else:
                should_pause = gripper_pos > gripper_range[0] + 300
                should_resume = gripper_pos < gripper_range[0] + 100

            if not self.is_paused and should_pause:
                self.is_paused = True
                self.dxl_controller.enable_torque()
                self.socketio_instance.emit(self.log_emit_id, {
                    'log': 'Teleoperation Paused',
                    'type': 'stdout'
                })
                print("Teleoperation Paused")

            elif self.is_paused and should_resume:
                self.is_paused = False
                self.dxl_controller.remove_torque()
                self.socketio_instance.emit(self.log_emit_id, {
                    'log': 'Teleoperation Resumed',
                    'type': 'stdout'
                })
                print("Teleoperation Resumed")
            #--------------------------------------------

            self.agent.move_step(self.target_pos, self.target_tool_pos)
            end = time.time()

        self.dxl_controller.remove_torque()
        self.dxl_controller.portHandler.closePort()