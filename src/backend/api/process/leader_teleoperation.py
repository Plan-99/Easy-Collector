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
from .subscribe_dynamixel import get_available_ports
from collections import defaultdict


class Leader():
    def __init__(self, agents, socketio_instance, teleop_setting, log_emit_id='leader_teleoperation') -> None:
        # ROS 노드 초기화ur5e/ur5e_scaled_pos_joint_traj_controller/command
        self.socketio_instance = socketio_instance
        self.log_emit_id = log_emit_id
        # self.origin = leader_robot_preset['origin']  # 다이나믹셀의 원점 위치
        # self.gripper_dxl_range = leader_robot_preset['gripper_dxl_range']  # 다이나믹셀의 원점 위치
        # self.sign_corrector = leader_robot_preset['sign_corrector']
        # self.dxl_ids = leader_robot_preset['dxl_ids']  # 다이나믹셀 ID
        # self.gripper_dxl_ids = leader_robot_preset.get('gripper_dxl_ids', [])
        # self.ema = float(leader_robot_preset.get('ema', 0.0)) # EMA 필터 값

        # # 트리거 그리퍼 설정 (기본값: 첫 번째 그리퍼)
        # self.trigger_gripper_index = 0
        # self.trigger_dxl_id = self.gripper_dxl_ids[self.trigger_gripper_index]

        self.is_paused = False

        self.address = 132  # 다이나믹셀의 현재 위치 주소 (주소 132번은 현재 위치)
        self.p_gain = 1
        self.is_synced = False
        self.agents = agents

        self.grouped_by_port = defaultdict(list)
        self.joint_map = teleop_setting['joint_map']

        grouped_by_port = self.group_joints_by_port()
        self.dxl_controllers = {}
        for port, joints in grouped_by_port.items():
            self.dxl_controllers[port] = DxlController(
                serial_port=port,
                dxl_ids=[item['dxl_id'] for item in joints],
                gripper_dxl_ids=[item['dxl_id'] for item in joints if item.get('is_gripper', False)],
            )

        self.ema = float(teleop_setting.get('ema', 0.0)) # EMA 필터 값


    def read_dxl_and_write_to_joint_map(self):
        group_by_port = self.group_joints_by_port()
        for port, joints in group_by_port.items():
            dxl_controller = self.dxl_controllers[port]
            positions = dxl_controller.read_all_dynamixel()
            for position in positions:
                joint = self.get_joint_by_dxl_id(port, position['id'])
                if joint is not None:
                    joint['dxl_position'] = position['position']

    def read_agent_and_write_to_joint_map(self):
        for agent in self.agents:
            joint_positions = agent.get_joint_states()
            for joint in self.joint_map:
                if joint['robot_id'] != agent.id:
                    continue
                joint_name = joint['joint_name']
                joint_index = agent.joint_names.index(joint_name)
                joint['agent_position'] = joint_positions[joint_index]


    def group_joints_by_port(self):
        grouped_by_port = defaultdict(list)
        for joint in self.joint_map:
            grouped_by_port[joint['port']].append(joint)

        return grouped_by_port
    
    def group_joints_by_agent(self):
        grouped_by_port = defaultdict(list)
        for joint in self.joint_map:
            grouped_by_port[joint['robot_id']].append(joint)

        return grouped_by_port
    
    def get_joint_by_joint_name(self, agent, joint_name):
        for joint in self.joint_map:
            if joint['robot_id'] != agent.id:
                continue
            if joint['joint_name'] == joint_name:
                return joint
        return None
    
    def get_joint_by_dxl_id(self, port, dxl_id):
        for joint in self.joint_map:
            if joint['port'] != port:
                continue
            if joint['dxl_id'] == dxl_id:
                return joint
        return None


    def get_rad_pos(self, joint):
        # pos = math.fmod((position - self.origin[dxl_id]), 4096)
        pos = (joint['dxl_position'] - joint['origin']) % 4096
        pos = pos / 4096 * 360
        if pos > 180:
            pos -= 360
        elif pos < -180:
            pos += 360
        return pos / 360 * 2 * math.pi * joint['sign']
    

    def rad_to_tick(self, rad, origin, sign):
        pos_deg = rad * 180.0 / math.pi * sign
        if pos_deg < 0:
            pos_deg += 360
        pos_steps = pos_deg / 360.0 * 4096.0
        position = (pos_steps + origin) % 4096
        return int(round(position))
    

    def sync_leader_robot(self):
        self.read_agent_and_write_to_joint_map()

        groups_by_port = self.group_joints_by_port()

        def sync_dxl_controller(port, dxl_joints):
            dxl_controller = self.dxl_controllers[port]
            for dxl_joint in dxl_joints:
                dxl_id = dxl_joint['dxl_id']
                agent_position = dxl_joint['agent_position']
                pos = self.rad_to_tick(agent_position, dxl_joint['origin'], dxl_joint['sign'])
                dxl_joint['dxl_goal_position'] = pos

            
                
            # for index, dxl_id in enumerate(self.dxl_ids):
            #     if dxl_id in self.tools:
            #         pos.append(follower_pos[index])
            #     else:
            #         pos.append(self.rad_to_tick(follower_pos[index], dxl_id))
            self.socketio_instance.emit(self.log_emit_id, {
                'log': f'Syncing Leader Robot',
                'type': 'stdout'
            })
            moved = dxl_controller.move_controller(dxl_joints)
            if moved:
                self.socketio_instance.emit(self.log_emit_id, {
                    'log': 'Will you start teleoperation? Close Gripper to Start!',
                    'type': 'stdout '
                })

                gripper_closed = False
                while not gripper_closed:
                    gripper_closed = True
                    for gripper_dxl_id in dxl_controller.gripper_dxl_ids:
                        gripper_pos = dxl_controller.read_dynamixel(gripper_dxl_id)  # 그리퍼의 현재 위치를 읽어오기
                        gripper_range = self.get_joint_by_dxl_id(port, gripper_dxl_id)['gripper_dxl_range']
                        if gripper_range[0] < gripper_range[1]:
                            if gripper_pos < gripper_range[1]:
                                gripper_closed = False
                        else:
                            if gripper_pos > gripper_range[1]:
                                gripper_closed = False
                
                dxl_controller.remove_torque()
            else:
                raise Exception
        
        for port, joints in groups_by_port.items():
            self.socketio_instance.start_background_task(
                target=sync_dxl_controller,
                port=port,
                dxl_joints=joints
            )

        time.sleep(0.1)

        while any([dc.controlled for dc in self.dxl_controllers.values()]):
            time.sleep(0.1)


    def get_gripper_pos(self, joint):
        gripper_pos_low = joint['joint_lower_bound']
        gripper_pos_high = joint['joint_upper_bound']
        gripper_pos = (joint['dxl_position'] - joint['gripper_dxl_range'][1]) / (joint['gripper_dxl_range'][0] - joint['gripper_dxl_range'][1]) * (gripper_pos_high - gripper_pos_low) + gripper_pos_low
        if gripper_pos < gripper_pos_low:
            return gripper_pos_low
        elif gripper_pos > gripper_pos_high:
            return gripper_pos_high
        return gripper_pos

        

    def position_pub(self, task_control):

        while not task_control['stop']:
            start = time.time()

            self.read_dxl_and_write_to_joint_map()

            group_by_agent = self.group_joints_by_agent()

            gripper_rad_pos = None
            for agent in self.agents:
                joint_list = group_by_agent[agent.id]
                for joint in joint_list:
                    joint_name = joint['joint_name']
                    joint_index = agent.joint_names.index(joint_name)
                    dxl_id = joint['dxl_id']
                    position = joint['dxl_position']
                    if joint.get('is_gripper', True):
                        joint['target_agent_position'] = self.get_gripper_pos(joint)

                    else:
                        target_pos = self.get_rad_pos(joint)

                        if 'prev_agent_position' not in joint:  # 첫 번째 루프에서 이전 위치 초기화
                            joint['prev_agent_position'] = target_pos

                        # EMA 필터 적용
                        joint['target_agent_position'] = self.ema * joint['prev_agent_position'] + (1 - self.ema) * target_pos
                        joint['prev_agent_position'] = joint['target_agent_position']


                action = agent.fetch_joint_map_to_action(joint_list)
                agent.move_joint_step(action)
                end = time.time()
            # self.target_pos[-1] = self.get_gripper_pos()
            #-----------------------------------------



            # 그리퍼를 벌리면 정지하도록 하는 코드--------------

            should_pause = [False] * len(self.agents)
            should_resume = [False] * len(self.agents)
            dxl_contoller_index = 0
            for port, dxl_controller in self.dxl_controllers.items():
                gripper_dxl_ids = dxl_controller.gripper_dxl_ids
                for dxl_id in gripper_dxl_ids:
                    gripper_joint = self.get_joint_by_dxl_id(port, dxl_id)
                    dxl_pos = gripper_joint['dxl_position']
                    gripper_range = gripper_joint['gripper_dxl_range']

                    if gripper_range[0] < gripper_range[1]:
                        should_pause[dxl_contoller_index] = dxl_pos < gripper_range[0] - 200
                        should_resume[dxl_contoller_index] = dxl_pos > gripper_range[0] - 100
                    else:
                        should_pause[dxl_contoller_index] = dxl_pos > gripper_range[0] + 200
                        should_resume[dxl_contoller_index] = dxl_pos < gripper_range[0] + 100

                dxl_contoller_index += 1

            if not self.is_paused and all(should_pause):
                self.is_paused = True
                for dxl_controller in self.dxl_controllers.values():
                    dxl_controller.enable_torque()
                self.socketio_instance.emit(self.log_emit_id, {
                    'log': 'Teleoperation Paused',
                    'type': 'stdout'
                })
                print("Teleoperation Paused")

            elif self.is_paused and all(should_resume):
                self.is_paused = False
                for dxl_controller in self.dxl_controllers.values():
                    dxl_controller.remove_torque()
                self.socketio_instance.emit(self.log_emit_id, {
                    'log': 'Teleoperation Resumed',
                    'type': 'stdout'
                })
                print("Teleoperation Resumed")

            # gripper_pos = positions[self.dxl_ids.index(self.trigger_dxl_id)]
            # gripper_range = self.gripper_dxl_range[self.trigger_gripper_index]

            # if gripper_range[0] < gripper_range[1]:
            #     should_pause = gripper_pos < gripper_range[0] - 300
            #     should_resume = gripper_pos > gripper_range[0] - 100
            # else:
            #     should_pause = gripper_pos > gripper_range[0] + 300
            #     should_resume = gripper_pos < gripper_range[0] + 100

            # if not self.is_paused and should_pause:
            #     self.is_paused = True
            #     self.dxl_controller.enable_torque()
            #     self.socketio_instance.emit(self.log_emit_id, {
            #         'log': 'Teleoperation Paused',
            #         'type': 'stdout'
            #     })
            #     print("Teleoperation Paused")

            # elif self.is_paused and should_resume:
            #     self.is_paused = False
            #     self.dxl_controller.remove_torque()
            #     self.socketio_instance.emit(self.log_emit_id, {
            #         'log': 'Teleoperation Resumed',
            #         'type': 'stdout'
            #     })
            #     print("Teleoperation Resumed")
            # #--------------------------------------------

        for dxl_controller in self.dxl_controllers.values():
            dxl_controller.remove_torque()
            dxl_controller.portHandler.closePort()
