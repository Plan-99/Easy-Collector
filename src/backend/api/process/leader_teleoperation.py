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
import sys


class Leader():
    def __init__(self, node: Node, agents, socketio_instance, teleop_setting) -> None:
        # ROS 노드 초기화ur5e/ur5e_scaled_pos_joint_traj_controller/command
        self.node = node
        self.socketio_instance = socketio_instance
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

        # 각 에이전트에 맞게 joint_map 업데이트
        self.udpate_joint_map_with_agent_info()

        grouped_by_port = self.group_joints_by_port()
        self.dxl_controllers = {}
        for port, joints in grouped_by_port.items():
            self.dxl_controllers[port] = DxlController(
                serial_port=port,
                dxl_ids=[item['dxl_id'] for item in joints],
                gripper_dxl_ids=[item['dxl_id'] for item in joints if item.get('is_gripper', False)],
            )

        self.ema = float(teleop_setting.get('ema', 0.0)) # EMA 필터 값


    def udpate_joint_map_with_agent_info(self):
        for joint_info in self.joint_map:
            for agent in self.agents:
                if agent.id == joint_info['robot_id']:
                    try:
                        j_idx = agent.joint_names.index(joint_info['joint_name'])
                        joint_info['joint_upper_bound'] = agent.joint_upper_bounds[j_idx]
                        joint_info['joint_lower_bound'] = agent.joint_lower_bounds[j_idx]
                    except ValueError:
                        continue


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
                if 'is_dummy_gripper' in joint and joint['is_dummy_gripper']:
                    continue
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
            if 'is_dummy_gripper' in joint and joint['is_dummy_gripper']:
                continue
            grouped_by_port[joint['robot_id']].append(joint)

        return grouped_by_port
    
    def get_joint_by_joint_name(self, agent, joint_name):
        for joint in self.joint_map:
            if 'is_dummy_gripper' in joint and joint['is_dummy_gripper']:
                continue
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
        self.is_synced = False
        self.read_agent_and_write_to_joint_map()

        groups_by_port = self.group_joints_by_port()

        def sync_dxl_controller(port, dxl_joints):
            dxl_controller = self.dxl_controllers[port]
            for dxl_joint in dxl_joints:
                if 'is_dummy_gripper' in dxl_joint and dxl_joint['is_dummy_gripper']:
                    continue
                dxl_id = dxl_joint['dxl_id']
                agent_position = dxl_joint['agent_position']
                pos = self.rad_to_tick(agent_position, dxl_joint['origin'], dxl_joint['sign'])
                dxl_joint['dxl_goal_position'] = pos

            
                
            # for index, dxl_id in enumerate(self.dxl_ids):
            #     if dxl_id in self.tools:
            #         pos.append(follower_pos[index])
            #     else:
            #         pos.append(self.rad_to_tick(follower_pos[index], dxl_id))

            print("Syncing Leader Robot")
            moved = dxl_controller.move_controller(dxl_joints)
            if moved:
                print("[NOTICE] Will you start teleoperation? Close Gripper to Start!")

                gripper_closed = False

                count = 0
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

                        if count == 0 and gripper_closed:
                            print("[ERROR] Teleoperation Failed. You have to keep controller's gripper opened")
                            raise Exception
                        
                    count = 1

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

        self.is_synced = True
        print("[SUCCESS] Leader Robot Synced Successfully!")


    def get_gripper_pos(self, joint):
        gripper_pos_low = joint['joint_lower_bound']
        gripper_pos_high = joint['joint_upper_bound']
        sign = joint['sign']
        if sign < 0:
            gripper_pos = gripper_pos_high - (joint['dxl_position'] - joint['gripper_dxl_range'][1]) / (joint['gripper_dxl_range'][0] - joint['gripper_dxl_range'][1]) * (gripper_pos_high - gripper_pos_low)
        else:
            gripper_pos = (joint['dxl_position'] - joint['gripper_dxl_range'][1]) / (joint['gripper_dxl_range'][0] - joint['gripper_dxl_range'][1]) * (gripper_pos_high - gripper_pos_low) + gripper_pos_low
        if gripper_pos < gripper_pos_low:
            return gripper_pos_low
        elif gripper_pos > gripper_pos_high:
            return gripper_pos_high
        return gripper_pos

        

    def position_pub(self, task_control):
        try:
            is_joint_trajectory = False
            for agent in self.agents:
                if agent.write_topic_msg == 'trajectory_msgs/JointTrajectory':
                    is_joint_trajectory = True
                    break
            if is_joint_trajectory:
                rate = self.node.create_rate(3)  # 50Hz
            else:
                rate = self.node.create_rate(25)  # 10Hz
            while rclpy.ok() and not task_control.get('stop', False) and not task_control.get('episode_stop', False):
                
                start = time.time()

                # 1. 하드웨어 읽기 작업 (여기서 SerialException 등이 발생할 확률이 높음)
                try:
                    self.read_dxl_and_write_to_joint_map()
                except Exception as e:
                    print(f"[ERROR] Dynamixel Read Failed: {e}", flush=True)
                    break # 읽기 실패 시 루프 중단

                group_by_agent = self.group_joints_by_agent()

                gripper_rad_pos = None
                for agent in self.agents:
                    joint_list = group_by_agent[agent.id]
                    for joint in joint_list:
                        if 'is_dummy_gripper' in joint and joint['is_dummy_gripper']:
                            continue
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
                should_pause = [False]
                should_resume = [False]
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
                    time.sleep(0.1)  # 토크가 걸릴 시간을 약간 줌
                    print("Teleoperation Paused")

                elif self.is_paused and all(should_resume):
                    self.is_paused = False
                    for dxl_controller in self.dxl_controllers.values():
                        dxl_controller.remove_torque()
                    time.sleep(0.1) 
                    print("Teleoperation Resumed")

                rate.sleep()

        except Exception as e:
            # 예상치 못한 전체 루프 에러 처리
            import traceback
            error_msg = traceback.format_exc()
            print(f"[CRITICAL ERROR] position_pub loop crashed:\n{error_msg}", flush=True)
            # SocketIOStream을 통해 프론트엔드로도 전송됨
            raise e
        
        finally:
            # 에러가 나든 정상 종료되든 반드시 실행되는 블록
            print("Cleaning up Leader Robot resources...", flush=True)
            for port, dxl_controller in self.dxl_controllers.items():
                try:
                    # 포트 닫기
                    dxl_controller.portHandler.closePort()
                    print(f"Port {port} closed safely.", flush=True)
                except:
                    print(f"Failed to close port {port} cleanly.", flush=True)


    def leader_teleop_workflow(self, task_control):
        """
        이 함수는 ProcessManager.start_function에 의해 백그라운드에서 실행됩니다.
        """
        print("[SYSTEM] Starting Leader Sync Process...")
        
        # 1. 동기화 실행 (내부의 while 루프 덕분에 완료될 때까지 여기서 블로킹됨)
        self.sync_leader_robot()

        
        # 만약 중간에 사용자가 중지 버튼을 눌렀다면 체크
        if task_control['stop']:
            print("[SYSTEM] Workflow stopped by user during sync.")
            return

        print("[SUCCESS] Sync completed. Transitioning to Teleoperation...")

        # 2. 곧바로 텔레옵(무한 루프) 실행
        # leader.position_pub 내부에 task_control을 체크하는 로직이 있으면 더욱 좋습니다.
        self.position_pub(task_control=task_control)
