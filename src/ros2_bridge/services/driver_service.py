# -*- coding: utf-8 -*-
"""
DriverService gRPC servicer.
ROS2 컨테이너에서 ros2 launch/run 프로세스를 관리한다.
기존 robot.py / sensor.py의 드라이버 시작 로직을 추출.
"""
import json
import os
import signal
import subprocess
import threading
import time

import grpc

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc


class DriverServiceServicer(pb_grpc.DriverServiceServicer):
    def __init__(self, node):
        """
        Args:
            node: rclpy.node.Node - ROS2 node for topic listing.
        """
        self.node = node
        self.processes = {}  # name -> subprocess.Popen
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Robot Driver
    # ------------------------------------------------------------------
    def StartRobotDriver(self, request, context):
        settings = json.loads(request.settings_json) if request.settings_json else {}
        robot_id = request.robot_id
        company = request.company
        rtype = request.type
        process_id = request.process_id

        # 기존 프로세스 정리
        self._stop(process_id)

        command = self._build_robot_command(company, rtype, robot_id, settings)
        if not command:
            return pb.DriverStatus(success=False, message=f'Unsupported company: {company}')

        # Piper CAN 설정
        if company == 'Piper':
            can_cmd = self._piper_can_setup(robot_id, settings)
            if can_cmd:
                self._start_subprocess(f'can_config_{robot_id}', can_cmd)
                time.sleep(1)

        proc = self._start_subprocess(process_id, command)
        if proc is None:
            return pb.DriverStatus(success=False, message='Failed to start process')

        # 보간 노드 시작: interpolation=True인 로봇
        if settings.get('interpolation'):
            interp_id = f'interp_{robot_id}'
            self._stop(interp_id)
            ns = f'ec_robot_{robot_id}'
            # write_topic에서 상대 토픽명만 추출 (e.g., /ec_robot_1/joint_states → joint_states)
            write_topic = settings.get('write_topic', '/joint_states').rsplit('/', 1)[-1]
            interp_cmd = [
                'python3', '-m', 'ros2_bridge.interpolation_node',
                '--ros-args', '-r', f'__ns:=/{ns}',
                '-p', f'output_topic:={write_topic}',
            ]
            self._start_subprocess(interp_id, interp_cmd)

        # JAKA servo enable
        if company == 'JAKA':
            self._jaka_servo_enable()

        return pb.DriverStatus(success=True, message='Driver started', pid=proc.pid)

    def StopRobotDriver(self, request, context):
        self._stop(request.name)
        # 보간 노드도 함께 정리 (robot_{id} → interp_{id})
        if request.name.startswith('robot_'):
            robot_id = request.name[len('robot_'):]
            self._stop(f'interp_{robot_id}')
        return pb.StatusResponse(success=True, message='Stopped')

    # ------------------------------------------------------------------
    # Sensor Driver
    # ------------------------------------------------------------------
    def StartSensorDriver(self, request, context):
        settings = json.loads(request.settings_json) if request.settings_json else {}
        sensor_id = request.sensor_id
        company = request.company
        stype = request.type
        process_id = request.process_id

        if stype == 'custom':
            return pb.DriverStatus(success=True, message='Custom sensor uses external topic')

        command = self._build_sensor_command(company, stype, sensor_id, settings)
        if not command:
            return pb.DriverStatus(success=False, message=f'Unsupported sensor company: {company}')

        proc = self._start_subprocess(process_id, command)
        if proc is None:
            return pb.DriverStatus(success=False, message='Failed to start sensor process')

        return pb.DriverStatus(success=True, message='Sensor driver started', pid=proc.pid)

    def StopSensorDriver(self, request, context):
        self._stop(request.name)
        return pb.StatusResponse(success=True, message='Stopped')

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------
    def ListProcesses(self, request, context):
        with self._lock:
            return pb.ProcessList(names=list(self.processes.keys()))

    def ListTopics(self, request, context):
        all_topics = self.node.get_topic_names_and_types()
        topics = [
            pb.TopicInfo(name=name, type=types[0])
            for name, types in all_topics
            if self.node.count_publishers(name) > 0
        ]
        return pb.TopicList(topics=topics)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _build_robot_command(self, company, rtype, robot_id, settings):
        ns = f'ec_robot_{robot_id}'
        ip = settings.get('ip_address', '192.168.1.10')

        if company == 'Piper':
            gripper_exist = 'true' if rtype == 'piper' else 'false'
            can_port = settings.get('can_port', 'can0')
            if can_port.startswith('can_'):
                can_port = 'can' + can_port[4:]
            return ['ros2', 'launch', 'piper', 'start_single_piper.launch.py',
                    f'namespace:={ns}', f'can_port:={can_port}',
                    'auto_enable:=true', 'rviz_ctrl_flag:=false',
                    f'gripper_exist:={gripper_exist}']

        if company == 'Rainbow Robotics':
            return ['ros2', 'launch', 'rbpodo_bringup', 'rbpodo.launch.py',
                    f'namespace:={ns}', f'robot_ip:={settings.get("ip", "10.0.2.27")}',
                    'use_fake_hardware:=false']

        if company == 'OnRobot':
            return ['ros2', 'launch', 'onrobot_rg_control', 'bringup.launch.py',
                    f'namespace:={ns}', f'gripper:={rtype}',
                    f'ip:={ip}', f'port:={settings.get("port", 41414)}',
                    f'changer_addr:={settings.get("changer_address", 5)}']

        if company == 'Robotiq':
            return ['ros2', 'launch', 'robotiq_description', 'robotiq_control.launch.py',
                    f'namespace:={ns}',
                    f'com_port:={settings.get("serial_port", "/dev/ttyUSB0")}']

        if company == 'Kinova':
            return ['ros2', 'launch', f'{rtype}_moveit_config', 'robot.launch.py',
                    f'namespace:={ns}', f'robot_ip:={ip}',
                    'launch_rviz:=false']

        if company == 'OMRON':
            return ['ros2', 'launch', 'tm_driver', 'tm_bringup.launch.py',
                    f'namespace:={ns}', f'robot_ip:={ip}']

        if company == 'JAKA':
            return ['ros2', 'launch', 'jaka_driver', 'robot_start.launch.py',
                    f'namespace:={ns}', f'ip:={ip}']

        if company == 'Fairino':
            return ['ros2', 'run', 'fairino_hardware', 'ros2_cmd_server',
                    '--ros-args', '-r', f'__ns:=/ec_robot_{robot_id}',
                    '-p', f'robot_ip:={settings.get("ip_address", "192.168.58.2")}']

        if company == 'Test':
            return ['ros2', 'launch', 'test_arm', 'test_arm.launch.py',
                    f'namespace:={ns}']

        return None

    def _build_sensor_command(self, company, stype, sensor_id, settings):
        ns = f'ec_sensor_{sensor_id}'

        if company == 'Intel':
            serial_no = settings.get('serial_number')
            if not serial_no:
                return None
            return ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                    f'camera_namespace:={ns}', f'serial_no:="{serial_no}"']

        if company == 'Logitec':
            device_index = settings.get('device_index')
            if device_index is None:
                return None
            return ['ros2', 'launch', 'webcam_publisher', 'webcam_publisher.launch.py',
                    f'namespace:={ns}', f'device_index:={device_index}']

        if company == 'Kinova':
            ip_address = settings.get('ip_address')
            if not ip_address:
                return None
            return ['ros2', 'launch', 'kinova_vision', 'kinova_vision.launch.py',
                    f'device:={ip_address}', f'camera:={ns}']

        return None

    def _piper_can_setup(self, robot_id, settings):
        script_path = os.path.expanduser('~/ros2_ws/src/piper_ros/can_activate_main.sh')
        if os.path.exists(script_path):
            return ['bash', script_path]
        return None

    def _jaka_servo_enable(self):
        """JAKA servo mode 활성화 - ROS2 서비스 호출"""
        try:
            from jaka_msgs.srv import ServoMoveEnable
            client = self.node.create_client(ServoMoveEnable, '/jaka_driver/servo_move_enable')
            if client.wait_for_service(timeout_sec=5.0):
                req = ServoMoveEnable.Request()
                req.enable = True
                client.call_async(req)
        except Exception as e:
            print(f"[WARN] JAKA servo enable failed: {e}")

    def _start_subprocess(self, name, command):
        with self._lock:
            if name in self.processes:
                self._stop_locked(name)

            try:
                popen_args = {
                    'stdout': subprocess.PIPE, 'stderr': subprocess.PIPE,
                    'text': True, 'bufsize': 1,
                }
                if os.name != 'nt':
                    popen_args['preexec_fn'] = os.setsid
                    command = ['stdbuf', '-oL'] + command

                proc = subprocess.Popen(command, **popen_args)
                self.processes[name] = proc

                # Background log reader
                def _log_reader(stream, label):
                    try:
                        for line in stream:
                            print(f"[{name}/{label}] {line.strip()}", flush=True)
                    except Exception:
                        pass

                threading.Thread(target=_log_reader, args=(proc.stdout, 'stdout'), daemon=True).start()
                threading.Thread(target=_log_reader, args=(proc.stderr, 'stderr'), daemon=True).start()

                # Wait for process end
                def _wait():
                    proc.wait()
                    with self._lock:
                        self.processes.pop(name, None)
                    print(f"Process '{name}' exited with code {proc.returncode}", flush=True)

                threading.Thread(target=_wait, daemon=True).start()
                return proc
            except Exception as e:
                print(f"[ERROR] Failed to start '{name}': {e}", flush=True)
                return None

    def _stop(self, name):
        with self._lock:
            self._stop_locked(name)

    def _stop_locked(self, name):
        proc = self.processes.pop(name, None)
        if proc is None:
            return
        try:
            if os.name != 'nt':
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            else:
                proc.terminate()
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        print(f"Process '{name}' terminated.", flush=True)

    def stop_all(self):
        with self._lock:
            for name in list(self.processes.keys()):
                self._stop_locked(name)
