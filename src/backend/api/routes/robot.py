from flask import Blueprint, request, current_app
from flask_socketio import Namespace, emit
from ...database.models.robot_model import Robot as RobotModel
import json
import time
import os
import subprocess
from ...bridge.remote_agent import RemoteAgent
from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
from ..process.subscribe_robot import subscribe_robot_topic
from ...configs.global_configs import SUPPORT_ROBOTS
from ..utils.runtime import attach_robot_runtime

# 1. Blueprint 생성
# 이 블루프린트는 카메라와 관련된 'HTTP' 라우트를 관리합니다.
robot_bp = Blueprint('robot', __name__)


class RobotNamespace(Namespace):
    def __init__(self, namespace, process_manager):
        super().__init__(namespace)
        self.pm = process_manager

    # def on_connect(self):
    #     print(f'Client connected to /robot namespace: {request.sid}')
    #     emit('response', {'data': '카메라 제어 채널에 연결되었습니다.'})

    # def on_disconnect(self):
    #     print(f'Client disconnected from /robot namespace: {request.sid}')


@robot_bp.route('/robots', methods=['GET'])
def get_robots():
    robots = RobotModel.with_('leader_robot_preset').where('hide', False).get()
    robots = [robot.to_dict() for robot in robots]

    processes = set(current_app.pm.list_processes())
    # ROS2 컨테이너의 드라이버 프로세스 목록도 포함
    try:
        client = get_bridge_client()
        ros2_procs = client.driver.ListProcesses(pb.Empty())
        processes.update(ros2_procs.names)
    except Exception:
        pass
    for robot in robots:
        attach_robot_runtime(robot, processes)

    return {
        'status': 'success',
        'robots': robots
    }, 200

@robot_bp.route('/robots:supporting', methods=['GET'])
def get_supporting_robots():
    support_robot_json = []
    for robot in SUPPORT_ROBOTS:
        robot_copy = robot.copy()
        robot_copy.pop('ik_setting', None)
        support_robot_json.append(robot_copy)
    return {
        'status': 'success',
        'robots': support_robot_json
    }, 200

@robot_bp.route('/robot/<id>', methods=['GET'])
def get_robot(id):
    robot = RobotModel.find(id)
    if robot:
        processes = set(current_app.pm.list_processes())
        robot = attach_robot_runtime(robot.to_dict(), processes)
        return {
            'status': 'success',
            'robot': robot
        }, 200
    else:
        return {
            'status': 'error',
            'message': 'Robot not found'
        }, 404

@robot_bp.route('/robot:start', methods=['POST'])
def start_robot():
    data = request.json
    process_id = data.get('process_id')
    id = data.get('id')
    type = data.get('type')
    company = data.get('company', '')
    settings = data.get('settings', {})

    # global_configs에서 파생되는 값을 settings에 주입 (드라이버 서비스에서 필요)
    robot_model = RobotModel.find(id)
    if robot_model:
        robot_dict = robot_model.to_dict()
        settings['interpolation'] = robot_dict.get('interpolation', False)
        settings['write_topic'] = robot_dict.get('write_topic', '')

    # 기존 구독 정리
    try:
        current_app.pm.stop_function('subscribe_robot_' + str(id))
    except Exception:
        pass

    print(f"[DEBUG start_robot] id={id} type={type} company='{company}' process_id={process_id}")

    # custom 로봇은 드라이버 시작 불필요 (외부 토픽 직접 구독)
    if type == 'custom':
        return {'status': 'success', 'message': 'Custom robot uses external topic'}, 200

    # ROS2 컨테이너에 드라이버 시작 요청 (gRPC)
    client = get_bridge_client()
    result = client.driver.StartRobotDriver(pb.DriverConfig(
        process_id=process_id,
        robot_id=int(id),
        type=type,
        company=company,
        settings_json=json.dumps(settings),
    ))

    if result.success:
        return {
            'status': 'success',
            'message': 'Robot process started',
            'pid': result.pid
        }, 200
    else:
        return {'status': 'error', 'message': result.message}, 500
    

@robot_bp.route('/robot:stop', methods=['POST'])
def stop_robot():
    robot_id = request.json.get('id')
    process_id = request.json.get('process_id')

    # ROS2 컨테이너에 드라이버 정지 요청
    client = get_bridge_client()
    client.driver.StopRobotDriver(pb.ProcessId(name=process_id))

    # 로컬 리소스 정리
    current_app.pm.stop_process('leader_teleoperation')
    current_app.pm.stop_function('subscribe_robot_' + str(robot_id))

    # RemoteAgent 정리
    agent = current_app.agents.pop(int(robot_id), None)
    if agent:
        agent.destroy()

    return {'status': 'success', 'message': 'Robot process stopped'}, 200


@robot_bp.route('/robot', methods=['POST'])
def create_robot():
    name = request.json.get('name')
    type = request.json.get('type')
    homepose = request.json.get('homepose', [])
    role = ''

    settings = {}

    if type == 'custom':
        role = request.json.get('role', '')
        settings = {
            'role': request.json.get('role', ''),
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', ''),
            'write_type': request.json.get('write_type', ''),
            'write_topic': request.json.get('write_topic', ''),
            'write_topic_msg': request.json.get('write_topic_msg', ''),
            'joint_names': request.json.get('joint_names', []),
            'joint_lower_bounds': request.json.get('joint_lower_bounds', []),
            'joint_upper_bounds': request.json.get('joint_upper_bounds', []),
            'tool_index': request.json.get('tool_index', []),
            'tool_inner': len(request.json.get('tool_index', [])) > 0,
        }
        _apply_ik_settings(request.json, settings)

    custom_fields = ['can_port', 'ip_address', 'port', 'changer_address', 'serial_port']
    for field in custom_fields:
        if field in request.json:
            settings[field] = request.json.get(field)

    if 'is_sim' in request.json:
        settings['is_sim'] = request.json.get('is_sim', False)

    # Normalize CAN port names (can0/can1) in case underscores are provided
    if 'can_port' in settings and settings['can_port'].startswith('can_'):
        settings['can_port'] = 'can' + settings['can_port'][4:]

    RobotModel.create(
        name=name,
        type=type,
        role=role,
        settings=settings,
        homepose=homepose
    )
    
    return {'status': 'success', 'message': 'Robot Created'}, 200


@robot_bp.route('/robot/<id>', methods=['PUT'])
def update_robot(id):
    name = request.json.get('name')
    type = request.json.get('type')

    robot = RobotModel.find(id)
    robot.name = name
    robot.type = type

    if 'homepose' in request.json:
        robot.homepose = request.json.get('homepose')

    if 'role' in request.json:
        robot.role = request.json.get('role')

    settings = robot.settings if robot.settings else {}
    if type == 'custom':
        settings = {
            'role': request.json.get('role', ''),
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', ''),
            'write_type': request.json.get('write_type', ''),
            'write_topic': request.json.get('write_topic', ''),
            'write_topic_msg': request.json.get('write_topic_msg', ''),
            'joint_names': request.json.get('joint_names', []),
            'joint_lower_bounds': request.json.get('joint_lower_bounds', []),
            'joint_upper_bounds': request.json.get('joint_upper_bounds', []),
            'tool_index': request.json.get('tool_index', []),
            'tool_inner': len(request.json.get('tool_index', [])) > 0,
        }
        _apply_ik_settings(request.json, settings)

    custom_fields = ['can_port', 'ip_address', 'port', 'changer_address', 'serial_port']
    for field in custom_fields:
        if field in request.json:
            settings[field] = request.json.get(field)

    if 'is_sim' in request.json:
        settings['is_sim'] = request.json.get('is_sim', False)

    robot.settings = settings
    robot.save()
    return {'status': 'success', 'message': 'Robot Updated'}, 200


@robot_bp.route('/robot/<id>', methods=['DELETE'])
def delete_robot(id):
    robot = RobotModel.find(id)
    robot.hide = True
    robot.save()
    return {'status': 'success', 'message': 'Robot Deleted'}, 200


@robot_bp.route('/robot/<id>/:move_to', methods=['POST'])
def move_robot(id):
    data = request.json
    robot = RobotModel.find(id)
    if not robot:
        return {'status': 'error', 'message': 'Robot not found'}, 404

    goal_pos = data.get('goal_pos')
    step_size = data.get('step_size', 0.1)

    if not goal_pos:
        return {'status': 'error', 'message': 'Action is required'}, 400
    
    print(current_app.agents)
    agent = current_app.agents[int(id)]

    agent.move_to(goal_pos, step_size)
    time.sleep(3)
    return {'status': 'success', 'message': 'Robot moved'}, 200


@robot_bp.route('/robot/<id>/:subscribe_robot', methods=['POST'])
def subscribe_robot(id):
    int_id = int(id)
    func_name = 'subscribe_robot_' + str(id)

    # 기존 RemoteAgent가 있으면 재사용
    existing_agent = current_app.agents.get(int_id)
    if existing_agent is not None:
        if func_name in current_app.pm.processes:
            return {'status': 'success', 'message': 'Already subscribed'}, 200
        current_app.pm.start_function(
            name=func_name,
            func=subscribe_robot_topic,
            socketio_instance=current_app.pm.socketio,
            agent=existing_agent,
        )
        return {'status': 'success', 'message': 'Subscribed to robot topic'}, 200

    robot = RobotModel.find(id).to_dict()
    agent = RemoteAgent(robot)
    current_app.agents[int_id] = agent

    current_app.pm.start_function(
        name=func_name,
        func=subscribe_robot_topic,
        socketio_instance=current_app.pm.socketio,
        agent=agent,
    )
    return {'status': 'success', 'message': 'Subscribed to robot topic'}, 200


@robot_bp.route('/robot/<id>/:unsubscribe_robot', methods=['POST'])
def unsubscribe_robot(id):
    current_app.pm.stop_function('subscribe_robot_' + str(id))
    # Agent와 ROS2 구독은 유지 — rclpy.spin 중 destroy하면 segfault
    # Agent는 다음 subscribe 시 재사용됨
    return {'status': 'success', 'message': 'Unsubscribed from robot topic'}, 200


def _apply_ik_settings(data: dict, settings: dict):
    """Extract IK JSON from request data into robot settings."""
    ik_json = data.get('ik_json', '')
    if ik_json and isinstance(ik_json, dict):
        settings['urdf_path'] = ik_json.get('urdf_path', '')
        settings['urdf_package_dir'] = ik_json.get('urdf_package_dir', '')
        settings['ik_setting'] = ik_json.get('ik_setting', {})
    else:
        settings.pop('urdf_path', None)
        settings.pop('urdf_package_dir', None)
        settings.pop('ik_setting', None)


def _ensure_can_interface(name: str):
    """
    Make sure the requested CAN interface exists.
    If 'canX' is missing but 'can_X' exists, rename it to 'canX'.
    """
    if not name.startswith("can"):
        return name
    # Already exists
    if os.path.exists(f"/sys/class/net/{name}"):
        return name
    # Try underscore variant
    num = name[3:]
    alt = f"can_{num}"
    if os.path.exists(f"/sys/class/net/{alt}"):
        try:
            subprocess.run(["ip", "link", "set", alt, "down"], check=True)
            subprocess.run(["ip", "link", "set", alt, "name", name], check=True)
            subprocess.run(["ip", "link", "set", name, "type", "can", "bitrate", "1000000"], check=True)
            subprocess.run(["ip", "link", "set", name, "up"], check=True)
            print(f"[PIPER] Renamed {alt} -> {name}")
        except Exception as e:
            print(f"[PIPER][WARN] Failed to rename {alt} -> {name}: {e}")
    return name
