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
robot_bp = Blueprint('robot', __name__)


class RobotNamespace(Namespace):
    def __init__(self, namespace, process_manager):
        super().__init__(namespace)
        self.pm = process_manager


@robot_bp.route('/robots', methods=['GET'])
def get_robots():
    robots = RobotModel.select().where(RobotModel.hide == False, RobotModel.deleted_at.is_null())
    robots = [robot.to_dict() for robot in robots]

    processes = set(current_app.pm.list_processes())
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
    support_robot_json = [robot.copy() for robot in SUPPORT_ROBOTS]
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

    robot_model = RobotModel.find(id)
    if robot_model:
        robot_dict = robot_model.to_dict()
        settings['interpolation'] = robot_dict.get('interpolation', False)
        settings['write_topic'] = robot_dict.get('write_topic', '')
        settings['sdk_control'] = robot_dict.get('sdk_control', False)
        settings['sdk_type'] = robot_dict.get('sdk_type', '')

    try:
        current_app.pm.stop_function('subscribe_robot_' + str(id))
    except Exception:
        pass

    print(f"[DEBUG start_robot] id={id} type={type} company='{company}' process_id={process_id}")
    socketio = current_app.extensions.get('socketio')
    def _log(msg, log_type='stdout'):
        print(f"[robot:{id}] {msg}", flush=True)
        if socketio:
            socketio.emit('task_log', {'id': process_id, 'message': msg, 'type': log_type})

    if type == 'custom':
        # custom 로봇은 외부 ROS2 토픽만 소비하므로 robot driver 는 띄우지
        # 않지만, settings.interpolation=True 면 interpolation_node 만 별도로
        # 띄워서 ec_joint_cmd → write_topic 사이에 보간을 끼워넣는다.
        if settings.get('interpolation'):
            write_topic = settings.get('write_topic') or ''
            write_topic_msg = settings.get('write_topic_msg') or 'sensor_msgs/JointState'
            if not write_topic:
                _log('interpolation 활성화 실패: write_topic 이 비어있음', 'error')
                return {
                    'status': 'error',
                    'message': 'interpolation 을 켜려면 write_topic 이 필요합니다.',
                }, 400
            try:
                client = get_bridge_client()
                result = client.driver.StartInterpolation(pb.InterpolationConfig(
                    robot_id=int(id),
                    output_topic=write_topic,
                    output_msg_type=write_topic_msg,
                    publish_rate=0.0,  # 0 → 노드 default(200Hz)
                ))
            except Exception as e:
                _log(f'StartInterpolation 호출 실패: {e}', 'error')
                return {'status': 'error', 'message': f'Bridge call failed: {e}'}, 500
            if not result.success:
                _log(f'interpolation_node 기동 실패: {result.message}', 'error')
                return {'status': 'error', 'message': result.message}, 500
            _log(f'interpolation_node started (pid={result.pid}) → {write_topic}')
            return {
                'status': 'success',
                'message': 'Custom robot ready (interpolation enabled)',
                'pid': result.pid,
            }, 200
        return {'status': 'success', 'message': 'Custom robot uses external topic'}, 200

    _log(f'Starting robot driver: {type} ({company})')

    client = get_bridge_client()
    result = client.driver.StartRobotDriver(pb.DriverConfig(
        process_id=process_id,
        robot_id=int(id),
        type=type,
        company=company,
        settings_json=json.dumps(settings),
    ))

    if result.success:
        _log(f'Driver started (pid={result.pid})')
        return {
            'status': 'success',
            'message': 'Robot process started',
            'pid': result.pid
        }, 200
    else:
        _log(f'Driver failed: {result.message}', 'error')
        return {'status': 'error', 'message': result.message}, 500


@robot_bp.route('/robot:stop', methods=['POST'])
def stop_robot():
    robot_id = request.json.get('id')
    process_id = request.json.get('process_id')

    client = get_bridge_client()
    client.driver.StopRobotDriver(pb.ProcessId(name=process_id))
    # Custom 로봇용 standalone interpolation_node 정리. 빌트인 로봇은
    # StopRobotDriver 가 내부적으로 같은 이름으로 _stop 을 호출하므로 중복은
    # idempotent (없는 process_id 면 no-op).
    try:
        client.driver.StopInterpolation(pb.ProcessId(name=f'interp_{robot_id}'))
    except Exception as e:
        print(f'[WARN] StopInterpolation failed for robot {robot_id}: {e}')

    current_app.pm.stop_process('leader_teleoperation')
    current_app.pm.stop_function('subscribe_robot_' + str(robot_id))

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
            # Optional: 보간 켜면 robot:start 시 standalone interpolation_node 가
            # ec_joint_cmd 를 받아 write_topic 으로 부드럽게 publish.
            'interpolation': bool(request.json.get('interpolation', False)),
        }
        _apply_ik_settings(request.json, settings)

    custom_fields = ['can_port', 'ip_address', 'port', 'changer_address', 'serial_port']
    for field in custom_fields:
        if field in request.json:
            settings[field] = request.json.get(field)

    if 'is_sim' in request.json:
        settings['is_sim'] = request.json.get('is_sim', False)

    if 'can_port' in settings and settings['can_port'].startswith('can_'):
        settings['can_port'] = 'can' + settings['can_port'][4:]

    RobotModel.create(
        name=name,
        type=type,
        role=role,
        settings=json.dumps(settings),
        homepose=json.dumps(homepose)
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

    settings = robot._settings if robot._settings else {}
    if type == 'custom':
        # 기존 settings 의 마커성 필드 (is_tutorial 등 운영 메타) 를 보존하기 위해
        # 기존 dict 위에 폼 입력값을 머지한다 — 통째로 덮어쓰면 시드 메타가 사라져
        # tutorial 행이 일반 custom 처럼 취급되는 사고가 난다.
        settings = dict(settings) if isinstance(settings, dict) else {}
        settings.update({
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
            'interpolation': bool(request.json.get('interpolation', settings.get('interpolation', False))),
        })
        _apply_ik_settings(request.json, settings)

    custom_fields = ['can_port', 'ip_address', 'port', 'changer_address', 'serial_port']
    for field in custom_fields:
        if field in request.json:
            settings[field] = request.json.get(field)

    if 'is_sim' in request.json:
        settings['is_sim'] = request.json.get('is_sim', False)

    if 'ee_offset' in request.json:
        ee_offset = request.json.get('ee_offset') or {}
        if ee_offset:
            settings['ee_offset'] = ee_offset
        else:
            settings.pop('ee_offset', None)

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
    if not name.startswith("can"):
        return name
    if os.path.exists(f"/sys/class/net/{name}"):
        return name
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
