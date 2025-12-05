from flask import Blueprint, request, current_app
from flask_socketio import Namespace, emit
from ...database.models.robot_model import Robot as RobotModel
import time
import os
from ...env.agent import Agent
from ..process.subscribe_robot import subscribe_robot_topic
from ...configs.global_configs import SUPPORT_ROBOTS

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

    for robot in robots:
        # 프로세스 ID 초기화
        robot['process_id'] = 'robot_' + str(robot['id'])

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
        return {
            'status': 'success',
            'robot': robot.to_dict()
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

    command = ''
    if company == 'Piper':
        script_path = os.path.expanduser('~/ros2_ws/src/piper_ros/can_activate_main.sh')
        current_app.pm.start_process(
            name='can_config',
            command=['bash', script_path],
            log_emit_id = 'log_' +  process_id
        )
        time.sleep(1)

        gripper_exist = 'true' if type == 'piper' else 'false'
        command = ['ros2', 'launch', 'piper', 'start_single_piper.launch.py', 
                   f'namespace:=ec_robot_{id}', 
                   f'can_port:={settings.get("can_port", "can_0")}', 
                   'auto_enable:=true', 'rviz_ctrl_flag:=false', 
                   f'gripper_exist:={gripper_exist}']

    if company == 'Rainbow Robotics':
        command = ['ros2', 'launch', 'rbpodo_bringup', 'rbpodo.launch.py', f'namespace:=ec_robot_{id}', f'robot_ip:={settings.get("ip", "10.1.1.1")}', 'use_fake_hardware:=true']

    if company == 'OnRobot':
        command = ['ros2', 'launch', 'onrobot_rg_control', 'bringup.launch.py',
                   f'namespace:=ec_robot_{id}',
                   f'gripper:={type}',
                   f'ip:={settings.get("ip_address", "10.0.2.27")}',
                   f'port:={settings.get("port", 502)}',
                   f'changer_addr:={settings.get("changer_address", 5)}'
        ]
    print(f"Attempting to start robot")

    process = current_app.pm.start_process(
        name=process_id,
        command=command,
    )

    agent = Agent(current_app.node, data)

    current_app.agents[id] = agent

    current_app.pm.start_function(
        name='subscribe_robot_' + str(id),
        node=current_app.node,
        func=subscribe_robot_topic,
        socketio_instance=current_app.pm.socketio,
        agent=agent,
    )

    if process:
        return {
            'status': 'success',
            'message': f'Robot process started',
            'pid': process.pid
        }, 200
    else:
        return {'status': 'error', 'message': f'Failed to start robot'}, 500
    

@robot_bp.route('/robot:stop', methods=['POST'])
def stop_robot():
    robot_id = request.json.get('id')
    process_id = request.json.get('process_id')
    current_app.pm.stop_process(process_id)
    current_app.pm.stop_process('leader_teleoperation')
    current_app.pm.stop_function('subscribe_robot_' + str(robot_id))
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
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', ''),
            'write_topic': request.json.get('write_topic', ''),
            'write_topic_msg': request.json.get('write_topic_msg', ''),
            'joint_names': request.json.get('joint_names', []),
            'joint_lower_bounds': request.json.get('joint_lower_bounds', []),
            'joint_upper_bounds': request.json.get('joint_upper_bounds', []),
        }

    custom_fields = ['can_port', 'ip_address', 'port', 'changer_address']
    for field in custom_fields:
        if field in request.json:
            settings[field] = request.json.get(field)


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
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', ''),
            'write_topic': request.json.get('write_topic', ''),
            'write_topic_msg': request.json.get('write_topic_msg', ''),
            'joint_names': request.json.get('joint_names', []),
            'joint_lower_bounds': request.json.get('joint_lower_bounds', []),
            'joint_upper_bounds': request.json.get('joint_upper_bounds', []),
        }


    if 'can_port' in request.json:
        settings['can_port'] = request.json.get('can_port', 'can_0')

    if 'ip_address' in request.json:
        settings['ip_address'] = request.json.get('ip_address', '')

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