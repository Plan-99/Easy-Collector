
from flask import Blueprint, request, current_app
from flask_socketio import Namespace, emit
from ...database.models.robot_model import Robot as RobotModel
import time
import os
from ...env.agent import Agent

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
    robots = RobotModel.with_('leader_robot_preset').get()
    robots = [robot.to_dict() for robot in robots]

    for robot in robots:
        # 프로세스 ID 초기화
        robot['process_id'] = 'robot_' + str(robot['id'])

    return {
        'status': 'success',
        'robots': robots
    }, 200

@robot_bp.route('/robot:start', methods=['POST'])
def start_robot():
    data = request.json
    process_id = data.get('process_id')
    id = data.get('id')
    type = data.get('type')

    command = ''
    if type == 'piper':
        script_path = os.path.expanduser('~/catkin_ws/src/piper_ros/can_activate.sh')
        current_app.pm.start_process(
            name='can_config',
            command=['bash', script_path , 'can0', '1000000'],
            log_emit_id = 'log_' +  process_id
        )
        time.sleep(1)
        command = ['roslaunch', 'piper', 'start_single_piper.launch', f'group_ns:=ec_robot_{id}']


    print(f"Attempting to start robot")

    process = current_app.pm.start_process(
        name=process_id,
        command=command,
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
    process_id = request.json.get('process_id')
    current_app.pm.stop_process(process_id)
    current_app.pm.stop_process('leader_teleoperation')
    return {'status': 'success', 'message': 'Robot process stopped'}, 200


@robot_bp.route('/robot', methods=['POST'])
def create_robot():
    name = request.json.get('name')
    type = request.json.get('type')
    settings = {}
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
    RobotModel.create(
        name=name,
        type=type,
        settings=settings
    )
    
    return {'status': 'success', 'message': 'Robot Created'}, 200


@robot_bp.route('/robot/<id>', methods=['PUT'])
def update_robot(id):
    name = request.json.get('name')
    type = request.json.get('type')

    robot = RobotModel.find(id)
    robot.name = name
    robot.type = type

    if type == 'custom':
        robot.settings = {
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', ''),
            'write_topic': request.json.get('write_topic', ''),
            'write_topic_msg': request.json.get('write_topic_msg', ''),
            'joint_names': request.json.get('joint_names', []),
            'joint_lower_bounds': request.json.get('joint_lower_bounds', []),
            'joint_upper_bounds': request.json.get('joint_upper_bounds', []),
        }

    robot.save()
    return {'status': 'success', 'message': 'Robot Updated'}, 200


@robot_bp.route('/robot/<id>', methods=['DELETE'])
def delete_robot(id):
    robot = RobotModel.find(id)

    robot.delete()
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
    
    agent = Agent(robot.to_dict())

    agent.move_to(goal_pos, step_size)
    return {'status': 'success', 'message': 'Robot moved'}, 200