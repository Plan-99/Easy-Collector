# routes/camera.py

from flask import Blueprint, request, current_app
from flask_socketio import Namespace, emit
from ...database.models.robot_model import RobotModel

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
    robots = RobotModel.find_all(to='dict')

    for robot in robots:
        # 프로세스 ID 초기화
        robot['process_id'] = 'robot_' + str(robot['id'])
        # robot['process_cmd'] = f'roslaunch realsense2_camera rs_camera.launch camera:={robot['name']} serial_no:={robot['serial_no']}'
    # 클라이언트로 센서 목록 반환

    return {
        'status': 'success',
        'robots': robots
    }, 200

@robot_bp.route('/robot:start', methods=['POST'])
def start_robot():
    data = request.json
    serial_no = data.get('serial_no')
    process_id = data.get('process_id')
    name = data.get('name')
    type = data.get('type')

    command = ''
    if type == 'realsense_camera':
        command = ['roslaunch', 'realsense2_camera', 'rs_camera.launch', f'camera:={name}', f'serial_no:={serial_no}']

    if not serial_no:
        return {'status': 'error', 'message': 'serial_no is required'}, 400

    print(f"Attempting to start robot {serial_no}")

    process = current_app.pm.start_process(
        name=process_id,
        command=command,
    )

    if process:
        return {
            'status': 'success',
            'message': f'Robot process started for serial {serial_no}',
            'pid': process.pid
        }, 200
    else:
        return {'status': 'error', 'message': f'Failed to start robot for serial {serial_no}'}, 500
    

@robot_bp.route('/robot:stop', methods=['POST'])
def stop_robot():
    process_id = request.json.get('process_id')
    current_app.pm.stop_process(process_id)
    return {'status': 'success', 'message': 'Robot process stopped'}, 200


@robot_bp.route('/robot', methods=['POST'])
def create_robot():
    serial_no = request.json.get('serial_no')
    name = request.json.get('name')
    type = request.json.get('type')

    robot = RobotModel(
        name=name,
        type=type,
        settings={
            'serial_number': serial_no
        }
    )
    
    robot.create()
    return {'status': 'success', 'message': 'Robot Created'}, 200


@robot_bp.route('/robot/<id>', methods=['PUT'])
def update_robot(id):
    serial_no = request.json.get('serial_no')
    name = request.json.get('name')
    type = request.json.get('type')

    robot = RobotModel.find_one({ 'id': id })
    robot.settings['serial_number'] = serial_no
    robot.name = name
    robot.type = type

    robot.update()
    return {'status': 'success', 'message': 'Robot Updated'}, 200


@robot_bp.route('/robot/<id>', methods=['DELETE'])
def delete_robot(id):
    robot = RobotModel.find_one({ 'id': id })

    robot.delete()
    return {'status': 'success', 'message': 'Robot Deleted'}, 200



