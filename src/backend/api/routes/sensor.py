# routes/camera.py

from flask import Blueprint, request, current_app
from flask_socketio import Namespace, emit
from ...database.models.sensor_model import SensorModel

# 1. Blueprint 생성
# 이 블루프린트는 카메라와 관련된 'HTTP' 라우트를 관리합니다.
sensor_bp = Blueprint('sensor', __name__)


class SensorNamespace(Namespace):
    def __init__(self, namespace, process_manager):
        super().__init__(namespace)
        self.pm = process_manager

    # def on_connect(self):
    #     print(f'Client connected to /sensor namespace: {request.sid}')
    #     emit('response', {'data': '카메라 제어 채널에 연결되었습니다.'})

    # def on_disconnect(self):
    #     print(f'Client disconnected from /sensor namespace: {request.sid}')


@sensor_bp.route('/sensors', methods=['GET'])
def get_sensors():
    sensors = SensorModel.find_all(to='dict')

    for sensor in sensors:
        # 프로세스 ID 초기화
        sensor['process_id'] = 'sensor_' + str(sensor['id'])
        # sensor['process_cmd'] = f'roslaunch realsense2_camera rs_camera.launch camera:={sensor['name']} serial_no:={sensor['serial_no']}'
    # 클라이언트로 센서 목록 반환

    return {
        'status': 'success',
        'sensors': sensors
    }, 200

@sensor_bp.route('/sensor:start', methods=['POST'])
def start_sensor():
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

    print(f"Attempting to start sensor {serial_no}")

    process = current_app.pm.start_process(
        name=process_id,
        command=command,
    )

    if process:
        return {
            'status': 'success',
            'message': f'Sensor process started for serial {serial_no}',
            'pid': process.pid
        }, 200
    else:
        return {'status': 'error', 'message': f'Failed to start sensor for serial {serial_no}'}, 500
    

@sensor_bp.route('/sensor:stop', methods=['POST'])
def stop_sensor():
    process_id = request.json.get('process_id')
    current_app.pm.stop_process(process_id)
    return {'status': 'success', 'message': 'Sensor process stopped'}, 200


@sensor_bp.route('/sensor', methods=['POST'])
def create_sensor():
    serial_no = request.json.get('serial_no')
    name = request.json.get('name')
    type = request.json.get('type')

    sensor = SensorModel(
        name=name,
        type=type,
        settings={
            'serial_number': serial_no
        }
    )
    
    sensor.create()
    return {'status': 'success', 'message': 'Sensor Created'}, 200


@sensor_bp.route('/sensor/<id>', methods=['PUT'])
def update_sensor(id):
    serial_no = request.json.get('serial_no')
    name = request.json.get('name')
    type = request.json.get('type')

    sensor = SensorModel.find_one({ 'id': id })
    sensor.settings['serial_number'] = serial_no
    sensor.name = name
    sensor.type = type

    sensor.update()
    return {'status': 'success', 'message': 'Sensor Updated'}, 200


@sensor_bp.route('/sensor/<id>', methods=['DELETE'])
def delete_sensor(id):
    sensor = SensorModel.find_one({ 'id': id })

    sensor.delete()
    return {'status': 'success', 'message': 'Sensor Deleted'}, 200



