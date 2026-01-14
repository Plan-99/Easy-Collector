from flask import Blueprint, request, current_app, jsonify
from flask_socketio import Namespace, emit
from ...database.models.sensor_model import Sensor as SensorModel
from ...configs.global_configs import SUPPORT_SENSORS
import json

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
    sensors = SensorModel.where('hide', False).get()
    sensors = [sensor.to_dict() for sensor in sensors]
    for sensor in sensors:
        sensor['process_id'] = 'sensor_' + str(sensor['id'])
        # sensor['process_cmd'] = f'roslaunch realsense2_camera rs_camera.launch camera:={sensor['name']} serial_no:={sensor['serial_no']}'

    return {
        'status': 'success',
        'sensors': sensors
    }, 200


@sensor_bp.route('/sensors:supporting', methods=['GET'])
def get_supporting_sensors():
    support_sensor_json = []
    for sensor in SUPPORT_SENSORS:
        sensor_copy = sensor.copy()
        support_sensor_json.append(sensor_copy)
    return {
        'status': 'success',
        'sensors': support_sensor_json
    }, 200


@sensor_bp.route('/sensor:start', methods=['POST'])
def start_sensor():
    print("Received request to start sensor")
    data = request.json
    process_id = data.get('process_id')
    id = data.get('id')
    type = data.get('type')

    command = []
    
    if type == 'realsense_d435_color':
        serial_no = data.get('serial_no', None)
        if serial_no is None:
            return {'status': 'error', 'message': 'serial_no is required'}, 400
        command = ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', f'camera_namespace:=ec_sensor_{id}', f'serial_no:="{serial_no}"']
    elif type == 'kinova_vision_color':
        ip_address = data.get('ip_address', None)
        if ip_address is None:
            return {'status': 'error', 'message': 'ip_address is required'}, 400
        command = ['ros2', 'launch', 'kinova_vision', 'kinova_vision.launch.py', f'device:={ip_address}', f'camera:=ec_sensor_{id}']

    print(f"Attempting to start sensor commmand: {' '.join(command)}")

    process = current_app.pm.start_process(
        name=process_id,
        command=command,
    )

    if process:
        return {
            'status': 'success',
            'message': f'Sensor process started for id {id}',
            'pid': process.pid
        }, 200
    else:
        return {'status': 'error', 'message': f'Failed to start sensor for id {id}'}, 500

    # if process:
    #     return {
    #         'status': 'success',
    #         'message': f'Sensor process started for serial {serial_no}',
    #         'pid': process.pid
    #     }, 200
    # else:
    #     return {'status': 'error', 'message': f'Failed to start sensor for serial {serial_no}'}, 500
    

@sensor_bp.route('/sensor:stop', methods=['POST'])
def stop_sensor():
    process_id = request.json.get('process_id')
    current_app.pm.stop_process(process_id)
    return {'status': 'success', 'message': 'Sensor process stopped'}, 200


@sensor_bp.route('/sensor', methods=['POST'])
def create_sensor():
    serial_no = request.json.get('serial_no')
    ip_address = request.json.get('ip_address')
    name = request.json.get('name')
    type = request.json.get('type')

    SensorModel.create(
        name=name,
        type=type,
        settings={
            'serial_number': serial_no,
            'ip_address': ip_address
        }
    )
    
    return {'status': 'success', 'message': 'Sensor Created'}, 200


@sensor_bp.route('/sensor/<id>', methods=['PUT'])
def update_sensor(id):
    serial_no = request.json.get('serial_no')
    ip_address = request.json.get('ip_address')
    name = request.json.get('name')
    type = request.json.get('type')

    sensor = SensorModel.find(id)
    sensor.settings = {
        'serial_number': serial_no,
        'ip_address': ip_address
    }
    sensor.name = name
    sensor.type = type

    sensor.save()
    return {'status': 'success', 'message': 'Sensor Updated'}, 200


@sensor_bp.route('/sensor/<id>', methods=['DELETE'])
def delete_sensor(id):
    sensor = SensorModel.find(id)
    sensor.hide = True
    sensor.save()
    return {'status': 'success', 'message': 'Sensor Deleted'}, 200

