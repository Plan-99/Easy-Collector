from flask import Blueprint, request, current_app, jsonify
from flask_socketio import Namespace, emit
from ...database.models.sensor_model import Sensor as SensorModel
from ...configs.global_configs import SUPPORT_SENSORS
from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
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
    company = data.get('company')
    settings = data.get('settings', {})

    # Custom sensors use external ROS topics — no process to start
    if type == 'custom':
        return {'status': 'success', 'message': 'Custom sensor uses external topic'}, 200

    # ROS2 컨테이너에 센서 드라이버 시작 요청 (gRPC)
    client = get_bridge_client()
    result = client.driver.StartSensorDriver(pb.SensorDriverConfig(
        process_id=process_id,
        sensor_id=int(id),
        type=type,
        company=company,
        settings_json=json.dumps(settings),
    ))

    if result.success:
        return {
            'status': 'success',
            'message': f'Sensor process started for id {id}',
            'pid': result.pid
        }, 200
    else:
        return {'status': 'error', 'message': result.message}, 500
    

@sensor_bp.route('/sensor:stop', methods=['POST'])
def stop_sensor():
    process_id = request.json.get('process_id')
    # ROS2 컨테이너에 센서 드라이버 정지 요청
    client = get_bridge_client()
    client.driver.StopSensorDriver(pb.ProcessId(name=process_id))
    return {'status': 'success', 'message': 'Sensor process stopped'}, 200


@sensor_bp.route('/sensor', methods=['POST'])
def create_sensor():
    name = request.json.get('name')
    type = request.json.get('type')

    if type == 'custom':
        settings = {
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', 'sensor_msgs/CompressedImage'),
            'resolution': [
                request.json.get('resolution_width', 640),
                request.json.get('resolution_height', 480)
            ],
        }
    else:
        serial_no = request.json.get('serial_no')
        ip_address = request.json.get('ip_address')
        device_index = request.json.get('device_index')
        settings = {
            'serial_number': serial_no,
            'ip_address': ip_address,
            'device_index': device_index
        }

    SensorModel.create(
        name=name,
        type=type,
        settings=settings
    )

    return {'status': 'success', 'message': 'Sensor Created'}, 200


@sensor_bp.route('/sensor/<id>', methods=['PUT'])
def update_sensor(id):
    name = request.json.get('name')
    type = request.json.get('type')

    sensor = SensorModel.find(id)
    sensor.name = name
    sensor.type = type

    if type == 'custom':
        sensor.settings = {
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', 'sensor_msgs/CompressedImage'),
            'resolution': [
                request.json.get('resolution_width', 640),
                request.json.get('resolution_height', 480)
            ],
        }
    else:
        serial_no = request.json.get('serial_no')
        ip_address = request.json.get('ip_address')
        device_index = request.json.get('device_index')
        sensor.settings = {
            'serial_number': serial_no,
            'ip_address': ip_address,
            'device_index': device_index
        }

    sensor.save()
    return {'status': 'success', 'message': 'Sensor Updated'}, 200


@sensor_bp.route('/sensor/<id>', methods=['DELETE'])
def delete_sensor(id):
    sensor = SensorModel.find(id)
    sensor.hide = True
    sensor.save()
    return {'status': 'success', 'message': 'Sensor Deleted'}, 200

