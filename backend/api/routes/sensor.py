from flask import Blueprint, request, current_app, jsonify
from flask_socketio import Namespace, emit
from ...database.models.sensor_model import Sensor as SensorModel
from ...configs.global_configs import SUPPORT_SENSORS
from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
import base64
import json

sensor_bp = Blueprint('sensor', __name__)


class SensorNamespace(Namespace):
    def __init__(self, namespace, process_manager):
        super().__init__(namespace)
        self.pm = process_manager


@sensor_bp.route('/sensors', methods=['GET'])
def get_sensors():
    sensors = SensorModel.select().where(SensorModel.hide == False, SensorModel.deleted_at.is_null())
    sensors = [sensor.to_dict() for sensor in sensors]
    for sensor in sensors:
        sensor['process_id'] = 'sensor_' + str(sensor['id'])

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

    if type == 'custom':
        return {'status': 'success', 'message': 'Custom sensor uses external topic'}, 200

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


@sensor_bp.route('/sensor/<id>/sam3_preview', methods=['POST'])
def sam3_preview(id):
    """One-shot SAM3 mask preview for the WorkspacePage Test button.

    Body: { text_prompts: [str], boxes: [[x1,y1,x2,y2]], mode: str, color: [r,g,b] }
    Returns: { status, image: 'data:image/png;base64,...' } on success.
    """
    import cv2
    import numpy as np
    from ...bridge.image_reader import ImageBridgeReader
    from ...utils.sam3_helper import is_extension_installed, preview_mask

    if not is_extension_installed():
        return {
            'status': 'error',
            'message': 'SAM3 extension is not installed. Install the sam3 module first.',
        }, 400

    body = request.json or {}
    cfg = {
        'enabled': True,
        'text_prompts': body.get('text_prompts') or [],
        'boxes': body.get('boxes') or [],
        'mode': body.get('mode', 'background'),
        'color': body.get('color') or [0, 0, 0],
    }

    reader = ImageBridgeReader()
    image, _ts = reader.read(f'sensor_{id}')
    if image is None:
        return {
            'status': 'error',
            'message': 'No frame available — make sure the sensor is running.',
        }, 404

    masked = preview_mask(image, cfg)
    if masked is None:
        return {
            'status': 'error',
            'message': 'SAM3 preview failed. Check HF token and prompts.',
        }, 500

    ok, buf = cv2.imencode('.png', masked)
    if not ok:
        return {'status': 'error', 'message': 'Encoding failed'}, 500
    b64 = base64.b64encode(buf.tobytes()).decode('ascii')
    return {
        'status': 'success',
        'image': f'data:image/png;base64,{b64}',
    }, 200
