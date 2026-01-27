import json
from flask import Blueprint, request, current_app, jsonify
from flask_socketio import Namespace, emit
from ...database.models.sensor_model import Sensor as SensorModel
from ...configs.global_configs import SUPPORT_SENSORS
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# 1. Blueprint 생성
# 이 블루프린트는 카메라와 관련된 'HTTP' 라우트를 관리합니다.
sensor_bp = Blueprint('sensor', __name__)


class SensorNamespace(Namespace):
    def __init__(self, namespace, process_manager, node):
        super().__init__(namespace)
        self.pm = process_manager
        self.node = node
        self.active_subs = {}

    def on_connect(self):
        print(f'Client connected to /sensor namespace: {request.sid}')
        # emit('response', {'data': '카메라 제어 채널에 연결되었습니다.'})

    def on_disconnect(self):
        print(f'Client disconnected from /sensor namespace: {request.sid}')

    def on_subscribe_topic(self, data):
        topic_name = data['topic']
        msg_type_str = data.get('type') # 예: 'sensor_msgs/msg/Imu'

        print(f"[ROS] Subscribe request: {topic_name} ({msg_type_str})")

        # 이미 구독 중이면 패스
        if topic_name in self.active_subs:
            print(f"[ROS] Already subscribed to {topic_name}")
            return

        try:
            # 1. 메시지 타입 로드 (예: 'sensor_msgs/msg/Imu' -> class)
            try:
                MsgType = get_message(msg_type_str)
            except Exception as e:
                print(f"[ERROR] Failed to load message type '{msg_type_str}': {e}")
                return

            # 2. 데이터 처리 콜백 함수
            def callback(msg):
                try:
                    # ROS 메시지를 Dictionary(JSON)로 변환
                    data_dict = message_to_ordereddict(msg)
                    
                    # 프론트엔드로 전송
                    self.emit('ros_data', {
                        'topic': topic_name,
                        'msg': data_dict
                    })
                    # 디버깅용 로그 (데이터가 들어오는지 확인용, 나중에 주석 처리)
                    # print(f"[DEBUG] Sent data for {topic_name}") 
                except Exception as e:
                    print(f"[ERROR] JSON conversion failed for {topic_name}: {e}")

            # 3. ★ 핵심: QoS 설정 (Sensor Data 호환성) ★
            # 센서 데이터는 보통 Best Effort / Volatile 조합을 사용합니다.
            sensor_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT, # 놓친 데이터는 무시 (실시간성 중요)
                durability=QoSDurabilityPolicy.VOLATILE,      # 지나간 데이터는 받지 않음
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1                                       # 최신 1개만 유지
            )

            # 4. 구독 생성
            sub = self.node.create_subscription(
                MsgType, 
                topic_name, 
                callback, 
                sensor_qos # <-- 수정된 QoS 적용
            )
            
            self.active_subs[topic_name] = sub
            print(f"[ROS] Successfully subscribed to {topic_name} with Best Effort QoS")

        except Exception as e:
            print(f"[ROS] Critical Error inside subscribe logic: {e}")

    # 구독 취소 요청
    def on_unsubscribe_topic(self, data):
        topic_name = data['topic']
        if topic_name in self.active_subs:
            # 구독 객체 삭제 (destroy)
            self.active_subs[topic_name].destroy()
            del self.active_subs[topic_name]
            print(f"[ROS] Unsubscribed from {topic_name}")


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

    command = []
    
    if company == 'Intel':
        serial_no = data.get('serial_no', None)
        if serial_no is None:
            return {'status': 'error', 'message': 'serial_no is required'}, 400
        command = ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', f'camera_namespace:=ec_sensor_{id}', f'serial_no:="{serial_no}"']
    elif company == 'Kinova':
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
    
    read_topic = request.json.get('read_topic')
    read_topic_msg = request.json.get('read_topic_msg')

    settings_dict = {
        'serial_number': serial_no,
        'ip_address': ip_address
    }

    if type == 'custom':
        settings_dict['read_topic'] = read_topic
        settings_dict['read_topic_msg'] = read_topic_msg

    SensorModel.create(
        name=name,
        type=type,
        settings=settings_dict
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

    if type == 'custom':
        sensor.settings = {
            'read_topic': request.json.get('read_topic'),
            'read_topic_msg': request.json.get('read_topic_msg')
        }

    sensor.save()
    return {'status': 'success', 'message': 'Sensor Updated'}, 200


@sensor_bp.route('/sensor/<id>', methods=['DELETE'])
def delete_sensor(id):
    sensor = SensorModel.find(id)
    sensor.hide = True
    sensor.save()
    return {'status': 'success', 'message': 'Sensor Deleted'}, 200

