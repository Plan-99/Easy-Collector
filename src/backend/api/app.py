# -*- coding: utf-8 -*-
# import eventlet
# eventlet.monkey_patch()  # eventlet를 사용하기 위해 필요한 패치
# from eventlet import tpool

from flask import Flask, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS

from .process_manager import ProcessManager
import time

from .routes.sensor import sensor_bp, SensorNamespace
from .routes.robot import robot_bp, RobotNamespace
from .routes.leader_robot import leader_robot_bp
from .routes.task import task_bp
from .routes.dataset import dataset_bp
from .routes.policy import policy_bp
from .routes.checkpoint import checkpoint_bp

import rclpy
from rclpy.node import Node

import usb.core
import usb.util

from orator import DatabaseManager, Model

import os
import argparse

import threading

argparse = argparse.ArgumentParser(description='Easy Collector Web API')
argparse.add_argument('--debug', action='store_true', help='Enable debug mode')
args = argparse.parse_args()
# 디버그 모드 설정
debug = args.debug

# Flask 앱과 SocketIO 객체 생성
app = Flask(__name__)
CORS(app)
app.config['SECRET_KEY'] = 'mysecretkey!' # 실제 운영 환경에서는 더 복잡한 키 사용
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

pm = ProcessManager(socketio, debug=debug)  # 프로세스 관리 객체 생성

rclpy.init(args=None)
node = Node("web_api_node")

app.register_blueprint(sensor_bp, url_prefix='/api')
app.register_blueprint(robot_bp, url_prefix='/api')
app.register_blueprint(leader_robot_bp, url_prefix='/api')
app.register_blueprint(task_bp, url_prefix='/api')
app.register_blueprint(dataset_bp, url_prefix='/api')
app.register_blueprint(policy_bp, url_prefix='/api')
app.register_blueprint(checkpoint_bp, url_prefix='/api')

socketio.on_namespace(SensorNamespace('/sensor', pm))
socketio.on_namespace(RobotNamespace('/robot', pm))


BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DB_DIR = os.path.join(BASE_DIR, 'backend/database')
DB_PATH = os.path.join(DB_DIR, 'main.db')

config = {
    'mysql': {
        'driver': 'sqlite',
        'database': DB_PATH,
    }
}

db = DatabaseManager(config)
Model.set_connection_resolver(db)


pcs = set()

pm.start_process(
    'streaming',
    ['python3', '-m', 'backend.api.streaming']
)

pm.start_process(
    'rosbridge_websocket',
    ['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml', 'port:=9090']
)

@app.route('/api/processes', methods=['GET'])
def list_processes():
    processes = pm.list_processes()
    return {
        'status': 'success',
        'processes': processes
    }, 200

@app.route('/api/devices', methods=['GET'])
def list_devices():
    devices = []
    usb_devices = usb.core.find(find_all=True)

    for dev in usb_devices:
        manufacturer = usb.util.get_string(dev, dev.iManufacturer)
        product = usb.util.get_string(dev, dev.iProduct)
        
        devices.append({
            'product': product,
            'manufacturer': manufacturer,
            'id_vendor': hex(dev.idVendor),
            'id_product': hex(dev.idProduct),
        })
        print(f"장치: {product} | 제조사: {manufacturer} | ID: {hex(dev.idVendor)}:{hex(dev.idProduct)}")

    return {
        'status': 'success',
        'devices': devices
    }

@app.route('/api/topics', methods=['GET'])
def list_topics():
    topics = node.get_topic_names_and_types()
    topic_list = [{'name': topic[0], 'type': topic[1][0]} for topic in topics]
    
    return {
        'status': 'success',
        'topics': topic_list
    }, 200

@app.route('/api/stop_process', methods=['POST'])
def stop_process():
    data = request.json
    if pm.processes.get(data['name']) is None:
        return {
            'status': 'error',
            'message': f"Process '{data['name']}' not found."
        }, 404
    
    if pm.processes[data['name']] == 'function':
        pm.stop_function(data['name'])
    else:
        pm.stop_process(data['name'])

    return {
        'status': 'success',
        'message': f"Process '{data['name']}' stopped."
    }, 200


# --- SocketIO 이벤트 핸들러 ---

# 'connect' 이벤트: 클라이언트가 처음 연결했을 때
@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    emit('response', {'data': '서버에 연결되었습니다!'})

# 'disconnect' 이벤트: 클라이언트 연결이 끊어졌을 때
@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')


@socketio.on('offer')
def handle_offer_event(sid, data):
    # data = request.json
    """클라이언트의 offer 요청을 처리 (올바른 방식)"""
    print('offer event received', sid, data)
    # return await sm.offer(data)


def main():
    app.node = node  # Flask 앱에 ROS 노드 할당
    app.pm = pm  # Flask 앱에 프로세스 관리 객체 할당
    
    try:
        # 3. 메인 스레드에서 웹 서버 실행
        print("Starting Flask-SocketIO server...")
        # rclpy.spin을 별도의 스레드에서 실행
        executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        executor_thread.start()
        # 이 함수는 서버가 종료(예: Ctrl+C)될 때까지 여기서 멈춥니다.
        socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False)

    finally:
        import subprocess
        print("\nServer is shutting down. Executing kill script...")
        result = subprocess.run(
            ['/bin/bash', '/root/src/kill.sh'], 
            check=True,
            capture_output=True, # 스크립트의 출력을 캡처합니다.
            text=True            # 출력을 텍스트(문자열)로 다룹니다.
        )
        print("kill.sh script executed successfully.")
        print(f"Script output:\n{result.stdout}")

if __name__ == '__main__':
    main()