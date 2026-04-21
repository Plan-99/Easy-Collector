# -*- coding: utf-8 -*-
import faulthandler
import sys

# Segfault 발생 시 traceback을 stderr에 출력
faulthandler.enable(file=sys.stderr, all_threads=True)

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
from .routes.vla import vla_bp
from .routes.teleoperator import teleoperator_bp
from .routes.assembly import assembly_bp
from .routes.remote_train import remote_train_bp

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    rclpy = None
    Node = None
    MultiThreadedExecutor = None

import usb.core
import usb.util

from orator import DatabaseManager, Model

import os
import argparse
import logging

import threading
from ..database.config.database import DATABASES

argparse = argparse.ArgumentParser(description='Easy Collector Web API')
argparse.add_argument('--debug', action='store_true', help='Enable debug mode')
args = argparse.parse_args()
# 디버그 모드 설정
# debug = args.debug
debug = True

class _HealthzFilter(logging.Filter):
    def filter(self, record):
        return 'GET /api/healthz' not in record.getMessage()

logging.getLogger('werkzeug').addFilter(_HealthzFilter())

# Flask 앱과 SocketIO 객체 생성
app = Flask(__name__)
CORS(app)
app.config['SECRET_KEY'] = 'mysecretkey!' # 실제 운영 환경에서는 더 복잡한 키 사용
# Use threading mode to avoid eventlet/ROS compatibility issues
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading',
    # logger=False,
    # engineio_logger=False,
    # allow_upgrades=False,  # disable websocket upgrade (Werkzeug can't serve websockets)
    # transports=["polling"],  # force long-polling for compatibility
)

pm = ProcessManager(socketio, debug=debug)  # 프로세스 관리 객체 생성

node = None
if HAS_ROS:
    rclpy.init(args=None)
    node = Node("web_api_node")

app.register_blueprint(sensor_bp, url_prefix='/api')
app.register_blueprint(robot_bp, url_prefix='/api')
app.register_blueprint(leader_robot_bp, url_prefix='/api')
app.register_blueprint(task_bp, url_prefix='/api')
app.register_blueprint(dataset_bp, url_prefix='/api')
app.register_blueprint(policy_bp, url_prefix='/api')
app.register_blueprint(checkpoint_bp, url_prefix='/api')
app.register_blueprint(vla_bp, url_prefix='/api')
app.register_blueprint(teleoperator_bp, url_prefix='/api')
app.register_blueprint(assembly_bp, url_prefix='/api')
app.register_blueprint(remote_train_bp, url_prefix='/api')

socketio.on_namespace(SensorNamespace('/sensor', pm))
socketio.on_namespace(RobotNamespace('/robot', pm))


db = DatabaseManager(DATABASES)
Model.set_connection_resolver(db)


pcs = set()

pm.start_process(
    'streaming',
    ['python3', '-m', 'backend.api.streaming']
)

# pm.start_process(
#     'rosbridge_websocket',
#     ['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml', 'port:=9090']
# )

@app.route('/api/healthz', methods=['GET'])
def healthz():
    return {
        'status': 'ok',
        'ros_ok': bool(rclpy.ok()) if HAS_ROS else False,
    }, 200

@app.route('/api/db/path', methods=['GET'])
def db_path():
    try:
        path = DATABASES.get('sqlite', {}).get('database')
        return {
            'status': 'success',
            'path': path,
        }, 200
    except Exception as e:
        return {
            'status': 'error',
            'message': str(e),
        }, 500

@app.route('/api/db/reload', methods=['POST'])
def db_reload():
    global db
    try:
        try:
            db.disconnect()
        except Exception:
            pass
        db = DatabaseManager(DATABASES)
        Model.set_connection_resolver(db)
        return {
            'status': 'success',
        }, 200
    except Exception as e:
        return {
            'status': 'error',
            'message': str(e),
        }, 500

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
    if not HAS_ROS or node is None:
        return {'status': 'success', 'topics': []}, 200
    all_topics = node.get_topic_names_and_types()
    active_topic_list = [
        {'name': topic_name, 'type': topic_types[0]}
        for topic_name, topic_types in all_topics
        if node.count_publishers(topic_name) > 0
    ]

    return {
        'status': 'success',
        'topics': active_topic_list
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


@socketio.on('move_robot_joint')
def handle_move_robot_joint_event(data):
    try:
        agent = app.agents[data['robot']['id']]
        agent.moved_by_ui = True
        if agent.move_lock:
            return
        agent.move_joint_step(data['goal_pos'])
    except Exception as e:
        print(f"[ERROR] move_robot_joint: {e}")

@socketio.on('move_robot_ee')
def handle_move_robot_ee_event(data):
    try:
        agent = app.agents[data['robot']['id']]
        agent.moved_by_ui = True
        if agent.move_lock:
            return
        agent.move_ee_step(data['goal_pos'])
    except Exception as e:
        print(f"[ERROR] move_robot_ee: {e}")

@socketio.on('move_robot_joint_delta')
def handle_move_robot_joint_delta_event(data):
    try:
        agent = app.agents[data['robot']['id']]
        agent.moved_by_ui = True
        if agent.move_lock:
            return
        agent.move_joint_delta_step(data['delta_pos'])
    except Exception as e:
        print(f"[ERROR] move_robot_joint_delta: {e}")

@socketio.on('move_robot_ee_delta')
def handle_move_robot_ee_delta_event(data):
    try:
        agent = app.agents[data['robot']['id']]
        agent.moved_by_ui = True
        if agent.move_lock:
            return
        delta_pos = {
            k: [float(v) for v in vals]
            for k, vals in data['delta_pos'].items()
        }
        agent.move_ee_delta_step(delta_pos, vel_arg=0.2)
    except Exception as e:
        print(f"[ERROR] move_robot_ee_delta: {e}")


def main():
    app.node = node  # Flask 앱에 ROS 노드 할당 (None if no ROS)
    app.pm = pm  # Flask 앱에 프로세스 관리 객체 할당
    app.agents = {}  # Flask 앱에 에이전트 딕셔너리 할당

    try:
        print("Starting Flask-SocketIO server...")
        if HAS_ROS and node is not None:
            ros_executor = MultiThreadedExecutor()
            ros_executor.add_node(node)
            executor_thread = threading.Thread(target=ros_executor.spin, daemon=True)
            executor_thread.start()
        else:
            print("[WARN] ROS 2 not available — running without ROS integration")
        # 이 함수는 서버가 종료(예: Ctrl+C)될 때까지 여기서 멈춥니다.
        socketio.run(
            app,
            host='0.0.0.0',
            port=5000,
            debug=False,
            allow_unsafe_werkzeug=True,
        )


    finally:
        import subprocess
        print("\nServer is shutting down. Executing kill script...")
        try:
            result = subprocess.run(
                ['/bin/bash', '/root/src/kill.sh'],
                capture_output=True,
                text=True,
                timeout=10,
            )
            print(f"kill.sh exited with code {result.returncode}")
            if result.stdout.strip():
                print(f"Script output:\n{result.stdout}")
        except Exception as e:
            print(f"[WARN] kill.sh failed: {e}")

if __name__ == '__main__':
    main()
