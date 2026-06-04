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
from .routes.sim import sim_bp
from .routes.robot_pose import robot_pose_bp
from .routes.remote_train import remote_train_bp
from .routes.tutorial import tutorial_bp
from .routes.dual_arm_test import dual_arm_test_bp
from .routes.dual_arm_assembly_test import dual_arm_assembly_test_bp
from .routes.remote_train import run_training_job
from .routes.planner import planner_bp
from .routes.curriculum import curriculum_bp
from .routes.module import module_bp
from .routes.gpu import gpu_bp

from ..bridge.client import get_bridge_client
from ..bridge.remote_agent import RemoteAgent
from .ros2_log_streamer import ROS2LogStreamer
from .topic_watcher import TopicWatcher
from .training_scheduler import TrainingScheduler

import usb.core
import usb.util

import os
import argparse
import logging

import threading
from ..database.config.database import db as peewee_db

argparse = argparse.ArgumentParser(description='Easy Collector Web API')
argparse.add_argument('--debug', action='store_true', help='Enable debug mode')
args = argparse.parse_args()
# 디버그 모드 설정
# debug = args.debug
debug = True

class _HealthzFilter(logging.Filter):
    _SILENCED = ('GET /api/healthz', 'GET /api/health ', 'GET / HTTP')

    def filter(self, record):
        msg = record.getMessage()
        return not any(s in msg for s in self._SILENCED)

logging.getLogger('werkzeug').addFilter(_HealthzFilter())

# Flask 앱과 SocketIO 객체 생성
app = Flask(__name__)
CORS(app)
app.config['SECRET_KEY'] = 'mysecretkey!' # 실제 운영 환경에서는 더 복잡한 키 사용
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading',
)

pm = ProcessManager(socketio, debug=debug)  # 프로세스 관리 객체 생성

bridge_client = get_bridge_client()
node = None  # ROS2 node는 ROS2 컨테이너에서 관리

# Topic watcher — 1Hz로 ros2_bridge에 ListTopics 호출하여 변경 시 'topics_changed' 발행.
# main()에서 bridge_client.wait_for_ready 후 start() 호출.
topic_watcher = TopicWatcher(socketio, bridge_client)

# Training scheduler — 학습 큐의 단일 진실원천. main()에서 start().
# Runner는 routes.remote_train.run_training_job — pure 실행자.
training_scheduler = TrainingScheduler(socketio, runner_fn=run_training_job)
app.training_scheduler = training_scheduler

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
app.register_blueprint(sim_bp, url_prefix='/api')
app.register_blueprint(robot_pose_bp, url_prefix='/api')
app.register_blueprint(remote_train_bp, url_prefix='/api')
app.register_blueprint(tutorial_bp, url_prefix='/api')
app.register_blueprint(dual_arm_test_bp, url_prefix='/api')
app.register_blueprint(dual_arm_assembly_test_bp, url_prefix='/api')
app.register_blueprint(planner_bp, url_prefix='/api')
app.register_blueprint(curriculum_bp, url_prefix='/api')
app.register_blueprint(module_bp, url_prefix='/api')
app.register_blueprint(gpu_bp, url_prefix='/api')

socketio.on_namespace(SensorNamespace('/sensor', pm))
socketio.on_namespace(RobotNamespace('/robot', pm))


peewee_db.connect(reuse_if_open=True)


pcs = set()

# 스트리밍은 ROS2 컨테이너에서 실행됨 (port 5002)

@app.route('/api/healthz', methods=['GET'])
@app.route('/api/health', methods=['GET'])
def healthz():
    return {
        'status': 'ok',
        'ros_ok': bridge_client.is_ready(),
    }, 200

@app.route('/', methods=['GET'])
def root():
    return {'status': 'ok'}, 200

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
    from ..bridge.generated import robot_bridge_pb2 as pb
    try:
        result = bridge_client.driver.ListTopics(pb.Empty())
        active_topic_list = [
            {'name': t.name, 'type': t.type}
            for t in result.topics
        ]
    except Exception:
        active_topic_list = []

    return {
        'status': 'success',
        'topics': active_topic_list
    }, 200

@app.route('/api/stop_process', methods=['POST'])
def stop_process():
    data = request.json
    entry = pm.processes.get(data['name'])
    if entry is None:
        return {
            'status': 'error',
            'message': f"Process '{data['name']}' not found."
        }, 404

    # 기존 코드가 `entry == 'function'`으로 비교하던 자리에 실제 type 필드를 검사.
    # entry는 {'type': 'function'|'subprocess', 'obj': ...} 형태.
    if entry.get('type') == 'function':
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
    # Topic snapshot을 즉시 보내서 클라이언트가 첫 push 이벤트 기다리지 않도록.
    try:
        emit('topics_changed', {'topics': topic_watcher.get_snapshot()})
    except Exception as e:
        print(f"[connect] failed to send topic snapshot: {e}")

# 'disconnect' 이벤트: 클라이언트 연결이 끊어졌을 때
@socketio.on('disconnect')
def handle_disconnect():
    # 핸들러 자체에서 예외가 새어나가면 flask-socketio 가 websocket cleanup
    # 도중 native (engineio C / gevent) 레이어로 던져 faulthandler 가 그
    # context 에서 죽는 케이스 관찰됨. 항상 안전하게 swallow.
    try:
        print('Client disconnected')
    except Exception:
        pass


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
    # Peewee: 테이블이 없으면 자동 생성
    from ..database.migrate import migrate
    migrate()

    # Tutorial robot/sensor rows are seeded eagerly at boot so the UI shows
    # them from the very first render — independent of whether the sim is
    # currently running.
    try:
        from .routes.tutorial import _ensure_tutorial_rows
        _ensure_tutorial_rows()
    except Exception as e:
        print(f"[WARN] Tutorial seeding skipped: {e}")

    # Dual-arm TEST workspaces are seeded eagerly too so they appear in the UI
    # workspace list without first hitting :start.
    try:
        from .routes.sim_test_common import ensure_rows as _ensure_sim_test_rows
        from ..configs.dual_arm_defaults import SPEC as _DUAL_ARM_SPEC
        from ..configs.dual_arm_assembly_defaults import SPEC as _DUAL_ARM_ASM_SPEC
        _ensure_sim_test_rows(_DUAL_ARM_SPEC)
        _ensure_sim_test_rows(_DUAL_ARM_ASM_SPEC)
    except Exception as e:
        print(f"[WARN] Dual-arm test seeding skipped: {e}")

    app.node = node  # 호환성 유지 (None)
    app.bridge_client = bridge_client  # gRPC bridge 클라이언트
    app.pm = pm  # Flask 앱에 프로세스 관리 객체 할당
    app.agents = {}  # Flask 앱에 에이전트 딕셔너리 할당 (RemoteAgent)

    try:
        # ROS2 bridge 연결 대기
        print("Waiting for ROS2 bridge connection...")
        bridge_client.wait_for_ready(timeout=60)

        # ROS2 컨테이너 로그 스트리밍 시작
        ros2_log_streamer = ROS2LogStreamer(socketio)
        ros2_log_streamer.start()

        # Topic watcher 시작 — bridge가 ready 된 다음에 시작해야 첫 ListTopics가 성공.
        topic_watcher.start()
        print("TopicWatcher started (1Hz, emits 'topics_changed' on diff)")

        # Training scheduler 시작 — 학습 큐 워커. enqueue 시 자동으로 깨어남.
        training_scheduler.start()
        print("TrainingScheduler started (single worker, FIFO queue)")

        # 재시작 직전에 학습 중이던 체크포인트들에 대해 polling 재개. backend
        # 가 죽었다 깨어나도 training_server 의 학습 결과를 놓치지 않게.
        try:
            from .routes.remote_train import resume_inflight_trainings
            resume_inflight_trainings(app, socketio)
        except Exception as _e:
            print(f"[resume_inflight] failed: {_e}")

        # WebRTC streaming 서버 시작 (port 5002, 별도 스레드)
        from .streaming import start_streaming_server
        streaming_thread = threading.Thread(
            target=start_streaming_server,
            kwargs={'grpc_streaming_stub': bridge_client.streaming},
            daemon=True,
        )
        streaming_thread.start()
        print("WebRTC streaming server started on port 5002")

        print("Starting Flask-SocketIO server...")
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
