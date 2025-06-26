# -*- coding: utf-8 -*-
import eventlet
eventlet.monkey_patch()  # eventlet를 사용하기 위해 필요한 패치

from flask import Flask, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS

from .process_manager import ProcessManager
import time

from .routes.sensor import sensor_bp, SensorNamespace
from .routes.robot import robot_bp, RobotNamespace

import rospy

# Flask 앱과 SocketIO 객체 생성
app = Flask(__name__)
CORS(app)
app.config['SECRET_KEY'] = 'mysecretkey!' # 실제 운영 환경에서는 더 복잡한 키 사용
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

pm = ProcessManager(socketio)  # 프로세스 관리 객체 생성

app.register_blueprint(sensor_bp, url_prefix='/api')
app.register_blueprint(robot_bp, url_prefix='/api')
app.pm = pm

socketio.on_namespace(SensorNamespace('/sensor', pm))
socketio.on_namespace(RobotNamespace('/sensor', pm))


pcs = set()

rospy.init_node("web_api_node", anonymous=True)

@app.route('/api/processes', methods=['GET'])
def list_processes():
    processes = pm.list_processes()
    return {
        'status': 'success',
        'processes': processes
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


# @app.route('/api/offer', methods=['POST'])
@socketio.on('offer')
def handle_offer_event(sid, data):
    # data = request.json
    """클라이언트의 offer 요청을 처리 (올바른 방식)"""
    print('offer event received', sid, data)
    # return await sm.offer(data)


# @socketio.on('ice_candidate')
# async def handle_ice_event(data):
#     """클라이언트의 ice_candidate를 처리 (올바른 방식)"""
#     if data:
#         await sm.handle_ice_candidate(request.sid, data)


# 서버 실행
if __name__ == '__main__':
    # Flask 앱을 직접 실행하는 대신, socketio.run()을 사용해야 합니다.
    # host='0.0.0.0'은 외부에서도 접속 가능하도록 설정합니다.
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False)


