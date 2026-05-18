# -*- coding: utf-8 -*-
"""
ROS2 컨테이너 로그 스트리머.
공유 메모리(/dev/shm/easytrainer/ros2.log)를 tail하여
SocketIO task_log 이벤트로 프론트엔드에 전달한다.

드라이버 로그는 [process_id/stdout] 접두사를 파싱하여
해당 process_id의 ProcessConsole에 자동으로 표시된다.
일반 브릿지 로그는 'ros2_bridge' ID로 전달된다.
"""
import os
import re
import threading
import time

_PROCESS_PREFIX_RE = re.compile(r'^\[([^/\]]+)/(stdout|stderr)\]\s*(.*)')

# 무시할 노이즈 패턴
_IGNORE_PATTERNS = [
    'logging.getLogger',
    'setLevel',
]


class ROS2LogStreamer:
    LOG_PATH = '/dev/shm/easytrainer/ros2.log'
    FALLBACK_ID = 'ros2_bridge'

    def __init__(self, socketio):
        self.socketio = socketio
        self._thread = None
        self._stop_event = threading.Event()

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._tail_loop, daemon=True)
        self._thread.start()
        print("[ROS2LogStreamer] Started tailing", self.LOG_PATH)

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _tail_loop(self):
        while not self._stop_event.is_set():
            if not os.path.exists(self.LOG_PATH):
                time.sleep(1)
                continue
            try:
                with open(self.LOG_PATH, 'r', encoding='utf-8', errors='replace') as f:
                    # 기존 내용 건너뛰기 (최근 50줄만 표시)
                    lines = f.readlines()
                    for line in lines[-50:]:
                        self._emit_line(line.strip())

                    # 이후 새 줄 실시간 추적
                    while not self._stop_event.is_set():
                        line = f.readline()
                        if not line:
                            time.sleep(0.2)
                            continue
                        self._emit_line(line.strip())
            except Exception as e:
                print(f"[ROS2LogStreamer] Error: {e}")
                time.sleep(2)

    def _emit_line(self, line):
        if not line:
            return
        for pattern in _IGNORE_PATTERNS:
            if pattern in line:
                return

        match = _PROCESS_PREFIX_RE.match(line)
        if match:
            process_id = match.group(1)
            stream_type = match.group(2)
            message = match.group(3)
            # rclpy logger 는 [INFO]/[WARN]/[ERROR] 를 모두 stderr 로 내보내므로
            # 스트림이 아니라 메시지 prefix 로 분류한다. prefix 가 아예 없는
            # stderr (Python 트레이스백 등) 만 'error' 로 떨군다.
            detected = _detect_type(message)
            if detected is not None:
                msg_type = detected
            elif stream_type == 'stderr':
                msg_type = 'error'
            else:
                msg_type = 'stdout'
            self.socketio.emit('task_log', {
                'id': process_id,
                'message': message,
                'type': msg_type,
            })
        else:
            self.socketio.emit('task_log', {
                'id': self.FALLBACK_ID,
                'message': line,
                'type': _detect_type(line) or 'stdout',
            })


def _detect_type(message):
    """severity prefix 가 감지되면 type 문자열을, 없으면 None 을 반환."""
    m = message.strip()
    if m.startswith('[ERROR]') or m.startswith('[FATAL]') or m.startswith('ERROR'):
        return 'error'
    if m.startswith('[WARNING]') or m.startswith('[WARN]'):
        return 'warning'
    if m.startswith('[SUCCESS]'):
        return 'success'
    if m.startswith('[NOTICE]'):
        return 'notice'
    if m.startswith('[INFO]') or m.startswith('[DEBUG]'):
        return 'stdout'
    return None
