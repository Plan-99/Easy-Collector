# -*- coding: utf-8 -*-
"""
Observation Service — 센서 토픽 구독 lifecycle 관리.
구독된 센서 이미지를 /dev/shm/easytrainer/에 지속적으로 기록.
backend에서는 ImageBridgeReader로 shm에서 직접 읽음 (gRPC 불필요).
"""
import json
import threading
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc
from .image_bridge import ImageBridgeWriter
from ..utils.image_parser import ros_image_to_numpy

# Sensor 이미지 토픽은 관례상 BEST_EFFORT 퍼블리시. RELIABLE 구독이면 incompatible.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ObsServiceServicer(pb_grpc.ObsServiceServicer):
    def __init__(self, node):
        # 이미 spin 중인 메인 bridge 노드를 공유한다 — 세션마다 새 Node를 만들면
        # DDS discovery가 제때 매칭되지 못해 publisher가 살아있어도 callback이
        # 한 번도 fire되지 않는 케이스가 있었다(record_episode timeout 원인).
        self._node = node
        self._sessions = {}  # session_id -> session dict
        self._next_id = 1
        self._lock = threading.Lock()
        self.image_bridge = ImageBridgeWriter()

    def StartSensors(self, request, context):
        sensors = json.loads(request.sensors_json) if request.sensors_json else []
        if not sensors:
            return pb.SensorSessionId(id=-1)

        with self._lock:
            session_id = self._next_id
            self._next_id += 1

        # Multi-view 지원: 같은 물리 sensor_id 가 workspace 내 여러 view 로
        # 등장할 수 있다. shm 키는 *물리 sensor_id 기준* (suffix 없음) 으로
        # 한 번만 쓴다 — raw frame 은 모든 view 가 공유하고, per-view 의
        # crop/rotate/resize 는 consumer (record_episode/checkpoint_test) 가
        # fetch_image_with_config 로 view_key 별로 적용한다.
        # 따라서 호출자가 같은 sensor_id 를 중복 전달해도 한 번만 subscribe.
        seen_ids: set = set()
        subs = []
        for sensor in sensors:
            sensor_id = sensor['id']
            if sensor_id in seen_ids:
                continue
            seen_ids.add(sensor_id)
            msg_type = CompressedImage
            if sensor.get('read_topic_msg') == 'sensor_msgs/Image':
                msg_type = Image
            topic = sensor['read_topic']
            if not topic:
                print(f"[ObsService] sensor {sensor_id} has empty read_topic, skipping", flush=True)
                continue

            sensor_key = f"sensor_{sensor_id}"

            def _cb(msg, key=sensor_key):
                try:
                    image = ros_image_to_numpy(msg)
                    if image is not None:
                        self.image_bridge.write(key, image, time.time())
                except Exception as e:
                    print(f"[ObsService] sensor callback error ({key}): {e}", flush=True)

            sub = self._node.create_subscription(msg_type, topic, _cb, _SENSOR_QOS)
            subs.append(sub)
            print(f"[ObsService] Subscribed: {topic} -> shm/{sensor_key}", flush=True)

        with self._lock:
            self._sessions[session_id] = {
                'sensors': sensors,
                'subscriptions': subs,
            }

        return pb.SensorSessionId(id=session_id)

    def StopSensors(self, request, context):
        with self._lock:
            session = self._sessions.pop(request.id, None)

        if session is None:
            return pb.StatusResponse(success=False, message="Session not found")

        # origin/integration_2이 ObsService를 재설계 — per-session Node를 안 만들고
        # 메인 bridge node를 공유. 그래서 StopSensors에서 destroy_node 자체가 사라졌고
        # subscription만 개별 destroy하면 됨. 이전 commit 58cfe1d에서 내가 fix하려던
        # InvalidHandle race condition은 이 구조 변경으로 자연스럽게 사라짐 (destroy할
        # node가 없으니 race 자체가 발생 안 함). server.py의 robust_spin은 그래도
        # defense-in-depth로 유지 (다른 경로의 transient error 방어).
        for sub in session.get('subscriptions', []):
            try:
                self._node.destroy_subscription(sub)
            except Exception as e:
                print(f"[ObsService] destroy_subscription error: {e}", flush=True)

        # Clean up shm files for this session's sensors. Multi-view: 같은
        # 물리 sensor_id 가 여러 view 로 등장할 수 있으니 dedup 후 한 번씩만 remove.
        _seen_cleanup: set = set()
        for sensor in session.get('sensors', []):
            sid = sensor['id']
            if sid in _seen_cleanup:
                continue
            _seen_cleanup.add(sid)
            sensor_key = f"sensor_{sid}"
            self.image_bridge.remove(sensor_key)

        print(f"[ObsService] Session {request.id} stopped", flush=True)
        return pb.StatusResponse(success=True, message="Sensors stopped")

    def destroy_all(self):
        with self._lock:
            for sid, session in self._sessions.items():
                for sub in session.get('subscriptions', []):
                    try:
                        self._node.destroy_subscription(sub)
                    except Exception:
                        pass
            self._sessions.clear()
        self.image_bridge.cleanup()
