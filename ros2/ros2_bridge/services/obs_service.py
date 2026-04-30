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

        subs = []
        for sensor in sensors:
            sensor_id = sensor['id']
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

        # 메인 노드에 등록된 subscription을 개별 destroy (노드 자체는 유지).
        for sub in session.get('subscriptions', []):
            try:
                self._node.destroy_subscription(sub)
            except Exception as e:
                print(f"[ObsService] destroy_subscription error: {e}", flush=True)

        # Clean up shm files for this session's sensors
        for sensor in session.get('sensors', []):
            sensor_key = f"sensor_{sensor['id']}"
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
