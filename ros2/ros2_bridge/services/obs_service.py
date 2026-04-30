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
    def __init__(self, ros_executor):
        self._executor = ros_executor
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

        # Create a dedicated ROS2 node for this session's sensor subscriptions.
        # gRPC threads can't reliably add subscriptions to the main node's executor.
        from rclpy.node import Node
        node = Node(f'obs_session_{session_id}')
        self._executor.add_node(node)

        subs = []
        for sensor in sensors:
            sensor_id = sensor['id']
            msg_type = CompressedImage
            if sensor.get('read_topic_msg') == 'sensor_msgs/Image':
                msg_type = Image
            topic = sensor['read_topic']

            sensor_key = f"sensor_{sensor_id}"

            def _cb(msg, key=sensor_key):
                try:
                    image = ros_image_to_numpy(msg)
                    if image is not None:
                        self.image_bridge.write(key, image, time.time())
                except Exception as e:
                    print(f"[ObsService] sensor callback error ({key}): {e}", flush=True)

            sub = node.create_subscription(msg_type, topic, _cb, _SENSOR_QOS)
            subs.append(sub)
            print(f"[ObsService] Subscribed: {topic} -> shm/{sensor_key}", flush=True)

        with self._lock:
            self._sessions[session_id] = {
                'sensors': sensors,
                'subscriptions': subs,
                'node': node,
            }

        return pb.SensorSessionId(id=session_id)

    def StopSensors(self, request, context):
        with self._lock:
            session = self._sessions.pop(request.id, None)

        if session is None:
            return pb.StatusResponse(success=False, message="Session not found")

        node = session.get('node')
        if node:
            # Order matters: destroy subscriptions BEFORE removing from executor +
            # destroying the node. This narrows the race window where the executor's
            # spin loop holds a reference to a subscription that's about to die.
            # The robust_spin in server.py catches the residual InvalidHandle (if any),
            # but the explicit subscription teardown here makes the race much less
            # likely to fire in the first place.
            try:
                for sub in session.get('subscriptions', []) or []:
                    try:
                        node.destroy_subscription(sub)
                    except Exception:
                        pass
            except Exception:
                pass
            try:
                self._executor.remove_node(node)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass

        # Clean up shm files for this session's sensors
        for sensor in session.get('sensors', []):
            sensor_key = f"sensor_{sensor['id']}"
            self.image_bridge.remove(sensor_key)

        print(f"[ObsService] Session {request.id} stopped", flush=True)
        return pb.StatusResponse(success=True, message="Sensors stopped")

    def destroy_all(self):
        with self._lock:
            for sid, session in self._sessions.items():
                node = session.get('node')
                if node:
                    try:
                        self._executor.remove_node(node)
                        node.destroy_node()
                    except Exception:
                        pass
            self._sessions.clear()
        self.image_bridge.cleanup()
