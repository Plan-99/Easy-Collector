# -*- coding: utf-8 -*-
"""
gRPC StreamingService: ROS2 카메라 토픽을 구독하고 JPEG 프레임을 server-streaming으로 전송.
backend 컨테이너의 WebRTC 서버가 이 스트림을 수신하여 클라이언트에 전달.
"""
import threading
import time
import uuid
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image

# Sensor 데이터는 관례상 BEST_EFFORT. RELIABLE publisher와도 호환됨(다운그레이드).
# RELIABLE 구독자는 BEST_EFFORT 퍼블리셔와 incompatible → 메시지 안 옴.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc


class _TopicSubscriber:
    """하나의 ROS2 이미지 토픽을 구독하고 최신 JPEG 프레임을 유지."""

    def __init__(self, node: Node, topic: str, msg_type: str, stream_id: str):
        self.stream_id = stream_id
        self.topic = topic
        self.node = node
        self._frame = None  # latest JPEG bytes
        self._lock = threading.Lock()
        self._active = True

        is_compressed = msg_type != 'sensor_msgs/Image'
        ros_type = CompressedImage if is_compressed else Image
        self._is_compressed = is_compressed

        self._sub = node.create_subscription(ros_type, topic, self._callback, _SENSOR_QOS)

    def _callback(self, msg):
        try:
            if self._is_compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                h, w = msg.height, msg.width
                encoding = msg.encoding
                np_arr = np.frombuffer(msg.data, np.uint8)
                if encoding in ('rgb8', 'bgr8'):
                    cv_image = np_arr.reshape((h, w, 3))
                    if encoding == 'rgb8':
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                elif encoding == 'mono8':
                    cv_image = np_arr.reshape((h, w))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                else:
                    cv_image = np_arr.reshape((h, w, 3))

            if cv_image is None or cv_image.size == 0:
                return

            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            h, w = cv_image.shape[:2]

            with self._lock:
                self._frame = (jpeg.tobytes(), w, h)
        except Exception as e:
            self.node.get_logger().error(f"[StreamingSvc] {self.stream_id} callback error: {e}")

    def get_frame(self):
        with self._lock:
            return self._frame

    def destroy(self):
        self._active = False
        try:
            self.node.destroy_subscription(self._sub)
        except Exception:
            pass


class StreamingServiceServicer(pb_grpc.StreamingServiceServicer):
    """gRPC StreamingService 구현."""

    def __init__(self, node: Node):
        self.node = node
        self._subscribers = {}  # stream_id -> _TopicSubscriber
        self._lock = threading.Lock()

    def SubscribeImage(self, request, context):
        """ROS2 토픽을 구독하고 JPEG 프레임을 server-streaming으로 전송."""
        topic = request.topic
        msg_type = request.msg_type or 'sensor_msgs/CompressedImage'
        stream_id = request.stream_id or str(uuid.uuid4())

        subscriber = _TopicSubscriber(self.node, topic, msg_type, stream_id)
        with self._lock:
            self._subscribers[stream_id] = subscriber

        self.node.get_logger().info(f"[StreamingSvc] Subscribed: {stream_id} → {topic}")

        try:
            while context.is_active():
                frame_data = subscriber.get_frame()
                if frame_data is not None:
                    jpeg_bytes, w, h = frame_data
                    yield pb.ImageFrame(
                        jpeg_data=jpeg_bytes,
                        width=w,
                        height=h,
                        stream_id=stream_id,
                    )
                time.sleep(1.0 / 30.0)  # ~30 FPS
        finally:
            with self._lock:
                self._subscribers.pop(stream_id, None)
            subscriber.destroy()
            self.node.get_logger().info(f"[StreamingSvc] Unsubscribed: {stream_id}")

    def UnsubscribeImage(self, request, context):
        stream_id = request.stream_id
        with self._lock:
            sub = self._subscribers.pop(stream_id, None)
        if sub:
            sub.destroy()
            return pb.StatusResponse(success=True, message=f"Unsubscribed: {stream_id}")
        return pb.StatusResponse(success=False, message=f"Not found: {stream_id}")

    def UpdateConfig(self, request, context):
        # TODO: crop/resize/rotate 설정 업데이트
        return pb.StatusResponse(success=True, message="OK")
