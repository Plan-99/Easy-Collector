# -*- coding: utf-8 -*-
"""
gRPC StreamingService: ROS2 카메라 토픽을 구독하고 JPEG 프레임을 server-streaming으로 전송.
backend 컨테이너의 WebRTC 서버가 이 스트림을 수신하여 클라이언트에 전달.

설계 — 토픽 별 구독 공유:
    여러 stream_id 가 같은 카메라 토픽을 구독해도 (e.g. 한 카메라의 multi-view
    viewport 10개) ROS subscriber 와 JPEG 처리는 토픽 당 한 번만 수행한다. 이전
    구조에선 stream_id 마다 새 ``_TopicSubscriber`` 를 만들어 N 배 decode+encode
    가 일어났고, CPU 가 saturate 되면 realsense USB 인터럽트 타이밍이 무너져
    V4L2 가 "No such device" 로 카메라를 잃는 사례가 발생.

JPEG passthrough:
    소스가 ``sensor_msgs/CompressedImage`` (이미 JPEG) 면 decode → re-encode 를
    하지 않고 raw bytes 를 그대로 전달. realsense / webcam 토픽은 거의 항상
    JPEG. 폭/높이는 SOF marker 만 살짝 파싱해서 얻음.
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

# Publisher 의 reliability 를 런타임에 query 해서 매칭한다. 이론상 RELIABLE pub +
# BEST_EFFORT sub 는 호환이지만, 실측에서 Cyclone/FastDDS 가 message 를 drop 하는
# 경우가 있어(특히 large CompressedImage) realsense 처럼 RELIABLE 인 publisher 에는
# 같은 RELIABLE 로 구독해야 안정적으로 흐른다. webcam_publisher(BE) 도 동일한 로직으로
# BE 로 떨어진다.
_DEFAULT_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _qos_matching_publisher(node, topic):
    """현재 publisher 의 reliability 에 맞춰 QoS 를 만든다. publisher 가 아직 없거나
    여러 개면 BEST_EFFORT 로 fallback (가장 호환성이 넓음)."""
    try:
        infos = node.get_publishers_info_by_topic(topic)
    except Exception:
        return _DEFAULT_SENSOR_QOS
    if not infos:
        return _DEFAULT_SENSOR_QOS
    rel = infos[0].qos_profile.reliability
    return QoSProfile(
        reliability=rel,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _jpeg_dimensions(data: bytes):
    """JPEG SOF marker 만 보고 (w, h) 추출. 전체 decode 보다 100배 이상 빠름.
    실패하면 (0, 0). camera 가 resolution 을 안 바꾸므로 한 번만 호출되도록
    호출자가 캐시."""
    n = len(data)
    if n < 4 or data[0] != 0xFF or data[1] != 0xD8:
        return (0, 0)
    i = 2
    while i + 8 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        # SOF0..SOF3, SOF5..SOF7, SOF9..SOF11, SOF13..SOF15
        if marker in (0xC0, 0xC1, 0xC2, 0xC3,
                      0xC5, 0xC6, 0xC7,
                      0xC9, 0xCA, 0xCB,
                      0xCD, 0xCE, 0xCF):
            # SOF: FF Cn LL LL P HH HH WW WW ...
            h = (data[i + 5] << 8) | data[i + 6]
            w = (data[i + 7] << 8) | data[i + 8]
            return (w, h)
        # skip this segment: length is 2 bytes after the marker
        if marker == 0xD8 or marker == 0xD9 or (0xD0 <= marker <= 0xD7):
            # SOI, EOI, RSTn — no length
            i += 2
            continue
        seg_len = (data[i + 2] << 8) | data[i + 3]
        i += 2 + seg_len
    return (0, 0)


from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc


class _SharedTopicSubscriber:
    """한 ROS2 이미지 토픽을 구독하고 최신 frame 을 유지. 여러 stream_id 가 공유.

    refcount 로 lifecycle 관리: ``acquire()`` 마다 ref ++, ``release()`` 가 0 이면
    실제 ``destroy()`` 호출 → ROS subscription 해제. add/release 는 모두
    servicer 의 락 안에서만 호출되는 것을 가정 (스레드 안전).
    """

    def __init__(self, node: Node, topic: str, msg_type: str):
        self.topic = topic
        self.node = node
        self._frame = None  # (jpeg_bytes, w, h)
        self._lock = threading.Lock()
        self._refs = 0

        is_compressed = msg_type != 'sensor_msgs/Image'
        ros_type = CompressedImage if is_compressed else Image
        self._is_compressed = is_compressed
        self._cached_dims = None  # (w, h) for CompressedImage — JPEG header 파싱은 1회만

        qos = _qos_matching_publisher(node, topic)
        node.get_logger().info(
            f"[StreamingSvc] subscribing {topic} reliability={qos.reliability}"
        )
        self._sub = node.create_subscription(ros_type, topic, self._callback, qos)

    def _callback(self, msg):
        try:
            if self._is_compressed:
                # Passthrough: 소스가 이미 JPEG. decode 후 re-encode 는 N 배 부하의
                # 주범이라 제거. width/height 는 SOF marker 만 파싱해서 얻고 캐시.
                raw = msg.data
                jpeg_bytes = raw if isinstance(raw, (bytes, bytearray)) else bytes(raw)
                if not jpeg_bytes:
                    return
                if self._cached_dims is None:
                    self._cached_dims = _jpeg_dimensions(jpeg_bytes)
                w, h = self._cached_dims
                with self._lock:
                    self._frame = (bytes(jpeg_bytes), w, h)
                return

            # Raw Image — JPEG 인코딩이 불가피하지만 토픽 당 한 번만 일어남.
            h_, w_ = msg.height, msg.width
            encoding = msg.encoding
            if encoding in ('16UC1', 'mono16'):
                # Depth (16-bit, mm). Colorize for viewing: percentile-normalize the
                # valid range → JET colormap; no-reading (0) pixels stay black.
                dt = '>u2' if getattr(msg, 'is_bigendian', 0) else '<u2'
                depth = np.frombuffer(msg.data, dt).reshape((h_, w_))
                valid = depth[depth > 0]
                if valid.size:
                    lo = float(np.percentile(valid, 2))
                    hi = float(np.percentile(valid, 98))
                else:
                    lo, hi = 0.0, 1.0
                if hi <= lo:
                    hi = lo + 1.0
                norm = np.clip((depth.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)
                cv_image = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
                cv_image[depth == 0] = 0
            else:
                np_arr = np.frombuffer(msg.data, np.uint8)
                if encoding in ('rgb8', 'bgr8'):
                    cv_image = np_arr.reshape((h_, w_, 3))
                    if encoding == 'rgb8':
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                elif encoding == 'mono8':
                    cv_image = np_arr.reshape((h_, w_))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                else:
                    cv_image = np_arr.reshape((h_, w_, 3))

            if cv_image is None or cv_image.size == 0:
                return

            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self._lock:
                self._frame = (jpeg.tobytes(), w_, h_)
        except Exception as e:
            self.node.get_logger().error(f"[StreamingSvc] callback error on {self.topic}: {e}")

    def get_frame(self):
        with self._lock:
            return self._frame

    # refcount 는 servicer 가 self.lock 안에서만 부른다 → 자체 락 불필요.
    def add_ref(self):
        self._refs += 1
        return self._refs

    def release_ref(self):
        self._refs -= 1
        return self._refs

    def destroy(self):
        try:
            self.node.destroy_subscription(self._sub)
        except Exception:
            pass


class StreamingServiceServicer(pb_grpc.StreamingServiceServicer):
    """gRPC StreamingService 구현. 토픽 별로 구독을 공유한다."""

    def __init__(self, node: Node):
        self.node = node
        self._by_topic: dict[str, _SharedTopicSubscriber] = {}
        # stream_id → topic. SubscribeImage 가 stream_id 를 발급할 때 기록하고
        # UnsubscribeImage 또는 finally 에서 release 할 토픽을 찾는다.
        self._stream_topic: dict[str, str] = {}
        # 같은 stream_id 에 대해 release 가 두 번 일어나는 걸 방지 (UnsubscribeImage
        # 가 외부에서 호출된 뒤 finally 에서 또 호출되는 케이스).
        self._lock = threading.Lock()

    def _acquire(self, topic: str, msg_type: str) -> _SharedTopicSubscriber:
        """topic 의 subscriber 를 얻거나 새로 만든다 + ref ++."""
        with self._lock:
            sub = self._by_topic.get(topic)
            if sub is None:
                sub = _SharedTopicSubscriber(self.node, topic, msg_type)
                self._by_topic[topic] = sub
            sub.add_ref()
            return sub

    def _release_stream(self, stream_id: str) -> bool:
        """stream_id 의 ref 를 release 한다. 이미 release 된 stream_id 면 False."""
        with self._lock:
            topic = self._stream_topic.pop(stream_id, None)
            if topic is None:
                return False
            sub = self._by_topic.get(topic)
            if sub is None:
                return True
            remaining = sub.release_ref()
            if remaining <= 0:
                self._by_topic.pop(topic, None)
                sub.destroy()
                self.node.get_logger().info(
                    f"[StreamingSvc] destroyed subscriber for {topic} (refs=0)"
                )
        return True

    def SubscribeImage(self, request, context):
        """ROS2 토픽을 구독하고 JPEG 프레임을 server-streaming으로 전송."""
        topic = request.topic
        msg_type = request.msg_type or 'sensor_msgs/CompressedImage'
        stream_id = request.stream_id or str(uuid.uuid4())

        sub = self._acquire(topic, msg_type)
        with self._lock:
            self._stream_topic[stream_id] = topic

        self.node.get_logger().info(
            f"[StreamingSvc] Subscribed: stream={stream_id} → {topic} (shared refs={sub._refs})"
        )

        try:
            while context.is_active():
                frame_data = sub.get_frame()
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
            self._release_stream(stream_id)
            self.node.get_logger().info(f"[StreamingSvc] stream {stream_id} ended")

    def UnsubscribeImage(self, request, context):
        stream_id = request.stream_id
        released = self._release_stream(stream_id)
        if released:
            return pb.StatusResponse(success=True, message=f"Unsubscribed: {stream_id}")
        return pb.StatusResponse(success=False, message=f"Not found: {stream_id}")

    def UpdateConfig(self, request, context):
        # TODO: crop/resize/rotate 설정 업데이트
        return pb.StatusResponse(success=True, message="OK")
