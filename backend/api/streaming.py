# -*- coding: utf-8 -*-
"""
WebRTC Streaming Server (backend 컨테이너, port 5002).
ros2 컨테이너에서 gRPC server-streaming으로 수신한 이미지 프레임을 WebRTC로 클라이언트에 전달.
시뮬레이션 프레임도 동일한 경로로 처리.
"""
import asyncio
import cv2
import numpy as np
import threading
import traceback
import uuid
import time

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration
from aiohttp import web
import aiohttp_cors
from av import VideoFrame

# Single source of truth for the per-sensor crop/rotate/resize pipeline.
# Used by inference, dataset replay, failure detection, and now the live
# WebRTC preview here — having one canonical implementation means a fix to
# the order/semantics only ever needs to land in one place.
from ..utils.image_parser import fetch_image_with_config

# --- 전역 변수 ---
pcs = set()
stream_config = {}
stream_tracks = {}  # stream_id → GRPCImageStreamTrack (config 업데이트 라우팅용)


class _TopicFrameSource:
    """topic 별 단일 SubscribeImage 호출 + 디코딩된 cv_image 캐시.

    같은 topic 의 여러 ``GRPCImageStreamTrack`` 이 한 source 를 공유한다 →
    ros2 gRPC worker 점유, JPEG decode, 네트워크 트래픽 모두 N → 1 로 축소.
    여러 view 가 카메라 한 대를 분할하는 multi-view 환경 (Planner / Curriculum
    Monitor) 에서 thread pool 포화 + CPU 부하의 주범이었음.

    Per-view crop/resize/rotate 는 각 track 의 ``recv()`` 에서 적용 (decode 후).
    """

    _instances_by_topic: dict[str, '_TopicFrameSource'] = {}
    _registry_lock = threading.Lock()

    def __init__(self, grpc_stub, topic: str, msg_type: str):
        self.topic = topic
        self.msg_type = msg_type
        self._grpc_stub = grpc_stub
        self._cv_image = None        # latest decoded frame (BGR)
        self._frame_seq = 0          # 마지막 frame 의 단조 시퀀스 (track 측 cache 비교용)
        self._refs = 0
        self._running = True
        # 활성 SubscribeImage 호출 핸들. release() 가 능동 .cancel() 하기 위해 보관.
        # 이게 없으면 topic 이 프레임 발행을 멈췄을 때 _loop 의 for 문이 다음
        # 프레임을 영영 기다리며 break 하지 못해 gRPC stream 이 누수된다.
        self._call = None
        # gRPC SubscribeImage 의 stream_id 는 **이 source 인스턴스마다 고유**해야 한다.
        # ros2 StreamingService 는 stream_id 를 키로 구독 lifecycle(refcount)을 관리하는데,
        # 고정 'src-<topic>' 를 쓰면 view churn 으로 옛 stream 종료와 새 stream 시작이
        # 겹칠 때 같은 키가 충돌 → 해제가 한 번만 먹혀 refcount 가 누수된다. 그러면
        # ros2 쪽 _SharedTopicSubscriber 가 영영 파괴/재생성되지 않아, sim 재시작 등으로
        # publisher 가 바뀌면 stale 구독이 재매칭하지 못한 채 0 프레임만 내보낸다.
        self._stream_id = f'src-{topic}-{uuid.uuid4().hex[:8]}'
        self._thread = threading.Thread(target=self._loop, name=f'src-{topic}', daemon=True)
        self._thread.start()

    def _loop(self):
        from ..bridge.generated import robot_bridge_pb2 as pb
        try:
            request = pb.SubscribeImageRequest(
                topic=self.topic,
                msg_type=self.msg_type,
                stream_id=self._stream_id,
            )
            self._call = self._grpc_stub.SubscribeImage(request)
            for image_frame in self._call:
                if not self._running:
                    break
                try:
                    np_arr = np.frombuffer(image_frame.jpeg_data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if cv_image is None:
                        continue
                    self._cv_image = cv_image
                    self._frame_seq += 1
                except Exception as e:
                    print(f"[GRPCStream:src] {self.topic} decode error: {e}")
        except Exception as e:
            if self._running:
                print(f"[GRPCStream:src] {self.topic} stream error: {e}")

    def get_frame(self):
        """Return (cv_image, seq) or (None, 0). track 이 polling 으로 호출."""
        return self._cv_image, self._frame_seq

    @classmethod
    def acquire(cls, grpc_stub, topic: str, msg_type: str) -> '_TopicFrameSource':
        with cls._registry_lock:
            src = cls._instances_by_topic.get(topic)
            if src is None:
                src = cls(grpc_stub, topic, msg_type)
                cls._instances_by_topic[topic] = src
                print(f"[GRPCStream:src] created for {topic}")
            src._refs += 1
            return src

    @classmethod
    def release(cls, src: '_TopicFrameSource'):
        if src is None:
            return
        with cls._registry_lock:
            src._refs -= 1
            if src._refs <= 0:
                cls._instances_by_topic.pop(src.topic, None)
                src._running = False
                # 능동 취소 — topic 이 프레임을 안 보내도 _loop 가 즉시 빠져나오게
                # 해서 gRPC server-stream(HTTP/2 stream)을 확실히 닫는다.
                call = src._call
                if call is not None:
                    try:
                        call.cancel()
                    except Exception:
                        pass
                print(f"[GRPCStream:src] released last ref for {src.topic}")


class GRPCImageStreamTrack(VideoStreamTrack):
    """
    gRPC server-streaming으로 수신한 JPEG 프레임을 WebRTC로 전달하는 트랙.
    센서 카메라와 시뮬레이션 모두 이 트랙을 사용.

    SubscribeImage 호출은 ``_TopicFrameSource`` 가 topic 별로 공유한다 — N
    개의 viewport 가 같은 토픽을 봐도 ros2 gRPC worker 한 개만 점유.
    """
    def __init__(self, grpc_stub, topic: str, msg_type: str, stream_id: str = None):
        super().__init__()
        self.stream_id = stream_id or str(uuid.uuid4())
        self._running = True
        self._config = {}  # crop, resize, rotate
        self._source = _TopicFrameSource.acquire(grpc_stub, topic, msg_type)
        self._last_seq = -1

    def _apply_config(self, cv_image):
        """Delegate to the canonical per-sensor transform pipeline.

        Per-view 의 crop/resize/rotate 는 같은 source 의 같은 frame 에 각자
        독립적으로 적용된다 — 동일 topic 의 viewport 들이 서로 다른 crop 을
        써도 source 디코딩은 한 번만.
        """
        try:
            out = fetch_image_with_config(cv_image, self._config or {})
            if out is None or getattr(out, 'size', 0) == 0:
                return cv_image
            return out
        except Exception as e:
            print(f"[GRPCStream] {self.stream_id} _apply_config error: {e}")
            return cv_image

    def update_config(self, config):
        self._config.update(config)

    async def recv(self):
        # source 의 latest frame 을 polling. 새 frame 이 생길 때까지 짧게 대기.
        while True:
            if not self._running:
                # PC 가 stop 한 경우 — 빈 frame 보내지 말고 그냥 yield 멈춤.
                # aiortc 가 next_timestamp 호출 후 frame 을 요구하므로 종료
                # 시그널을 명시적으로 raise.
                raise ConnectionError('track stopped')
            cv_image, seq = self._source.get_frame()
            if cv_image is not None and seq != self._last_seq:
                self._last_seq = seq
                break
            await asyncio.sleep(0.01)
        cv_image = self._apply_config(cv_image)
        frame = VideoFrame.from_ndarray(cv_image, format="bgr24")
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def stop(self):
        self._running = False
        _TopicFrameSource.release(self._source)
        self._source = None
        super().stop()


class SimImageStreamTrack(VideoStreamTrack):
    """
    PyBullet 시뮬레이션 프레임을 WebRTC로 스트리밍하는 트랙.
    SimEngine의 render_frame()에서 프레임을 가져옴.
    """
    def __init__(self, sim_engine, stream_id: str = None, fps: int = 30):
        super().__init__()
        self.sim_engine = sim_engine
        self.stream_id = stream_id or str(uuid.uuid4())
        self.frame = None
        self._fps = fps
        self._running = True
        self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
        self._render_thread.start()

    def _render_loop(self):
        interval = 1.0 / self._fps
        while self._running:
            try:
                bgr = self.sim_engine.render_frame()
                if bgr is not None:
                    self.frame = VideoFrame.from_ndarray(bgr, format="bgr24")
            except Exception:
                pass
            time.sleep(interval)

    async def recv(self):
        while self.frame is None:
            await asyncio.sleep(0.01)
        frame = self.frame
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def stop(self):
        self._running = False
        super().stop()


# --- HTTP Handlers ---

async def offer(request):
    """센서 카메라용 WebRTC offer — gRPC로 ros2에서 프레임 수신."""
    params = await request.json()
    topic = params.get('topic')
    config = params.get('config', {})
    msg_type = params.get('msg_type', 'sensor_msgs/CompressedImage')

    grpc_stub = request.app.get('grpc_streaming_stub')
    if grpc_stub is None:
        return web.json_response({"error": "gRPC streaming not available"}, status=500)

    # localhost 연결 — 외부 STUN 불필요. host candidate 만 gather 하면 IPv6 STUN
    # ~5s 타임아웃 지연 없이 즉시 연결된다 (프론트 useWebRTC.js 와 동일 정책).
    pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    pcs.add(pc)

    stream_id = str(uuid.uuid4())

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            pcs.discard(pc)
            stream_config.pop(stream_id, None)
            stream_tracks.pop(stream_id, None)

    try:
        await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))

        stream_config[stream_id] = config
        video_track = GRPCImageStreamTrack(
            grpc_stub=grpc_stub,
            topic=topic,
            msg_type=msg_type,
            stream_id=stream_id,
        )
        stream_tracks[stream_id] = video_track
        if config:
            video_track.update_config(config)
        pc.addTrack(video_track)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
            "stream_id": stream_id,
        })
    except Exception as e:
        print(f"Error in offer: {e}\n{traceback.format_exc()}")
        return web.json_response({"error": str(e)}, status=500)


async def sim_offer(request):
    """시뮬레이션용 WebRTC offer — backend의 SimEngine에서 프레임 수신."""
    from .routes.sim import get_sim_engine
    sim_engine = get_sim_engine()
    if not sim_engine or not sim_engine.is_running:
        return web.json_response({"error": "Simulation not running"}, status=400)

    params = await request.json()
    # localhost 연결 — 외부 STUN 불필요 (offer() 와 동일 정책).
    pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            pcs.discard(pc)

    try:
        await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))

        stream_id = str(uuid.uuid4())
        fps = params.get('fps', 30)
        video_track = SimImageStreamTrack(sim_engine=sim_engine, stream_id=stream_id, fps=fps)
        pc.addTrack(video_track)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
            "stream_id": stream_id,
        })
    except Exception as e:
        print(f"Error in sim_offer: {e}\n{traceback.format_exc()}")
        return web.json_response({"error": str(e)}, status=500)


async def add_config(request):
    """스트림 설정 업데이트 (crop, resize, rotate)."""
    params = await request.json()
    stream_id = params.get('stream_id')
    config = params.get('config', {})
    if stream_id in stream_config:
        stream_config[stream_id].update(config)
    track = stream_tracks.get(stream_id)
    if track is not None:
        track.update_config(config)
        print(f"[Streaming] add_config stream_id={stream_id} config={config} merged={track._config}")
    else:
        print(f"[Streaming] add_config stream_id={stream_id} NO TRACK (known_ids={list(stream_tracks.keys())})")
    return web.json_response({"status": "success"})


# --- App Factory ---

def create_aiohttp_app(grpc_streaming_stub=None, sim_engine=None):
    app = web.Application()
    app['grpc_streaming_stub'] = grpc_streaming_stub
    app['sim_engine'] = sim_engine

    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True, expose_headers="*", allow_headers="*"
        )
    })

    offer_route = app.router.add_post("/offer", offer)
    sim_offer_route = app.router.add_post("/sim/offer", sim_offer)
    add_config_route = app.router.add_post("/add_config", add_config)
    cors.add(offer_route)
    cors.add(sim_offer_route)
    cors.add(add_config_route)

    return app


def start_streaming_server(grpc_streaming_stub=None, sim_engine=None):
    """Start the WebRTC streaming server on port 5002 (in a non-main thread)."""
    app = create_aiohttp_app(grpc_streaming_stub, sim_engine)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, '0.0.0.0', 5002)
    loop.run_until_complete(site.start())
    print("[Streaming] WebRTC server listening on port 5002")
    loop.run_forever()
