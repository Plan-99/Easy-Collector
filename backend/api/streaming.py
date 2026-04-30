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

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
from aiohttp import web
import aiohttp_cors
from av import VideoFrame

# --- 전역 변수 ---
pcs = set()
stream_config = {}


class GRPCImageStreamTrack(VideoStreamTrack):
    """
    gRPC server-streaming으로 수신한 JPEG 프레임을 WebRTC로 전달하는 트랙.
    센서 카메라와 시뮬레이션 모두 이 트랙을 사용.
    """
    def __init__(self, grpc_stub, topic: str, msg_type: str, stream_id: str = None):
        super().__init__()
        self.stream_id = stream_id or str(uuid.uuid4())
        self.frame = None
        self._running = True
        self._config = {}  # crop, resize, rotate

        # gRPC streaming을 백그라운드 스레드에서 실행
        self._thread = threading.Thread(
            target=self._grpc_stream_loop,
            args=(grpc_stub, topic, msg_type),
            daemon=True,
        )
        self._thread.start()

    def _grpc_stream_loop(self, stub, topic, msg_type):
        """gRPC SubscribeImage를 호출하고 프레임을 수신."""
        from ..bridge.generated import robot_bridge_pb2 as pb

        try:
            request = pb.SubscribeImageRequest(
                topic=topic,
                msg_type=msg_type,
                stream_id=self.stream_id,
            )
            for image_frame in stub.SubscribeImage(request):
                if not self._running:
                    break

                # JPEG → OpenCV BGR
                np_arr = np.frombuffer(image_frame.jpeg_data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is None:
                    continue

                # Apply config (crop, rotate, resize)
                cv_image = self._apply_config(cv_image)

                self.frame = VideoFrame.from_ndarray(cv_image, format="bgr24")
        except Exception as e:
            if self._running:
                print(f"[GRPCStream] {self.stream_id} error: {e}")

    def _apply_config(self, cv_image):
        """crop, rotate, resize 적용."""
        config = self._config
        h, w = cv_image.shape[:2]

        # Crop
        if config.get('cropped_area'):
            area = config['cropped_area']
            x1 = max(0, min(area[0], w - 1))
            y1 = max(0, min(area[1], h - 1))
            x2 = max(x1 + 1, min(area[2], w))
            y2 = max(y1 + 1, min(area[3], h))
            cv_image = cv_image[y1:y2, x1:x2]
            if cv_image.size == 0:
                return cv_image

        # Rotate
        angle = config.get('rotate', 0)
        if angle in [90, 180, 270]:
            rotation_map = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
            cv_image = cv2.rotate(cv_image, rotation_map[angle])

        # Resize
        if config.get('resize'):
            target_size = config['resize']
            cv_image = cv2.resize(cv_image, (target_size[0], target_size[1]))

        return cv_image

    def update_config(self, config):
        self._config.update(config)

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

    ice_servers = [RTCIceServer(urls="stun:stun.l.google.com:19302")]
    pc = RTCPeerConnection(RTCConfiguration(iceServers=ice_servers))
    pcs.add(pc)

    stream_id = str(uuid.uuid4())

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            pcs.discard(pc)
            stream_config.pop(stream_id, None)

    try:
        await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))

        stream_config[stream_id] = config
        video_track = GRPCImageStreamTrack(
            grpc_stub=grpc_stub,
            topic=topic,
            msg_type=msg_type,
            stream_id=stream_id,
        )
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
    ice_servers = [RTCIceServer(urls="stun:stun.l.google.com:19302")]
    pc = RTCPeerConnection(RTCConfiguration(iceServers=ice_servers))
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
