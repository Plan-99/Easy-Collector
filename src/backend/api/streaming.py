# -*- coding: utf-8 -*-
import asyncio
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
from aiohttp import web
import aiohttp_cors
import traceback
from av import VideoFrame
import uuid
import threading #  threading 모듈 추가

# --- 전역 변수 ---
pcs = set()
stream_config = {}

# --- 1. ROSImageStreamTrack 클래스 수정 ---
# Node를 상속하지 않고, 생성자에서 node 객체를 전달받습니다.
class ROSImageStreamTrack(VideoStreamTrack):
    """
    공유된 ROS 노드를 사용하여 CompressedImage 또는 Image 토픽을 구독하는 클래스.
    """
    def __init__(self, node: Node, initial_topic: str, stream_id: str = None, msg_type: str = 'sensor_msgs/CompressedImage'):
        super().__init__()
        self.node = node
        self.topic = initial_topic
        self.subscriber = None
        self.frame = None
        self.stream_id = stream_id
        self.is_compressed = msg_type != 'sensor_msgs/Image'
        self.start_subscriber()

    def start_subscriber(self):
        if self.subscriber:
            self.node.destroy_subscription(self.subscriber)
        ros_msg_type = CompressedImage if self.is_compressed else Image
        self.subscriber = self.node.create_subscription(
            ros_msg_type, self.topic, self.ros_callback, 1
        )
        self.node.get_logger().info(f"Stream {self.stream_id}: Subscribed to ROS topic: {self.topic} ({'CompressedImage' if self.is_compressed else 'Image'})")

    def ros_callback(self, msg):
        try:
            if self.is_compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # sensor_msgs/Image
                h, w = msg.height, msg.width
                encoding = msg.encoding
                np_arr = np.frombuffer(msg.data, np.uint8)
                if encoding in ('rgb8', 'bgr8'):
                    cv_image = np_arr.reshape((h, w, 3))
                    if encoding == 'rgb8':
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                elif encoding == 'rgba8':
                    cv_image = np_arr.reshape((h, w, 4))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
                elif encoding == 'bgra8':
                    cv_image = np_arr.reshape((h, w, 4))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
                elif encoding == 'mono8':
                    cv_image = np_arr.reshape((h, w))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                elif encoding == 'mono16':
                    np_arr = np.frombuffer(msg.data, np.uint16).reshape((h, w))
                    cv_image = (np_arr / 256).astype(np.uint8)
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                else:
                    # fallback: try as bgr8
                    cv_image = np_arr.reshape((h, w, 3))
            
            # 1. 디코딩 실패 확인
            if cv_image is None or cv_image.size == 0:
                return

            h, w = cv_image.shape[:2] # 원본 해상도 확인
            config = stream_config.get(self.stream_id, {})
            
            # 2. Crop 안전 장치
            if config.get('cropped_area'):
                area = config['cropped_area'] # [x1, y1, x2, y2]
                
                # 좌표 정규화 및 이미지 경계 제한
                x1 = max(0, min(area[0], w - 1))
                y1 = max(0, min(area[1], h - 1))
                x2 = max(x1 + 1, min(area[2], w))
                y2 = max(y1 + 1, min(area[3], h))
                
                cv_image = cv_image[y1:y2, x1:x2]

            # 3. 슬라이싱 후 이미지가 비었는지 최종 확인
            if cv_image.size == 0:
                self.node.get_logger().warn(f"Empty image after cropping! Area: {config.get('cropped_area')}")
                return

            # 4. Rotate
            angle = config.get('rotate', 0)
            if angle in [90, 180, 270]:
                rotation_map = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
                cv_image = cv2.rotate(cv_image, rotation_map[angle])

            # 5. Resize (이제 안전함)
            if config.get('resize'):
                target_size = config['resize']
                cv_image = cv2.resize(cv_image, (target_size[0], target_size[1]))
            
                
            self.frame = VideoFrame.from_ndarray(cv_image, format="bgr24")
        
        except Exception as e:
            self.node.get_logger().error(f"Error in ros_callback: {e}\n{traceback.format_exc()}")


    async def recv(self):
        # 최신 프레임이 있을 때까지 잠시 대기 (비동기)
        while self.frame is None:
            await asyncio.sleep(0.01)
            
        frame = self.frame
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def stop(self):
        if self.subscriber:
            self.node.destroy_subscription(self.subscriber)
            self.node.get_logger().info(f"Stream {self.stream_id}: Unsubscribed from ROS topic: {self.topic}")
        super().stop()


async def offer(request):
    # aiohttp 앱 객체에서 ROS 노드를 가져옵니다.
    node = request.app['ros_node']
    
    params = await request.json()
    topic = params.get('topic')
    config = params.get('config', {})
    msg_type = params.get('msg_type', 'sensor_msgs/CompressedImage')
    ice_servers = [RTCIceServer(urls="stun:stun.l.google.com:19302")]
    pc = RTCPeerConnection(RTCConfiguration(iceServers=ice_servers))
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state is {pc.connectionState}")
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            pcs.discard(pc)
            # stream_config에서도 정리
            for track in pc.getTransceivers():
                if hasattr(track.receiver, 'track') and hasattr(track.receiver.track, 'stream_id'):
                    stream_config.pop(track.receiver.track.stream_id, None)

    try:
        await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))

        stream_id = str(uuid.uuid4())
        stream_config[stream_id] = config

        # 2. ROSImageStreamTrack 생성 시 공유 노드 객체 전달
        video_track = ROSImageStreamTrack(node=node, initial_topic=topic, stream_id=stream_id, msg_type=msg_type)
        pc.addTrack(video_track)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type, 'stream_id': stream_id})
    except Exception as e:
        print(f"Error in offer: {e}\n{traceback.format_exc()}")
        return web.json_response({"error": str(e)}, status=500)

async def add_config(request):
    params = await request.json()
    stream_id = params.get('stream_id')
    config = params.get('config', {})
    if stream_id in stream_config and 'resize' in config:
        stream_config[stream_id]['resize'] = config['resize']
    if stream_id in stream_config and 'cropped_area' in config:
        stream_config[stream_id]['cropped_area'] = config['cropped_area']
    if stream_id in stream_config and 'rotate' in config:
        stream_config[stream_id]['rotate'] = config['rotate']

    return web.json_response({"status": "success"})


# 3. ROS 노드를 관리하는 함수들
def ros_main(args=None):
    rclpy.init(args=args)
    ros_node = Node("webrtc_ros_bridge_node")
    
    # asyncio 루프에 노드를 전달하기 위한 future
    future = asyncio.Future()
    app = create_aiohttp_app(ros_node, future)

    # ROS 노드 스핀을 위한 별도 스레드 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    # aiohttp 앱 실행
    web.run_app(app, port=5002)
    
    # 앱 종료 시 정리
    ros_node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

# aiohttp 앱을 생성하는 헬퍼 함수
def create_aiohttp_app(ros_node, future):
    app = web.Application()
    app['ros_node'] = ros_node # 앱 객체에 ROS 노드 저장
    
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True, expose_headers="*", allow_headers="*"
        )
    })
    
    offer_route = app.router.add_post("/offer", offer)
    add_config_route = app.router.add_post("/add_config", add_config)
    cors.add(offer_route)
    cors.add(add_config_route)
    
    return app

if __name__ == "__main__":
    ros_main()