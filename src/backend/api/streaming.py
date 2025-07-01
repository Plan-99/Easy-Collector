import asyncio
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaBlackhole
from aiohttp import web
import aiohttp_cors
import sys
import os
import traceback
from av import VideoFrame


# --- 전역 변수 ---
pcs = set()

# --- ROSImageStreamTrack 클래스 (변경 없음) ---
class ROSImageStreamTrack(VideoStreamTrack):
    """
    ROS CompressedImage 토픽을 구독하여 비디오 프레임을 제공하는 클래스.
    """
    def __init__(self, initial_topic: str, pub_term: int=1):
        super().__init__()
        self.topic = initial_topic
        self.subscriber = None
        # self.frame_queue = asyncio.Queue(maxsize=2)
        self.start_subscriber()
        self.pub_term = pub_term
        self.cnt = 0
        self.frame = None

    def start_subscriber(self):
        if self.subscriber:
            self.subscriber.unregister()
        self.subscriber = rospy.Subscriber(
            self.topic, CompressedImage, self.ros_callback, queue_size=1
        )
        rospy.loginfo(f"Subscribed to ROS topic: {self.topic}")

    def ros_callback(self, msg: CompressedImage):
        # self.cnt += 1
        # if self.cnt % self.pub_term != 0:
        #     return 
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logwarn(f"Failed to decode image from topic {self.topic}")
                return

            self.frame = VideoFrame.from_ndarray(cv_image, format="bgr24")
            
            # if self.frame_queue.full():
            #     self.frame_queue.get_nowait()
            # self.frame_queue.put_nowait(frame)
        except Exception as e:
            rospy.logerr(f"Error in ros_callback for topic {self.topic}: {e}\n{traceback.format_exc()}")

    async def recv(self):
        # frame = await self.frame_queue.get()
        frame = self.frame
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def stop(self):
        if self.subscriber:
            self.subscriber.unregister()
            rospy.loginfo(f"Unsubscribed from ROS topic: {self.topic}")
        super().stop()


async def offer(request):
    params = await request.json()
    sensor = params.get('sensor')
    
    # STUN 서버 추가

    ice_servers = [RTCIceServer(urls="stun:stun.l.google.com:19302")]
    rtc_config = RTCConfiguration(iceServers=ice_servers)
    pc = RTCPeerConnection(rtc_config)

    pcs.add(pc)

    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        print("New ICE Candidate:", candidate)

    try:
        await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))
        print("Remote description set successfully")
    except Exception as e:
        print("Error setting remote description:", e, traceback.format_exc())
        return web.json_response({"error": "Failed to set remote description"}, status=500)

    try:
        # 🔹 두 개의 비디오 트랙 추가
        video_track = ROSImageStreamTrack(initial_topic=sensor['topic'])

        pc.addTrack(video_track)

        answer = await pc.createAnswer()
    except Exception as e:
        print("Error creating answer:", e, traceback.format_exc())
        return web.json_response({"error": "Failed to create answer"}, status=500)

    await asyncio.sleep(1)  # 🔹 ICE Candidate가 모일 때까지 기다림

    try:

        await pc.setLocalDescription(answer)
        if pc.localDescription is None:
            print("Error: `pc.localDescription` is None!")
            return web.json_response({"error": "Local Description is None"}, status=500)
        print("Local description set successfully")
    except Exception as e:
        print("Error setting local description:", e, traceback.format_exc())
        return web.json_response({"error": "Failed to set local description"}, status=500)

    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})


async def cleanup():
    while True:
        await asyncio.sleep(10)
        for pc in pcs:
            if pc.connectionState == "closed":
                pcs.discard(pc)

if __name__ == "__main__":
    rospy.init_node("webrtc_ros_stream")

    app = web.Application()

    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*"
        )
    })
    route = app.router.add_post("/offer", offer)
    cors.add(route)

    loop = asyncio.get_event_loop()
    loop.create_task(cleanup())

    web.run_app(app, port=5001)
