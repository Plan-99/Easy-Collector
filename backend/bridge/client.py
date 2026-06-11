# -*- coding: utf-8 -*-
"""
gRPC Bridge Client singleton.
메인 컨테이너에서 ROS2 컨테이너의 gRPC 서버에 연결한다.
"""
import grpc
import threading
import time

from .generated import robot_bridge_pb2_grpc as pb_grpc

GRPC_HOST = 'localhost'
GRPC_PORT = 50051

_instance = None
_lock = threading.Lock()


class BridgeClient:
    """ROS2 Bridge gRPC 클라이언트 (싱글톤)."""

    def __init__(self, host=GRPC_HOST, port=GRPC_PORT):
        self._target = f'{host}:{port}'
        # control-plane 호출(ListProcesses 등 unary)과 streaming(SubscribeImage
        # 등 장수명 server-stream)을 **별도 채널**로 분리한다. 같은 채널을 공유하면
        # 누수된 streaming stream 이 HTTP/2 동시 스트림 한도를 갉아먹어 control
        # unary 호출이 무한 대기(deadline 도 못 먹힘)하는 wedge 가 발생했었다.
        self._channel = None          # control-plane channel
        self._stream_channel = None   # streaming/obs channel
        self._agent_stub = None
        self._driver_stub = None
        self._env_stub = None
        self._ros_proxy_stub = None
        self._vive_stub = None
        self._uncertainty_stub = None
        self._streaming_stub = None
        self._obs_stub = None
        self._connect()

    def _channel_options(self):
        return [
            ('grpc.max_send_message_length', 100 * 1024 * 1024),
            ('grpc.max_receive_message_length', 100 * 1024 * 1024),
            ('grpc.keepalive_time_ms', 10000),
            ('grpc.keepalive_timeout_ms', 5000),
        ]

    def _connect(self):
        # control-plane channel: unary/짧은 호출 전용 (agent/driver/env/ros_proxy/
        # vive/uncertainty). streaming stream 과 격리되어 절대 stream-exhaustion 으로
        # wedge 되지 않는다.
        self._channel = grpc.insecure_channel(self._target, options=self._channel_options())
        # streaming channel: SubscribeImage / Obs 등 장수명 server-stream 전용.
        self._stream_channel = grpc.insecure_channel(self._target, options=self._channel_options())

        self._agent_stub = pb_grpc.AgentServiceStub(self._channel)
        self._driver_stub = pb_grpc.DriverServiceStub(self._channel)
        self._env_stub = pb_grpc.EnvServiceStub(self._channel)
        self._ros_proxy_stub = pb_grpc.ROSProxyStub(self._channel)
        self._vive_stub = pb_grpc.ViveServiceStub(self._channel)
        self._uncertainty_stub = pb_grpc.UncertaintyServiceStub(self._channel)
        self._streaming_stub = pb_grpc.StreamingServiceStub(self._stream_channel)
        self._obs_stub = pb_grpc.ObsServiceStub(self._stream_channel)
        # AgentService 의 unary 호출(GetJointStates/Move*)은 control 채널을 쓰지만,
        # SubscribeRobotState(장수명 server-stream)는 **stream 채널** 전용 스텁으로
        # 분리한다. 안 그러면 누적된 robot-state 스트림이 control 채널의 HTTP/2
        # 동시 스트림 슬롯을 소진해 ListProcesses 같은 control unary 가 wedge 된다
        # (SubscribeImage 와 동일한 실패 모드).
        self._agent_stream_stub = pb_grpc.AgentServiceStub(self._stream_channel)

    @property
    def agent(self):
        return self._agent_stub

    @property
    def agent_stream(self):
        """SubscribeRobotState 등 AgentService 의 server-streaming 전용 (stream 채널)."""
        return self._agent_stream_stub

    @property
    def driver(self):
        return self._driver_stub

    @property
    def env(self):
        return self._env_stub

    @property
    def ros_proxy(self):
        return self._ros_proxy_stub

    @property
    def vive(self):
        return self._vive_stub

    @property
    def uncertainty(self):
        return self._uncertainty_stub

    @property
    def streaming(self):
        return self._streaming_stub

    @property
    def obs(self):
        return self._obs_stub

    def wait_for_ready(self, timeout=60):
        """gRPC 서버가 준비될 때까지 대기."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                grpc.channel_ready_future(self._channel).result(timeout=2)
                print(f"[BridgeClient] Connected to ROS2 bridge at {self._target}")
                return True
            except grpc.FutureTimeoutError:
                print(f"[BridgeClient] Waiting for ROS2 bridge at {self._target}...")
                time.sleep(1)
        print(f"[BridgeClient] Failed to connect to ROS2 bridge after {timeout}s")
        return False

    def is_ready(self):
        """gRPC 채널 상태 확인."""
        try:
            grpc.channel_ready_future(self._channel).result(timeout=1)
            return True
        except Exception:
            return False

    def close(self):
        if self._channel:
            self._channel.close()
            self._channel = None
        if self._stream_channel:
            self._stream_channel.close()
            self._stream_channel = None


def get_bridge_client(host=GRPC_HOST, port=GRPC_PORT):
    """BridgeClient 싱글톤 인스턴스 반환."""
    global _instance
    if _instance is None:
        with _lock:
            if _instance is None:
                _instance = BridgeClient(host, port)
    return _instance
