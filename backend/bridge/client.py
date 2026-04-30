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
        self._channel = None
        self._agent_stub = None
        self._driver_stub = None
        self._env_stub = None
        self._ros_proxy_stub = None
        self._vive_stub = None
        self._uncertainty_stub = None
        self._streaming_stub = None
        self._obs_stub = None
        self._connect()

    def _connect(self):
        self._channel = grpc.insecure_channel(
            self._target,
            options=[
                ('grpc.max_send_message_length', 100 * 1024 * 1024),
                ('grpc.max_receive_message_length', 100 * 1024 * 1024),
                ('grpc.keepalive_time_ms', 10000),
                ('grpc.keepalive_timeout_ms', 5000),
            ],
        )
        self._agent_stub = pb_grpc.AgentServiceStub(self._channel)
        self._driver_stub = pb_grpc.DriverServiceStub(self._channel)
        self._env_stub = pb_grpc.EnvServiceStub(self._channel)
        self._ros_proxy_stub = pb_grpc.ROSProxyStub(self._channel)
        self._vive_stub = pb_grpc.ViveServiceStub(self._channel)
        self._uncertainty_stub = pb_grpc.UncertaintyServiceStub(self._channel)
        self._streaming_stub = pb_grpc.StreamingServiceStub(self._channel)
        self._obs_stub = pb_grpc.ObsServiceStub(self._channel)

    @property
    def agent(self):
        return self._agent_stub

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


def get_bridge_client(host=GRPC_HOST, port=GRPC_PORT):
    """BridgeClient 싱글톤 인스턴스 반환."""
    global _instance
    if _instance is None:
        with _lock:
            if _instance is None:
                _instance = BridgeClient(host, port)
    return _instance
