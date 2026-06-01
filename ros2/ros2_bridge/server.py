# -*- coding: utf-8 -*-
"""
EasyTrainer ROS2 Bridge gRPC Server.
ROS2 컨테이너의 메인 진입점.
"""
import signal
import sys
import threading
from concurrent import futures

import grpc
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .generated import robot_bridge_pb2_grpc as pb_grpc
from .services.agent_service import AgentServiceServicer
from .services.driver_service import DriverServiceServicer
from .services.ros_proxy_service import ROSProxyServicer
from .services.vive_service import ViveServiceServicer
from .services.uncertainty_service import UncertaintyServiceServicer
from .services.streaming_service import StreamingServiceServicer
from .services.obs_service import ObsServiceServicer
from .services.env_service import EnvServiceServicer

GRPC_PORT = 50051


def serve():
    # ROS2 초기화
    rclpy.init(args=None)
    node = Node('easytrainer_ros2_bridge')
    print(f"[ROS2 Bridge] ROS2 node '{node.get_name()}' initialized")

    # ROS2 executor (별도 스레드에서 spin).
    # NOTE: raw `executor.spin` propagates `rclpy.InvalidHandle` when a node/subscription
    # is destroyed mid-iteration (race between ObsService.StopSensors → destroy_node and
    # the executor still holding a reference to a callback waiter). That kills the spin
    # thread → ALL ROS callbacks stop (StreamingService/AgentService/etc.) → image
    # streaming dies after the first sensor session ends.
    # Wrap spin in a loop that catches InvalidHandle and keeps spinning.
    ros_executor = MultiThreadedExecutor()
    ros_executor.add_node(node)

    def _robust_spin():
        from rclpy._rclpy_pybind11 import InvalidHandle
        while rclpy.ok():
            try:
                ros_executor.spin_once(timeout_sec=0.1)
            except InvalidHandle:
                # Race with node/subscription destruction — recoverable.
                # Next spin_once iteration won't see the dead handle anymore.
                continue
            except Exception as e:
                # Don't kill the streaming service for unrelated transient errors.
                print(f"[ROS2 Bridge] spin warning: {type(e).__name__}: {e}", flush=True)
                continue

    spin_thread = threading.Thread(target=_robust_spin, daemon=True)
    spin_thread.start()

    # gRPC 서비스 인스턴스 생성
    agent_servicer = AgentServiceServicer(node)
    driver_servicer = DriverServiceServicer(node)
    ros_proxy_servicer = ROSProxyServicer(node)
    vive_servicer = ViveServiceServicer(node, agent_servicer)
    uncertainty_servicer = UncertaintyServiceServicer(node)
    streaming_servicer = StreamingServiceServicer(node)
    obs_servicer = ObsServiceServicer(node)
    env_servicer = EnvServiceServicer(node, agent_servicer)
    # ObsService 가 sensor 토픽을 shm 으로 쏟아내고, EnvService 의
    # WaitForImages 가 같은 ImageBridgeWriter 로 frame 도착 여부를 검사한다.
    env_servicer.set_image_bridge(obs_servicer.image_bridge)

    # gRPC 서버 설정.
    #
    # max_workers 가 너무 작으면 streaming RPC (``SubscribeImage`` 매 viewport,
    # ``SubscribeRobotState`` 매 로봇, ``WatchTopics`` 등) 가 worker thread 를
    # 영구 점유하면서 ``ListProcesses`` / ``ListTopics`` 같은 fast unary 쿼리가
    # 큐에서 대기 → 라우트 30s 행. multi-view planner / curriculum monitor
    # 사용 시 viewport 가 10+ 으로 쉽게 늘어나므로 넉넉히 잡아둔다. 메모리
    # 부담은 거의 없고 (idle thread 는 1MiB 스택 정도) thread 누수도 daemon
    # 으로 종료 시 정리됨.
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=128),
        options=[
            ('grpc.max_send_message_length', 100 * 1024 * 1024),  # 100MB
            ('grpc.max_receive_message_length', 100 * 1024 * 1024),
        ],
    )
    pb_grpc.add_AgentServiceServicer_to_server(agent_servicer, server)
    pb_grpc.add_DriverServiceServicer_to_server(driver_servicer, server)
    pb_grpc.add_ROSProxyServicer_to_server(ros_proxy_servicer, server)
    pb_grpc.add_ViveServiceServicer_to_server(vive_servicer, server)
    pb_grpc.add_UncertaintyServiceServicer_to_server(uncertainty_servicer, server)
    pb_grpc.add_StreamingServiceServicer_to_server(streaming_servicer, server)
    pb_grpc.add_ObsServiceServicer_to_server(obs_servicer, server)
    pb_grpc.add_EnvServiceServicer_to_server(env_servicer, server)

    server.add_insecure_port(f'[::]:{GRPC_PORT}')
    server.start()
    print(f"[ROS2 Bridge] gRPC server listening on port {GRPC_PORT}")

    # Graceful shutdown
    shutdown_event = threading.Event()

    def _shutdown(signum, frame):
        print(f"\n[ROS2 Bridge] Received signal {signum}, shutting down...")
        shutdown_event.set()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    shutdown_event.wait()

    # Cleanup
    print("[ROS2 Bridge] Stopping services...")
    driver_servicer.stop_all()
    vive_servicer.destroy()
    uncertainty_servicer.destroy()
    agent_servicer.destroy_all()
    obs_servicer.destroy_all()

    server.stop(grace=5)
    ros_executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print("[ROS2 Bridge] Shutdown complete.")


if __name__ == '__main__':
    serve()
