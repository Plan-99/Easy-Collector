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
from .services.env_service import EnvServiceServicer
from .services.ros_proxy_service import ROSProxyServicer
from .services.vive_service import ViveServiceServicer
from .services.uncertainty_service import UncertaintyServiceServicer

GRPC_PORT = 50051


def serve():
    # ROS2 초기화
    rclpy.init(args=None)
    node = Node('easytrainer_ros2_bridge')
    print(f"[ROS2 Bridge] ROS2 node '{node.get_name()}' initialized")

    # ROS2 executor (별도 스레드에서 spin)
    ros_executor = MultiThreadedExecutor()
    ros_executor.add_node(node)
    spin_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    spin_thread.start()

    # gRPC 서비스 인스턴스 생성
    agent_servicer = AgentServiceServicer(node)
    driver_servicer = DriverServiceServicer(node)
    env_servicer = EnvServiceServicer(node, agent_servicer)
    ros_proxy_servicer = ROSProxyServicer(node)
    vive_servicer = ViveServiceServicer(node, agent_servicer)
    uncertainty_servicer = UncertaintyServiceServicer(node)

    # gRPC 서버 설정
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=20),
        options=[
            ('grpc.max_send_message_length', 100 * 1024 * 1024),  # 100MB
            ('grpc.max_receive_message_length', 100 * 1024 * 1024),
        ],
    )
    pb_grpc.add_AgentServiceServicer_to_server(agent_servicer, server)
    pb_grpc.add_DriverServiceServicer_to_server(driver_servicer, server)
    pb_grpc.add_EnvServiceServicer_to_server(env_servicer, server)
    pb_grpc.add_ROSProxyServicer_to_server(ros_proxy_servicer, server)
    pb_grpc.add_ViveServiceServicer_to_server(vive_servicer, server)
    pb_grpc.add_UncertaintyServiceServicer_to_server(uncertainty_servicer, server)

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
    env_servicer.destroy_all()

    server.stop(grace=5)
    ros_executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print("[ROS2 Bridge] Shutdown complete.")


if __name__ == '__main__':
    serve()
