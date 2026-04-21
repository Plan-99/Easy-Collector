# -*- coding: utf-8 -*-
"""
UncertaintyService gRPC 서비스 구현.
OTI-RL의 UncertaintySubscriber를 ROS2 컨테이너에서 관리한다.
"""
import threading

import grpc

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc


class UncertaintyServiceServicer(pb_grpc.UncertaintyServiceServicer):
    def __init__(self, node):
        self._node = node
        self._subscriber = None
        self._executor = None
        self._spin_thread = None

    def StartSubscriber(self, request, context):
        if self._subscriber is not None:
            return pb.StatusResponse(success=True, message='Already running')

        try:
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import Float64

            # 인라인 subscriber 생성 (oti_rl.py의 ROS2 의존성 제거를 위해)
            self._subscriber = self._node.create_subscription(
                Float64,
                '/failure_detection/uncertainty_score',
                self._callback,
                10
            )
            self._latest_score = 0.0
            return pb.StatusResponse(success=True, message='Subscriber started')
        except Exception as e:
            return pb.StatusResponse(success=False, message=str(e))

    def _callback(self, msg):
        self._latest_score = msg.data

    def GetLatestScore(self, request, context):
        score = getattr(self, '_latest_score', 0.0)
        return pb.UncertaintyScore(score=score)

    def SetScore(self, request, context):
        """메인 컨테이너의 failure_detection에서 직접 score를 설정."""
        self._latest_score = request.score
        return pb.Empty()

    def StopSubscriber(self, request, context):
        if self._subscriber is not None:
            self._node.destroy_subscription(self._subscriber)
            self._subscriber = None
            self._latest_score = 0.0
        return pb.StatusResponse(success=True, message='Subscriber stopped')

    def destroy(self):
        """서버 종료 시 호출."""
        if self._subscriber is not None:
            try:
                self._node.destroy_subscription(self._subscriber)
            except Exception:
                pass
            self._subscriber = None
