# -*- coding: utf-8 -*-
"""
ROSProxy gRPC 서비스 구현.
ROS2 서비스를 동적으로 호출하는 프록시.
"""
import json
import importlib
import threading
import time

import grpc

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc


class ROSProxyServicer(pb_grpc.ROSProxyServicer):
    def __init__(self, node):
        self._node = node
        self._clients = {}  # cache: (service_name, service_type) -> client

    def _get_service_class(self, service_type: str):
        """
        ROS2 서비스 타입 문자열에서 Python 클래스를 동적으로 import.
        e.g. "std_srvs/srv/Trigger" -> std_srvs.srv.Trigger
        """
        parts = service_type.split('/')
        if len(parts) != 3:
            raise ValueError(f"Invalid service type format: {service_type}. Expected 'pkg/srv/Type'")
        pkg, _, cls_name = parts
        module = importlib.import_module(f'{pkg}.srv')
        return getattr(module, cls_name)

    def CallService(self, request, context):
        service_name = request.service_name
        service_type = request.service_type
        request_json = request.request_json

        try:
            srv_class = self._get_service_class(service_type)
        except Exception as e:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details(f"Failed to load service type '{service_type}': {e}")
            return pb.ROSServiceResponse(success=False, response_json=json.dumps({'error': str(e)}))

        # 클라이언트 캐시
        cache_key = (service_name, service_type)
        if cache_key not in self._clients:
            self._clients[cache_key] = self._node.create_client(srv_class, service_name)

        client = self._clients[cache_key]

        # 서비스 대기 (최대 10초)
        if not client.wait_for_service(timeout_sec=10.0):
            return pb.ROSServiceResponse(
                success=False,
                response_json=json.dumps({'error': f'Service {service_name} not available after 10s'})
            )

        # 요청 생성
        req = srv_class.Request()
        if request_json:
            req_data = json.loads(request_json)
            for key, value in req_data.items():
                if hasattr(req, key):
                    setattr(req, key, value)

        # 동기 호출
        future = client.call_async(req)

        # 최대 60초 대기
        import time
        timeout = 60.0
        elapsed = 0.0
        while not future.done() and elapsed < timeout:
            time.sleep(0.1)
            elapsed += 0.1

        if not future.done():
            return pb.ROSServiceResponse(
                success=False,
                response_json=json.dumps({'error': f'Service call timed out after {timeout}s'})
            )

        result = future.result()

        # 응답을 JSON으로 직렬화
        response_dict = {}
        for field in result.get_fields_and_field_types():
            val = getattr(result, field, None)
            try:
                json.dumps(val)
                response_dict[field] = val
            except (TypeError, ValueError):
                response_dict[field] = str(val)

        return pb.ROSServiceResponse(
            success=True,
            response_json=json.dumps(response_dict)
        )

    def _get_msg_class(self, msg_type: str):
        """
        ROS2 메시지 타입 문자열에서 Python 클래스를 동적으로 import.
        지원 포맷: "std_msgs/msg/String" 또는 "std_msgs/String"
        """
        parts = msg_type.split('/')
        if len(parts) == 3:
            pkg, _, cls_name = parts
        elif len(parts) == 2:
            pkg, cls_name = parts
        else:
            raise ValueError(f"Invalid msg type format: {msg_type}")
        module = importlib.import_module(f'{pkg}.msg')
        return getattr(module, cls_name)

    def WaitForTopic(self, request, context):
        """지정된 토픽의 메시지 1건이 도착할 때까지 블로킹 대기."""
        topic_name = request.topic_name
        msg_type = request.msg_type
        timeout = request.timeout

        try:
            msg_class = self._get_msg_class(msg_type)
        except Exception as e:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details(f"Failed to load msg type '{msg_type}': {e}")
            return pb.WaitForTopicResponse(success=False, message_json=json.dumps({'error': str(e)}))

        event = threading.Event()
        received = {'msg': None}

        def _callback(msg):
            if not event.is_set():
                received['msg'] = msg
                event.set()

        sub = self._node.create_subscription(msg_class, topic_name, _callback, 10)

        try:
            # timeout <= 0: wait forever, but still honor gRPC cancellation via polling
            if timeout and timeout > 0:
                triggered = event.wait(timeout=timeout)
            else:
                # Poll so RPC 취소/서버 종료 시 빠져나갈 수 있음
                triggered = False
                while not triggered:
                    triggered = event.wait(timeout=0.5)
                    if not triggered and not context.is_active():
                        break
        finally:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass

        if not triggered:
            return pb.WaitForTopicResponse(
                success=False,
                message_json=json.dumps({'error': 'timeout or cancelled'})
            )

        msg = received['msg']
        msg_dict = {}
        try:
            for field in msg.get_fields_and_field_types():
                val = getattr(msg, field, None)
                try:
                    json.dumps(val)
                    msg_dict[field] = val
                except (TypeError, ValueError):
                    msg_dict[field] = str(val)
        except Exception:
            msg_dict = {'raw': str(msg)}

        return pb.WaitForTopicResponse(
            success=True,
            message_json=json.dumps(msg_dict)
        )
