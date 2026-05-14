# -*- coding: utf-8 -*-
"""
ROSProxy gRPC 서비스 구현.
ROS2 서비스를 동적으로 호출하는 프록시.
"""
import array
import json
import importlib
import grpc

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc


def _to_jsonable(val):
    """Best-effort conversion of a ROS msg field value into a json-serialisable
    Python primitive.

    ROS IDL maps ``float64[]`` to ``array.array('d', ...)`` (or numpy arrays
    when numpy is available), and nested messages to generated Python classes
    whose fields are themselves ROS types. ``json.dumps`` does not know how to
    handle either, so we recursively unwrap them here.
    """
    if isinstance(val, (str, int, float, bool)) or val is None:
        return val
    if isinstance(val, (list, tuple)):
        return [_to_jsonable(v) for v in val]
    if isinstance(val, dict):
        return {k: _to_jsonable(v) for k, v in val.items()}
    if isinstance(val, array.array):
        return list(val)
    # numpy.ndarray (optional dep — guard the import)
    tolist = getattr(val, 'tolist', None)
    if callable(tolist):
        try:
            return _to_jsonable(tolist())
        except Exception:
            pass
    # ROS message: walk its fields.
    get_fields = getattr(val, 'get_fields_and_field_types', None)
    if callable(get_fields):
        return {f: _to_jsonable(getattr(val, f, None)) for f in get_fields()}
    return str(val)


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

        # 응답을 JSON으로 직렬화 (array.array, numpy, nested msg 모두 처리)
        response_dict = {
            field: _to_jsonable(getattr(result, field, None))
            for field in result.get_fields_and_field_types()
        }

        return pb.ROSServiceResponse(
            success=True,
            response_json=json.dumps(response_dict)
        )
