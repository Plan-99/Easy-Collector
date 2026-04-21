# -*- coding: utf-8 -*-
"""
ViveService gRPC 서비스 구현.
ViveController를 ROS2 컨테이너에서 관리한다.
"""
import grpc

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc


class ViveServiceServicer(pb_grpc.ViveServiceServicer):
    def __init__(self, node, agent_servicer):
        self._node = node
        self._agent_servicer = agent_servicer
        self._vive = None

    def CreateVive(self, request, context):
        try:
            from backend.env.vive_controller import ViveController

            # agent_servicer에서 실제 Agent 객체들 가져오기
            agents = []
            for agent_id in request.agent_ids:
                agent = self._agent_servicer._agents.get(agent_id)
                if agent is None:
                    return pb.StatusResponse(success=False, message=f'Agent {agent_id} not found')
                agents.append(agent)

            self._vive = ViveController(
                node=self._node,
                socketio_instance=_NoopSocketIO(),
                agents=agents,
                move_robot=request.move_robot,
                role=request.role or 'right_wrist',
                scale_factor=request.scale_factor or 2.0,
                rotation_scale_factor=request.rotation_scale_factor or 1.0,
                step_rate=request.step_rate or 40,
                smoothing_alpha=request.smoothing_alpha or 0.1,
                deadzone_pos=request.deadzone_pos or 0.001,
                deadzone_rot=request.deadzone_rot or 0.001,
            )
            return pb.StatusResponse(success=True, message='ViveController created')
        except Exception as e:
            return pb.StatusResponse(success=False, message=str(e))

    def WaitForReady(self, request, context):
        if self._vive is None:
            return pb.StatusResponse(success=False, message='ViveController not created')
        timeout = request.timeout if request.timeout > 0 else 30.0
        ready = self._vive.wait_for_ready(timeout=timeout)
        return pb.StatusResponse(success=ready, message='ready' if ready else 'timeout')

    def SetOrigin(self, request, context):
        if self._vive is None:
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details('ViveController not created')
            return pb.Empty()
        self._vive.set_origin()
        return pb.Empty()

    def StartTeleop(self, request, context):
        if self._vive is None:
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details('ViveController not created')
            return pb.Empty()
        self._vive.start_teleop()
        return pb.Empty()

    def StopTeleop(self, request, context):
        if self._vive is None:
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details('ViveController not created')
            return pb.Empty()
        self._vive.stop_teleop()
        return pb.Empty()

    def ConsumeDelta(self, request, context):
        if self._vive is None:
            return pb.DeltaValues(values=[0.0] * 6)
        delta = self._vive.consume_delta()
        return pb.DeltaValues(values=delta)

    def GetOffset(self, request, context):
        if self._vive is None:
            return pb.DeltaValues(values=[0.0] * 6)
        offset = self._vive.get_offset()
        return pb.DeltaValues(values=offset)

    def DestroyVive(self, request, context):
        if self._vive is not None:
            self._vive.destroy()
            self._vive = None
        return pb.Empty()

    def destroy(self):
        """서버 종료 시 호출."""
        if self._vive is not None:
            self._vive.destroy()
            self._vive = None


class _NoopSocketIO:
    """ViveController의 socketio_instance 대체. 이벤트는 메인 컨테이너에서 처리."""
    def emit(self, event, data=None):
        pass

    def start_background_task(self, target, **kwargs):
        pass
