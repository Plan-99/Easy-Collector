# -*- coding: utf-8 -*-
"""
RemoteViveController — ViveController의 gRPC 프록시.
메인 컨테이너에서 ROS2 컨테이너의 ViveService를 호출한다.
"""
from .client import get_bridge_client
from .generated import robot_bridge_pb2 as pb


class RemoteViveController:
    """ViveController의 원격 프록시. record_episode.py에서 사용."""

    def __init__(self, socketio_instance, agents, move_robot=False,
                 scale_factor=2.0, step_rate=40, **kwargs):
        self._socketio = socketio_instance
        self._client = get_bridge_client()

        config = pb.ViveConfig(
            agent_ids=[a.id for a in agents],
            move_robot=move_robot,
            role=kwargs.get('role', 'right_wrist'),
            scale_factor=scale_factor,
            rotation_scale_factor=kwargs.get('rotation_scale_factor', 1.0),
            step_rate=step_rate,
            smoothing_alpha=kwargs.get('smoothing_alpha', 0.1),
            deadzone_pos=kwargs.get('deadzone_pos', 0.001),
            deadzone_rot=kwargs.get('deadzone_rot', 0.001),
        )
        resp = self._client.vive.CreateVive(config)
        if not resp.success:
            raise RuntimeError(f'Failed to create ViveController: {resp.message}')

    def wait_for_ready(self, timeout=30.0):
        resp = self._client.vive.WaitForReady(pb.ViveReadyRequest(timeout=timeout))
        if resp.success:
            self._socketio.emit('vive_node_ready', {})
        else:
            self._socketio.emit('vive_node_error', {'message': 'VIVE controller not detected'})
        return resp.success

    def set_origin(self):
        self._client.vive.SetOrigin(pb.Empty())

    def start_teleop(self):
        self._client.vive.StartTeleop(pb.Empty())

    def stop_teleop(self):
        self._client.vive.StopTeleop(pb.Empty())

    def consume_delta(self):
        resp = self._client.vive.ConsumeDelta(pb.Empty())
        return list(resp.values)

    def get_offset(self):
        resp = self._client.vive.GetOffset(pb.Empty())
        return list(resp.values)

    def destroy(self):
        try:
            self._client.vive.DestroyVive(pb.Empty())
        except Exception:
            pass

    @property
    def is_synced(self):
        """ViveController는 sync 대기가 필요 없음."""
        return True
