# -*- coding: utf-8 -*-
"""
EnvService gRPC servicer.
기존 Env 클래스를 gRPC로 래핑. 이미지는 공유 메모리를 통해 전송.
"""
import json
import threading
import time

import grpc

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc
from .image_bridge import ImageBridgeWriter


class EnvServiceServicer(pb_grpc.EnvServiceServicer):
    def __init__(self, node, agent_servicer):
        """
        Args:
            node: rclpy.node.Node
            agent_servicer: AgentServiceServicer - agent 인스턴스 접근용
        """
        self.node = node
        self.agent_servicer = agent_servicer
        self.envs = {}  # env_id -> Env instance
        self._next_id = 1
        self._lock = threading.Lock()
        self.image_bridge = ImageBridgeWriter()

    def CreateEnv(self, request, context):
        from backend.env.env import Env

        sensors = json.loads(request.sensors_json) if request.sensors_json else []
        agents = []
        for aid in request.agent_ids:
            agent = self.agent_servicer.get_agent(aid)
            if agent is None:
                return pb.EnvId(id=-1)
            agents.append(agent)

        env = Env(
            self.node, agents, sensors,
            language_instruction=request.language_instruction or None,
            virtual_agents=request.virtual_agents,
        )

        with self._lock:
            env_id = self._next_id
            self._next_id += 1
            self.envs[env_id] = env

        return pb.EnvId(id=env_id)

    def DestroyEnv(self, request, context):
        with self._lock:
            env = self.envs.pop(request.id, None)
        if env:
            env.destroy()
            return pb.StatusResponse(success=True, message='Env destroyed')
        return pb.StatusResponse(success=False, message='Env not found')

    def GetObservation(self, request, context):
        env = self.envs.get(request.id)
        if env is None:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details('Env not found')
            return pb.Observation()

        obs = env.get_observation()
        return self._obs_to_proto(obs, env)

    def Reset(self, request, context):
        env = self.envs.get(request.id)
        if env is None:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            return pb.Observation()

        obs = env.reset()
        return self._obs_to_proto(obs, env)

    def RecordStep(self, request, context):
        env = self.envs.get(request.id)
        if env is None:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            return pb.Observation()

        obs = env.record_step()
        return self._obs_to_proto(obs, env)

    def _obs_to_proto(self, obs, env):
        """Observation dict -> protobuf Observation.
        이미지는 공유 메모리에 기록하고 키만 전달.
        """
        robot_states = obs.get('robot_states', {})
        # numpy array -> list 변환
        robot_states_serializable = {}
        for rid, state in robot_states.items():
            robot_states_serializable[str(rid)] = {
                k: (v.tolist() if hasattr(v, 'tolist') else v)
                for k, v in state.items()
            }

        images_proto = []
        for key, img in obs.get('images', {}).items():
            if img is not None:
                self.image_bridge.write(key, img, time.time())
                h, w = img.shape[:2]
                c = img.shape[2] if img.ndim == 3 else 1
                images_proto.append(pb.ImageData(
                    sensor_key=key,
                    width=w,
                    height=h,
                    channels=c,
                    shm_name=key,
                ))
            else:
                images_proto.append(pb.ImageData(sensor_key=key))

        return pb.Observation(
            robot_states_json=json.dumps(robot_states_serializable),
            images=images_proto,
            language_instruction=obs.get('language_instruction', '') or '',
        )

    def destroy_all(self):
        with self._lock:
            for env in self.envs.values():
                env.destroy()
            self.envs.clear()
        self.image_bridge.cleanup()
