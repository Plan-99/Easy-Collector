# -*- coding: utf-8 -*-
"""
RemoteEnv - Env 클래스의 gRPC + 공유 메모리 프록시.
기존 Env와 동일한 인터페이스를 제공.
"""
import collections
import json
from types import SimpleNamespace

from .generated import robot_bridge_pb2 as pb
from .client import get_bridge_client
from .image_reader import ImageBridgeReader


class RemoteEnv:
    """Env와 동일한 public 인터페이스를 gRPC + shared memory로 구현."""

    def __init__(self, agents, sensors, language_instruction=None, virtual_agents=False):
        """
        Args:
            agents: list of RemoteAgent instances
            sensors: list of sensor config dicts
            language_instruction: str or None
            virtual_agents: bool
        """
        self.agents = agents
        self.sensors = sensors
        self.language_instruction = language_instruction
        self.virtual_agents = virtual_agents

        self._image_reader = ImageBridgeReader()

        # gRPC로 Env 생성
        client = get_bridge_client()
        agent_ids = [a.id for a in agents]
        result = client.env.CreateEnv(pb.EnvConfig(
            agent_ids=agent_ids,
            sensors_json=json.dumps(sensors),
            language_instruction=language_instruction or '',
            virtual_agents=virtual_agents,
        ))
        self._env_id = result.id

    def destroy(self):
        try:
            client = get_bridge_client()
            client.env.DestroyEnv(pb.EnvId(id=self._env_id))
        except Exception as e:
            print(f"[RemoteEnv] destroy error: {e}")

    def get_observation(self):
        client = get_bridge_client()
        obs_proto = client.env.GetObservation(pb.EnvId(id=self._env_id))
        return self._proto_to_obs(obs_proto)

    def reset(self):
        client = get_bridge_client()
        obs_proto = client.env.Reset(pb.EnvId(id=self._env_id))
        obs = self._proto_to_obs(obs_proto)
        return SimpleNamespace(observation=obs)

    def record_step(self):
        client = get_bridge_client()
        obs_proto = client.env.RecordStep(pb.EnvId(id=self._env_id))
        obs = self._proto_to_obs(obs_proto)
        return SimpleNamespace(observation=obs)

    def _proto_to_obs(self, obs_proto):
        """protobuf Observation -> dict (기존 Env.get_observation() 형식)."""
        obs = collections.OrderedDict()

        # Robot states
        robot_states = json.loads(obs_proto.robot_states_json) if obs_proto.robot_states_json else {}
        # key를 int로 변환 (기존 코드와 호환)
        obs['robot_states'] = {int(k): v for k, v in robot_states.items()}

        # Images - 공유 메모리에서 읽기
        images = {}
        for img_data in obs_proto.images:
            if img_data.shm_name:
                image, _ = self._image_reader.read(img_data.shm_name)
                images[img_data.sensor_key] = image
            else:
                images[img_data.sensor_key] = None
        obs['images'] = images

        obs['language_instruction'] = obs_proto.language_instruction or None
        return obs
