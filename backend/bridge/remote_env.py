# -*- coding: utf-8 -*-
"""
RemoteEnv — backend 측 Env (record/inference loop) 의 gRPC 프록시.

기존 backend/env/env.py::Env 가 step 마다 6+ 회 gRPC 호출 (joint_states,
joint_actions, ee_pos, joint_vel, joint_effort + per-sensor) 을 했다면,
RemoteEnv 는 ros2 컨테이너의 EnvService 한 번 호출(`RecordStep`)만 하고
이미지는 shm 에서 zero-copy 로 직접 읽는다.

센서 lifecycle (구독 시작/종료) 은 RemoteObs (ObsService 래퍼) 가 담당.
"""
import collections
import json
from types import SimpleNamespace

from .generated import robot_bridge_pb2 as pb
from .client import get_bridge_client
from .image_reader import ImageBridgeReader
from .remote_obs import RemoteObs


class RemoteEnv:
    """기존 Env 와 동일한 public 인터페이스 — drop-in 교체 가능."""

    def __init__(self, agents, sensors, language_instruction=None,
                 virtual_agents=False, tutorial=False):
        """
        Args:
            agents: list of RemoteAgent instances (current_app.agents 로부터)
            sensors: list of sensor config dicts
            language_instruction: str or None
            virtual_agents: bool — record_episode 의 vive_only 모드용 zero-fill
            tutorial: bool — Reset 시 mujoco world 자동 reset
        """
        self.agents = agents
        self.sensors = sensors
        self.language_instruction = language_instruction
        self.virtual_agents = virtual_agents
        self.tutorial = tutorial

        # 센서 구독: 기존 Env 와 동일하게 자동 시작 (record_episode 등이 별도
        # 으로 부르지 않아도 첫 frame 이 흐르도록)
        self._obs = RemoteObs(sensors)
        self._image_reader = ImageBridgeReader()

        # 서버사이드 env 핸들 생성
        client = get_bridge_client()
        result = client.env.CreateEnv(pb.EnvConfig(
            agent_ids=[a.id for a in agents],
            sensors_json=json.dumps(sensors),
            language_instruction=language_instruction or '',
            virtual_agents=virtual_agents,
            tutorial=tutorial,
        ))
        self._env_id = result.id

    # ------------------------------------------------------------------
    # 기존 Env 호환 인터페이스
    # ------------------------------------------------------------------
    def initialize(self):
        """과거 Env.initialize 호환용 — Reset RPC 가 tutorial 처리를 포함하므로
        보통 호출 불필요. 명시적으로 부르면 한 번 reset.
        """
        if not self.tutorial:
            return
        # Reset 을 호출하되 observation 결과는 버림 — 호출자가 별도로 reset() 을
        # 부르는 패턴(record_episode 의 reset 호출) 과 호환.
        self.reset()

    def get_observation(self):
        client = get_bridge_client()
        obs_proto = client.env.GetObservation(pb.EnvId(id=self._env_id))
        return self._proto_to_obs(obs_proto)

    def reset(self):
        client = get_bridge_client()
        obs_proto = client.env.Reset(pb.EnvId(id=self._env_id))
        return SimpleNamespace(observation=self._proto_to_obs(obs_proto))

    def record_step(self):
        client = get_bridge_client()
        obs_proto = client.env.RecordStep(pb.EnvId(id=self._env_id))
        return SimpleNamespace(observation=self._proto_to_obs(obs_proto))

    def wait_for_images(self, timeout=10.0):
        """모든 센서가 첫 프레임을 받을 때까지 대기. 서버사이드 + 클라이언트
        polling 둘 다 동작 — 서버사이드가 잘 돌아도 backend shm read 측에
        파일 캐시 race 가 있을 수 있어 RemoteObs.wait_for_images 도 같이 호출.
        """
        client = get_bridge_client()
        try:
            res = client.env.WaitForImages(pb.WaitForImagesRequest(
                env_id=self._env_id, timeout=float(timeout),
            ))
            if not res.success:
                # 서버 timeout — 굳이 client polling 한 번 더 시도해보고 결과 반영.
                return self._obs.wait_for_images(timeout=1.0)
        except Exception as e:
            print(f"[RemoteEnv] wait_for_images RPC failed: {e}")
            return self._obs.wait_for_images(timeout=timeout)
        return True

    def destroy(self):
        try:
            client = get_bridge_client()
            client.env.DestroyEnv(pb.EnvId(id=self._env_id))
        except Exception as e:
            print(f"[RemoteEnv] destroy error: {e}")
        try:
            self._obs.stop()
        except Exception as e:
            print(f"[RemoteEnv] obs stop error: {e}")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _proto_to_obs(self, obs_proto):
        """protobuf Observation -> dict (기존 Env.get_observation() 형식)."""
        obs = collections.OrderedDict()

        robot_states_raw = (
            json.loads(obs_proto.robot_states_json)
            if obs_proto.robot_states_json else {}
        )
        # key 를 int 로 변환 (기존 코드 호환 — env.agents[i].id 가 int).
        obs['robot_states'] = {int(k): v for k, v in robot_states_raw.items()}

        # 이미지: 서버는 shm_name 만 metadata 로 채워서 보낸다 → backend 가
        # ImageBridgeReader 로 직접 읽어서 numpy 배열 만듦 (zero-copy).
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
