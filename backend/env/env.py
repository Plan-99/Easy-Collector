# -*- coding: utf-8 -*-
"""
Env — backend 로컬 환경.
RemoteAgent(로컬 캐시) + RemoteObs(shm 직접 읽기)를 조합하여
record_step 시 gRPC 호출 0회로 observation을 생성한다.
"""
import collections
from types import SimpleNamespace

from ..bridge.remote_obs import RemoteObs


class Env:
    def __init__(self, agents, sensors, language_instruction=None, virtual_agents=False, tutorial=False):
        self.agents = agents
        self.sensors = sensors
        self.language_instruction = language_instruction
        self.virtual_agents = virtual_agents
        self.tutorial = tutorial
        self.obs = RemoteObs(sensors)

    def get_observation(self):
        return collections.OrderedDict(
            robot_states=self._get_robot_states(),
            images=self.obs.get_images(),
            language_instruction=self.language_instruction,
        )

    def initialize(self):
        """환경 초기화 — tutorial sim의 world objects(큐브 등)를 시작 위치로 리셋.

        `tutorial=False`면 no-op. tutorial 모드인데 reset이 실패하면 예외를
        그대로 전파해 호출자가 인지/대응할 수 있게 한다.
        """
        if not self.tutorial:
            return
        from ..api.routes.tutorial import reset_tutorial_world
        ok, msg = reset_tutorial_world()
        if not ok:
            raise RuntimeError(f'Tutorial world reset failed: {msg}')

    def reset(self):
        self.initialize()
        return SimpleNamespace(observation=self.get_observation())

    def record_step(self):
        return SimpleNamespace(observation=self.get_observation())

    def _get_robot_states(self):
        states = {}
        for agent in self.agents:
            if self.virtual_agents and agent.role == 'single_arm':
                zeros = [0.0] * agent.joint_len
                states[agent.id] = {
                    'qpos': zeros,
                    'qaction': zeros,
                    'eepos': None,
                    'qvel': zeros,
                    'qeffort': zeros,
                }
            else:
                # 캐시(agent.joint_states 등)는 subscribe_state_stream이 동작할 때만
                # 채워지지만 현재 그 스트림은 호출되지 않으므로 None — 늘 fresh
                # gRPC fetch로 가져온다 (record/inference 둘 다 None이면 np 호출 깨짐).
                qpos = agent.get_joint_states()
                qaction = agent.get_joint_actions()
                states[agent.id] = {
                    'qpos': qpos if qpos is not None else [0.0] * agent.joint_len,
                    'qaction': qaction if qaction is not None else [0.0] * agent.joint_len,
                    'eepos': agent.get_ee_position(),
                    'qvel': agent.get_joint_vel(),
                    'qeffort': agent.get_joint_effort(),
                }
        return states

    def wait_for_images(self, timeout=10.0):
        """Wait until all sensors have their first frame."""
        return self.obs.wait_for_images(timeout=timeout)

    def destroy(self):
        self.obs.stop()
