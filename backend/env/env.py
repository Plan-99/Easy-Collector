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
    def __init__(self, agents, sensors, language_instruction=None, virtual_agents=False):
        self.agents = agents
        self.sensors = sensors
        self.language_instruction = language_instruction
        self.virtual_agents = virtual_agents
        self.obs = RemoteObs(sensors)

    def get_observation(self):
        return collections.OrderedDict(
            robot_states=self._get_robot_states(),
            images=self.obs.get_images(),
            language_instruction=self.language_instruction,
        )

    def reset(self):
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
                states[agent.id] = {
                    'qpos': agent.joint_states,
                    'qaction': agent.joint_actions,
                    'eepos': agent.ee_pos,
                    'qvel': agent.get_joint_vel(),
                    'qeffort': agent.get_joint_effort(),
                }
        return states

    def wait_for_images(self, timeout=10.0):
        """Wait until all sensors have their first frame."""
        return self.obs.wait_for_images(timeout=timeout)

    def destroy(self):
        self.obs.stop()
