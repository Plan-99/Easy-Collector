# -*- coding: utf-8 -*-
"""
EnvService — 센서/로봇 observation 빌더.

기존 backend/env/env.py 가 record/inference 한 step 마다 backend → bridge 로
6+ 번 gRPC 라운드트립 (get_joint_states / get_joint_actions / get_ee_position /
get_joint_vel / get_joint_effort / + 이미지 shm 메타) 을 했었는데, 이 서비스가
한 번의 RecordStep RPC 로 모든 robot_state 를 in-process 로 모아 직렬화해 돌려
주고, 이미지는 shm_name 만 metadata 로 전달 (backend 가 zero-copy 로 직접 read).

Tutorial 모드 처리:
    Reset 시 /tutorial/reset_world ros2 service 를 호출해 큐브 등 movable
    object 를 시작 위치로 복원. 호출 자체는 ROSProxy 와 동일 패턴
    (client.call_async + future.done() polling).
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
            node: rclpy Node — tutorial reset 서비스 호출용.
            agent_servicer: AgentServiceServicer — agent_id → Agent 인스턴스 lookup.
        """
        self.node = node
        self.agent_servicer = agent_servicer
        self.envs = {}  # env_id -> dict
        self._next_id = 1
        self._lock = threading.Lock()
        self._tutorial_reset_client = None
        # WaitForImages 는 ImageBridgeWriter 가 shm 에 쓴 직후의 frame 을 검사하면
        # 되므로 같은 인스턴스를 공유한다 (ObsServiceServicer 가 사용하는 그것).
        # __init__ 에서는 None 으로 두고 set_image_bridge() 로 주입.
        self._image_bridge = None

    def set_image_bridge(self, image_bridge: ImageBridgeWriter):
        """server.py 가 ObsService 와 같은 ImageBridgeWriter 인스턴스를 공유."""
        self._image_bridge = image_bridge

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def CreateEnv(self, request, context):
        agent_ids = list(request.agent_ids)
        sensors = json.loads(request.sensors_json) if request.sensors_json else []
        with self._lock:
            env_id = self._next_id
            self._next_id += 1
            self.envs[env_id] = {
                'agent_ids': agent_ids,
                'sensors': sensors,
                'language_instruction': request.language_instruction or '',
                'virtual_agents': bool(request.virtual_agents),
                'tutorial': bool(request.tutorial),
            }
        print(f"[EnvService] CreateEnv id={env_id} agents={agent_ids} "
              f"sensors={[s.get('id') for s in sensors]} "
              f"tutorial={bool(request.tutorial)}", flush=True)
        return pb.EnvId(id=env_id)

    def DestroyEnv(self, request, context):
        with self._lock:
            removed = self.envs.pop(request.id, None)
        if removed is None:
            return pb.StatusResponse(success=False, message='Env not found')
        print(f"[EnvService] DestroyEnv id={request.id}", flush=True)
        return pb.StatusResponse(success=True, message='Env destroyed')

    # ------------------------------------------------------------------
    # Observation building
    # ------------------------------------------------------------------
    def GetObservation(self, request, context):
        env = self._get_env(request.id, context)
        if env is None:
            return pb.Observation()
        return self._build_observation(env)

    def Reset(self, request, context):
        env = self._get_env(request.id, context)
        if env is None:
            return pb.Observation()
        if env['tutorial']:
            ok, msg = self._reset_tutorial_world()
            if not ok:
                # 실패해도 observation 은 돌려준다 (호출자가 RuntimeError 처리할 수
                # 있게 status 를 metadata 로 추가). 현재 backend 의 Env.initialize 는
                # 실패시 raise 하므로 여기서도 그 패턴을 따라 grpc 에러로 변환.
                context.set_code(grpc.StatusCode.INTERNAL)
                context.set_details(f'Tutorial world reset failed: {msg}')
                return pb.Observation()
        return self._build_observation(env)

    def RecordStep(self, request, context):
        env = self._get_env(request.id, context)
        if env is None:
            return pb.Observation()
        return self._build_observation(env)

    def WaitForImages(self, request, context):
        env = self._get_env(request.env_id, context)
        if env is None:
            return pb.StatusResponse(success=False, message='Env not found')
        if self._image_bridge is None:
            return pb.StatusResponse(success=False, message='ImageBridge not wired')
        sensors = env['sensors'] or []
        timeout = request.timeout if request.timeout > 0 else 5.0
        deadline = time.time() + timeout
        keys = [f"sensor_{s['id']}" for s in sensors]
        while time.time() < deadline:
            if all(self._image_bridge.has_frame(k) for k in keys):
                return pb.StatusResponse(success=True, message='All frames ready')
            time.sleep(0.05)
        missing = [k for k in keys if not self._image_bridge.has_frame(k)]
        return pb.StatusResponse(
            success=False,
            message=f'Timeout after {timeout}s; missing={missing}',
        )

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _build_observation(self, env):
        robot_states = {}
        for agent_id in env['agent_ids']:
            agent = self.agent_servicer.get_agent(agent_id)
            if agent is None:
                continue
            joint_len = agent.joint_len
            if env['virtual_agents'] and getattr(agent, 'role', None) == 'single_arm':
                zeros = [0.0] * joint_len
                robot_states[str(agent_id)] = {
                    'qpos': zeros,
                    'qaction': zeros,
                    'eepos': None,
                    'qvel': zeros,
                    'qeffort': zeros,
                }
                continue
            qpos = agent.get_joint_states()
            qaction = agent.get_joint_actions()
            robot_states[str(agent_id)] = {
                'qpos': qpos if qpos is not None else [0.0] * joint_len,
                'qaction': qaction if qaction is not None else [0.0] * joint_len,
                'eepos': agent.get_ee_position(),
                'qvel': agent.get_joint_vel(),
                'qeffort': agent.get_joint_effort(),
            }

        # 이미지는 shm 포인터만 전달 — backend 가 ImageBridgeReader 로 직접 read.
        images = []
        for sensor in env['sensors']:
            sensor_key = f"sensor_{sensor['id']}"
            images.append(pb.ImageData(sensor_key=sensor_key, shm_name=sensor_key))

        return pb.Observation(
            robot_states_json=json.dumps(robot_states),
            images=images,
            language_instruction=env['language_instruction'] or '',
        )

    def _reset_tutorial_world(self):
        """/tutorial/reset_world (std_srvs/srv/Empty) 호출. (success, message)."""
        try:
            from std_srvs.srv import Empty
            if self._tutorial_reset_client is None:
                self._tutorial_reset_client = self.node.create_client(
                    Empty, '/tutorial/reset_world'
                )
            client = self._tutorial_reset_client
            if not client.wait_for_service(timeout_sec=5.0):
                return False, 'service /tutorial/reset_world not available'
            future = client.call_async(Empty.Request())
            timeout = 10.0
            elapsed = 0.0
            while not future.done() and elapsed < timeout:
                time.sleep(0.05)
                elapsed += 0.05
            if not future.done():
                return False, f'service call timed out after {timeout}s'
            return True, ''
        except Exception as e:
            return False, str(e)

    def _get_env(self, env_id, context):
        env = self.envs.get(env_id)
        if env is None:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details(f'Env {env_id} not found')
        return env
