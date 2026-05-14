# -*- coding: utf-8 -*-
"""
AgentService gRPC servicer.
기존 Agent 클래스를 gRPC로 래핑하여 원격에서 로봇 제어 가능하게 한다.
"""
import json
import threading
import time

import grpc

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc


class AgentServiceServicer(pb_grpc.AgentServiceServicer):
    def __init__(self, node):
        self.node = node
        self.agents = {}  # agent_id -> Agent instance
        self._lock = threading.Lock()

    def _get_agent(self, agent_id, context):
        agent = self.agents.get(agent_id)
        if agent is None:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details(f'Agent {agent_id} not found')
        return agent

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def CreateAgent(self, request, context):
        from ros2_bridge.env.agent import Agent
        robot = json.loads(request.robot_json)
        robot_id = robot['id']

        with self._lock:
            if robot_id in self.agents:
                return pb.AgentId(id=robot_id)
            agent = Agent(self.node, robot)
            self.agents[robot_id] = agent
        return pb.AgentId(id=robot_id)

    def DestroyAgent(self, request, context):
        with self._lock:
            agent = self.agents.pop(request.id, None)
        if agent:
            agent.destroy()
            return pb.StatusResponse(success=True, message='Agent destroyed')
        return pb.StatusResponse(success=False, message='Agent not found')

    # ------------------------------------------------------------------
    # Movement Commands
    # ------------------------------------------------------------------
    def MoveJointStep(self, request, context):
        agent = self._get_agent(request.agent_id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        vel = request.velocity_arg if request.HasField('velocity_arg') else None
        agent.move_joint_step(list(request.action), from_ee=request.from_ee, velocity_arg=vel)
        return pb.StatusResponse(success=True, message='OK')

    def MoveJointDeltaStep(self, request, context):
        agent = self._get_agent(request.agent_id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        agent.move_joint_delta_step(list(request.action))
        return pb.StatusResponse(success=True, message='OK')

    def MoveEEStep(self, request, context):
        agent = self._get_agent(request.agent_id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        target_ee = json.loads(request.target_ee_json)
        vel = request.velocity_arg if request.HasField('velocity_arg') else None
        agent.move_ee_step(target_ee, vel_arg=vel)
        return pb.StatusResponse(success=True, message='OK')

    def MoveEEDeltaStep(self, request, context):
        agent = self._get_agent(request.agent_id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        delta_ee = json.loads(request.delta_ee_json)
        vel = request.velocity_arg if request.HasField('velocity_arg') else None
        tool_positions = json.loads(request.tool_positions_json) if request.tool_positions_json else None
        agent.move_ee_delta_step(delta_ee, vel_arg=vel, tool_positions=tool_positions)
        return pb.StatusResponse(success=True, message='OK')

    def MoveTo(self, request, context):
        agent = self._get_agent(request.agent_id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        # 비동기 — agent.move_to 가 thread 띄우고 즉시 return.
        # step_size 는 deprecated (proto 호환만 남김), duration/hz 만 사용.
        agent.move_to(
            list(request.target_pos),
            duration=request.duration,
            hz=request.hz,
        )
        return pb.StatusResponse(success=True, message='Move started (async)')

    def MoveEETo(self, request, context):
        agent = self._get_agent(request.agent_id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        target_ee = json.loads(request.target_ee_json)
        # 비동기 — agent.move_ee_to 가 IK 푼 뒤 move_to thread 띄우고 즉시 return.
        agent.move_ee_to(
            target_ee,
            duration=request.duration or 5.0,
            hz=request.hz or 100,
        )
        return pb.StatusResponse(success=True, message='Move started (async)')

    def CancelMoveTo(self, request, context):
        agent = self._get_agent(request.id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        agent.cancel_move_to()
        return pb.StatusResponse(success=True, message='Cancel signal sent')

    # ------------------------------------------------------------------
    # State Queries
    # ------------------------------------------------------------------
    def GetJointStates(self, request, context):
        agent = self._get_agent(request.id, context)
        if agent is None:
            return pb.JointValues()
        js = agent.get_joint_states()
        return pb.JointValues(values=js if js else [])

    def GetJointActions(self, request, context):
        agent = self._get_agent(request.id, context)
        if agent is None:
            return pb.JointValues()
        ja = agent.get_joint_actions()
        return pb.JointValues(values=ja if ja else [])

    def GetJointVel(self, request, context):
        agent = self._get_agent(request.id, context)
        if agent is None:
            return pb.JointValues()
        return pb.JointValues(values=agent.get_joint_vel())

    def GetJointEffort(self, request, context):
        agent = self._get_agent(request.id, context)
        if agent is None:
            return pb.JointValues()
        return pb.JointValues(values=agent.get_joint_effort())

    def GetEEPosition(self, request, context):
        agent = self._get_agent(request.id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        ep = agent.get_ee_position()
        return pb.StatusResponse(success=True, message=json.dumps(ep) if ep else '')

    def GetEETarget(self, request, context):
        agent = self._get_agent(request.id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        et = agent.get_ee_target()
        return pb.StatusResponse(success=True, message=json.dumps(et) if et else '')

    # ------------------------------------------------------------------
    # IK
    # ------------------------------------------------------------------
    def ResetIKSolver(self, request, context):
        agent = self._get_agent(request.agent_id, context)
        if agent is None:
            return pb.StatusResponse(success=False, message='Agent not found')
        agent.reset_ik_solver(list(request.q))
        return pb.StatusResponse(success=True, message='OK')

    def FetchJointMapToAction(self, request, context):
        agent = self._get_agent(request.agent_id, context)
        if agent is None:
            return pb.JointValues()
        joint_map = json.loads(request.joint_map_json)
        action = agent.fetch_joint_map_to_action(joint_map)
        return pb.JointValues(values=action)

    # ------------------------------------------------------------------
    # Streaming
    # ------------------------------------------------------------------
    def SubscribeRobotState(self, request, context):
        """10Hz로 로봇 상태를 스트리밍한다."""
        agent_id = request.agent_id
        stale_after = 8.0

        while context.is_active():
            agent = self.agents.get(agent_id)
            if agent is None:
                time.sleep(0.5)
                continue

            now = time.time()
            last_js = agent.last_joint_update or 0
            js = agent.get_joint_states()
            ja = agent.get_joint_actions()
            ep = agent.get_ee_position()
            et = agent.get_ee_target()
            connected = (now - last_js) <= stale_after and js is not None

            state = pb.RobotState(
                agent_id=agent_id,
                joint_states=js if js else [],
                joint_actions=ja if ja else [],
                ee_pos_json=json.dumps(ep) if ep else '',
                ee_target_json=json.dumps(et) if et else '',
                connected=connected,
                joint_vel=agent.get_joint_vel(),
                joint_effort=agent.get_joint_effort(),
            )
            yield state
            time.sleep(0.1)  # 10Hz

    def get_agent(self, agent_id):
        """외부에서 Agent 인스턴스에 접근할 때 사용 (EnvService 등)."""
        return self.agents.get(agent_id)

    def destroy_all(self):
        with self._lock:
            for agent in self.agents.values():
                agent.destroy()
            self.agents.clear()
