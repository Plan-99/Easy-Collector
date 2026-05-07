# -*- coding: utf-8 -*-
"""
RemoteAgent - Agent 클래스의 gRPC 프록시.
기존 Agent와 동일한 인터페이스를 제공하되, 실제 동작은 ROS2 컨테이너에 위임.
"""
import json
import time
import threading

from .generated import robot_bridge_pb2 as pb
from .client import get_bridge_client


class RemoteAgent:
    """Agent와 동일한 public 인터페이스를 gRPC로 구현."""

    def __init__(self, robot):
        """
        Args:
            robot: dict - 로봇 설정 (DB 모델에서 가져온 것)
        """
        self.id = robot['id']
        self.robot = robot
        self.joint_len = len(robot['joint_names'])
        self.joint_names = robot['joint_names']
        self.joint_upper_bounds = robot.get('joint_upper_bounds')
        self.joint_lower_bounds = robot.get('joint_lower_bounds')
        self.role = robot.get('role', 'single_arm')
        self.tool_inner = robot.get('tool_inner', False)
        self.robot_type = robot['type']
        self.robot_company = robot['company']
        self.is_sim = robot.get('is_sim', False)
        self.ik_solver = True if robot.get('ik_available', False) else None
        self.last_ee_delta = None

        # ee_names: IK가 있으면 ik_setting의 ee_definitions에서 추출
        self.ee_names = []
        if self.ik_solver:
            ik_setting = robot.get('ik_setting') or (robot.get('settings') or {}).get('ik_setting')
            if ik_setting and 'ee_definitions' in ik_setting:
                self.ee_names = [ed[0] for ed in ik_setting['ee_definitions']]

        # 상태 캐시 (subscribe 스트리밍에서 업데이트)
        self.joint_states = None
        self.joint_actions = None
        self.ee_pos = None
        self.ee_target = None
        self.last_joint_update = None
        self.moved_by_ui = False
        self.move_lock = False
        # move_to 는 비동기라 server-side 진행 상태를 매번 RPC 로 폴링하면 비싸다.
        # 대신 호출자가 알려준 duration 으로 로컬 deadline 을 둬서 그 시간 동안만
        # is_moving=True 로 보고한다. cancel_move_to 호출 시 즉시 0 으로 리셋.
        self._move_deadline = 0.0  # monotonic seconds

        # gRPC로 Agent 생성
        client = get_bridge_client()
        result = client.agent.CreateAgent(
            pb.RobotConfig(robot_json=json.dumps(robot))
        )
        self._agent_id = result.id

    def destroy(self):
        try:
            client = get_bridge_client()
            client.agent.DestroyAgent(pb.AgentId(id=self._agent_id))
        except Exception as e:
            print(f"[RemoteAgent] destroy error: {e}")

    # ------------------------------------------------------------------
    # Movement Commands
    # ------------------------------------------------------------------
    def move_joint_step(self, action, from_ee=False, velocity_arg=None):
        client = get_bridge_client()
        req = pb.MoveJointRequest(
            agent_id=self._agent_id,
            action=action,
            from_ee=from_ee,
        )
        if velocity_arg is not None:
            req.velocity_arg = velocity_arg
        client.agent.MoveJointStep(req)

    def move_joint_delta_step(self, delta_action):
        client = get_bridge_client()
        client.agent.MoveJointDeltaStep(
            pb.MoveJointRequest(agent_id=self._agent_id, action=delta_action)
        )

    def move_ee_step(self, target_ee_dict, vel_arg=None):
        client = get_bridge_client()
        req = pb.MoveEERequest(
            agent_id=self._agent_id,
            target_ee_json=json.dumps(target_ee_dict),
        )
        if vel_arg is not None:
            req.velocity_arg = vel_arg
        client.agent.MoveEEStep(req)

    def move_ee_delta_step(self, delta_ee_dict, vel_arg=None, tool_positions=None):
        client = get_bridge_client()
        req = pb.MoveEEDeltaRequest(
            agent_id=self._agent_id,
            delta_ee_json=json.dumps(delta_ee_dict),
            tool_positions_json=json.dumps(tool_positions) if tool_positions else '',
        )
        if vel_arg is not None:
            req.velocity_arg = vel_arg
        client.agent.MoveEEDeltaStep(req)

    def move_to(self, target_pos, duration=5.0, hz=100.0, step_size=None):
        """Async — 즉시 return. is_moving 폴링 또는 cancel_move_to 로 제어.

        step_size 는 backwards-compat 용 (무시됨). duration/hz 만 사용.
        """
        client = get_bridge_client()
        client.agent.MoveTo(pb.MoveToRequest(
            agent_id=self._agent_id,
            target_pos=list(target_pos),
            duration=float(duration),
            hz=float(hz),
        ))
        self._move_deadline = time.monotonic() + float(duration)

    def cancel_move_to(self):
        """진행 중인 move_to 를 즉시 중단 신호 — server-side thread 가
        다음 step 직전에 abort 검사 후 종료. is_moving 도 곧 False 로."""
        client = get_bridge_client()
        client.agent.CancelMoveTo(pb.AgentId(id=self._agent_id))
        self._move_deadline = 0.0

    @property
    def is_moving(self):
        """move_to 가 비동기로 진행 중인지. duration 기반 로컬 추정."""
        return time.monotonic() < self._move_deadline

    # ------------------------------------------------------------------
    # State Queries
    # ------------------------------------------------------------------
    def get_joint_states(self):
        try:
            client = get_bridge_client()
            result = client.agent.GetJointStates(pb.AgentId(id=self._agent_id))
            values = list(result.values)
            if values:
                self.last_joint_update = time.time()
                return values
            return None
        except Exception:
            return None

    def get_joint_actions(self):
        try:
            client = get_bridge_client()
            result = client.agent.GetJointActions(pb.AgentId(id=self._agent_id))
            values = list(result.values)
            return values if values else None
        except Exception:
            return None

    def get_joint_vel(self):
        try:
            client = get_bridge_client()
            result = client.agent.GetJointVel(pb.AgentId(id=self._agent_id))
            return list(result.values)
        except Exception:
            return [0.0] * self.joint_len

    def get_joint_effort(self):
        try:
            client = get_bridge_client()
            result = client.agent.GetJointEffort(pb.AgentId(id=self._agent_id))
            return list(result.values)
        except Exception:
            return [0.0] * self.joint_len

    def get_ee_position(self):
        try:
            client = get_bridge_client()
            result = client.agent.GetEEPosition(pb.AgentId(id=self._agent_id))
            return json.loads(result.message) if result.message else None
        except Exception:
            return None

    def get_ee_target(self):
        try:
            client = get_bridge_client()
            result = client.agent.GetEETarget(pb.AgentId(id=self._agent_id))
            return json.loads(result.message) if result.message else None
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Joint/Tool Split (순수 로직, gRPC 불필요)
    # ------------------------------------------------------------------
    def get_joint_and_tool_pos(self, full_joint_positions):
        if full_joint_positions is None:
            return None, None
        if self.role == 'tool':
            return None, full_joint_positions
        if self.role == 'single_arm':
            if self.tool_inner:
                return full_joint_positions[:-1], [full_joint_positions[-1]]
            return full_joint_positions, None
        elif self.role == 'dual_arm':
            mid = len(full_joint_positions) // 2
            left = full_joint_positions[:mid]
            right = full_joint_positions[mid:]
            if self.tool_inner:
                arm_joints = left[:-1] + right[:-1]
                tool_joints = [left[-1], right[-1]]
                return arm_joints, tool_joints
            return full_joint_positions, None
        return full_joint_positions, None

    def compute_fk_delta(self, qaction, qpos):
        if not self.ik_solver or qaction is None or qpos is None:
            return None
        try:
            client = get_bridge_client()
            result = client.agent.ComputeFKDelta(pb.FKDeltaRequest(
                agent_id=self._agent_id,
                qaction=qaction,
                qpos=qpos,
            ))
            if result.message:
                return json.loads(result.message)
            return None
        except Exception:
            return None

    # ------------------------------------------------------------------
    # IK
    # ------------------------------------------------------------------
    def reset_ik_solver(self, q):
        client = get_bridge_client()
        client.agent.ResetIKSolver(pb.ResetIKRequest(
            agent_id=self._agent_id, q=q,
        ))

    def fetch_joint_map_to_action(self, joint_map):
        client = get_bridge_client()
        result = client.agent.FetchJointMapToAction(pb.FetchJointMapRequest(
            agent_id=self._agent_id,
            joint_map_json=json.dumps(joint_map),
        ))
        return list(result.values)

    # ------------------------------------------------------------------
    # Streaming helper
    # ------------------------------------------------------------------
    def subscribe_state_stream(self, callback, stop_flag):
        """gRPC 서버 스트리밍을 구독하여 상태를 콜백으로 전달.
        subscribe_robot_topic의 대체.

        Args:
            callback: callable(state_dict)
            stop_flag: dict with 'stop' key
        """
        client = get_bridge_client()
        try:
            stream = client.agent.SubscribeRobotState(
                pb.SubscribeRequest(agent_id=self._agent_id)
            )
            for state in stream:
                if stop_flag.get('stop'):
                    break
                state_dict = {
                    'joint_states': list(state.joint_states) if state.joint_states else None,
                    'joint_actions': list(state.joint_actions) if state.joint_actions else None,
                    'ee_pos': json.loads(state.ee_pos_json) if state.ee_pos_json else None,
                    'ee_target': json.loads(state.ee_target_json) if state.ee_target_json else None,
                    'connected': state.connected,
                }
                # 로컬 캐시 업데이트
                self.joint_states = state_dict['joint_states']
                self.joint_actions = state_dict['joint_actions']
                self.ee_pos = state_dict['ee_pos']
                self.ee_target = state_dict['ee_target']
                if state_dict['connected']:
                    self.last_joint_update = time.time()

                callback(state_dict)
        except Exception as e:
            if not stop_flag.get('stop'):
                print(f"[RemoteAgent] State stream error: {e}")
