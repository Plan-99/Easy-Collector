"""FollowJointTrajectory ↔ IsaacSim 브릿지 추상 클래스.

어떤 로봇이든 controller_topics만 정의하면 동일한 브릿지 로직을 재사용할 수 있습니다.
"""

from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import time


class TrajectoryBridge(Node, ABC):
    """FollowJointTrajectory 액션 서버 → /joint_command 퍼블리셔 브릿지.

    서브클래스에서 controller_topics를 정의하면 각 토픽에 대한 액션 서버가 생성됩니다.
    """

    def __init__(self, node_name: str = "trajectory_bridge"):
        super().__init__(node_name)

        self._cb_group = ReentrantCallbackGroup()

        # IsaacSim 관절 명령 퍼블리셔
        self._cmd_pub = self.create_publisher(
            JointState, self.command_topic, 10
        )

        # 현재 관절 상태 구독
        self._current_positions: dict[str, float] = {}
        # 마지막으로 명령한 관절 위치 (publish 시 전체 관절 합성용)
        self._commanded_positions: dict[str, float] = {}
        self.create_subscription(
            JointState, self.state_topic, self._joint_state_cb, 10,
            callback_group=self._cb_group,
        )

        # 각 controller topic에 대해 액션 서버 생성
        self._action_servers: list[ActionServer] = []
        for topic in self.controller_topics:
            server = ActionServer(
                self,
                FollowJointTrajectory,
                topic,
                self._execute_cb,
                callback_group=self._cb_group,
            )
            self._action_servers.append(server)

        self.get_logger().info(f"{self.__class__.__name__} 시작됨")
        for topic in self.controller_topics:
            self.get_logger().info(f"  액션 서버: {topic}")
        self.get_logger().info(f"  퍼블리쉬:  {self.command_topic}")
        self.get_logger().info(f"  구독:      {self.state_topic}")

    @property
    @abstractmethod
    def controller_topics(self) -> list[str]:
        """FollowJointTrajectory 액션 서버를 생성할 토픽 리스트."""

    @property
    def mimic_joints(self) -> dict[str, tuple[str, float, float]]:
        """Mimic joint 매핑: {mimic_joint: (source_joint, multiplier, offset)}.

        서브클래스에서 오버라이드하면 trajectory 명령 시 mimic joint도 함께 전송됩니다.
        """
        return {}

    @property
    def command_topic(self) -> str:
        """IsaacSim에 관절 명령을 보낼 토픽."""
        return "/simulation/joint_command"

    @property
    def state_topic(self) -> str:
        """IsaacSim으로부터 관절 상태를 받을 토픽."""
        return "/simulation/joint_states"

    def _joint_state_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._current_positions[name] = msg.position[i]

    def _append_mimic(self, names: list[str], positions: list[float]) -> tuple[list[str], list[float]]:
        """명령에 mimic joint를 추가."""
        if not self.mimic_joints:
            return names, positions
        src_map = {n: p for n, p in zip(names, positions)}
        extra_names = []
        extra_positions = []
        for mimic_name, (src_name, mult, offset) in self.mimic_joints.items():
            if src_name in src_map and mimic_name not in src_map:
                extra_names.append(mimic_name)
                extra_positions.append(mult * src_map[src_name] + offset)
        return names + extra_names, positions + extra_positions

    def _publish_command(self, traj_names: list[str], traj_positions: list[float]):
        """trajectory 관절 + 나머지 관절을 합쳐서 전체 관절 명령을 publish.

        traj_names/traj_positions: 현재 trajectory에서 제어 중인 관절.
        나머지 관절은 마지막 명령값(_commanded_positions)을 유지.
        """
        # trajectory 관절 → commanded 갱신
        for n, p in zip(traj_names, traj_positions):
            self._commanded_positions[n] = p

        # 전체 관절 합성: commanded + current (최초 명령 전 관절용)
        # mimic joint는 _append_mimic에서 올바른 값으로 추가하므로 여기서 제외
        mimic_names = set(self.mimic_joints.keys())
        merged = {k: v for k, v in self._current_positions.items()
                  if k not in mimic_names}
        merged.update(self._commanded_positions)

        # mimic 관절은 source joint 기반으로 계산하여 추가
        names = list(merged.keys())
        positions = list(merged.values())
        names, positions = self._append_mimic(names, positions)

        cmd = JointState()
        cmd.name = names
        cmd.position = positions
        self._cmd_pub.publish(cmd)

    def _execute_cb(self, goal_handle):
        """FollowJointTrajectory 액션 실행: trajectory 포인트를 순서대로 IsaacSim에 전달."""
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names

        self.get_logger().info(
            f"Trajectory 수신: {len(trajectory.points)} 포인트, "
            f"관절: {joint_names}"
        )

        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = joint_names
        start_time = time.time()

        for i, point in enumerate(trajectory.points):
            target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            while (time.time() - start_time) < target_time:
                elapsed = time.time() - start_time
                if i > 0:
                    prev_time = (
                        trajectory.points[i - 1].time_from_start.sec
                        + trajectory.points[i - 1].time_from_start.nanosec * 1e-9
                    )
                else:
                    prev_time = 0.0

                dt = target_time - prev_time
                alpha = min(1.0, (elapsed - prev_time) / dt) if dt > 0 else 1.0

                if i > 0:
                    prev_positions = list(trajectory.points[i - 1].positions)
                else:
                    prev_positions = [
                        self._current_positions.get(n, 0.0) for n in joint_names
                    ]

                interp_positions = [
                    p + alpha * (t - p)
                    for p, t in zip(prev_positions, point.positions)
                ]

                self._publish_command(joint_names, interp_positions)

                time.sleep(0.01)  # 100Hz

            # 최종 포인트 위치 퍼블리쉬
            self._publish_command(joint_names, list(point.positions))

            # 피드백 전송
            feedback_msg.desired = point
            actual = JointTrajectoryPoint()
            actual.positions = [
                self._current_positions.get(n, 0.0) for n in joint_names
            ]
            feedback_msg.actual = actual
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("Trajectory 실행 완료")
        return result
