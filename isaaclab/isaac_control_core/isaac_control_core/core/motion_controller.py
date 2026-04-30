"""모션 컨트롤러 추상 베이스 클래스.

모든 모션 플래너(MoveIt, cuRobo 등)가 구현해야 하는 공통 인터페이스.
ROS2 노드, TF, 관절 상태, 오브젝트 추적 등 공통 기능을 제공하고,
실제 모션 플래닝/실행은 서브클래스에서 구현한다.
"""

from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage

from isaac_control_core.core.robot import RobotConfig


class MotionController(Node, ABC):
    """모션 컨트롤러 추상 베이스.

    공통 기능:
    - ROS2 노드 (TF, joint_states, object_markers 구독)
    - 현재 EE orientation 조회
    - 오브젝트 위치 추적

    서브클래스 구현 필수:
    - move_to_joint(), move_to_pose(), move_linear(), set_gripper()
    - _wait_for_planner() (플래너별 초기화 대기)

    Args:
        robot_config: 로봇 설정 (관절 이름, 토픽, 프레임 등).
        object_names: 추적할 오브젝트 이름 리스트 (Marker 토픽 구독).
        node_name: ROS2 노드 이름.
    """

    def __init__(
        self,
        robot_config: RobotConfig,
        object_names: list[str] | None = None,
        node_name: str = "motion_controller",
    ):
        super().__init__(node_name)
        self._robot_config = robot_config

        self._cb_group = ReentrantCallbackGroup()

        # TF 리스너 (현재 EE 자세 조회용)
        self._tf_buffer = Buffer()
        self.create_subscription(
            TFMessage, "/simulation/tf",
            lambda msg: [self._tf_buffer.set_transform(t, "simulation") for t in msg.transforms],
            10, callback_group=self._cb_group,
        )
        self.create_subscription(
            TFMessage, "/simulation/tf_static",
            lambda msg: [self._tf_buffer.set_transform_static(t, "simulation") for t in msg.transforms],
            10, callback_group=self._cb_group,
        )

        # 현재 관절 상태
        self._current_joint_state = None
        self.create_subscription(
            JointState, "/simulation/joint_states", self._joint_state_cb, 10,
            callback_group=self._cb_group,
        )

        # 오브젝트 위치 + orientation (Marker 구독)
        self._object_positions: dict[str, tuple[float, float, float]] = {}
        self._object_orientations: dict[str, tuple[float, float, float, float]] = {}  # xyzw
        for name in (object_names or []):
            self.create_subscription(
                Marker, f"/simulation/object_markers/{name}", self._marker_cb, 10,
                callback_group=self._cb_group,
            )

        self._tracked_objects = list(object_names or [])

    @property
    def robot_config(self) -> RobotConfig:
        return self._robot_config

    @property
    def object_positions(self) -> dict[str, tuple[float, float, float]]:
        return self._object_positions

    @property
    def object_orientations(self) -> dict[str, tuple[float, float, float, float]]:
        """오브젝트 orientation (xyzw quaternion)."""
        return self._object_orientations

    # ── 콜백 ──────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        self._current_joint_state = msg

    def _marker_cb(self, msg: Marker):
        self._object_positions[msg.ns] = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self._object_orientations[msg.ns] = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    # ── 공통 대기 ─────────────────────────────────────────────

    def wait_for_ready(self):
        """모든 필요한 데이터가 수신될 때까지 대기.

        1. 서브클래스의 _wait_for_planner() 호출 (플래너별 서버/서비스 대기)
        2. joint_states 수신 대기
        3. 오브젝트 위치 수신 대기
        """
        self._wait_for_planner()

        self.get_logger().info("joint_states 대기 중...")
        while self._current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.5)

        js = self._current_joint_state
        for i, name in enumerate(js.name):
            self.get_logger().info(f"  [joint_state] {name} = {js.position[i]:.6f}")

        if self._tracked_objects:
            self.get_logger().info(
                f"오브젝트 위치 대기 중 ({', '.join(self._tracked_objects)})..."
            )
            while not all(
                name in self._object_positions for name in self._tracked_objects
            ):
                rclpy.spin_once(self, timeout_sec=0.5)

            for name in self._tracked_objects:
                self.get_logger().info(f"  {name} 위치: {self._object_positions[name]}")

        self.get_logger().info("모든 준비 완료!")

    # ── 공통 헬퍼 ─────────────────────────────────────────────

    def _get_arm_positions(self) -> list[float]:
        """현재 arm 관절 위치를 순서대로 반환."""
        js = self._current_joint_state
        pos_map = {js.name[i]: js.position[i] for i in range(len(js.name))}
        return [pos_map.get(n, 0.0) for n in self._robot_config.arm_joint_names]

    def get_current_ee_orientation(self) -> tuple[float, float, float, float] | None:
        """TF에서 현재 EE 프레임의 orientation (x, y, z, w)을 조회."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._robot_config.base_frame,
                self._robot_config.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            q = t.transform.rotation
            return (q.x, q.y, q.z, q.w)
        except Exception as e:
            self.get_logger().warn(f"EE orientation 조회 실패: {e}")
            return None

    @property
    def gripper_length_override(self) -> float | None:
        """플래너별 gripper_length 오버라이드. None이면 RobotConfig 값 사용."""
        return None

    @property
    def needs_orientation_correction(self) -> bool:
        """move_to_pose 후 orientation 보정이 필요한지 여부.

        MoveIt: True (MoveGroup 폴백 시 orientation 부정확할 수 있음)
        cuRobo: False (move_to_pose가 이미 정확한 orientation으로 플래닝)
        """
        return True

    # ── 서브클래스 구현 필수 ──────────────────────────────────

    @abstractmethod
    def _wait_for_planner(self):
        """플래너별 서버/서비스 대기. wait_for_ready()에서 호출됨."""

    @abstractmethod
    def move_to_joint(self, joint_positions: dict) -> bool:
        """관절 목표 위치로 이동."""

    @abstractmethod
    def move_to_pose(
        self, x: float, y: float, z: float,
        ox: float | None = None, oy: float | None = None,
        oz: float | None = None, ow: float | None = None,
    ) -> bool:
        """포즈 목표로 이동."""

    @abstractmethod
    def move_linear(
        self, x: float, y: float, z: float,
        ox: float | None = None, oy: float | None = None,
        oz: float | None = None, ow: float | None = None,
        velocity_scaling: float = 0.3,
        acceleration_scaling: float = 0.3,
    ) -> bool:
        """직선 경로로 이동."""

    @abstractmethod
    def set_gripper(self, width: float) -> bool:
        """그리퍼를 지정 너비로 이동."""
