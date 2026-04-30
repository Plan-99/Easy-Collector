"""로봇 설정 추상 클래스."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class GripperConfig:
    """그리퍼 관절 설정."""
    joint_names: list[str]
    open_width: float
    close_width: float
    controller_topic: str


@dataclass
class RobotConfig(ABC):
    """로봇 설정 추상 베이스 클래스.

    새 로봇을 추가하려면 이 클래스를 상속하고 모든 필드를 정의하세요.
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """로봇 이름 (예: 'panda')."""

    @property
    @abstractmethod
    def arm_joint_names(self) -> list[str]:
        """Arm 관절 이름 리스트 (순서 중요)."""

    @property
    @abstractmethod
    def home_joints(self) -> dict[str, float]:
        """홈 포지션 관절 값."""

    @property
    @abstractmethod
    def arm_controller_topic(self) -> str:
        """Arm FollowJointTrajectory 액션 토픽."""

    @property
    @abstractmethod
    def gripper(self) -> GripperConfig:
        """그리퍼 설정."""

    @property
    @abstractmethod
    def base_frame(self) -> str:
        """베이스 프레임 ID (예: 'panda_link0')."""

    @property
    @abstractmethod
    def ee_frame(self) -> str:
        """엔드이펙터 프레임 ID (예: 'panda_link8')."""

    @property
    @abstractmethod
    def move_group_name(self) -> str:
        """MoveIt2 그룹 이름 (예: 'panda_arm')."""

    @property
    @abstractmethod
    def gripper_length(self) -> float:
        """엔드이펙터 → 그리퍼 끝 오프셋."""

    @property
    def grasp_orientation(self) -> tuple[float, float, float, float] | None:
        """그래스프 시 EE orientation (x, y, z, w). None이면 자동 감지."""
        return None
