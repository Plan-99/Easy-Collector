"""Piper 로봇 설정."""

from isaac_control_core.core.robot import RobotConfig, GripperConfig


class PiperConfig(RobotConfig):
    """Piper 6-DOF 로봇 설정 (그리퍼 포함)."""

    @property
    def name(self) -> str:
        return "piper"

    @property
    def arm_joint_names(self) -> list[str]:
        return [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
        ]

    @property
    def home_joints(self) -> dict[str, float]:
        return {
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
            "joint5": 0.0,
            "joint6": 0.0,
        }

    @property
    def arm_controller_topic(self) -> str:
        return "/arm_controller/follow_joint_trajectory"

    @property
    def gripper(self) -> GripperConfig:
        return GripperConfig(
            joint_names=["joint7"],
            open_width=0.085,
            close_width=0.0,
            controller_topic="/gripper_controller/follow_joint_trajectory",
        )

    @property
    def base_frame(self) -> str:
        return "base_link"

    @property
    def ee_frame(self) -> str:
        return "tcp"

    @property
    def move_group_name(self) -> str:
        return "arm"

    @property
    def gripper_length(self) -> float:
        return -0.05

    @property
    def grasp_orientation(self) -> tuple[float, float, float, float]:
        """Top-down 그래스프 orientation (x, y, z, w).

        gripper_base Z축(approach 방향)이 base_link -Z(아래)를 향하도록 하는 쿼터니언.
        Y축 180도 회전: Z=[0,0,-1](아래), Y=[0,1,0](핑거 방향).
        """
        return (0.0, 1.0, 0.0, 0.0)
