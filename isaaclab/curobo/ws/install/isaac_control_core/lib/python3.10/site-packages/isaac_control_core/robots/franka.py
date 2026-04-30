"""Franka Panda 로봇 설정."""

from isaac_control_core.core.robot import RobotConfig, GripperConfig


class FrankaConfig(RobotConfig):
    """Franka Emika Panda 로봇 설정."""

    @property
    def name(self) -> str:
        return "panda"

    @property
    def arm_joint_names(self) -> list[str]:
        return [
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7",
        ]

    @property
    def home_joints(self) -> dict[str, float]:
        return {
            "panda_joint1": 0.0,
            "panda_joint2": -0.785,
            "panda_joint3": 0.0,
            "panda_joint4": -2.356,
            "panda_joint5": 0.0,
            "panda_joint6": 1.571,
            "panda_joint7": 0.785,
        }

    @property
    def arm_controller_topic(self) -> str:
        return "/panda_arm_controller/follow_joint_trajectory"

    @property
    def gripper(self) -> GripperConfig:
        return GripperConfig(
            joint_names=["panda_finger_joint1", "panda_finger_joint2"],
            open_width=0.04,
            close_width=0.001,
            controller_topic="/panda_hand_controller/follow_joint_trajectory",
        )

    @property
    def base_frame(self) -> str:
        return "panda_link0"

    @property
    def ee_frame(self) -> str:
        return "panda_link8"

    @property
    def move_group_name(self) -> str:
        return "panda_arm"

    @property
    def gripper_length(self) -> float:
        return 0.103
