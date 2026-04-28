"""Franka Panda용 TrajectoryBridge 구현."""

from isaac_robot_control.core.bridge import TrajectoryBridge


class FrankaTrajectoryBridge(TrajectoryBridge):
    """Franka Panda arm + hand controller 브릿지."""

    @property
    def controller_topics(self) -> list[str]:
        return [
            "/panda_arm_controller/follow_joint_trajectory",
            "/panda_hand_controller/follow_joint_trajectory",
        ]
