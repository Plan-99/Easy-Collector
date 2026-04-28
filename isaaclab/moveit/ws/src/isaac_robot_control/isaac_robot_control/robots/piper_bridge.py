"""Piper용 TrajectoryBridge 구현.

그리퍼 값 매핑: 실제 로봇 0~0.085 ↔ 시뮬레이션 0~0.035.
MoveIt/외부에서는 실제 로봇 스케일(0~0.085)을 사용하고,
IsaacSim과의 통신에서 자동 변환합니다.
"""

from isaac_robot_control.core.bridge import TrajectoryBridge
from sensor_msgs.msg import JointState


# 그리퍼 값 매핑 상수
GRIPPER_REAL_MAX = 0.085
GRIPPER_SIM_MAX = 0.035
GRIPPER_JOINTS = {"joint7", "joint8"}


def _sim_to_real(value: float) -> float:
    """시뮬레이션 그리퍼 값 → 실제 로봇 값."""
    return value * (GRIPPER_REAL_MAX / GRIPPER_SIM_MAX)


def _real_to_sim(value: float) -> float:
    """실제 로봇 그리퍼 값 → 시뮬레이션 값."""
    return value * (GRIPPER_SIM_MAX / GRIPPER_REAL_MAX)


class PiperTrajectoryBridge(TrajectoryBridge):
    """Piper arm + gripper controller 브릿지."""

    @property
    def controller_topics(self) -> list[str]:
        return [
            "/arm_controller/follow_joint_trajectory",
            "/gripper_controller/follow_joint_trajectory",
        ]

    @property
    def mimic_joints(self) -> dict[str, tuple[str, float, float]]:
        """joint8은 joint7의 mimic (multiplier=-1, offset=0)."""
        return {
            "joint8": ("joint7", -1.0, 0.0),
        }

    def _joint_state_cb(self, msg: JointState):
        """sim → real 변환 후 저장. MoveIt에는 실제 로봇 스케일로 보임."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                val = msg.position[i]
                if name in GRIPPER_JOINTS:
                    val = _sim_to_real(val)
                self._current_positions[name] = val

    def _publish_command(self, traj_names: list[str], traj_positions: list[float]):
        """real → sim 변환 후 publish. MoveIt에서 오는 실제 값을 sim 값으로 변환."""
        # 그리퍼 관절 값을 sim 스케일로 변환
        converted_positions = [
            _real_to_sim(p) if n in GRIPPER_JOINTS else p
            for n, p in zip(traj_names, traj_positions)
        ]

        # 부모 클래스의 publish 로직 사용
        super()._publish_command(traj_names, converted_positions)
