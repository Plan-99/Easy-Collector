"""Motor-in-Box Task 구현.

오브젝트를 잡아서 박스(바구니) 안에 넣는 작업.
pick 후 EE-오브젝트 상대 오프셋을 계산하여,
그립 위치에 관계없이 정확히 박스 위치에 놓이도록 보정한다.

evaluate: 박스 안에 들어갔으면 성공 (XY 가깝고, pick Z가 place Z보다 높지 않음).
"""

from __future__ import annotations

import math
import rclpy
from typing import TYPE_CHECKING

from isaac_control_core.core.task import BaseTask
from isaac_control_core.utils.skills import RobotSkills

if TYPE_CHECKING:
    from isaac_control_core.core.controller import RobotController


APPROACH_OFFSET = 0.1
GRASP_OFFSET = 0.01
LIFT_OFFSET = 0.1


class MotorInBoxTask(BaseTask):
    """Motor-in-Box Task.

    Args:
        controller: 로봇 컨트롤러.
        pick_object: 잡을 오브젝트 이름 (Marker ns).
        place_target: 넣을 박스 오브젝트 이름 (Marker ns).
        stack_offset: 박스 z 위에 추가할 높이 (기본 0.05m).
    """

    def __init__(
        self,
        controller: RobotController,
        pick_object: str,
        place_target: str,
        stack_offset: float = 0.0,
    ):
        super().__init__(controller)
        self._pick_object = pick_object
        self._place_target = place_target
        self._stack_offset = stack_offset

    @property
    def pick_object(self) -> str:
        return self._pick_object

    @property
    def place_target(self) -> str:
        return self._place_target

    def validate(self) -> bool:
        """pick_object와 place_target의 위치가 수신되었는지 확인."""
        positions = self._controller.object_positions
        if self._pick_object not in positions:
            self.logger.error(f"오브젝트 '{self._pick_object}' 위치를 찾을 수 없습니다.")
            return False
        if self._place_target not in positions:
            self.logger.error(f"오브젝트 '{self._place_target}' 위치를 찾을 수 없습니다.")
            return False
        return True

    def execute(self) -> bool:
        """Motor-in-Box 전체 시퀀스 실행."""
        if not self.validate():
            return False

        # 오브젝트 위치 최신화 (리셋 직후 stale 방지)
        for _ in range(30):
            rclpy.spin_once(self._controller, timeout_sec=0.05)

        skills = RobotSkills(self._controller)

        pick_pos = self._controller.object_positions[self._pick_object]
        place_pos = self._controller.object_positions[self._place_target]

        cx, cy, cz = pick_pos
        px, py, pz = place_pos

        self.logger.info("=" * 50)
        self.logger.info("  Motor-in-Box 시작")
        self.logger.info(f"  {self._pick_object}:    ({cx:.3f}, {cy:.3f}, {cz:.3f})")
        self.logger.info(f"  {self._place_target}: ({px:.3f}, {py:.3f}, {pz:.3f})")
        self.logger.info("=" * 50)

        initial_joints = skills.get_current_arm_joints()
        self.logger.info(f"  초기 관절: {initial_joints}")

        # Pick: approach → descend → close_gripper → lift (작은 lift)
        self.logger.info(f"\n[Pick] {self._pick_object}")
        if not skills.pick(cx, cy, cz,
                           approach_offset=0.1,
                           grasp_offset=GRASP_OFFSET,
                           lift_offset=LIFT_OFFSET):
            self.logger.error("Pick 실패!")
            return False

        # Pick 후 초기 자세로 복귀 (안전한 이송 경로 확보)
        self.logger.info("\n[이송] 초기 자세로 복귀")
        skills.go_to_joints(initial_joints)

        # Place in box (상대좌표 보정, orientation 느슨)
        self.logger.info(f"\n[Place in] {self._place_target}")
        if not skills.place_on_object(
            self._pick_object,
            self._place_target,
            stack_offset=self._stack_offset,
            approach_offset=0.02,
        ):
            self.logger.error("Place 실패!")
            return False

        # 초기 위치 복귀
        self.logger.info("\n[완료] 초기 위치로 복귀")
        skills.go_to_joints(initial_joints)

        self.logger.info("\n" + "=" * 50)
        self.logger.info("  Motor-in-Box 모션 완료!")
        self.logger.info("=" * 50)
        return True

    def evaluate(self) -> bool:
        """pick_object가 place_target(박스) 안에 들어갔는지 판정.

        XY 거리가 8cm 이내이고,
        pick_object Z가 place_target Z보다 많이 높지 않으면 성공.
        (박스 안에 들어가면 pick Z <= place Z)
        """
        ctrl = self._controller

        for _ in range(10):
            rclpy.spin_once(ctrl, timeout_sec=0.1)

        positions = ctrl.object_positions
        if self._pick_object not in positions or self._place_target not in positions:
            self.logger.error("evaluate: 오브젝트 위치를 읽을 수 없습니다.")
            return False

        pick_pos = positions[self._pick_object]
        place_pos = positions[self._place_target]

        dx = pick_pos[0] - place_pos[0]
        dy = pick_pos[1] - place_pos[1]
        xy_dist = math.sqrt(dx * dx + dy * dy)
        z_diff = pick_pos[2] - place_pos[2]

        self.logger.info(f"[판정] {self._pick_object} 위치: "
                         f"({pick_pos[0]:.3f}, {pick_pos[1]:.3f}, {pick_pos[2]:.3f})")
        self.logger.info(f"[판정] {self._place_target} 위치: "
                         f"({place_pos[0]:.3f}, {place_pos[1]:.3f}, {place_pos[2]:.3f})")
        self.logger.info(f"[판정] XY 거리: {xy_dist:.3f}m, Z 차이: {z_diff:.3f}m")

        # 박스 안: XY 가깝고, pick이 place보다 많이 높지 않음
        success = xy_dist < 0.08 and z_diff < 0.03
        return success
