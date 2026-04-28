"""Pick-and-Place Task 구현.

오브젝트를 잡아서 목표 위치에 놓는 작업.
pick_object, place_target을 인자로 받아 범용적으로 사용 가능.
"""

from __future__ import annotations

import math
import rclpy
from typing import TYPE_CHECKING
from scipy.spatial.transform import Rotation as R

from isaac_control_core.core.task import BaseTask
from isaac_control_core.utils.skills import RobotSkills

if TYPE_CHECKING:
    from isaac_control_core.core.controller import RobotController


# 높이 오프셋 (오브젝트 z 좌표에 더해짐, ee_frame 기준)
APPROACH_OFFSET = 0.1
GRASP_OFFSET = 0.01
LIFT_OFFSET = 0.1
PLACE_OFFSET = 0.08


class PickAndPlaceTask(BaseTask):
    """Pick-and-Place Task.

    Args:
        controller: 로봇 컨트롤러.
        pick_object: 잡을 오브젝트 이름 (Marker ns).
        place_target: 놓을 목표 오브젝트 이름 (Marker ns).
    """

    def __init__(
        self,
        controller: RobotController,
        pick_object: str,
        place_target: str,
        grasp_yaw: str | float = "auto",
    ):
        """
        Args:
            grasp_yaw: 그리퍼 yaw 설정.
                - "auto": 오브젝트 yaw에 자동 정렬
                - "vertical" 또는 90: 세로(90도) 방향으로 잡기
                - "horizontal" 또는 0: 가로(0도) 방향으로 잡기 (기본 grasp)
                - float: 지정한 각도(도)만큼 회전하여 잡기
        """
        super().__init__(controller)
        self._pick_object = pick_object
        self._place_target = place_target
        self._grasp_yaw = grasp_yaw

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
        """Pick-and-Place 전체 시퀀스 실행."""
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
        self.logger.info("  Pick-and-Place 시작")
        self.logger.info(f"  {self._pick_object}:    ({cx:.3f}, {cy:.3f}, {cz:.3f})")
        self.logger.info(f"  {self._place_target}: ({px:.3f}, {py:.3f}, {pz:.3f})")
        self.logger.info("=" * 50)

        # 현재 arm 관절 위치 저장 (마지막에 복귀용)
        initial_joints = skills.get_current_arm_joints()
        self.logger.info(f"  초기 관절: {initial_joints}")

        # grasp yaw 계산
        pick_ori_kwargs = {}
        grasp_yaw_rad = None

        if self._grasp_yaw == "auto":
            # 오브젝트 yaw에 자동 정렬
            obj_ori = self._controller.object_orientations.get(self._pick_object)
            if obj_ori:
                ox, oy, oz, ow = obj_ori
                rot = R.from_quat([ox, oy, oz, ow])
                local_x_in_world = rot.apply([1, 0, 0])
                grasp_yaw_rad = math.atan2(local_x_in_world[1], local_x_in_world[0])
        elif self._grasp_yaw == "horizontal":
            grasp_yaw_rad = 0.0
        elif self._grasp_yaw == "vertical":
            grasp_yaw_rad = math.pi / 2
        elif isinstance(self._grasp_yaw, (int, float)):
            grasp_yaw_rad = math.radians(float(self._grasp_yaw))

        if grasp_yaw_rad is not None:
            # 평행 그리퍼: yaw와 yaw+180°는 동일 grasp → [-90°, 90°]로 정규화
            while grasp_yaw_rad > math.pi / 2:
                grasp_yaw_rad -= math.pi
            while grasp_yaw_rad < -math.pi / 2:
                grasp_yaw_rad += math.pi

            # top-down(Y축 180°) + Z축 yaw (gripper Z가 반전이므로 부호 반전)
            aligned_rot = R.from_euler('YZ', [math.pi, -grasp_yaw_rad])
            aq = aligned_rot.as_quat()  # xyzw

            pick_ori_kwargs = {"ox": aq[0], "oy": aq[1], "oz": aq[2], "ow": aq[3]}
            self.logger.info(f"  [grasp] yaw={math.degrees(grasp_yaw_rad):.1f}°, "
                             f"ori=({aq[0]:.3f}, {aq[1]:.3f}, {aq[2]:.3f}, {aq[3]:.3f})")

        # Pick: approach → descend → close_gripper → lift
        self.logger.info(f"\n[Pick] {self._pick_object}")
        if not skills.pick(cx, cy, cz,
                           approach_offset=APPROACH_OFFSET,
                           grasp_offset=GRASP_OFFSET,
                           lift_offset=LIFT_OFFSET,
                           **pick_ori_kwargs):
            self.logger.error("Pick 실패!")
            return False

        # Pick 후 초기 자세로 복귀 (안전한 이송 경로 확보)
        self.logger.info("\n[이송] 초기 자세로 복귀")
        skills.go_to_joints(initial_joints)

        # Place: approach → descend → open_gripper → retreat
        self.logger.info(f"\n[Place] {self._place_target}")
        if not skills.place(px, py, pz,
                            approach_offset=APPROACH_OFFSET,
                            place_offset=PLACE_OFFSET,
                            lift_offset=LIFT_OFFSET):
            self.logger.error("Place 실패!")
            return False

        # 초기 위치 복귀
        self.logger.info("\n[완료] 초기 위치로 복귀")
        skills.go_to_joints(initial_joints)

        self.logger.info("\n" + "=" * 50)
        self.logger.info("  Pick-and-Place 모션 완료!")
        self.logger.info("=" * 50)
        return True

    def evaluate(self) -> bool:
        """pick_object가 place_target 위에 있는지 판정.

        XY 거리가 place_target 크기 이내이고,
        pick_object Z가 place_target Z보다 위에 있으면 성공.
        """
        ctrl = self._controller

        # 최신 위치 수신을 위해 잠시 spin
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

        # place_target 반경 이내 + pick_object가 위에 있어야 성공
        xy_threshold = 0.08  # 8cm 이내
        success = xy_dist < xy_threshold and z_diff > 0
        return success
