"""
AgileX Piper – Peg-in-Hole 환경.

Piper 로봇이 원통형 Peg를 집어 Hole(소켓) 안에 삽입하는 태스크.

오브젝트 구성:
  - Peg      : 가는 원통 (반지름 6mm, 높이 50mm) — 로봇이 집어야 할 대상
  - HoleBase : 넓고 납작한 판 — 구멍이 있는 고정물(fixture) 역할
  - HoleRing : Peg보다 살짝 큰 링 형태의 짧은 원통 — 삽입 목표 위치 표시

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/piper_peg_in_hole.py
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

# Piper URDF/메쉬 경로
PIPER_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/piper_description"
PIPER_URDF = str(PIPER_PKG / "urdf/piper_description.urdf")
PIPER_MESHES = str(PIPER_PKG / "meshes")


class PiperPegInHole(BaseEnv):

    def setup_scene(self):
        self.add_robot_from_urdf(
            name="piper",
            urdf_path=PIPER_URDF,
            prim_path="/World/Piper",
            color=(0.1, 0.1, 0.1),
        )
        self.add_mimic_joint("joint8", "joint7", multiplier=-1.0)

        # 테이블 — spawn_range를 좁혀서 로봇 작업 영역 내에 오브젝트 배치
        self.add_table(
            center=[0.18, 0.0, 0.012],
            scale=[0.6, 0.5, 0.04],
            spawn_range=[[0.25, -0.12], [0.40, 0.12]],
        )

        # Peg — 가는 원통 (로봇이 잡아서 삽입할 대상)
        self.add_object(
            "Peg",
            size=0.012,       # 지름 12mm
            height=0.05,      # 높이 50mm
            color=(0.8, 0.8, 0.85),  # 은색 메탈릭
            shape="cylinder",
        )

        # HoleBase — 구멍 뚫린 판을 표현하는 넓고 납작한 원통
        self.add_object(
            "HoleBase",
            size=0.08,        # 지름 80mm
            height=0.015,     # 높이 15mm
            color=(0.25, 0.25, 0.3),  # 짙은 회색
            shape="cylinder",
        )

        # HoleRing — 삽입 목표를 시각적으로 표시하는 링 (Peg보다 약간 큰 원통)
        self.add_object(
            "HoleRing",
            size=0.018,       # 지름 18mm (Peg 12mm + 여유 6mm)
            height=0.016,     # HoleBase보다 살짝 높게
            color=(0.9, 0.2, 0.2),  # 빨간색 — 목표 위치 강조
            shape="cylinder",
        )

        # 카메라 — 작업 영역을 내려다보는 위치
        self.add_camera(
            position=[1.8, 0.0, 1.6],
            orientation=[0.0, 40.0, 180.0],
        )


if __name__ == "__main__":
    PiperPegInHole(headless=False).run()
