"""
AgileX Piper + 테이블 + 오브젝트 환경.

URDF를 IsaacSim URDF Importer로 자동 변환하여 로드.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/piper_env1.py
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

# Piper URDF/메쉬 경로
PIPER_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/piper_description"
PIPER_URDF = str(PIPER_PKG / "urdf/piper_description.urdf")
PIPER_MESHES = str(PIPER_PKG / "meshes")


class PiperEnv1(BaseEnv):

    def setup_scene(self):
        self.add_table(
            center=[0.18, 0.0, 0.012],
            scale=[0.6, 0.5, 0.04],
            color=(0.1, 0.1, 0.1),
            spawn_range=[[0.43, 0.18], [0.18, -0.18]],
        )

        # 테이블 표면 z = 0.012 + 0.04/2 = 0.032
        self.add_robot_from_urdf(
            name="piper",
            urdf_path=PIPER_URDF,
            prim_path="/World/Piper",
            position=[0.0, 0.0, 0.032],
            color=(0.01, 0.01, 0.01),
        )
        self.add_mimic_joint("joint8", "joint7", multiplier=-1.0)

        self.add_object("RedCube", size=0.03, color=(1.0, 0.3, 0.2), shape="cube")
        self.add_object("GreenCube", size=0.033, color=(0.2, 0.9, 0.3), shape="cube")

        self.add_camera(
            position=[1.8, 0.0, 1.6],
            orientation=[0.0, 40.0, 180.0],
        )


if __name__ == "__main__":
    PiperEnv1(headless=False).run()
