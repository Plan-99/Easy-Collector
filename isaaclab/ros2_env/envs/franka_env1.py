"""
Franka Panda + 테이블 + 오브젝트 환경.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/franka_env1.py
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv


class FrankaEnv1(BaseEnv):

    def setup_scene(self):
        self.add_robot("franka")

        self.add_table(
            center=[0.32, 0.0, 0.02],
            scale=[0.8, 0.6, 0.05],
        )

        self.add_object("RedCube", size=0.03, color=(1.0, 0.3, 0.2), shape="cube")
        self.add_object("GreenCube", size=0.05, color=(0.2, 0.9, 0.3), shape="cube")
        self.add_object("BlueSphere", size=0.05, color=(0.2, 0.4, 1.0), shape="sphere")
        self.add_object("WhitePlate", size=0.12, color=(0.95, 0.95, 0.95), shape="cylinder", height=0.015)

        self.add_camera(
            position=[2.9, 0.0, 2.3],
            orientation=[0.0, 40.0, 180.0],
        )


if __name__ == "__main__":
    FrankaEnv1(headless=False).run()
