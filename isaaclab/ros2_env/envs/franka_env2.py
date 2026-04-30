"""
Franka Panda + 테이블 + USD 에셋 오브젝트 환경 예시.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/franka_env2.py
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

# IsaacSim Nucleus 에셋 경로
NUCLEUS = "omniverse://localhost/NVIDIA/Assets/Isaac/4.2"


class FrankaEnv2(BaseEnv):

    def setup_scene(self):
        self.add_robot("franka")

        self.add_table(
            center=[0.32, 0.0, 0.02],
            scale=[0.8, 0.6, 0.05],
        )

        # 프리미티브 오브젝트
        self.add_object("RedCube", size=0.04, color=(1.0, 0.2, 0.2), shape="cube")

        # USD 에셋 오브젝트 (Nucleus 서버)
        self.add_object("YellowBlock", size=0.05,
                        usd_path=f"{NUCLEUS}/Props/Blocks/yellow_block.usd",
                        scale=0.5)
        self.add_object("BlueBlock", size=0.05,
                        usd_path=f"{NUCLEUS}/Props/Blocks/blue_block.usd",
                        scale=0.5)
        self.add_object("DexCube", size=0.06,
                        usd_path=f"{NUCLEUS}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                        scale=0.7)

        self.add_camera(
            position=[2.9, 0.0, 2.3],
            orientation=[0.0, 40.0, 180.0],
        )


if __name__ == "__main__":
    FrankaEnv2(headless=False).run()
