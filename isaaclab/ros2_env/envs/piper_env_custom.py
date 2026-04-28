"""
AgileX Piper + 테이블 + Custom USD 오브젝트 환경.

custom_usds/usd 디렉토리의 USD 에셋을 오브젝트로 사용.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/piper_env_custom.py
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

# Piper URDF/메쉬 경로
PIPER_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/piper_description"
PIPER_URDF = str(PIPER_PKG / "urdf/piper_description.urdf")

# Custom USD 경로
USD_DIR = Path(__file__).resolve().parents[2] / "custom_usds/usd"


class PiperEnvCustom(BaseEnv):

    def setup_scene(self):
        self.add_table(
            center=[0.365, 0.0, 0.012],
            scale=[0.9, 1.0, 0.04],
            color=(0.5, 0.2, 0.1),
            spawn_range=[[0.43, 0.18], [0.18, -0.18]],
        )

        # 테이블 표면 z = 0.012 + 0.04/2 = 0.032
        self.add_robot_from_urdf(
            name="piper",
            urdf_path=PIPER_URDF,
            prim_path="/World/Piper",
            position=[0.0, 0.0, 0.032],
            color=(0.1, 0.1, 0.1),
        )
        self.add_mimic_joint("joint8", "joint7", multiplier=-1.0)

        # position_range: [[x_min, y_min, z], [x_max, y_max, z]]
        # z = 테이블 표면(0.032) + 오브젝트 반높이
        # self.add_object("RedCube", size=0.05, color=(1.0, 0.3, 0.2), shape="cube",
        #                 position_range=[[0.18, -0.18, 0.047], [0.43, 0.18, 0.047]])
        # self.add_object("GreenCube", size=0.05, color=(0.2, 0.9, 0.3), shape="cube",
        #                 position_range=[[0.18, -0.18, 0.052], [0.43, 0.18, 0.052]])
        # self.add_object("WhitePlate", size=0.12, color=(0.95, 0.95, 0.95), shape="cylinder", height=0.015,
        #                 position_range=[[0.18, -0.18, 0.040], [0.43, 0.18, 0.040]])
        self.add_object("WhiteBox", usd_path=str(USD_DIR / "white_box.usd"), size=0.2, scale=0.6,
                position_range=[[0.18, -0.18, 0.052], [0.43, 0.18, 0.052]],
                orientation_range=[[-90, -45, 180], [-90, 45, 180]],
                collision_approximation="convexDecomposition")

        # 스펀지 — 큐브 범위 안
        self.add_object("BlueSponge", usd_path=str(USD_DIR / "blue_sponge.usd"), size=0.08, scale=0.5,
                        position_range=[[0.18, -0.18, 0.052], [0.43, 0.18, 0.052]],
                        orientation_range=[[90, -10, -10], [90, 10, 10]])
        self.add_object("PinkSponge", usd_path=str(USD_DIR / "pink_sponge.usd"), size=0.08, scale=0.5,
                        position_range=[[0.18, -0.18, 0.052], [0.43, 0.18, 0.052]],
                        orientation_range=[[90, -10, -10], [90, 10, 10]])
        self.add_object("RedSponge", usd_path=str(USD_DIR / "red_sponge.usd"), size=0.08, scale=0.5,
                        position_range=[[0.18, -0.18, 0.052], [0.43, 0.18, 0.052]],
                        orientation_range=[[90, -10, -10], [90, 10, 10]])

        # GreyCube — 큐브 범위 안
        self.add_object("GreyCube", usd_path=str(USD_DIR / "grey_cube.usd"), size=0.08, scale=0.5,
                        position_range=[[0.18, -0.18, 0.052], [0.43, 0.18, 0.052]], color=(0.1, 0.1, 0.1))

        # Mouse, Keyboard — 테이블 위, 큐브 범위 바깥 (x=0.50~0.75)
        self.add_object("Mouse", usd_path=str(USD_DIR / "mouse.usd"), size=0.1, scale=0.5,
                        position_range=[[0.50, -0.18, 0.052], [0.75, 0.18, 0.052]],
                        orientation_range=[[-90, 0, -180], [-90, 0, -180]])
        self.add_object("Keyboard", usd_path=str(USD_DIR / "keyboard.usd"), size=0.2, scale=0.5,
                        position_range=[[0.50, -0.18, 0.052], [0.75, 0.18, 0.052]],
                        orientation_range=[[-90, 0, -180], [-90, 0, -180]])

        # 배경 오브젝트 (물리 없음, 고정 위치)
        # self.add_bg_object("Monitor_bg", usd_path=str(USD_DIR / "monitor.usd"),
        #                    position=(0.4, -0.4, 0.201), orientation=(-90, 0, 180), scale=1.0)
        self.add_bg_object("Clamp_bg", usd_path=str(USD_DIR / "clamp_for_robot.usd"),
                           position=(-0.108, 0.00741, -0.02808), orientation=(-25.213, -75.653, -117.797), scale=0.6)
        self.add_bg_object("GreyShelf", box_size=(0.3, 0.24, 0.02),
                           position=(0.00943, -0.003, 0.0272))

        # 고정 카메라 (외부 시점)
        self.add_camera(
            position=[0.40257, 1.20247, 1.60051],
            orientation=[37.97461, 1.0, 173.42998],
            position_delta=[0.08, 0.05, 0.05],
            orientation_delta=[2.0, 2.0, 5.0],
            name="external_cam",
        )

        # 손목 부착 카메라
        self.add_camera(
            position=[-0.41487, 0.00531, -0.03578],
            orientation=[114, 0, -90],
            parent_prim="/piper/gripper_base",
            name="wrist_cam",
        )

        # 조명 도메인 랜덤화
        self.set_lighting_randomization(
            intensity_range=[800, 2500],
            color_temp_range=[3500, 6500],
            additional_lights=2,
            additional_intensity_range=[200, 800],
        )


if __name__ == "__main__":
    PiperEnvCustom(headless=False).run()
