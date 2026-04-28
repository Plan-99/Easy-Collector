"""
AgileX Piper V100 (넓은 그리퍼) + 테이블 + Custom USD 오브젝트 환경.

Usage:
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/pi0_env.py
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/pi0_env.py --headless
    /workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/pi0_env.py --standard  # 표준 Piper 사용
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_env.base_env import BaseEnv

# Piper URDF/메쉬 경로
PIPER_PKG = Path(__file__).resolve().parents[2] / "ros_pkgs/piper_description"
PIPER_URDF_V100 = str(PIPER_PKG / "urdf/piper_description_v100.urdf")
PIPER_URDF_STD = str(PIPER_PKG / "urdf/piper_description.urdf")

# Custom USD 경로
USD_DIR = Path(__file__).resolve().parents[2] / "custom_usds/usd"


class Pi0Env(BaseEnv):

    def __init__(self, headless=False, use_v100=True):
        self._use_v100 = use_v100
        super().__init__(headless=headless)

    def setup_scene(self):
        self.add_table(
            center=[0.365, 0.0, 0.012],
            scale=[0.9, 1.4, 0.04],
            color=(0.03, 0.03, 0.03),
            spawn_range=[[0.43, 0.18], [0.18, -0.18]],
        )

        # 테이블 표면 z = 0.012 + 0.04/2 = 0.032
        urdf = PIPER_URDF_V100 if self._use_v100 else PIPER_URDF_STD
        self.add_robot_from_urdf(
            name="piper",
            urdf_path=urdf,
            prim_path="/World/Piper",
            position=[0.0, 0.0, 0.032],
            color=(0.01, 0.01, 0.01),
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
        # WhitePlate를 먼저 배치 (큰 오브젝트 우선 → 겹침 방지)
        self.add_object("WhitePlate", size=0.22, color=(0.95, 0.95, 0.95), shape="cylinder", height=0.01,
                position_range=[[0.2, 0.0, 0.052], [0.50, 0.35, 0.052]])
        
        # 큐브들은 WhitePlate 배치 후 겹치지 않는 곳에 자동 배치
        self.add_object("RedCube", size=0.05, color=(0.8, 0.0, 0.0), shape="cube",
                position_range=[[0.20226725928386902, -0.1331026392452162, 0.05699997395277153],[0.3164861738692305, 0.04731225567373524, 0.05699997395277101]],
                orientation_range=[[0, 0, -45], [0, 0, 45]])
        self.add_object("BlueCube", size=0.05, color=(0.0, 0.03, 0.1), shape="cube",
                position_range=[[0.2, -0.35, 0.052], [0.50, 0, 0.052]],
                orientation_range=[[0, 0, -45], [0, 0, 45]])
        self.add_object("GreenCube", size=0.05, color=(0.0, 0.03, 0.0), shape="cube",
                position_range=[[0.2, -0.35, 0.052], [0.50, 0, 0.052]],
                orientation_range=[[0, 0, -45], [0, 0, 45]])
        self.add_object("AlienDoll", usd_path=str(USD_DIR / "alien_doll.usd"), size=0.08, scale=0.1,
                        position_range=[[0.2, -0.35, 0.052], [0.5, 0, 0.052]], orientation_range=[[0, 0, -135], [0, 0, -45]],
                        friction=3.0, mass=0.02)
        
        self.add_bg_object("Clamp_bg", usd_path=str(USD_DIR / "clamp_for_robot.usd"),
                           position=(-0.108, 0.00741, -0.02808), orientation=(-25.213, -75.653, -117.797), scale=0.6)
        self.add_bg_object("GreyShelf", box_size=(0.3, 0.24, 0.02),
                           position=(0.00943, -0.003, 0.0272))

        # 고정 카메라 (외부 시점)
        self.add_camera(
            position=[0.42129395649021645, -1.6332931339129597, 1.3261807891504858],
            orientation=[51.059105, 1.2660791, 3.953346],
            position_delta=[0.03, 0.03, 0.03],
            orientation_delta=[1, 1, 1],
            name="side_cam",
        )

        self.add_camera(
            position=[0.8889916444797262, 0.08290099598718931, 2.0569620163434847],
            orientation=[16.736338, 2.5316212, 89.90968],
            position_delta=[0.03, 0.03, 0.03],
            orientation_delta=[1, 1, 1],
            name="top_cam",
        )

        # 손목 부착 카메라 (V100은 gripper_base 없음 → link6에 부착)
        link = "link6" if self._use_v100 else "gripper_base"
        wrist_parent = f"{self._robot_prim_path}/{link}"
        self.add_camera(
            position=[0.012212855875360258, 0.1438242841130981, 0.07214914261067958],
            orientation=[128.89171, 0.80611223, 176.2427],
            parent_prim=wrist_parent,
            name="wrist_cam",
            focal_length=13.0,  # 초광각 (~120도 FOV)
        )

        # 조명 도메인 랜덤화
        self.set_lighting_randomization(
            intensity_range=[1300, 2000],
            color_temp_range=[5500, 6500],
            additional_lights=2,
            additional_intensity_range=[200, 800],
        )

        # 배경 색상 랜덤화 (검정~진한 회색)
        self.set_background_randomization(brightness_range=[0.0, 0.3])


if __name__ == "__main__":
    headless = "--headless" in sys.argv
    use_v100 = "--standard" not in sys.argv  # 기본: V100, --standard로 표준 Piper
    Pi0Env(headless=headless, use_v100=use_v100).run()
