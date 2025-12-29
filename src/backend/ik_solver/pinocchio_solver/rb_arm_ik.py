import numpy as np
from .ik_solver import IK_Solver
import os
from pathlib import Path

cwd = os.getcwd()  # 예: /root/src
parent_of_cwd = os.path.dirname(cwd)  # 예: /root

class Piper_ArmIK(IK_Solver):
    def __init__(self):

        urdf_path = os.path.join(parent_of_cwd, 'ros2_ws/src/piper_ros/src/piper_description/urdf/piper_description.urdf')
        pdir = Path(parent_of_cwd) / 'ros2_ws/src/piper_ros/src/piper_description/'
        package_dir = [str(pdir), str(pdir.parent)]
        # urdf_path = '../../assets/g1/g1_body23.urdf'
        # package_dir = '../../assets/g1/'

        # 1. 잠글 관절들 정의
        abs_urdf_path = os.path.abspath(urdf_path)
        abs_package_dir = os.path.abspath(package_dir) if package_dir else "None"
        
        joints_to_lock = [
            'joint6_to_gripper_base', 'joint7', 'joint8'
        ]
        # 2. EE 프레임 정의 (기준 관절, 오프셋)
        tool_def = ('ee', 'joint7', None)

        ee_definitions = [tool_def]
        
        # 3. 비용함수 가중치 정의
        cost_weights = {
            'trans': 50,
            'rot': 1.0,
            'reg': 0.02,
            'smooth': 0.1,
            'viz_axis_width': 20
        }

        # 4. 부모 클래스 생성자 호출
        super().__init__(
            urdf_path=urdf_path,
            package_dir=package_dir,
            joints_to_lock=joints_to_lock,
            ee_definitions=ee_definitions,
            cost_weights=cost_weights,
            use_scaling=False,
            Visualization=False
        )
