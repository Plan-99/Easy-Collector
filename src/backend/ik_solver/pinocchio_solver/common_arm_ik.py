import numpy as np
from .ik_solver import IK_Solver
import os # 파일 상단에 import os 추가

import os

cwd = os.getcwd()  # 예: /root/src
parent_of_cwd = os.path.dirname(cwd)  # 예: /root

class Common_ArmIK(IK_Solver):
    def __init__(self, urdf_path=None, urdf_package_dir=None, joints_to_lock=None, ee_definitions=None):

        urdf_path = parent_of_cwd + urdf_path
        package_dir = parent_of_cwd + urdf_package_dir
        
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
