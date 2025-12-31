import numpy as np
from .ik_solver import IK_Solver
import os
from pathlib import Path

cwd = os.getcwd()  # 예: /root/src
parent_of_cwd = os.path.dirname(cwd)  # 예: /root

class Common_ArmIK(IK_Solver):
    def __init__(self, urdf_path=None, urdf_package_dir=None, joints_to_lock=None, ee_definitions=None):

        base = Path(parent_of_cwd)

        def _normalize(p: str) -> Path:
            return Path(p) if os.path.isabs(p) else (base / p.lstrip("/"))

        # Avoid double-prefixing when absolute paths are provided via config
        if urdf_path:
            urdf_path = str(_normalize(urdf_path))

        package_dir = None
        if urdf_package_dir:
            package_dirs = []
            items = urdf_package_dir if isinstance(urdf_package_dir, (list, tuple)) else [urdf_package_dir]
            for item in items:
                p = _normalize(str(item))
                package_dirs.append(str(p))
                package_dirs.append(str(p.parent))
            # Deduplicate while preserving order
            seen = set()
            package_dir = []
            for p in package_dirs:
                if p not in seen:
                    seen.add(p)
                    package_dir.append(p)
        
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
