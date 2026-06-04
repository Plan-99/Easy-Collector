import numpy as np
from .ik_solver import IK_Solver
import os
from pathlib import Path

cwd = os.getcwd()  # 예: /root/src
parent_of_cwd = os.path.dirname(cwd)  # 예: /root

class Common_ArmIK(IK_Solver):
    def __init__(self, urdf_path=None, urdf_package_dir=None, joints_to_lock=None,
                 ee_definitions=None, gravity_compensate=0.0,
                 position_cost=1.0, orientation_cost=1.0):

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
            'trans': 10000.0,  # 매우 높음
            'rot': 5000.0,    # 높음
            'reg': 0.01,       # 매우 낮음 (0은 아니되, 방해 안 될 정도)
            'smooth': 1.0     # 낮음
        }

        self.gravity_compensate = gravity_compensate

        print(f'[IK_Solver] Initialized with urdf_path={urdf_path}, package_dir={package_dir}, joints_to_lock={joints_to_lock}, ee_definitions={ee_definitions}, gravity_compensate={gravity_compensate}, position_cost={position_cost}, orientation_cost={orientation_cost}')

        # 4. 부모 클래스 생성자 호출
        super().__init__(
            urdf_path=urdf_path,
            package_dir=package_dir,
            joints_to_lock=joints_to_lock,
            ee_definitions=ee_definitions,
            cost_weights=cost_weights,
            use_scaling=False,
            Visualization=False,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
        )

    def solve_ik(self, target_poses: dict, **kwargs):
        if self.gravity_compensate != 0.0:
            target_poses = {
                name: [pose[0], pose[1], pose[2] + self.gravity_compensate] + list(pose[3:])
                for name, pose in target_poses.items()
            }
        return super().solve_ik(target_poses, **kwargs)
