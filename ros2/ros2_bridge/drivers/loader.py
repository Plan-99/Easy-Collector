# -*- coding: utf-8 -*-
"""Dynamic driver loader.

module.json 에 명시된 entrypoint 를 읽어 사용자 driver 를 importlib 로
로드한다. 빌트인은 모두 interp 로 운영하니 보통 custom robot 만 이 경로를
탄다.

Lookup 경로:
  EASYTRAINER_DATA_DIR/modules/robots/<module_id>/  (보통
  /opt/easytrainer/modules/robots/<module_id>/)

entrypoint 형식: "<module_path>:<class_name>"
  - module_path: 위 경로 기준 .py 파일 (e.g. "driver" → driver.py).
                 sub-folder 도 가능 ("pkg/driver" → pkg/driver.py).
  - class_name: RobotDriver 의 서브클래스명.
"""
import importlib.util
import os
import traceback
from typing import Optional

from .base import RobotDriver


def _modules_root() -> str:
    """module 폴더 루트. backend module_loader 와 같은 방식."""
    root = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
    # 호스트의 modules/robots/ 가 컨테이너에 mount 되는 위치. 마운트 안 되어
    # 있으면 None 으로 떨어져 driver 를 찾지 못하므로 기본 경로도 시도.
    candidates = [
        os.path.join(root, 'modules', 'robots'),
        os.path.join(root, 'project', 'modules', 'robots'),
        '/root/modules/robots',
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    # 마지막 fallback — 정확하지 않을 수 있으나 driver 가 없는 케이스에서는
    # 어차피 None 으로 떨어진다.
    return candidates[0]


def load_driver(robot: dict, node=None) -> Optional[RobotDriver]:
    """robot dict (`driver` 필드 포함) 에서 entrypoint 를 읽어 driver 를 로드.

    조건:
      driver.kind == 'custom' AND driver.entrypoint 가 명시되어 있을 때만 로드.
      그 외 (interp 사용, sdk 모드, 빈 driver) 는 None 반환.

    Args:
        robot: RobotConfig dict (`driver` 키 포함).
        node:  agent 가 들고 있는 rclpy.node.Node — driver __init__ 에 전달.

    Returns:
        RobotDriver instance 또는 None (entrypoint 없거나 import 실패 시).
    """
    driver_meta = (robot.get('driver') if isinstance(robot.get('driver'), dict)
                   else None)
    # 호환: 이전 로직은 driver 필드를 _flatten 으로 풀어 top-level 에 넣었다.
    # 그 경우 robot.get('kind'), robot.get('entrypoint') 를 직접 본다.
    if driver_meta is None:
        if not robot.get('entrypoint'):
            return None
        driver_meta = {
            'kind': robot.get('kind') or 'custom',
            'entrypoint': robot.get('entrypoint'),
            'config': robot.get('driver_config') or {},
        }

    if (driver_meta.get('kind') or '') != 'custom':
        return None
    entrypoint = driver_meta.get('entrypoint') or ''
    if ':' not in entrypoint:
        print(f"[driver_loader] invalid entrypoint '{entrypoint}' — "
              f"format must be '<file>:<class>'")
        return None
    module_path, class_name = entrypoint.split(':', 1)

    module_id = robot.get('module_id') or ''
    if not module_id:
        print(f"[driver_loader] robot {robot.get('name')} has no module_id "
              f"but driver.kind='custom' — module 폴더 위치를 알 수 없음")
        return None

    base = _modules_root()
    candidate_dirs = [
        os.path.join(base, module_id),
        # 'robot_my_arm' 같은 prefix 를 떼서 폴더명 매칭 시도
        os.path.join(base, module_id.split('_', 1)[1]) if '_' in module_id else None,
    ]
    src_path = None
    for d in [c for c in candidate_dirs if c]:
        cand = os.path.join(d, *module_path.split('/')) + '.py'
        if os.path.isfile(cand):
            src_path = cand
            break
    if src_path is None:
        print(f"[driver_loader] cannot locate driver file for entrypoint "
              f"'{entrypoint}' in {candidate_dirs}")
        return None

    try:
        spec = importlib.util.spec_from_file_location(
            f"_easytrainer_driver_{module_id}", src_path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls = getattr(module, class_name, None)
        if cls is None:
            print(f"[driver_loader] class '{class_name}' not found in {src_path}")
            return None
        if not issubclass(cls, RobotDriver):
            print(f"[driver_loader] class '{class_name}' must subclass RobotDriver")
            return None
        return cls(node, robot, driver_meta.get('config') or {})
    except Exception as e:
        print(f"[driver_loader] failed to load {entrypoint} from {src_path}: {e}")
        traceback.print_exc()
        return None
