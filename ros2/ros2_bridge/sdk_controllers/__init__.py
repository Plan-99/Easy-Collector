# -*- coding: utf-8 -*-
"""
SDK Controller factory.
로봇 SDK 컨트롤러를 타입별로 생성한다.

사용자 모듈은 `ros2/robot_sdk/<id>/controller.py` 안에 BaseSDKController 서브클래스를
넣으면 자동 인식된다. 모듈 이름이 sdk_type과 같거나, 파일에 `SDK_TYPE = "..."` 또는
클래스에 `sdk_type = "..."` 어트리뷰트가 있으면 매칭.
"""
import os
import sys
import importlib.util
from typing import Optional, Type

from . import base as _base_mod
from .base import BaseSDKController

# 사용자 controller.py가 `from base import BaseSDKController` 로 임포트해도
# dispatcher가 보는 것과 동일한 클래스 객체가 되도록 sys.modules에 alias 등록.
# (alias가 없으면 별도 모듈 객체가 만들어져 issubclass 검사가 실패한다.)
sys.modules.setdefault('base', _base_mod)


_BUILTIN: dict[str, str] = {
    'piper': 'piper_controller.PiperSDKController',
    'fairino': 'fairino_controller.FairinoSDKController',
}


def create_sdk_controller(sdk_type: str, config: dict) -> BaseSDKController:
    """sdk_type에 맞는 SDK 컨트롤러 인스턴스를 생성한다."""
    cls = _resolve_builtin(sdk_type) or _resolve_user_module(sdk_type)
    if cls is None:
        raise ValueError(f"Unknown SDK type: {sdk_type}")
    return cls(config)


def _resolve_builtin(sdk_type: str) -> Optional[Type[BaseSDKController]]:
    spec = _BUILTIN.get(sdk_type)
    if not spec:
        return None
    mod_name, cls_name = spec.split('.')
    mod = importlib.import_module(f'.{mod_name}', package=__package__)
    return getattr(mod, cls_name)


def _user_sdk_roots() -> list[str]:
    """사용자 SDK 컨트롤러가 들어갈 수 있는 후보 디렉터리들."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # 호스트 dev 트리: ros2/ros2_bridge/sdk_controllers → ../../robot_sdk
        os.path.normpath(os.path.join(here, '..', '..', 'robot_sdk')),
        # 컨테이너 마운트
        '/root/robot_sdk',
        # 영속 호스트 경로
        '/opt/easytrainer/project/ros2/robot_sdk',
    ]
    return [p for p in candidates if os.path.isdir(p)]


def _resolve_user_module(sdk_type: str) -> Optional[Type[BaseSDKController]]:
    """`<root>/<id>/controller.py`를 스캔해 sdk_type이 일치하는 BaseSDKController 서브클래스를 반환."""
    seen: set[str] = set()
    for root in _user_sdk_roots():
        try:
            for entry in os.listdir(root):
                ctrl_path = os.path.join(root, entry, 'controller.py')
                if entry in seen or not os.path.isfile(ctrl_path):
                    continue
                seen.add(entry)
                cls = _load_controller(ctrl_path, entry, sdk_type)
                if cls is not None:
                    return cls
        except OSError:
            continue
    return None


def _load_controller(path: str, module_dir: str, sdk_type: str) -> Optional[Type[BaseSDKController]]:
    """controller.py 모듈을 임포트하고 sdk_type에 매칭되는 클래스를 찾는다."""
    sdk_root = os.path.dirname(os.path.dirname(path))   # robot_sdk
    pkg_dir = os.path.dirname(path)                     # robot_sdk/<id>
    here = os.path.dirname(os.path.abspath(__file__))   # ros2_bridge/sdk_controllers
    # 사용자 controller.py에서 `from base import BaseSDKController` 형태로
    # 임포트할 수 있도록 sdk_controllers/ 자체도 sys.path에 추가한다.
    for p in (here, sdk_root, pkg_dir):
        if p not in sys.path:
            sys.path.insert(0, p)

    spec_name = f'_easytrainer_user_sdk__{module_dir}'
    spec = importlib.util.spec_from_file_location(spec_name, path)
    if spec is None or spec.loader is None:
        return None
    try:
        module = importlib.util.module_from_spec(spec)
        # 사용자가 import 없이 BaseSDKController를 직접 참조해도 동작하도록 주입.
        module.__dict__.setdefault('BaseSDKController', BaseSDKController)
        sys.modules[spec_name] = module
        spec.loader.exec_module(module)
    except Exception as e:
        print(f"[sdk_controllers] failed to import {path}: {e}", flush=True)
        return None

    declared = getattr(module, 'SDK_TYPE', None) or module_dir
    if declared != sdk_type:
        return None

    for attr in dir(module):
        obj = getattr(module, attr)
        if (isinstance(obj, type)
                and issubclass(obj, BaseSDKController)
                and obj is not BaseSDKController):
            return obj
    return None
