# -*- coding: utf-8 -*-
"""
SDK Controller factory.
로봇 SDK 컨트롤러를 sdk_type에 맞춰 동적으로 로드한다.

빌트인 컨트롤러는 두지 않는다. 모든 SDK 컨트롤러는 로봇 모듈 안에
`<module>/sdk/controller.py` 형태로 들어가며, 설치되면
`ros2/robot_sdk/<module_id>/controller.py`에 풀린다.

controller.py에는 `SDK_TYPE = "..."` 상수와 `BaseSDKController` 서브클래스가
있어야 한다. (SDK_TYPE이 없으면 폴더명을 fallback으로 사용한다.)
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


def create_sdk_controller(sdk_type: str, config: dict) -> BaseSDKController:
    """sdk_type에 맞는 SDK 컨트롤러 인스턴스를 생성한다."""
    cls = _resolve(sdk_type)
    if cls is None:
        raise ValueError(f"Unknown SDK type: {sdk_type}")
    return cls(config)


def _controller_dirs() -> list[str]:
    """controller.py가 들어있을 수 있는 디렉터리 후보들.

    설치/런타임 레이아웃과 호스트 dev 트리(SoT) 둘 다 지원한다.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    dirs: list[str] = []

    # 설치 / 런타임: <robot_sdk>/<module_id>/controller.py
    install_roots = [
        os.path.normpath(os.path.join(here, '..', '..', 'robot_sdk')),  # 호스트 dev 트리의 설치 결과
        '/root/robot_sdk',                                              # 컨테이너 마운트
        '/opt/easytrainer/project/ros2/robot_sdk',                      # 호스트 영속 경로
    ]
    for root in install_roots:
        if not os.path.isdir(root):
            continue
        try:
            for entry in os.listdir(root):
                dirs.append(os.path.join(root, entry))
        except OSError:
            continue

    # 호스트 dev 트리 (모듈 SoT): modules/<category>/<id>/sdk/controller.py
    modules_root = os.path.normpath(os.path.join(here, '..', '..', '..', 'modules'))
    if os.path.isdir(modules_root):
        for category in ('robots', 'extensions'):
            cat_dir = os.path.join(modules_root, category)
            if not os.path.isdir(cat_dir):
                continue
            try:
                for entry in os.listdir(cat_dir):
                    dirs.append(os.path.join(cat_dir, entry, 'sdk'))
            except OSError:
                continue

    return dirs


def _resolve(sdk_type: str) -> Optional[Type[BaseSDKController]]:
    """후보 디렉터리에서 SDK_TYPE이 일치하는 BaseSDKController 서브클래스를 찾는다."""
    seen: set[str] = set()
    for pkg_dir in _controller_dirs():
        ctrl_path = os.path.join(pkg_dir, 'controller.py')
        if not os.path.isfile(ctrl_path):
            continue
        key = os.path.realpath(ctrl_path)
        if key in seen:
            continue
        seen.add(key)
        cls = _load_controller(ctrl_path, sdk_type)
        if cls is not None:
            return cls
    return None


def _load_controller(path: str, sdk_type: str) -> Optional[Type[BaseSDKController]]:
    """controller.py 모듈을 임포트하고 sdk_type에 매칭되는 클래스를 찾는다."""
    pkg_dir = os.path.dirname(path)
    sdk_root = os.path.dirname(pkg_dir)
    here = os.path.dirname(os.path.abspath(__file__))   # ros2_bridge/sdk_controllers
    # 사용자 controller.py에서 `from base import BaseSDKController` 형태로
    # 임포트할 수 있도록 sdk_controllers/ 자체도 sys.path에 추가한다.
    for p in (here, sdk_root, pkg_dir):
        if p not in sys.path:
            sys.path.insert(0, p)

    # 같은 spec name이 robot_sdk/와 modules/ dev 트리에서 충돌하지 않도록
    # 디렉터리 경로 일부를 포함한 고유 이름을 만든다.
    tag = '__'.join(os.path.normpath(pkg_dir).strip(os.sep).split(os.sep)[-3:])
    spec_name = f'_easytrainer_user_sdk__{tag}'
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

    declared = getattr(module, 'SDK_TYPE', None) or os.path.basename(pkg_dir)
    if declared != sdk_type:
        return None

    for attr in dir(module):
        obj = getattr(module, attr)
        if (isinstance(obj, type)
                and issubclass(obj, BaseSDKController)
                and obj is not BaseSDKController):
            return obj
    return None
