# -*- coding: utf-8 -*-
"""
ROS2 컨테이너 전용 로봇 설정 — 호환성 wrapper.

이전엔 in-code dict (`ROBOT_CONFIGS`) 로 빌트인 9 로봇의 URDF/IK 정보를 들고
있었으나, 이제는 `module_loader.get_robot_config(robot_type)` 가 single source
of truth (project/modules/*.json) 에서 직접 가져온다.

이 파일은 레거시 import (`from ..configs.robot_configs import get_robot_config`)
호환을 위해 wrapper 만 제공.
"""
from .module_loader import get_robot_config  # noqa: F401  (re-exported)
