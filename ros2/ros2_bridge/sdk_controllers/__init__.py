# -*- coding: utf-8 -*-
"""
SDK Controller factory.
로봇 SDK 컨트롤러를 타입별로 생성한다.
"""
from .base import BaseSDKController


def create_sdk_controller(sdk_type: str, config: dict) -> BaseSDKController:
    """sdk_type에 맞는 SDK 컨트롤러 인스턴스를 생성한다."""
    if sdk_type == 'piper':
        from .piper_controller import PiperSDKController
        return PiperSDKController(config)
    if sdk_type == 'fairino':
        from .fairino_controller import FairinoSDKController
        return FairinoSDKController(config)
    raise ValueError(f"Unknown SDK type: {sdk_type}")
