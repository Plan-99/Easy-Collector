# -*- coding: utf-8 -*-
"""
Robot driver plugin system.

Custom robot 가 interpolation_node 를 사용하지 않을 때 (interpolation=False)
agent 가 호출할 transport 로직. 사용자가 module 안에 driver 코드를 짜서 농축.

빌트인 robot 은 모두 interpolation=True 로 운영하는 것이 권장 정책 — 즉 이
driver path 는 사실상 custom robot 전용이다.

Loader 사용법:
    from ros2_bridge.drivers import load_driver
    driver = load_driver(robot_dict)  # None 이면 transport 미설정
    driver.write_joints(action, vel_arg=...)
    driver.destroy()

Driver entrypoint 는 module.json 에 다음과 같이 명시:
    "driver": {
        "kind": "custom",
        "entrypoint": "driver:MyRobotDriver",
        "config": {"port": 8080}        // optional, __init__ 에 전달
    }

entrypoint 의 file 부분은 module 폴더 (`/opt/easytrainer/modules/robots/<id>/`)
기준 상대 경로 (e.g. "driver" → driver.py). class 부분은 RobotDriver 의
서브클래스명.
"""
from .base import RobotDriver
from .loader import load_driver

__all__ = ['RobotDriver', 'load_driver']
