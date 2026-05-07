# -*- coding: utf-8 -*-
"""
Module manifest loader (ros2_bridge 측).

backend 의 module_loader.py 와 같은 manifest (`project/modules/*.json`) 에서
**IK 섹션** 만 뽑아 ROS2 Agent 가 요구하는 형태로 노출한다.

기존 `robot_configs.ROBOT_CONFIGS` (numpy 객체 포함 dict) 를 대체. URDF 경로
+ ik_setting 형태로 반환하여 agent.py 의 `Common_ArmIK(**ik_setting)` 호출이
변경 없이 동작.
"""
import json
import os
import numpy as np


def _modules_dir() -> str:
    root = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
    return os.path.join(root, 'project', 'modules')


def _iter_manifests():
    d = _modules_dir()
    if not os.path.isdir(d):
        return
    for fname in sorted(os.listdir(d)):
        if not fname.endswith('.json'):
            continue
        try:
            with open(os.path.join(d, fname)) as f:
                yield json.load(f)
        except Exception as e:
            print(f"[module_loader] skip {fname}: {e}", flush=True)


def _ik_to_ros2_format(ik: dict) -> dict:
    """JSON 의 ik 섹션 → agent.py 가 받는 dict 형태.

    변환점:
      - ee_definitions: list of dict({name,parent,offset:list|None})
                      → list of tuple(name, parent, np.ndarray|None)
                      (Common_ArmIK 가 받는 historical 시그니처)
      - 그 외 (joints_to_lock, gravity_compensate 등) 은 그대로 통과.
    """
    out = {}
    for k, v in ik.items():
        if k == 'ee_definitions':
            converted = []
            for d in (v or []):
                offset = d.get('offset')
                if isinstance(offset, list):
                    offset = np.array(offset).T
                converted.append((d.get('name'), d.get('parent'), offset))
            out[k] = converted
        elif k in ('urdf_path', 'urdf_package_dir'):
            continue  # 분리해서 별도 키로
        else:
            out[k] = v
    return out


def get_robot_config(robot_type: str) -> dict | None:
    """기존 robot_configs.get_robot_config 호환.
    Returns:
        {'urdf_path': str, 'urdf_package_dir': str, 'ik_setting': {...}} or None.
    """
    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'robot':
            continue
        for robot in manifest.get('robots') or []:
            if robot.get('type') != robot_type:
                continue
            ik = robot.get('ik')
            if not ik or not ik.get('urdf_path'):
                return None
            return {
                'urdf_path': ik['urdf_path'],
                'urdf_package_dir': ik.get('urdf_package_dir', ''),
                'ik_setting': _ik_to_ros2_format(ik),
            }
    return None


def get_robot_driver_launch(robot_type: str) -> dict | None:
    """주어진 robot_type 의 module.json 에서 driver.launch 블록을 반환.

    스키마:
      {"command": "launch"|"run" (default "launch"),
       "package": str,
       "launch_file": str (command=launch),
       "executable": str (command=run),
       "args": {key: value_template, ...},
       "ros_args": {...} (command=run 전용)}
    값 템플릿은 `{ip_address}`, `{robot_id}`, `{namespace}`, `{key|default}` 같은
    placeholder 를 포함 가능. driver_service 가 settings 와 결합해 치환.
    """
    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'robot':
            continue
        for robot in manifest.get('robots') or []:
            if robot.get('type') != robot_type:
                continue
            launch = (robot.get('driver') or {}).get('launch')
            if isinstance(launch, dict) and launch.get('package'):
                return launch
    return None


def get_robot_driver_hooks(robot_type: str) -> dict | None:
    """robot_type 의 driver pre/post launch 훅을 반환.

    스키마:
      {"pre_launch": [{"type": "script",
                        "path": "can_activate_main.sh",     # 모듈 루트 기준 상대
                        "root": "ros2" | "sdk",             # default: "ros2"
                        "wait_after": 1.0}],
       "post_launch": [{"type": "ros_service",
                         "service": "/jaka_driver/servo_move_enable",
                         "service_type": "jaka_msgs/srv/ServoMoveEnable",
                         "request": {"enable": true},
                         "wait_before": 5.0,
                         "timeout": 5.0}]}

    - pre_launch.script: bash 로 실행. driver 시작 직전.
      `path` 가 / 로 시작하면 absolute, 아니면 root 기준 상대.
        root="ros2" → /root/ros2_ws/src/<module_id>/<path>
        root="sdk"  → /root/robot_sdk/<module_id>/<path>
    - post_launch.ros_service: ROS 2 서비스 호출. driver 시작 직후.

    반환값에 `module_id` 가 포함되어 path resolution 에 사용된다.
    """
    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'robot':
            continue
        for robot in manifest.get('robots') or []:
            if robot.get('type') != robot_type:
                continue
            drv = robot.get('driver') or {}
            pre = drv.get('pre_launch') or []
            post = drv.get('post_launch') or []
            if pre or post:
                return {
                    'pre_launch': pre,
                    'post_launch': post,
                    'module_id': manifest.get('id', ''),
                }
    return None


def get_sensor_driver_launch(sensor_type: str) -> dict | None:
    """sensor_type 의 module.json 에서 driver.launch 블록을 반환.

    sensor manifest 는 `sensors[]` 배열을 가진다 (robots[] 와 같은 형태).
    """
    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'sensor':
            continue
        for sensor in manifest.get('sensors') or []:
            if sensor.get('type') != sensor_type:
                continue
            launch = (sensor.get('driver') or {}).get('launch')
            if isinstance(launch, dict) and launch.get('package'):
                return launch
    return None
