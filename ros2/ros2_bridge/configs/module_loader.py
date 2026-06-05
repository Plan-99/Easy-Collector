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


# ---------------------------------------------------------------------------
# Path placeholder substitution: {ros2_root} / {sdk_root}
#
# manifest 의 urdf_path / urdf_package_dir / driver.launch.args 등에서 install
# 폴더의 절대 경로를 module_id 에 의존하지 않고 portable 하게 표기하기 위한 placeholder.
# - {ros2_root} → 실제 ros2 install 폴더 (보통 /root/ros2_ws/src/<id 또는 vendor명>)
# - {sdk_root}  → 실제 sdk install 폴더 (보통 /root/robot_sdk/<id 또는 vendor명>)
# install 폴더 이름이 module_id 와 다를 수 있어 (옛 마켓 모듈 piper 등) 동적 lookup.
# ---------------------------------------------------------------------------

def _resolve_install_root(module_id: str, kind: str = 'ros2') -> str:
    """모듈의 실제 install 폴더 경로. 디스크에서 module.json id 매칭으로 찾고,
    없으면 module_id 를 폴더명으로 가정해 fallback.
    """
    base = '/root/robot_sdk' if kind == 'sdk' else '/root/ros2_ws/src'
    if os.path.isdir(base):
        try:
            for entry in os.listdir(base):
                d = os.path.join(base, entry)
                mj = os.path.join(d, 'module.json')
                if not os.path.isfile(mj):
                    continue
                try:
                    with open(mj) as f:
                        if json.load(f).get('id') == module_id:
                            return d
                except Exception:
                    continue
        except OSError:
            pass
    return os.path.join(base, module_id)


def _substitute_paths(value, ros2_root: str, sdk_root: str):
    """문자열 / dict / list 안의 {ros2_root} / {sdk_root} placeholder 를 재귀 치환."""
    if isinstance(value, str):
        if '{ros2_root}' in value:
            value = value.replace('{ros2_root}', ros2_root)
        if '{sdk_root}' in value:
            value = value.replace('{sdk_root}', sdk_root)
        return value
    if isinstance(value, dict):
        return {k: _substitute_paths(v, ros2_root, sdk_root) for k, v in value.items()}
    if isinstance(value, list):
        return [_substitute_paths(v, ros2_root, sdk_root) for v in value]
    return value


def _resolve_manifest_paths(manifest: dict) -> dict:
    """manifest 전체에 대해 {ros2_root}/{sdk_root} placeholder 치환."""
    mid = manifest.get('id', '')
    if not mid:
        return manifest
    ros2_root = _resolve_install_root(mid, 'ros2')
    sdk_root = _resolve_install_root(mid, 'sdk')
    return _substitute_paths(manifest, ros2_root, sdk_root)


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
        manifest = _resolve_manifest_paths(manifest)
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
        manifest = _resolve_manifest_paths(manifest)
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
        manifest = _resolve_manifest_paths(manifest)
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


def get_robot_driver_remote(robot_type: str) -> dict | None:
    """robot_type 의 driver.remote 블록을 반환 (원격 SSH 드라이버).

    원격 모드: 로봇 자체 PC(온보드 PC)에서 ROS 드라이버를 실행하고 EasyTrainer 는
    같은 DDS 도메인에서 토픽만 주고받는 운용. driver_service 가 settings 의
    host_field(기본 'ssh_host') 가 채워져 있으면 이 블록으로 동작한다.

    스키마:
      {"enabled_when": "ssh_host",            # settings 에 이 키가 차 있으면 원격 모드
       "host_field": "ssh_host",              # 호스트명/IP settings 키
       "user_field": "ssh_user",              # (optional) 사용자 settings 키
       "port_field": "ssh_port",              # (optional) 포트 settings 키
       "password_field": "ssh_password",      # (optional) 비밀번호 settings 키.
                                              #   채워지면 sshpass 로 암호 인증, 비면 키 인증.
                                              #   (ros2 컨테이너에 apt 'sshpass' 필요)
       "default_user": "root",                # user_field 비었을 때
       "default_port": 22,
       "ros_domain_id": 30,                   # (optional) 바깥 ssh 셸에 export
       "read_topic": "/joint_states",         # (optional) 원격 모드 토픽 override
       "write_topic": "/leader/joint_trajectory",
       "write_topic_msg": "trajectory_msgs/JointTrajectory",
       "payload": [{"src": "{ros2_root}/remote/foo", "dst": "/remote/path/foo"}],
       "provision": [{"name": str,
                      "check": "<sh; exit 0 이면 이미 완료 → skip>",   # optional
                      "run": "<sh>",
                      "timeout": 600,                                   # optional, 초
                      "optional": false}],                             # optional, 실패해도 계속
       "launch": "<원격에서 blocking 으로 도는 드라이버 실행 sh 명령>"}

    문자열 값은 {ssh_host}, {robot_id}, {namespace}, {key|default} placeholder 를
    포함 가능 — driver_service 가 settings 와 결합해 치환한다.

    반환값에 `module_id` 가 포함된다 (payload src 의 {ros2_root} 해석에 사용).
    """
    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'robot':
            continue
        manifest = _resolve_manifest_paths(manifest)
        for robot in manifest.get('robots') or []:
            if robot.get('type') != robot_type:
                continue
            remote = (robot.get('driver') or {}).get('remote')
            if isinstance(remote, dict) and (remote.get('launch') or remote.get('provision')):
                out = dict(remote)
                out['module_id'] = manifest.get('id', '')
                return out
    return None


def get_sensor_driver_launch(sensor_type: str) -> dict | None:
    """sensor_type 의 module.json 에서 driver.launch 블록을 반환.

    sensor manifest 는 `sensors[]` 배열을 가진다 (robots[] 와 같은 형태).
    """
    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'sensor':
            continue
        manifest = _resolve_manifest_paths(manifest)
        for sensor in manifest.get('sensors') or []:
            if sensor.get('type') != sensor_type:
                continue
            launch = (sensor.get('driver') or {}).get('launch')
            if isinstance(launch, dict) and launch.get('package'):
                return launch
    return None
