# -*- coding: utf-8 -*-
"""
Module manifest loader (backend 측).

`project/modules/*.json` 의 module manifest 들을 단일 진실원천으로 사용해
빌트인/외부 로봇 정의를 모두 일관된 흐름으로 노출한다. 기존
`global_configs._ALL_ROBOTS` 의 in-code dict 를 대체.

Manifest 스키마 (관련 부분만):
    {
      "id": "robot_piper",
      "category": "robot",
      "robots": [
        {
          "type": "piper",                 # RobotModel.type
          "spec": {company, role, joint_*, tool_*, ik_available, custom_fields},
          "driver": {kind, read_topic, write_topic, ...},
          "ik": {urdf_path, urdf_package_dir, ee_definitions, joints_to_lock}
        },
        ...
      ]
    }

이 로더는 `spec` + `driver` + module-level 메타(`module_id`)를 머지해서 기존
`_ALL_ROBOTS` dict 와 동일한 flat shape 으로 돌려준다. 호출자(예: SUPPORT_ROBOTS,
robot_model.get_robot_type_info)는 코드 변경 없이 그대로 동작.

`ik` 섹션은 backend 에서는 ee_definitions 만 frontend default 표시용으로 노출
(`get_default_ee_definitions`). URDF 로딩/IK 솔버 init 은 ros2 컨테이너의 별도
loader 가 담당.
"""
import json
import os
from typing import Iterable


# 빌트인 로봇 — 정식 `modules/robots/<id>/` 모듈로 모두 흡수돼서 비어 있음.
# (이전에 'test_arm' 이 이 자리에서 카탈로그에 항상 노출되며 옛 extensions/test_arm
#  ROS 패키지로 띄우려고 했지만, 위자드/정식 모듈 시스템과 충돌해 제거.)
_BUILTIN_ROBOTS: list[dict] = []


def _modules_dir() -> str:
    """현재 컨테이너에서 installed manifest 경로. 양쪽 컨테이너에 같은 host 경로
    가 mount 되어 있어 backend/ros2 가 같은 파일을 본다.
    """
    root = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
    return os.path.join(root, 'project', 'modules')


def _iter_manifests() -> Iterable[dict]:
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
            # 깨진 manifest 하나 때문에 전체가 죽지 않도록 skip + log
            print(f"[module_loader] skip {fname}: {e}")


def _resolve_install_root(module_id: str, kind: str = 'ros2') -> str:
    """모듈의 실제 install 폴더 (backend 컨테이너 기준 경로).
    backend 는 host 의 /opt/easytrainer/project/... 를 그대로 보지만, 호환을 위해
    동적 lookup 한 후 못 찾으면 module_id 폴더명을 fallback 으로 사용.
    """
    data_root = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
    if kind == 'sdk':
        base = os.path.join(data_root, 'project', 'ros2', 'robot_sdk')
    else:
        base = os.path.join(data_root, 'project', 'ros2', 'ros2_ws', 'src')
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
    """문자열 / dict / list 안의 {ros2_root} / {sdk_root} 재귀 치환."""
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
    """manifest 전체에 {ros2_root}/{sdk_root} placeholder 치환 적용."""
    mid = manifest.get('id', '')
    if not mid:
        return manifest
    ros2_root = _resolve_install_root(mid, 'ros2')
    sdk_root = _resolve_install_root(mid, 'sdk')
    return _substitute_paths(manifest, ros2_root, sdk_root)


def _flatten(robot_entry: dict, module_id: str | None) -> dict:
    """nested(spec/driver) → flat dict (기존 _ALL_ROBOTS 호환)."""
    flat = {'name': robot_entry['type']}
    if module_id:
        flat['module_id'] = module_id
    spec = robot_entry.get('spec') or {}
    driver = robot_entry.get('driver') or {}
    flat.update(spec)
    flat.update(driver)  # interpolation, sdk_control, sdk_type, read_topic, ... 흡수
    return flat


def load_all_robots() -> list[dict]:
    """builtin + 모든 manifest 의 robots[] 를 평탄화해 합친 리스트.

    backend 의 `_ALL_ROBOTS` 와 동일 shape — module_id 가 있고 manifest 가 없으면
    이 리스트엔 등장하지 않는다 (기존 _get_support_robots 의 필터링은 더 이상
    필요 없음 — 이제 manifest 가 곧 설치 여부).
    """
    out = list(_BUILTIN_ROBOTS)
    out = [_flatten(r, None) for r in out]
    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'robot':
            continue
        manifest = _resolve_manifest_paths(manifest)
        module_id = manifest.get('id')
        for robot in manifest.get('robots') or []:
            out.append(_flatten(robot, module_id))
    return out


def load_all_sensors() -> list[dict]:
    """모든 manifest 의 sensors[] 를 평탄화해 합친 리스트.

    sensors[] 는 sensor 모듈뿐 아니라 robot 모듈에도 포함될 수 있다 (예: Kinova
    의 vision sensor 는 robot_kinova 모듈 안에 함께 들어간다). 따라서 category
    필터 없이 모든 manifest 를 본다.

    반환 dict 는 기존 `_ALL_SENSORS` flat shape (name=type, +module_id) 를 유지해
    `SUPPORT_SENSORS`/`get_sensor_by_name` 등 옛 호출처가 변경 없이 동작.
    """
    out: list[dict] = []
    for manifest in _iter_manifests():
        manifest = _resolve_manifest_paths(manifest)
        module_id = manifest.get('id')
        for sensor in manifest.get('sensors') or []:
            out.append(_flatten(sensor, module_id))
    return out


def get_default_ee_definitions(robot_type: str) -> list[dict]:
    """frontend RobotPage 의 IK 폼 default 표시용. 각 정의는
    {name, parent, offset} dict. offset 은 list 또는 None."""
    # builtin 에는 IK 정보가 없으므로 manifest 만 검색.
    for manifest in _iter_manifests():
        if (manifest.get('category') or '') != 'robot':
            continue
        for robot in manifest.get('robots') or []:
            if robot.get('type') != robot_type:
                continue
            ik = robot.get('ik') or {}
            return [
                {
                    'name': d.get('name'),
                    'parent': d.get('parent'),
                    'offset': d.get('offset'),
                }
                for d in (ik.get('ee_definitions') or [])
            ]
    return []
