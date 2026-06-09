# -*- coding: utf-8 -*-
"""Demo simulation catalog API.

The repo-root ``demo/`` folder is a catalog of self-contained simulation demos
shown at robot events (no hardware required). Each demo is a sub-folder with a
``manifest.json`` describing how to launch its environment and how to collect
data from it. This blueprint reads that folder and drives the launch via the
gRPC bridge — the exact mechanism tutorial mode uses (see ``tutorial.py``).

Endpoints:
    GET  /api/demo/list      - [{id, name, description, cameras, ...}, ...]
    GET  /api/demo/status    - {running, demo_id, has_topics, process_id}
    POST /api/demo:start     - body {demo_id} → launch that demo's environment
    POST /api/demo:stop      - stop the running demo sim
"""
import json
import os
from pathlib import Path

from flask import Blueprint, jsonify, request

from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
from ...configs.sim_defaults import (
    SIM_AGENT_NAME,
    SIM_ROBOT,
    SIM_SENSORS,
    SIM_WORKSPACE_DEFAULT_SENSOR_CFG,
    SIM_WORKSPACE_HOMEPOSE,
)
from ...database.models.assembly_model import Assembly as AssemblyModel
from ...database.models.planner_model import Planner as PlannerModel
from ...database.models.robot_model import Robot as RobotModel
from ...database.models.sensor_model import Sensor as SensorModel
from ...database.models.task_model import Task as TaskModel

# planner block fields keyed by robot id (see planner_run.py). When a demo
# workspace's single arm is swapped to the sim robot, these must follow.
_ROBOT_KEYED_BLOCK_FIELDS = ('positions', 'deltas', 'tool_positions', 'observe_positions')


demo_bp = Blueprint('demo', __name__)


def _resolve_demo_dir():
    """Locate the repo-root ``demo/`` folder across the several runtime layouts.

    - Dev container: backend is bind-mounted at ``/root/backend`` (so ``__file__``
      resolves to ``/root/...`` and parents[3] is ``/`` — useless), but the
      ``demo/`` folder is synced to ``/opt/easytrainer/project/demo`` by
      ``scripts/quick_apply.sh`` and ``/opt/easytrainer`` is always mounted.
    - DEB / launcher install: backend runs from ``<project>/backend`` so
      ``parents[3]/demo`` points straight at it.
    - Override with ``EASYTRAINER_DEMO_DIR`` if neither fits.
    """
    data_dir = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
    candidates = [
        os.environ.get('EASYTRAINER_DEMO_DIR'),
        Path(__file__).resolve().parents[3] / 'demo',
        Path(data_dir) / 'project' / 'demo',
    ]
    for cand in candidates:
        if cand and Path(cand).is_dir():
            return Path(cand)
    # Fall back to the project-root guess even if it doesn't exist yet.
    return Path(__file__).resolve().parents[3] / 'demo'


# ---------------------------------------------------------------------------
# Manifest loading
# ---------------------------------------------------------------------------

def _load_manifests():
    """Return {demo_id: manifest_dict} for every valid demo/<id>/manifest.json.

    Silently skips folders without a manifest or with broken JSON so one bad
    demo never takes down the whole list.
    """
    out = {}
    demo_dir = _resolve_demo_dir()
    if not demo_dir.is_dir():
        return out
    for child in sorted(demo_dir.iterdir()):
        manifest = child / 'manifest.json'
        if not manifest.is_file():
            continue
        try:
            data = json.loads(manifest.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        demo_id = data.get('id') or child.name
        data['id'] = demo_id
        out[demo_id] = data
    return out


def _public_view(manifest):
    """Trim a manifest down to what the UI needs."""
    return {
        'id': manifest.get('id'),
        'name': manifest.get('name'),
        'name_en': manifest.get('name_en'),
        'description': manifest.get('description'),
        'description_en': manifest.get('description_en'),
        'task_language': manifest.get('task_language'),
        'cameras': manifest.get('cameras', []),
        'topic_prefix': manifest.get('topic_prefix'),
        'process_id': (manifest.get('launch') or {}).get('process_id'),
    }


# ---------------------------------------------------------------------------
# Bridge helpers (mirror tutorial.py)
# ---------------------------------------------------------------------------

def _running_process_ids():
    try:
        client = get_bridge_client()
        result = client.driver.ListProcesses(pb.Empty())
        return set(result.names)
    except Exception:
        return set()


def _topics_active(topic_prefix):
    if not topic_prefix:
        return False
    try:
        client = get_bridge_client()
        result = client.driver.ListTopics(pb.Empty())
        names = {t.name for t in result.topics}
    except Exception:
        return False
    return f'{topic_prefix.rstrip("/")}/joint_states' in names


def _running_demo(manifests):
    """Return the manifest of the currently-running demo, or None."""
    active = _running_process_ids()
    for manifest in manifests.values():
        pid = (manifest.get('launch') or {}).get('process_id')
        if pid and pid in active:
            return manifest
    return None


# ---------------------------------------------------------------------------
# Demo-sim device seeding (mirror of tutorial.py, but is_tutorial=False).
#
# Demo sims need their OWN device set: a device flagged `is_tutorial` makes
# record_episode's env.reset() snap the movable objects (peg) back to the home
# keyframe on every episode, wiping the planner's restoration. So we seed a
# parallel `sim_arm` / `sim_wrist_cam*` set (is_sim=True, is_tutorial=False) and
# repoint the demo's workspace onto it. Identified by `is_sim and not
# is_tutorial` (robot) / `sim_camera_slug` (sensors) — never by name alone.
# ---------------------------------------------------------------------------

def _settings_dict(model):
    raw = model.settings
    if isinstance(raw, str):
        try:
            return json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return {}
    return raw or {}


def _is_sim_device(model):
    # `demo_sim_device` is the unambiguous marker — is_sim alone also matches
    # dual_arm_test / other custom sim robots, so we must NOT key off it.
    return bool(_settings_dict(model).get('demo_sim_device'))


def _find_sim_robot():
    candidates = RobotModel.select().where(
        RobotModel.hide == False,  # noqa: E712
        RobotModel.deleted_at.is_null(),
        RobotModel.type == SIM_ROBOT['type'],
    )
    for row in candidates:
        if _is_sim_device(row):
            return row
    return None


def _find_sim_sensors():
    candidates = SensorModel.select().where(
        SensorModel.hide == False,  # noqa: E712
        SensorModel.deleted_at.is_null(),
    )
    return [row for row in candidates
            if _is_sim_device(row) and _settings_dict(row).get('sim_camera_slug')]


def _find_sim_sensor_by_slug(slug):
    for row in _find_sim_sensors():
        if _settings_dict(row).get('sim_camera_slug') == slug:
            return row
    return None


def _find_sim_assembly():
    return AssemblyModel.select().where(
        AssemblyModel.name == SIM_AGENT_NAME,
        AssemblyModel.hide == False,  # noqa: E712
        AssemblyModel.deleted_at.is_null(),
    ).first()


def _ensure_sim_assembly(robot):
    assembly = _find_sim_assembly()
    if assembly is None:
        assembly = AssemblyModel.create(name=SIM_AGENT_NAME, left_arm_id=robot.id)
    elif assembly.left_arm_id != robot.id:
        assembly.left_arm_id = robot.id
        assembly.save()
    return assembly


def _ensure_sim_rows():
    """Idempotently create the demo-sim robot/sensors/assembly. Returns
    (robot, sensors, assembly). Mirrors tutorial.py's _ensure_tutorial_rows but
    flags every row is_sim=True / is_tutorial=False."""
    robot = _find_sim_robot()
    if robot is None:
        robot = RobotModel.create(
            name=SIM_ROBOT['name'],
            type=SIM_ROBOT['type'],
            settings=json.dumps(SIM_ROBOT['settings']),
            homepose=json.dumps(SIM_ROBOT['homepose']),
        )
    else:
        # Keep topics/IK in sync with the single source (sim_defaults), but
        # always force is_tutorial off — that is the whole point of this set.
        current = _settings_dict(robot)
        for k, v in SIM_ROBOT['settings'].items():
            current[k] = v
        current['is_tutorial'] = False
        current['is_sim'] = True
        robot.settings = json.dumps(current)
        robot.save()

    sensors = []
    for spec in SIM_SENSORS:
        slug = spec['settings']['sim_camera_slug']
        row = _find_sim_sensor_by_slug(slug)
        if row is None:
            row = SensorModel.create(
                name=spec['name'],
                type=spec['type'],
                settings=json.dumps(spec['settings']),
            )
        else:
            current = _settings_dict(row)
            for k, v in spec['settings'].items():
                current[k] = v
            current['is_tutorial'] = False
            current['is_sim'] = True
            row.settings = json.dumps(current)
            row.save()
        sensors.append(row)

    assembly = _ensure_sim_assembly(robot)
    return robot, sensors, assembly


def _repoint_workspace_to_sim(workspace_name, robot, sensors, assembly):
    """Swap a demo workspace's robot/sensors onto the sim device set.

    Idempotent: forces assembly_id + sensor_ids onto the sim devices and merges
    default sensor/robot config for any newly-added slot (existing user tweaks
    like episode_len / home_pose are preserved)."""
    if not workspace_name:
        return None
    ws = TaskModel.select().where(
        TaskModel.name == workspace_name,
        TaskModel.deleted_at.is_null(),
    ).order_by(TaskModel.id.desc()).first()
    if ws is None:
        return None

    sensor_ids = [s.id for s in sensors]
    ws.assembly_id = assembly.id
    ws.sensor_ids = json.dumps(sensor_ids)

    raw = ws.settings
    if isinstance(raw, str):
        try:
            current = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            current = {}
    else:
        current = raw or {}
    if not isinstance(current, dict):
        current = {}
    current['is_tutorial'] = False
    # Rebuild robots/sensors maps onto the sim ids (drop stale tutorial-device
    # keys so record_episode reads only sim sensors → env.tutorial=False).
    old_robots = current.get('robots') or {}
    home_pose = None
    for v in old_robots.values():
        if isinstance(v, dict) and v.get('home_pose'):
            home_pose = v['home_pose']
            break
    current['robots'] = {
        str(robot.id): {'home_pose': home_pose or list(SIM_WORKSPACE_HOMEPOSE)}
    }
    current['sensors'] = {
        str(sid): dict(SIM_WORKSPACE_DEFAULT_SENSOR_CFG) for sid in sensor_ids
    }
    ws.settings = json.dumps(current)
    ws.save()

    # 플랜 블록의 로봇-키 필드 + visual_reach 의 sensor_id 도 sim 디바이스로 따라가게
    # 한다 (안 그러면 joint_position 은 옛 로봇 id 를, visual_reach 는 옛 센서 id 를
    # 찾다 실패 — 후자는 _wrist_stream_geometry 가 센서를 못 찾아 검출 0px 로 깨진다).
    _remap_workspace_blocks(ws.id, robot.id, sensors,
                            single_arm=(assembly.right_arm_id is None))
    return ws


def _sensor_slug(sensor_id):
    """센서 행의 카메라 slug (sim_camera_slug 또는 tutorial_camera_slug)."""
    try:
        row = SensorModel.get_by_id(sensor_id)
    except Exception:
        return None
    s = _settings_dict(row)
    return s.get('sim_camera_slug') or s.get('tutorial_camera_slug')


def _remap_workspace_blocks(ws_id, new_robot_id, sim_sensors, single_arm=True):
    """이 워크스페이스를 대상으로 하는 모든 planner 블록을 sim 디바이스로 remap.

    - 로봇-키 dict 필드(positions/deltas/tool_positions/observe_positions) → new_robot_id.
      single-arm sim 데모 전제: 키 1개 dict 만 안전 교체(2개↑면 dual-arm 가능성 → skip).
    - visual_reach 의 sensor_id → 같은 카메라 slug 의 sim 센서 id (slug 매칭, 실패 시
      depth 가 있는 첫 sim 센서로 폴백).
    이미 대상 값이면 no-op."""
    new_key = str(new_robot_id)
    # slug → sim sensor id, + detection 폴백용(첫 has_depth/rgbd 센서, 없으면 첫 센서)
    slug_to_sim = {}
    detect_fallback = None
    for s in sim_sensors:
        sd = _settings_dict(s)
        slug = sd.get('sim_camera_slug')
        if slug:
            slug_to_sim[slug] = s.id
        if detect_fallback is None and (sd.get('has_depth') or sd.get('rgbd_service')):
            detect_fallback = s.id
    if detect_fallback is None and sim_sensors:
        detect_fallback = sim_sensors[0].id
    sim_sensor_ids = {s.id for s in sim_sensors}

    for planner in PlannerModel.select().where(PlannerModel.deleted_at.is_null()):
        dirty = False
        for field in ('plans', 'plan', 'blocks'):
            raw = getattr(planner, field, None)
            if not raw:
                continue
            try:
                data = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
            groups = data if isinstance(data, list) else [data]
            for grp in groups:
                blocks = grp.get('blocks') if isinstance(grp, dict) else None
                if not blocks:
                    continue
                for blk in blocks:
                    if str(blk.get('workspace_id')) != str(ws_id):
                        continue
                    for f in _ROBOT_KEYED_BLOCK_FIELDS:
                        d = blk.get(f)
                        if not isinstance(d, dict) or not d:
                            continue
                        if not single_arm or len(d) != 1:
                            continue
                        (old_key, val), = d.items()
                        if old_key != new_key:
                            blk[f] = {new_key: val}
                            dirty = True
                    # visual_reach sensor_id 재매핑 (slug 기준)
                    if blk.get('type') == 'visual_reach':
                        old_sid = blk.get('sensor_id')
                        if old_sid is not None and old_sid not in sim_sensor_ids:
                            new_sid = slug_to_sim.get(_sensor_slug(old_sid)) or detect_fallback
                            if new_sid is not None and new_sid != old_sid:
                                blk['sensor_id'] = new_sid
                                dirty = True
            if dirty:
                setattr(planner, field, json.dumps(data))
        if dirty:
            planner.save()


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@demo_bp.route('/demo/list', methods=['GET'])
def demo_list():
    manifests = _load_manifests()
    return jsonify({
        'status': 'success',
        'demos': [_public_view(m) for m in manifests.values()],
    }), 200


@demo_bp.route('/demo/status', methods=['GET'])
def demo_status():
    manifests = _load_manifests()
    running = _running_demo(manifests)
    return jsonify({
        'status': 'success',
        'running': running is not None,
        'demo_id': running.get('id') if running else None,
        'process_id': (running.get('launch') or {}).get('process_id') if running else None,
        'has_topics': _topics_active(running.get('topic_prefix')) if running else False,
    }), 200


@demo_bp.route('/demo:start', methods=['POST'])
def demo_start():
    body = request.get_json(silent=True) or {}
    demo_id = body.get('demo_id')
    if not demo_id:
        return jsonify({'status': 'error', 'message': 'demo_id is required'}), 400

    manifests = _load_manifests()
    manifest = manifests.get(demo_id)
    if manifest is None:
        return jsonify({'status': 'error', 'message': f'unknown demo_id: {demo_id}'}), 404

    launch = manifest.get('launch') or {}
    process_id = launch.get('process_id')
    package = launch.get('package')
    launch_file = launch.get('launch_file')
    if not (process_id and package and launch_file):
        return jsonify({'status': 'error',
                        'message': f'demo {demo_id} has an incomplete launch manifest'}), 500

    # Stop any other demo first so two demos never fight over the same topics.
    active = _running_process_ids()
    for other in manifests.values():
        opid = (other.get('launch') or {}).get('process_id')
        if opid and opid != process_id and opid in active:
            try:
                get_bridge_client().driver.StopLaunch(pb.ProcessId(name=opid))
            except Exception:
                pass

    args = dict(launch.get('args') or {})
    # Allow the caller to override show_viewer (events: window on; CI: headless).
    if 'show_viewer' in body and body['show_viewer'] is not None:
        args['show_viewer'] = 'true' if bool(body['show_viewer']) else 'false'

    client = get_bridge_client()
    try:
        result = client.driver.StartLaunch(pb.LaunchConfig(
            process_id=process_id,
            package=package,
            launch_file=launch_file,
            args_json=json.dumps(args),
        ))
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500

    if not result.success:
        return jsonify({'status': 'error', 'message': result.message}), 500

    # Sim 활성화 = 데모 전용 디바이스(is_sim, NOT is_tutorial) 보장 + 데모
    # 워크스페이스를 그 디바이스로 교체. 실패해도 sim 자체는 떴으므로 치명적이지
    # 않게 처리(메시지에 경고만 실어 반환).
    sim_warning = None
    sim_devices = None
    try:
        robot, sensors, assembly = _ensure_sim_rows()
        ws = _repoint_workspace_to_sim(
            manifest.get('workspace_name'), robot, sensors, assembly)
        sim_devices = {
            'robot_id': robot.id,
            'sensor_ids': [s.id for s in sensors],
            'assembly_id': assembly.id,
            'workspace_id': ws.id if ws else None,
        }
    except Exception as e:
        sim_warning = f'sim device seeding failed: {e}'

    payload = {
        'status': 'success',
        'message': f'Demo "{demo_id}" starting',
        'demo_id': demo_id,
        'process_id': process_id,
        'pid': result.pid,
        'sim_devices': sim_devices,
    }
    if sim_warning:
        payload['warning'] = sim_warning
    return jsonify(payload), 200


@demo_bp.route('/demo:stop', methods=['POST'])
def demo_stop():
    body = request.get_json(silent=True) or {}
    manifests = _load_manifests()

    # Stop the explicitly-requested demo, or whatever demo is currently running.
    demo_id = body.get('demo_id')
    target = manifests.get(demo_id) if demo_id else _running_demo(manifests)
    if target is None:
        # Nothing to stop is a success (idempotent).
        return jsonify({'status': 'success', 'message': 'No demo running'}), 200

    process_id = (target.get('launch') or {}).get('process_id')
    client = get_bridge_client()
    try:
        client.driver.StopLaunch(pb.ProcessId(name=process_id))
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500

    return jsonify({'status': 'success', 'message': f'Demo "{target.get("id")}" stopped'}), 200
