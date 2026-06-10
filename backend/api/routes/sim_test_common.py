# -*- coding: utf-8 -*-
"""
Shared seeding + sim-control + route factory for MuJoCo dual-arm TEST modes.

These test modes (`dual_arm_test`, `dual_arm_assembly_test`) verify that the
양팔(dual-arm) control paths work end-to-end against a bundled MuJoCo world,
exactly like tutorial mode does for the single arm. They are intentionally
modeled on backend/api/routes/tutorial.py — robot/sensor/assembly/workspace
rows are seeded as `type='custom'` (no hardware driver), and a `ros2 launch`
of the mujoco_world package publishes the virtual robot(s) + cameras.

To avoid copy-pasting tutorial.py twice, the seeding/control is driven by a
declarative SPEC dict (see configs/dual_arm_defaults.py /
configs/dual_arm_assembly_defaults.py). Identity is carried in settings:
    robots/sensors/workspace  ->  settings['sim_test_env'] == spec['env']
    robots/sensors            ->  settings['sim_test_slug'] == per-row slug
    assembly                  ->  matched by name (Assembly has no settings col)

Each SPEC produces a Flask Blueprint with:
    GET  /api/<env>/status
    POST /api/<env>:start          (optional body {show_viewer: bool})
    POST /api/<env>:stop
    POST /api/<env>:reset          (snap to home keyframe)
    POST /api/<env>:reset_world    (reset movable objects only)
"""
import json

from flask import Blueprint, jsonify, request

from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
from ...database.models.assembly_model import Assembly as AssemblyModel
from ...database.models.robot_model import Robot as RobotModel
from ...database.models.sensor_model import Sensor as SensorModel
from ...database.models.task_model import Task as TaskModel


# Robot settings keys that the SPEC is the single source of truth for — always
# overwritten on re-seed so topic/IK changes propagate even to existing rows.
_ROBOT_AUTHORITATIVE_KEYS = (
    'is_sim', 'sim_test_env', 'sim_test_slug', 'role',
    'read_topic', 'read_topic_msg',
    'write_type', 'write_topic', 'write_topic_msg',
    'joint_names', 'joint_lower_bounds', 'joint_upper_bounds',
    'tool_index', 'tool_inner', 'interpolation',
    'urdf_path', 'urdf_package_dir', 'ik_setting',
)


def _settings_dict(model):
    raw = model.settings
    if isinstance(raw, str):
        try:
            return json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return {}
    return raw or {}


# ---------------------------------------------------------------------------
# Robots
# ---------------------------------------------------------------------------

def _find_robot(env, slug):
    rows = RobotModel.select().where(
        RobotModel.hide == False,  # noqa: E712
        RobotModel.deleted_at.is_null(),
    )
    for row in rows:
        s = _settings_dict(row)
        if s.get('sim_test_env') == env and s.get('sim_test_slug') == slug:
            return row
    return None


def _ensure_robot(env, spec_robot):
    slug = spec_robot['settings']['sim_test_slug']
    row = _find_robot(env, slug)
    if row is None:
        row = RobotModel.create(
            name=spec_robot['name'],
            type=spec_robot['type'],
            settings=json.dumps(spec_robot['settings']),
            homepose=json.dumps(spec_robot.get('homepose', [])),
        )
    else:
        current = _settings_dict(row)
        for k in _ROBOT_AUTHORITATIVE_KEYS:
            if k in spec_robot['settings']:
                current[k] = spec_robot['settings'][k]
        for k, v in spec_robot['settings'].items():
            current.setdefault(k, v)
        row.settings = json.dumps(current)
        row.save()
    return row


# ---------------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------------

def _find_sensor(env, slug):
    rows = SensorModel.select().where(
        SensorModel.hide == False,  # noqa: E712
        SensorModel.deleted_at.is_null(),
    )
    for row in rows:
        s = _settings_dict(row)
        if s.get('sim_test_env') == env and s.get('sim_test_slug') == slug:
            return row
    return None


def _ensure_sensor(env, spec_sensor):
    slug = spec_sensor['settings']['sim_test_slug']
    row = _find_sensor(env, slug)
    if row is None:
        row = SensorModel.create(
            name=spec_sensor['name'],
            type=spec_sensor['type'],
            settings=json.dumps(spec_sensor['settings']),
        )
    else:
        current = _settings_dict(row)
        current['read_topic'] = spec_sensor['settings']['read_topic']
        current['read_topic_msg'] = spec_sensor['settings']['read_topic_msg']
        current['sim_test_env'] = env
        current['sim_test_slug'] = slug
        for k, v in spec_sensor['settings'].items():
            current.setdefault(k, v)
        row.settings = json.dumps(current)
        row.save()
    return row


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------

def _find_assembly(name):
    return AssemblyModel.select().where(
        AssemblyModel.name == name,
        AssemblyModel.hide == False,  # noqa: E712
        AssemblyModel.deleted_at.is_null(),
    ).first()


def _ensure_assembly(spec, robots_by_slug):
    a_spec = spec['assembly']
    name = a_spec['name']

    def _rid(slug_key):
        slug = a_spec.get(slug_key)
        return robots_by_slug[slug].id if slug else None

    left_id = _rid('left_slug')
    right_id = _rid('right_slug')
    left_tool_id = _rid('left_tool_slug')
    right_tool_id = _rid('right_tool_slug')

    assembly = _find_assembly(name)
    if assembly is None:
        assembly = AssemblyModel.create(
            name=name,
            left_arm_id=left_id,
            right_arm_id=right_id,
            left_tool_id=left_tool_id,
            right_tool_id=right_tool_id,
        )
    else:
        # Single source of truth: always re-point the slots (auto-repair).
        assembly.left_arm_id = left_id
        assembly.right_arm_id = right_id
        assembly.left_tool_id = left_tool_id
        assembly.right_tool_id = right_tool_id
        assembly.save()
    return assembly


# ---------------------------------------------------------------------------
# Workspace (Task)
# ---------------------------------------------------------------------------

def _find_workspace(env):
    for row in TaskModel.select().where(TaskModel.deleted_at.is_null()):
        raw = row.settings
        if isinstance(raw, str):
            try:
                data = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
        else:
            data = raw or {}
        if data.get('sim_test_env') == env:
            return row
    return None


def _ensure_workspace(spec, robots_by_slug, sensors):
    env = spec['env']
    ws_spec = spec['workspace']
    sensor_ids = [s.id for s in sensors]

    default_sensor_cfg = {
        str(sid): dict(ws_spec['default_sensor_cfg']) for sid in sensor_ids
    }
    # home_pose per robot id (slug -> pose in spec)
    robots_cfg = {}
    for slug, pose in ws_spec['home_poses'].items():
        rid = robots_by_slug[slug].id
        robots_cfg[str(rid)] = {'home_pose': list(pose)}

    workspace = _find_workspace(env)
    if workspace is None:
        settings = {
            'sim_test_env': env,
            'robots': robots_cfg,
            'sensors': default_sensor_cfg,
        }
        return TaskModel.create(
            name=ws_spec['name'],
            assembly_id=spec['_assembly_id'],
            episode_len=ws_spec['episode_len'],
            sensor_ids=json.dumps(sensor_ids),
            settings=json.dumps(settings),
            home_pose=json.dumps({}),
        )

    workspace.assembly_id = spec['_assembly_id']
    workspace.sensor_ids = json.dumps(sensor_ids)
    if not workspace.episode_len:
        workspace.episode_len = ws_spec['episode_len']
    current = _settings_dict(workspace)
    if not isinstance(current, dict):
        current = {}
    current['sim_test_env'] = env
    current.setdefault('robots', {})
    current.setdefault('sensors', {})
    for slug, pose in ws_spec['home_poses'].items():
        rid = str(robots_by_slug[slug].id)
        if rid not in current['robots']:
            current['robots'][rid] = {'home_pose': list(pose)}
        elif 'home_pose' not in current['robots'][rid]:
            current['robots'][rid]['home_pose'] = list(pose)
    for sid in sensor_ids:
        if str(sid) not in current['sensors']:
            current['sensors'][str(sid)] = dict(ws_spec['default_sensor_cfg'])
    workspace.settings = json.dumps(current)
    workspace.save()
    return workspace


def ensure_rows(spec, apply_binding=False):
    """Idempotently seed robot/sensor/assembly/workspace rows for a SPEC.

    Returns (robots_by_slug, sensors, assembly, workspace).

    canonical 모드 (spec['canonical_env'] 지정 시): 환경 전용 로봇/센서를 새로 시드하지
    않고 표준 디바이스(sim_arm/sim_arm_2/cam/cam_2)를 그 환경 바인딩으로 쓴다.
    spec['canonical_robots']/['canonical_cameras'] 가 SPEC slug → 표준 slug 매핑.

    apply_binding: True 면 표준 디바이스의 토픽/설정을 이 환경으로 **리포인트**
    한다(환경 활성화 :start). False(기본)면 디바이스를 조회만 하고 assembly/workspace
    만 구성한다 — startup 시딩이 표준 디바이스의 기본(tutorial) 바인딩을 덮어쓰지
    않도록."""
    canonical_env = spec.get('canonical_env')
    if canonical_env:
        from .tutorial import repoint_sim_devices, find_sim_devices
        if apply_binding:
            robots_c, cameras_c = repoint_sim_devices(canonical_env)
        else:
            robots_c, cameras_c = find_sim_devices(canonical_env)
        robots_by_slug = {
            spec_slug: robots_c[can_slug]
            for spec_slug, can_slug in spec['canonical_robots'].items()
            if can_slug in robots_c
        }
        sensors = []
        for cam_spec in spec['sensors']:
            can = spec['canonical_cameras'].get(cam_spec['settings']['sim_test_slug'])
            if can and can in cameras_c:
                sensors.append(cameras_c[can])
        assembly = _ensure_assembly(spec, robots_by_slug)
        spec['_assembly_id'] = assembly.id
        workspace = _ensure_workspace(spec, robots_by_slug, sensors)
        return robots_by_slug, sensors, assembly, workspace

    robots_by_slug = {}
    for spec_robot in spec['robots']:
        row = _ensure_robot(spec['env'], spec_robot)
        robots_by_slug[spec_robot['settings']['sim_test_slug']] = row

    sensors = [_ensure_sensor(spec['env'], s) for s in spec['sensors']]

    assembly = _ensure_assembly(spec, robots_by_slug)
    spec['_assembly_id'] = assembly.id
    workspace = _ensure_workspace(spec, robots_by_slug, sensors)
    return robots_by_slug, sensors, assembly, workspace


# ---------------------------------------------------------------------------
# Sim process control (via gRPC bridge)
# ---------------------------------------------------------------------------

def _is_running(spec):
    try:
        client = get_bridge_client()
        result = client.driver.ListProcesses(pb.Empty())
        return spec['launch']['process_id'] in set(result.names)
    except Exception:
        return False


def _topics_active(spec):
    try:
        client = get_bridge_client()
        result = client.driver.ListTopics(pb.Empty())
        names = {t.name for t in result.topics}
    except Exception:
        return False
    for spec_robot in spec['robots']:
        if spec_robot['settings'].get('read_topic') in names:
            return True
    return any(s['settings']['read_topic'] in names for s in spec['sensors'])


def _call_service(service_type, service_name, request_json=''):
    try:
        client = get_bridge_client()
        resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
            service_type=service_type,
            service_name=service_name,
            request_json=request_json,
        ))
        return bool(resp.success), resp.response_json or ''
    except Exception as e:
        return False, f'Bridge call failed: {e}'


# ---------------------------------------------------------------------------
# Blueprint factory
# ---------------------------------------------------------------------------

def make_sim_test_blueprint(spec):
    """Build a Flask Blueprint exposing status/start/stop/reset for a SPEC."""
    env = spec['env']
    bp = Blueprint(f'sim_test_{env}', __name__)
    launch = spec['launch']
    # The reset/reset_world services are namespaced under the first robot's
    # topic prefix (single-group) or the launch topic_prefix.
    reset_prefix = launch.get('reset_prefix', launch['topic_prefix']).rstrip('/')

    @bp.route(f'/{env}/status', methods=['GET'])
    def status():
        robots_by_slug, sensors, assembly, workspace = ensure_rows(spec)
        return jsonify({
            'status': 'success',
            'running': _is_running(spec),
            'has_topics': _topics_active(spec),
            'robot_ids': {slug: r.id for slug, r in robots_by_slug.items()},
            'sensor_ids': [s.id for s in sensors],
            'assembly_id': assembly.id if assembly else None,
            'workspace_id': workspace.id if workspace else None,
        }), 200

    @bp.route(f'/{env}:start', methods=['POST'])
    def start():
        # 활성화 시에만 표준 디바이스를 이 환경으로 리포인트(토픽/IK 교체).
        robots_by_slug, sensors, assembly, workspace = ensure_rows(spec, apply_binding=True)
        body = request.get_json(silent=True) or {}
        args = dict(launch.get('extra_args', {}))
        args['topic_prefix'] = launch['topic_prefix']
        if 'show_viewer' in body and body['show_viewer'] is not None:
            args['show_viewer'] = 'true' if bool(body['show_viewer']) else 'false'
        client = get_bridge_client()
        try:
            result = client.driver.StartLaunch(pb.LaunchConfig(
                process_id=launch['process_id'],
                package=launch['package'],
                launch_file=launch['launch_file'],
                args_json=json.dumps(args),
            ))
        except Exception as e:
            return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500
        if not result.success:
            return jsonify({'status': 'error', 'message': result.message}), 500
        return jsonify({
            'status': 'success',
            'message': f'{env} world starting',
            'pid': result.pid,
            'robot_ids': {slug: r.id for slug, r in robots_by_slug.items()},
            'sensor_ids': [s.id for s in sensors],
            'assembly_id': assembly.id,
            'workspace_id': workspace.id,
        }), 200

    @bp.route(f'/{env}:stop', methods=['POST'])
    def stop():
        client = get_bridge_client()
        try:
            client.driver.StopLaunch(pb.ProcessId(name=launch['process_id']))
        except Exception as e:
            return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500
        return jsonify({'status': 'success', 'message': f'{env} world stopped'}), 200

    @bp.route(f'/{env}:reset', methods=['POST'])
    def reset():
        if not _is_running(spec):
            return jsonify({'status': 'error', 'message': f'{env} sim not running'}), 400
        ok, msg = _call_service(
            'std_srvs/srv/Trigger', f'{reset_prefix}/reset', '')
        if not ok:
            return jsonify({'status': 'error', 'message': msg}), 400
        return jsonify({'status': 'success', 'message': msg or 'reset'}), 200

    @bp.route(f'/{env}:reset_world', methods=['POST'])
    def reset_world():
        ok, msg = _call_service(
            'std_srvs/srv/Empty', f'{reset_prefix}/reset_world', '{}')
        if not ok:
            return jsonify({'status': 'error', 'message': f'Reset failed: {msg}'}), 500
        return jsonify({'status': 'success', 'message': f'{env} world objects reset'}), 200

    return bp
