# -*- coding: utf-8 -*-
"""
dual_arm_plank demo routes.

Two independent single-arm robots (left/right) sharing one MuJoCo sim, each
wired to its OWN workspace (no combined Assembly). Built on the canonical sim
device model: the standard sim_arm/sim_arm_2 + cam/cam_2/cam_3/cam_4 rows are
repointed to the dual_arm_plank topics on :start (configs/sim_devices.py).

Endpoints:
    GET  /api/dual_arm_plank/status
    POST /api/dual_arm_plank:start        (optional body {show_viewer: bool})
    POST /api/dual_arm_plank:stop
    POST /api/dual_arm_plank:reset         (snap to home keyframe)
    POST /api/dual_arm_plank:reset_world   (reset movable objects only)

Seeds (idempotent): standard cam_4 row (if missing), two single-arm assemblies,
two workspaces (dual_arm_plank_left / dual_arm_plank_right).
"""
import json

from flask import Blueprint, jsonify, request

from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
from ...configs.dual_arm_plank_defaults import (
    DEFAULT_SENSOR_CFG,
    ENV,
    EPISODE_LEN,
    LAUNCH,
    RESET_PREFIX,
    WORKSPACES,
)
from ...database.models.assembly_model import Assembly as AssemblyModel
from ...database.models.sensor_model import Sensor as SensorModel
from ...database.models.task_model import Task as TaskModel
from .tutorial import (
    _find_sensor_by_sim_slug,
    _settings_dict,
    find_sim_devices,
    repoint_sim_devices,
)


dual_arm_plank_bp = Blueprint('dual_arm_plank', __name__)

_WS_MARKER = 'sim_plank_env'  # workspace settings flag identifying this demo's tasks


# ---------------------------------------------------------------------------
# Device seeding
# ---------------------------------------------------------------------------

def _ensure_cam4():
    """Ensure the standard cam_4 sensor row exists (the 4th sim camera). Its
    topic/depth binding is applied by repoint_sim_devices on :start; here we just
    guarantee the row is present so the binding has something to repoint."""
    row = _find_sensor_by_sim_slug('cam_4')
    if row is not None:
        return row
    return SensorModel.create(
        name='cam_4',
        type='custom',
        settings=json.dumps({
            'sim_device_slug': 'cam_4',
            'is_sim': True,
            'is_tutorial': False,
            'read_topic': '/da_plank_right/right_wrist_cam2/image_raw/compressed',
            'read_topic_msg': 'sensor_msgs/CompressedImage',
            'resolution': [640, 480],
        }),
    )


# ---------------------------------------------------------------------------
# Assembly (single-arm, one per workspace)
# ---------------------------------------------------------------------------

def _ensure_single_arm_assembly(name, arm_id):
    assembly = AssemblyModel.select().where(
        AssemblyModel.name == name,
        AssemblyModel.hide == False,  # noqa: E712
        AssemblyModel.deleted_at.is_null(),
    ).first()
    if assembly is None:
        return AssemblyModel.create(name=name, left_arm_id=arm_id)
    if assembly.left_arm_id != arm_id:
        assembly.left_arm_id = arm_id
        assembly.right_arm_id = None
        assembly.save()
    return assembly


# ---------------------------------------------------------------------------
# Workspace (Task) — one per arm
# ---------------------------------------------------------------------------

def _find_plank_workspace(name):
    for row in TaskModel.select().where(TaskModel.deleted_at.is_null()):
        s = _settings_dict(row)
        if s.get(_WS_MARKER) and row.name == name:
            return row
    return None


def _ensure_plank_workspace(ws_spec, assembly, arm_row, sensor_rows):
    sensor_ids = [s.id for s in sensor_rows]
    default_sensor_cfg = {str(sid): dict(DEFAULT_SENSOR_CFG) for sid in sensor_ids}
    robots_cfg = {str(arm_row.id): {'home_pose': list(ws_spec['home_pose'])}}

    workspace = _find_plank_workspace(ws_spec['name'])
    if workspace is None:
        settings = {
            _WS_MARKER: True,
            'robots': robots_cfg,
            'sensors': default_sensor_cfg,
        }
        return TaskModel.create(
            name=ws_spec['name'],
            assembly_id=assembly.id,
            episode_len=EPISODE_LEN,
            sensor_ids=json.dumps(sensor_ids),
            settings=json.dumps(settings),
            home_pose=json.dumps({}),
        )

    workspace.assembly_id = assembly.id
    workspace.sensor_ids = json.dumps(sensor_ids)
    if not workspace.episode_len:
        workspace.episode_len = EPISODE_LEN
    current = _settings_dict(workspace)
    if not isinstance(current, dict):
        current = {}
    current[_WS_MARKER] = True
    current.setdefault('robots', {})
    current.setdefault('sensors', {})
    rid = str(arm_row.id)
    if rid not in current['robots']:
        current['robots'][rid] = {'home_pose': list(ws_spec['home_pose'])}
    elif 'home_pose' not in current['robots'][rid]:
        current['robots'][rid]['home_pose'] = list(ws_spec['home_pose'])
    for sid in sensor_ids:
        if str(sid) not in current['sensors']:
            current['sensors'][str(sid)] = dict(DEFAULT_SENSOR_CFG)
    workspace.settings = json.dumps(current)
    workspace.save()
    return workspace


def ensure_plank_rows(apply_binding=False):
    """Idempotently seed cam_4 + two single-arm assemblies + two workspaces.

    apply_binding=True repoints the standard devices to the dual_arm_plank topics
    (done on :start); False only looks them up (status)."""
    _ensure_cam4()
    if apply_binding:
        robots_c, cameras_c = repoint_sim_devices(ENV)
    else:
        robots_c, cameras_c = find_sim_devices(ENV)

    workspaces = []
    for ws_spec in WORKSPACES:
        arm_row = robots_c.get(ws_spec['arm_slug'])
        sensor_rows = [cameras_c[s] for s in ws_spec['sensor_slugs'] if s in cameras_c]
        if arm_row is None or len(sensor_rows) != len(ws_spec['sensor_slugs']):
            # Devices not seeded yet (first boot before tutorial seeding ran);
            # skip — a later call once standard rows exist will complete it.
            continue
        assembly = _ensure_single_arm_assembly(ws_spec['assembly_name'], arm_row.id)
        workspace = _ensure_plank_workspace(ws_spec, assembly, arm_row, sensor_rows)
        workspaces.append(workspace)
    return workspaces


# ---------------------------------------------------------------------------
# Sim process control (via gRPC bridge)
# ---------------------------------------------------------------------------

def _is_running():
    try:
        client = get_bridge_client()
        result = client.driver.ListProcesses(pb.Empty())
        return LAUNCH['process_id'] in set(result.names)
    except Exception:
        return False


def _topics_active():
    try:
        client = get_bridge_client()
        result = client.driver.ListTopics(pb.Empty())
        names = {t.name for t in result.topics}
    except Exception:
        return False
    return '/da_plank_left/joint_states' in names or '/da_plank_right/joint_states' in names


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
# Routes
# ---------------------------------------------------------------------------

@dual_arm_plank_bp.route('/dual_arm_plank/status', methods=['GET'])
def status():
    workspaces = ensure_plank_rows()
    return jsonify({
        'status': 'success',
        'running': _is_running(),
        'has_topics': _topics_active(),
        'workspace_ids': [w.id for w in workspaces],
        'workspaces': [{'id': w.id, 'name': w.name} for w in workspaces],
    }), 200


@dual_arm_plank_bp.route('/dual_arm_plank:start', methods=['POST'])
def start():
    workspaces = ensure_plank_rows(apply_binding=True)
    body = request.get_json(silent=True) or {}
    args = {'topic_prefix': LAUNCH['topic_prefix']}
    if 'show_viewer' in body and body['show_viewer'] is not None:
        args['show_viewer'] = 'true' if bool(body['show_viewer']) else 'false'
    client = get_bridge_client()
    try:
        result = client.driver.StartLaunch(pb.LaunchConfig(
            process_id=LAUNCH['process_id'],
            package=LAUNCH['package'],
            launch_file=LAUNCH['launch_file'],
            args_json=json.dumps(args),
        ))
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500
    if not result.success:
        return jsonify({'status': 'error', 'message': result.message}), 500
    return jsonify({
        'status': 'success',
        'message': 'dual_arm_plank world starting',
        'pid': result.pid,
        'workspace_ids': [w.id for w in workspaces],
        'workspaces': [{'id': w.id, 'name': w.name} for w in workspaces],
    }), 200


@dual_arm_plank_bp.route('/dual_arm_plank:stop', methods=['POST'])
def stop():
    client = get_bridge_client()
    try:
        client.driver.StopLaunch(pb.ProcessId(name=LAUNCH['process_id']))
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500
    return jsonify({'status': 'success', 'message': 'dual_arm_plank world stopped'}), 200


@dual_arm_plank_bp.route('/dual_arm_plank:reset', methods=['POST'])
def reset():
    if not _is_running():
        return jsonify({'status': 'error', 'message': 'dual_arm_plank sim not running'}), 400
    ok, msg = _call_service('std_srvs/srv/Trigger', f'{RESET_PREFIX}/reset', '')
    if not ok:
        return jsonify({'status': 'error', 'message': msg}), 400
    return jsonify({'status': 'success', 'message': msg or 'reset'}), 200


@dual_arm_plank_bp.route('/dual_arm_plank:reset_world', methods=['POST'])
def reset_world():
    ok, msg = _call_service('std_srvs/srv/Empty', f'{RESET_PREFIX}/reset_world', '{}')
    if not ok:
        return jsonify({'status': 'error', 'message': f'Reset failed: {msg}'}), 500
    return jsonify({'status': 'success', 'message': 'dual_arm_plank world objects reset'}), 200
