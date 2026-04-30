# -*- coding: utf-8 -*-
"""
Tutorial mode API routes.

Tutorial mode boots a bundled MuJoCo simulation (ros2/ros2_ws/src/mujoco_world)
that publishes a virtual robot and RGB camera as standard ROS2 topics. The
tutorial_arm / tutorial_camera DB rows are seeded as `type='custom'` so the
rest of EasyTrainer treats them like ordinary external-topic devices — no
hardware is required to run through the full workflow.

Endpoints:
    GET  /api/tutorial/status   - {running, robot_id, sensor_id, has_topics}
    POST /api/tutorial:start    - launch sim + ensure DB rows exist
    POST /api/tutorial:stop     - stop sim launch (DB rows preserved)
"""
import json

from flask import Blueprint, jsonify

from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
from ...configs.tutorial_defaults import (
    TUTORIAL_LAUNCH_FILE,
    TUTORIAL_LAUNCH_PACKAGE,
    TUTORIAL_LAUNCH_PROCESS_ID,
    TUTORIAL_ROBOT,
    TUTORIAL_SENSORS,
    TUTORIAL_TOPIC_PREFIX,
)
from ...database.models.robot_model import Robot as RobotModel
from ...database.models.sensor_model import Sensor as SensorModel


tutorial_bp = Blueprint('tutorial', __name__)


# ---------------------------------------------------------------------------
# DB seeding
# ---------------------------------------------------------------------------

def _settings_dict(model):
    """Return the parsed settings dict for a Robot/Sensor row."""
    raw = model.settings
    if isinstance(raw, str):
        try:
            return json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return {}
    return raw or {}


def _find_tutorial_robot():
    """Look for an existing tutorial robot regardless of its name (rename-safe)."""
    candidates = RobotModel.select().where(
        RobotModel.hide == False,  # noqa: E712
        RobotModel.deleted_at.is_null(),
        RobotModel.type == TUTORIAL_ROBOT['type'],
    )
    for row in candidates:
        if _settings_dict(row).get('is_tutorial'):
            return row
    return None


def _find_tutorial_sensors():
    """Return all rows flagged as tutorial sensors (rename-safe)."""
    candidates = SensorModel.select().where(
        SensorModel.hide == False,  # noqa: E712
        SensorModel.deleted_at.is_null(),
    )
    return [row for row in candidates if _settings_dict(row).get('is_tutorial')]


def _find_tutorial_sensor_by_slug(slug: str):
    """Find a tutorial sensor row matching the camera slug (top_cam / front_cam)."""
    for row in _find_tutorial_sensors():
        if _settings_dict(row).get('tutorial_camera_slug') == slug:
            return row
    return None


def _ensure_tutorial_rows():
    """Create tutorial robot/sensor DB rows if they don't already exist.

    Idempotent — safe to call at every backend startup.

    For tutorial robot/sensor settings:
      - missing keys from the latest defaults are merged in (handles new
        fields added across versions, e.g. urdf_path / ik_setting / topic
        renames). Existing keys are preserved so user edits aren't clobbered.
      - 토픽 등 우리가 단일 진실원천(tutorial_defaults.py)으로 관리해야 하는
        키는 항상 현재 default로 덮어쓴다 — 카메라 토픽 이름이 바뀌어도
        구독이 깨지지 않도록 하기 위함.

    Stale tutorial sensor rows (e.g. legacy `tutorial_camera` from before the
    top/front split) are soft-hidden so they no longer pollute the UI list.
    """
    robot = _find_tutorial_robot()
    if robot is None:
        # Robot.role is a getter-only @property; for custom robots it falls
        # back to settings['role'], which is what we set below.
        robot = RobotModel.create(
            name=TUTORIAL_ROBOT['name'],
            type=TUTORIAL_ROBOT['type'],
            settings=json.dumps(TUTORIAL_ROBOT['settings']),
            homepose=json.dumps(TUTORIAL_ROBOT['homepose']),
        )
    else:
        current = _settings_dict(robot)
        # Authoritative keys: 단일 진실원천(tutorial_defaults.py)이 항상 이김.
        # 특히 is_sim은 record_episode의 home pose 이동 분기를 결정하므로 누락되면
        # 매우 느린 step-by-step 이동이 발생 — 항상 강제 반영해야 한다.
        for k in ('is_sim', 'is_tutorial'):
            if k in TUTORIAL_ROBOT['settings']:
                current[k] = TUTORIAL_ROBOT['settings'][k]
        # 그 외 새로 추가된 default 키는 머지(기존 사용자 편집 보존)
        for k, v in TUTORIAL_ROBOT['settings'].items():
            current.setdefault(k, v)
        robot.settings = json.dumps(current)
        robot.save()

    sensors = []
    expected_slugs = {s['settings']['tutorial_camera_slug'] for s in TUTORIAL_SENSORS}
    for spec in TUTORIAL_SENSORS:
        slug = spec['settings']['tutorial_camera_slug']
        row = _find_tutorial_sensor_by_slug(slug)
        if row is None:
            row = SensorModel.create(
                name=spec['name'],
                type=spec['type'],
                settings=json.dumps(spec['settings']),
            )
        else:
            current = _settings_dict(row)
            # Authoritative keys: 현재 default를 항상 반영 (토픽 이름 등)
            current['read_topic'] = spec['settings']['read_topic']
            current['read_topic_msg'] = spec['settings']['read_topic_msg']
            current['tutorial_camera_slug'] = slug
            current['is_tutorial'] = True
            # 그 외 새로 추가된 default 키는 머지
            for k, v in spec['settings'].items():
                current.setdefault(k, v)
            row.settings = json.dumps(current)
            row.save()
        sensors.append(row)

    # Hide legacy tutorial sensors that don't match any current slug
    # (e.g. the old single `tutorial_camera` row).
    for row in _find_tutorial_sensors():
        slug = _settings_dict(row).get('tutorial_camera_slug')
        if slug not in expected_slugs:
            row.hide = True
            row.save()

    return robot, sensors


# ---------------------------------------------------------------------------
# Sim process control (via gRPC bridge)
# ---------------------------------------------------------------------------

def _is_sim_running():
    try:
        client = get_bridge_client()
        result = client.driver.ListProcesses(pb.Empty())
        return TUTORIAL_LAUNCH_PROCESS_ID in set(result.names)
    except Exception:
        return False


def reset_tutorial_world():
    """Tell the running MuJoCo world to snap back to its home keyframe.

    Returns (success: bool, message: str). No-op (returns success=False with a
    'not running' message) when the sim isn't up — callers can ignore it.
    """
    if not _is_sim_running():
        return False, 'Tutorial sim not running'
    try:
        client = get_bridge_client()
        resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
            service_type='std_srvs/srv/Trigger',
            service_name=f'{TUTORIAL_TOPIC_PREFIX}/reset',
            request_json='',
        ))
        return bool(resp.success), resp.response_json or ''
    except Exception as e:
        return False, f'Bridge call failed: {e}'


def _topics_active():
    """Check whether the sim's published topics are visible on the ROS graph."""
    try:
        client = get_bridge_client()
        result = client.driver.ListTopics(pb.Empty())
        names = {t.name for t in result.topics}
    except Exception:
        return False
    if f'{TUTORIAL_TOPIC_PREFIX}/joint_states' in names:
        return True
    return any(spec['settings']['read_topic'] in names for spec in TUTORIAL_SENSORS)


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@tutorial_bp.route('/tutorial/status', methods=['GET'])
def tutorial_status():
    robot = _find_tutorial_robot()
    sensors = _find_tutorial_sensors()
    return jsonify({
        'status': 'success',
        'running': _is_sim_running(),
        'has_topics': _topics_active(),
        'robot_id': robot.id if robot else None,
        'sensor_ids': [s.id for s in sensors],
    }), 200


@tutorial_bp.route('/tutorial:start', methods=['POST'])
def tutorial_start():
    """Launch the bundled MuJoCo world. DB rows are guaranteed by app startup,
    but call _ensure_tutorial_rows() again as a safety net (idempotent)."""
    robot, sensors = _ensure_tutorial_rows()

    client = get_bridge_client()
    args = {
        'topic_prefix': TUTORIAL_TOPIC_PREFIX,
    }
    try:
        result = client.driver.StartLaunch(pb.LaunchConfig(
            process_id=TUTORIAL_LAUNCH_PROCESS_ID,
            package=TUTORIAL_LAUNCH_PACKAGE,
            launch_file=TUTORIAL_LAUNCH_FILE,
            args_json=json.dumps(args),
        ))
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500

    if not result.success:
        return jsonify({'status': 'error', 'message': result.message}), 500

    return jsonify({
        'status': 'success',
        'message': 'Tutorial world starting',
        'pid': result.pid,
        'robot_id': robot.id,
        'sensor_ids': [s.id for s in sensors],
    }), 200


def reset_tutorial_world() -> tuple[bool, str]:
    """Reset movable objects (cube, etc.) in the tutorial sim. Robot is untouched.

    Returns (success, message). 호출자(라우트, record_episode 등)가 결과 보고만
    필요하면 그대로 사용; 실패해도 sim 자체는 계속 돈다.
    """
    try:
        client = get_bridge_client()
        result = client.ros_proxy.CallService(pb.ROSServiceRequest(
            service_type='std_srvs/srv/Empty',
            service_name=f'{TUTORIAL_TOPIC_PREFIX}/reset_world',
            request_json='{}',
        ))
        return bool(result.success), result.response_json or ''
    except Exception as e:
        return False, str(e)


@tutorial_bp.route('/tutorial:reset_world', methods=['POST'])
def tutorial_reset_world():
    """Manually trigger an object-only reset (cube position etc.)."""
    ok, msg = reset_tutorial_world()
    if not ok:
        return jsonify({'status': 'error', 'message': f'Reset failed: {msg}'}), 500
    return jsonify({'status': 'success', 'message': 'Tutorial world objects reset'}), 200


@tutorial_bp.route('/tutorial:stop', methods=['POST'])
def tutorial_stop():
    """Stop the bundled MuJoCo world. DB rows are kept (toggle stays idempotent).

    Note: 명시적으로 subscribe_robot_* 워커를 stop_function 하지 않는다.
    sim 토픽이 사라지면 subscribe_robot_topic이 예외 → break로 자연 종료하며,
    그 시점에 동시에 일어나는 gRPC 스트림 종료 / engineio 웹소켓 송신과
    socketio.emit이 race를 이뤄 werkzeug threaded 모드의 C 확장 조합에서
    SIGSEGV를 낸 적이 있었다(2026-04 사례). 자연 종료 경로가 더 안전.
    """
    client = get_bridge_client()
    try:
        client.driver.StopLaunch(pb.ProcessId(name=TUTORIAL_LAUNCH_PROCESS_ID))
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500

    return jsonify({'status': 'success', 'message': 'Tutorial world stopped'}), 200


@tutorial_bp.route('/tutorial:reset', methods=['POST'])
def tutorial_reset():
    """Snap the MuJoCo world back to the home keyframe."""
    ok, message = reset_tutorial_world()
    if not ok:
        return jsonify({'status': 'error', 'message': message}), 400
    return jsonify({'status': 'success', 'message': message or 'reset'}), 200
