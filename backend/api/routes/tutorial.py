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

from flask import Blueprint, jsonify, request

from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
from ...configs.tutorial_defaults import (
    TUTORIAL_AGENT_NAME,
    TUTORIAL_LAUNCH_FILE,
    TUTORIAL_LAUNCH_PACKAGE,
    TUTORIAL_LAUNCH_PROCESS_ID,
    TUTORIAL_ROBOT,
    TUTORIAL_SENSORS,
    TUTORIAL_TOPIC_PREFIX,
    TUTORIAL_WORKSPACE_DEFAULT_SENSOR_CFG,
    TUTORIAL_WORKSPACE_EPISODE_LEN,
    TUTORIAL_WORKSPACE_HOMEPOSE,
    TUTORIAL_WORKSPACE_NAME,
)
from ...database.models.assembly_model import Assembly as AssemblyModel
from ...database.models.robot_model import Robot as RobotModel
from ...database.models.sensor_model import Sensor as SensorModel
from ...database.models.task_model import Task as TaskModel


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
        # is_sim은 record_episode의 home pose 이동 분기를 결정. read_topic 등
        # ROS 토픽 매핑은 비어있으면 useRobot 의 topicStore.isPublished('') 가
        # 항상 false 라 "토픽 OFF" 로 잘못 표시된다 — sim 토픽 이름이 바뀌어도
        # 자동 복구되도록 항상 default 로 덮어쓴다.
        AUTHORITATIVE_KEYS = (
            'is_sim', 'is_tutorial', 'role',
            'read_topic', 'read_topic_msg',
            'write_type', 'write_topic', 'write_topic_msg',
            'joint_names', 'joint_lower_bounds', 'joint_upper_bounds',
            'tool_index', 'tool_inner', 'interpolation',
            'urdf_path', 'urdf_package_dir', 'ik_setting',
        )
        for k in AUTHORITATIVE_KEYS:
            if k in TUTORIAL_ROBOT['settings']:
                current[k] = TUTORIAL_ROBOT['settings'][k]
        # 그 외 새로 추가된 default 키는 머지(기존 사용자 편집 보존)
        for k, v in TUTORIAL_ROBOT['settings'].items():
            current.setdefault(k, v)
        robot.settings = json.dumps(current)
        robot.save()

    # Hide stale tutorial_arm-named rows that don't carry the is_tutorial flag
    # (e.g. very old seeded rows from before is_tutorial existed). 그대로 두면
    # robots 목록에 중복으로 나타나 사용자가 잘못된 행 (settings 비어있는 옛날
    # 행) 을 클릭해 IK 가 없다고 표시되는 사고가 난다.
    stale = RobotModel.select().where(
        RobotModel.name == TUTORIAL_ROBOT['name'],
        RobotModel.id != robot.id,
        RobotModel.hide == False,  # noqa: E712
        RobotModel.deleted_at.is_null(),
    )
    for row in stale:
        row.hide = True
        row.save()

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

    assembly = _ensure_tutorial_assembly(robot)
    workspace = _ensure_tutorial_workspace(assembly, robot, sensors)

    return robot, sensors, assembly, workspace


def _find_tutorial_assembly():
    """Look up the tutorial assembly by name (Assembly has no settings col)."""
    return AssemblyModel.select().where(
        AssemblyModel.name == TUTORIAL_AGENT_NAME,
        AssemblyModel.hide == False,  # noqa: E712
        AssemblyModel.deleted_at.is_null(),
    ).first()


def _ensure_tutorial_assembly(robot):
    """Idempotently create/repair the tutorial Assembly (single-arm, tutorial_arm).

    `left_arm_id` 는 항상 현재 tutorial_arm robot id로 강제 — robot row가 재생성
    되었거나 사용자가 다른 로봇을 임시로 슬롯에 끼워둔 경우에도 자동 복구.
    """
    assembly = _find_tutorial_assembly()
    if assembly is None:
        assembly = AssemblyModel.create(
            name=TUTORIAL_AGENT_NAME,
            left_arm_id=robot.id,
        )
    elif assembly.left_arm_id != robot.id:
        assembly.left_arm_id = robot.id
        assembly.save()
    return assembly


def _find_tutorial_workspace():
    """Look up the tutorial Task row, identified by `is_tutorial` in settings."""
    candidates = TaskModel.select().where(TaskModel.deleted_at.is_null())
    for row in candidates:
        raw = row.settings
        if isinstance(raw, str):
            try:
                data = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
        else:
            data = raw or {}
        if data.get('is_tutorial'):
            return row
    return None


def _ensure_tutorial_workspace(assembly, robot, sensors):
    """Idempotently create/repair the tutorial Task row.

    워크스페이스에는 (1) tutorial_agent assembly, (2) 두 tutorial 카메라,
    (3) home_pose / 센서 설정 default 가 포함된다. 사용자가 episode_len 등을
    바꿔뒀다면 그 값은 보존하고, 비어있는 슬롯만 default로 채운다 (assembly,
    sensor_ids는 단일 진실원천이므로 항상 강제).
    """
    workspace = _find_tutorial_workspace()
    sensor_ids = [s.id for s in sensors]
    default_sensor_cfg = {
        str(sid): dict(TUTORIAL_WORKSPACE_DEFAULT_SENSOR_CFG)
        for sid in sensor_ids
    }
    default_robots_cfg = {
        str(robot.id): {'home_pose': list(TUTORIAL_WORKSPACE_HOMEPOSE)},
    }

    if workspace is None:
        settings = {
            'is_tutorial': True,
            'robots': default_robots_cfg,
            'sensors': default_sensor_cfg,
        }
        workspace = TaskModel.create(
            name=TUTORIAL_WORKSPACE_NAME,
            assembly_id=assembly.id,
            episode_len=TUTORIAL_WORKSPACE_EPISODE_LEN,
            sensor_ids=json.dumps(sensor_ids),
            settings=json.dumps(settings),
            home_pose=json.dumps({}),
        )
        return workspace

    # 기존 row가 있으면 단일 진실원천 키만 강제 동기화
    workspace.assembly_id = assembly.id
    workspace.sensor_ids = json.dumps(sensor_ids)
    if not workspace.episode_len:
        workspace.episode_len = TUTORIAL_WORKSPACE_EPISODE_LEN

    current = workspace._settings if hasattr(workspace, '_settings') else {}
    if not isinstance(current, dict):
        current = {}
    current['is_tutorial'] = True
    current.setdefault('robots', {})
    current.setdefault('sensors', {})
    # robot 슬롯이 비어 있으면 default home_pose로 채움
    if str(robot.id) not in current['robots']:
        current['robots'][str(robot.id)] = {'home_pose': list(TUTORIAL_WORKSPACE_HOMEPOSE)}
    elif 'home_pose' not in current['robots'][str(robot.id)]:
        current['robots'][str(robot.id)]['home_pose'] = list(TUTORIAL_WORKSPACE_HOMEPOSE)
    # 센서 설정도 누락된 slug만 default로 채움
    for sid in sensor_ids:
        if str(sid) not in current['sensors']:
            current['sensors'][str(sid)] = dict(TUTORIAL_WORKSPACE_DEFAULT_SENSOR_CFG)
    workspace.settings = json.dumps(current)
    workspace.save()
    return workspace


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
    assembly = _find_tutorial_assembly()
    workspace = _find_tutorial_workspace()
    return jsonify({
        'status': 'success',
        'running': _is_sim_running(),
        'has_topics': _topics_active(),
        'robot_id': robot.id if robot else None,
        'sensor_ids': [s.id for s in sensors],
        'assembly_id': assembly.id if assembly else None,
        'workspace_id': workspace.id if workspace else None,
    }), 200


@tutorial_bp.route('/tutorial:start', methods=['POST'])
def tutorial_start():
    """Launch the bundled MuJoCo world. DB rows are guaranteed by app startup,
    but call _ensure_tutorial_rows() again as a safety net (idempotent).

    Optional body:
        show_viewer (bool): false 로 주면 native viewer 창 없이 headless로 기동
            (offscreen 카메라는 정상 동작). 자동화/CI 환경에서 사용.
            누락/null 이면 기존 동작(true) 유지.
    """
    robot, sensors, assembly, workspace = _ensure_tutorial_rows()

    body = request.get_json(silent=True) or {}
    args = {
        'topic_prefix': TUTORIAL_TOPIC_PREFIX,
    }
    if 'show_viewer' in body and body['show_viewer'] is not None:
        # ros2 launch 의 args 는 CLI 문자열로 직렬화된다 ('show_viewer:=false').
        # 노드 declare_parameter 가 bool 디폴트라 ROS2 가 'true'/'false' 문자열을
        # bool 로 coerce 해주므로 소문자 string 형태로 보낸다.
        args['show_viewer'] = 'true' if bool(body['show_viewer']) else 'false'

    client = get_bridge_client()
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
        'assembly_id': assembly.id,
        'workspace_id': workspace.id,
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
