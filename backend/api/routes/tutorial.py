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
from ...configs.sim_devices import SIM_ENV_BINDINGS
from ...configs.tutorial_defaults import (
    SIM_ARM_2,
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


def _find_robot_by_sim_slug(slug: str):
    """Find a canonical sim robot row by its `sim_device_slug` (rename-safe)."""
    candidates = RobotModel.select().where(
        RobotModel.hide == False,  # noqa: E712
        RobotModel.deleted_at.is_null(),
        RobotModel.type == 'custom',
    )
    for row in candidates:
        if _settings_dict(row).get('sim_device_slug') == slug:
            return row
    return None


def _find_sensor_by_sim_slug(slug: str):
    """Find a canonical sim camera row by its `sim_device_slug` (rename-safe)."""
    candidates = SensorModel.select().where(
        SensorModel.hide == False,  # noqa: E712
        SensorModel.deleted_at.is_null(),
    )
    for row in candidates:
        if _settings_dict(row).get('sim_device_slug') == slug:
            return row
    return None


def repoint_sim_devices(env_id):
    """주어진 시뮬 환경의 바인딩(SIM_ENV_BINDINGS)을 표준 디바이스로 갈아끼운다.

    각 표준 디바이스(slug)의 settings 를 그 환경 설정으로 **완전히 교체**한다
    (sim_device_slug / is_sim 은 강제 유지). 환경 전환 시 옛 키가 남지 않도록
    merge 가 아닌 replace.

    반환: ({robot_slug: RobotModel}, {camera_slug: SensorModel}) — 워크스페이스
    리포인트/플래너 블록 remap 에 쓰도록 실제 행을 돌려준다.
    알 수 없는 env 면 ({}, {})."""
    binding = SIM_ENV_BINDINGS.get(env_id)
    if not binding:
        return {}, {}
    robots = {}
    for slug, settings in (binding.get('robots') or {}).items():
        row = _find_robot_by_sim_slug(slug)
        if row is None:
            continue
        s = dict(settings)
        s['sim_device_slug'] = slug
        s['is_sim'] = True
        row.settings = json.dumps(s)
        row.save()
        robots[slug] = row
    cameras = {}
    for slug, settings in (binding.get('cameras') or {}).items():
        row = _find_sensor_by_sim_slug(slug)
        if row is None:
            continue
        s = dict(settings)
        s['sim_device_slug'] = slug
        s['is_sim'] = True
        row.settings = json.dumps(s)
        row.save()
        cameras[slug] = row
    return robots, cameras


def find_sim_devices(env_id):
    """주어진 환경 바인딩에 속한 표준 디바이스 행들을 **수정 없이** 조회만 한다.
    (startup 시 assembly/workspace 만 구성하고 토픽 리포인트는 활성화 때만 하려고
    repoint 와 분리.) 반환: ({robot_slug: RobotModel}, {camera_slug: SensorModel})."""
    binding = SIM_ENV_BINDINGS.get(env_id)
    if not binding:
        return {}, {}
    robots = {}
    for slug in (binding.get('robots') or {}):
        row = _find_robot_by_sim_slug(slug)
        if row is not None:
            robots[slug] = row
    cameras = {}
    for slug in (binding.get('cameras') or {}):
        row = _find_sensor_by_sim_slug(slug)
        if row is not None:
            cameras[slug] = row
    return robots, cameras


def _ensure_sim_arm_2():
    """두 번째 표준 sim 암(sim_arm_2)을 시드. 단일팔이며 양팔 시뮬 환경에서
    오른팔로 쓰인다(환경 활성화 시 토픽 리포인트). Idempotent — 매 부팅 호출 안전.
    설치 시 로봇 페이지에 sim_arm 과 함께 보이도록 startup 에 시드한다."""
    slug = SIM_ARM_2['settings']['sim_device_slug']
    row = _find_robot_by_sim_slug(slug)
    if row is None:
        row = RobotModel.create(
            name=SIM_ARM_2['name'],
            type=SIM_ARM_2['type'],
            settings=json.dumps(SIM_ARM_2['settings']),
            homepose=json.dumps(SIM_ARM_2['homepose']),
        )
    else:
        current = _settings_dict(row)
        # 토픽/IK 등은 환경 활성화가 리포인트하므로 여기선 머지만(사용자 편집 보존).
        for k, v in SIM_ARM_2['settings'].items():
            current.setdefault(k, v)
        current['sim_device_slug'] = slug
        current['is_sim'] = True
        row.settings = json.dumps(current)
        row.name = SIM_ARM_2['name']
        row.save()
    return row


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
    # 표준 로봇 sim_arm 존재 보장. sim_device_slug 로 찾고(환경 바인딩이 토픽/
    # is_tutorial 을 바꿔도 안정적 — 데모 활성화 후에도 같은 행을 찾는다), 없으면
    # 레거시 tutorial_arm(is_tutorial) 을 마이그레이션, 그래도 없으면 생성.
    # 실제 토픽/IK/is_tutorial 바인딩은 아래 repoint_sim_devices('tutorial') 가 적용.
    robot = _find_robot_by_sim_slug('arm') or _find_tutorial_robot()
    if robot is None:
        robot = RobotModel.create(
            name=TUTORIAL_ROBOT['name'],
            type=TUTORIAL_ROBOT['type'],
            settings=json.dumps(TUTORIAL_ROBOT['settings']),
            homepose=json.dumps(TUTORIAL_ROBOT['homepose']),
        )
    else:
        # 표준 이름/슬러그로 마이그레이션 (tutorial_arm → sim_arm). ID 보존 →
        # 기존 워크스페이스/데이터셋/커리큘럼 참조가 깨지지 않는다.
        current = _settings_dict(robot)
        current['sim_device_slug'] = 'arm'
        current.setdefault('is_sim', True)
        robot.settings = json.dumps(current)
        robot.name = TUTORIAL_ROBOT['name']
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

    # 두 번째 표준 sim 암(sim_arm_2)도 설치 시 함께 시드 — 로봇 페이지에 노출.
    _ensure_sim_arm_2()

    # 표준 카메라 cam/cam_2/cam_3 존재 보장. sim_device_slug 로 찾고(안정적), 없으면
    # 레거시 tutorial_camera_slug 로 마이그레이션, 그래도 없으면 생성. 토픽 바인딩은
    # 아래 repoint('tutorial') 가 적용.
    sensors = []
    for spec in TUTORIAL_SENSORS:
        slug = spec['settings']['sim_device_slug']
        tut_slug = spec['settings']['tutorial_camera_slug']
        row = _find_sensor_by_sim_slug(slug) or _find_tutorial_sensor_by_slug(tut_slug)
        if row is None:
            row = SensorModel.create(
                name=spec['name'],
                type=spec['type'],
                settings=json.dumps(spec['settings']),
            )
        else:
            current = _settings_dict(row)
            current['sim_device_slug'] = slug
            row.settings = json.dumps(current)
            # 표준 이름으로 마이그레이션 (tutorial_top_cam → cam 등). ID 보존.
            row.name = spec['name']
            row.save()
        sensors.append(row)

    # 레거시 sensor 정리 — sim_device_slug 가 없는 옛 tutorial 카메라(top/front
    # 분리 전 단일 `tutorial_camera` 등)는 숨긴다. 표준 카메라는 slug 가 있어 안전.
    for row in _find_tutorial_sensors():
        if not _settings_dict(row).get('sim_device_slug'):
            row.hide = True
            row.save()

    # 레거시 sim 디바이스 정리 — 이제 모든 시뮬 환경이 표준 디바이스(sim_arm/
    # sim_arm_2/cam*)를 공유(repoint)하므로 환경 전용 행이 필요 없다:
    #   - demo_sim_device : 데모 전용 sim_arm / sim_wrist_cam*
    #   - sim_test_env    : dual_arm_test_robot, dual_arm_assembly_left/right + 그 카메라
    # 표준 디바이스에는 이 마커가 없어 영향 없음.
    for RowModel in (RobotModel, SensorModel):
        for row in RowModel.select().where(
            RowModel.hide == False,  # noqa: E712
            RowModel.deleted_at.is_null(),
        ):
            s = _settings_dict(row)
            if s.get('demo_sim_device') or s.get('sim_test_env'):
                row.hide = True
                row.save()

    # 은퇴한 dual_arm_test(14관절 단일로봇) 워크스페이스 soft-delete — 두 단일팔
    # 방식(dual_arm_assembly_test)으로 대체됐다. dual_arm_assembly_test 워크스페이스는
    # canonical 시딩이 표준 디바이스로 리포인트해 유지하므로 건드리지 않는다.
    for ws in TaskModel.select().where(TaskModel.deleted_at.is_null()):
        if _settings_dict(ws).get('sim_test_env') == 'dual_arm_test':
            ws.delete_instance()  # SoftDeleteModel → deleted_at 세팅(soft)

    # 표준 디바이스에 tutorial 바인딩 적용(토픽/IK/is_tutorial 복원). 데모 등 다른
    # 환경을 쓴 뒤에도 tutorial 시딩 시 항상 tutorial 토픽/동작으로 되돌린다.
    repoint_sim_devices('tutorial')

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
