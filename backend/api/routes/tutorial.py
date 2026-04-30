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

from flask import Blueprint, current_app, jsonify

from ...bridge.client import get_bridge_client
from ...bridge.generated import robot_bridge_pb2 as pb
from ...configs.tutorial_defaults import (
    TUTORIAL_LAUNCH_FILE,
    TUTORIAL_LAUNCH_PACKAGE,
    TUTORIAL_LAUNCH_PROCESS_ID,
    TUTORIAL_ROBOT,
    TUTORIAL_SENSOR,
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


def _find_tutorial_sensor():
    candidates = SensorModel.select().where(
        SensorModel.hide == False,  # noqa: E712
        SensorModel.deleted_at.is_null(),
        SensorModel.type == TUTORIAL_SENSOR['type'],
    )
    for row in candidates:
        if _settings_dict(row).get('is_tutorial'):
            return row
    return None


def _ensure_tutorial_rows():
    """Create tutorial robot/sensor DB rows if they don't already exist.

    Also migrates already-seeded tutorial rows by merging in any newer keys
    from TUTORIAL_ROBOT/SENSOR['settings'] that the existing row is missing
    (e.g. urdf_path / ik_setting added after first install). Existing keys
    are preserved so user edits aren't clobbered.
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
        added = {k: v for k, v in TUTORIAL_ROBOT['settings'].items() if k not in current}
        if added:
            current.update(added)
            robot.settings = json.dumps(current)
            robot.save()

    sensor = _find_tutorial_sensor()
    if sensor is None:
        sensor = SensorModel.create(
            name=TUTORIAL_SENSOR['name'],
            type=TUTORIAL_SENSOR['type'],
            settings=json.dumps(TUTORIAL_SENSOR['settings']),
        )
    else:
        current = _settings_dict(sensor)
        added = {k: v for k, v in TUTORIAL_SENSOR['settings'].items() if k not in current}
        if added:
            current.update(added)
            sensor.settings = json.dumps(current)
            sensor.save()

    return robot, sensor


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


def _topics_active():
    """Check whether the sim's published topics are visible on the ROS graph."""
    try:
        client = get_bridge_client()
        result = client.driver.ListTopics(pb.Empty())
        names = {t.name for t in result.topics}
    except Exception:
        return False
    return (
        f'{TUTORIAL_TOPIC_PREFIX}/joint_states' in names
        or f'{TUTORIAL_TOPIC_PREFIX}/camera/image_raw/compressed' in names
    )


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@tutorial_bp.route('/tutorial/status', methods=['GET'])
def tutorial_status():
    robot = _find_tutorial_robot()
    sensor = _find_tutorial_sensor()
    return jsonify({
        'status': 'success',
        'running': _is_sim_running(),
        'has_topics': _topics_active(),
        'robot_id': robot.id if robot else None,
        'sensor_id': sensor.id if sensor else None,
    }), 200


@tutorial_bp.route('/tutorial:start', methods=['POST'])
def tutorial_start():
    """Launch the bundled MuJoCo world and seed tutorial DB rows."""
    robot, sensor = _ensure_tutorial_rows()

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
        'sensor_id': sensor.id,
    }), 200


@tutorial_bp.route('/tutorial:stop', methods=['POST'])
def tutorial_stop():
    """Stop the bundled MuJoCo world. DB rows are kept (toggle stays idempotent)."""
    client = get_bridge_client()
    try:
        client.driver.StopLaunch(pb.ProcessId(name=TUTORIAL_LAUNCH_PROCESS_ID))
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Bridge call failed: {e}'}), 500

    # Also tear down any robot/sensor subscriptions that may still be active.
    robot = _find_tutorial_robot()
    if robot is not None:
        try:
            current_app.pm.stop_function(f'subscribe_robot_{robot.id}')
        except Exception:
            pass

    return jsonify({'status': 'success', 'message': 'Tutorial world stopped'}), 200
