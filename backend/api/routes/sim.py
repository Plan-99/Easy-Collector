# -*- coding: utf-8 -*-
"""
Simulation API routes.
시뮬레이션 시작/정지/관절 제어 등 센서 로직과 완전히 분리된 API.
"""
from flask import Blueprint, request, jsonify, current_app, Response
import os
import cv2
import time

sim_bp = Blueprint('sim', __name__)


# 모듈 레벨 공유 — streaming 서버(별도 스레드)에서도 접근 가능
_shared_sim_engine = None


def get_sim_engine():
    """Get the shared SimEngine instance (streaming 서버에서도 사용)."""
    return _shared_sim_engine


def _get_engine():
    """Get or create the shared SimEngine instance."""
    global _shared_sim_engine
    engine = getattr(current_app, '_sim_engine', None)
    if engine is None:
        from ...sim.engine import SimEngine
        engine = SimEngine()
        current_app._sim_engine = engine
        _shared_sim_engine = engine
    return engine


@sim_bp.route('/sim/start', methods=['POST'])
def sim_start():
    """Start a simulation with a robot ID.

    Body:
        robot_id: int — robot ID from database
        cam_config: dict (optional) — {width, height, fov, distance, yaw, pitch, target}
    """
    from ...database.models.robot_model import Robot
    from ...sim.urdf_registry import get_urdf_path

    data = request.json or {}
    robot_id = data.get('robot_id')

    if not robot_id:
        return jsonify({'error': 'robot_id is required'}), 400

    # Fetch robot from DB
    robot = Robot.find(robot_id)
    if not robot:
        return jsonify({'error': f'Robot not found: {robot_id}'}), 404

    urdf_path = get_urdf_path(robot.type)

    if not urdf_path:
        return jsonify({'error': f'No URDF configured for robot: {robot.type}'}), 404

    if not os.path.isfile(urdf_path):
        return jsonify({'error': f'URDF not found: {urdf_path}'}), 404

    engine = _get_engine()
    cam_config = data.get('cam_config', None)

    try:
        engine.start(urdf_path, cam_config)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

    # Register with streaming server
    streaming_app = getattr(current_app, '_streaming_app', None)
    if streaming_app:
        streaming_app['sim_engine'] = engine

    return jsonify({
        'status': 'running',
        'joints': engine.get_joint_info(),
        'robot_id': robot_id,
        'robot_type': robot.type,
    })


@sim_bp.route('/sim/stop', methods=['POST'])
def sim_stop():
    """Stop the simulation."""
    engine = _get_engine()
    engine.stop()
    return jsonify({'status': 'stopped'})


@sim_bp.route('/sim/status', methods=['GET'])
def sim_status():
    """Get simulation status."""
    engine = _get_engine()
    if not engine.is_running:
        return jsonify({'status': 'stopped'})

    return jsonify({
        'status': 'running',
        'joints': engine.get_joint_states(),
    })


@sim_bp.route('/sim/joints', methods=['POST'])
def sim_set_joints():
    """Set joint target positions.

    Body:
        positions: dict[str(joint_index), float] — {0: 1.57, 1: -0.5, ...}
    """
    engine = _get_engine()
    if not engine.is_running:
        return jsonify({'error': 'Simulation not running'}), 400

    data = request.json or {}
    positions = data.get('positions', {})

    # Convert string keys to int
    positions = {int(k): float(v) for k, v in positions.items()}
    engine.set_joint_positions(positions)

    return jsonify({'status': 'success'})


@sim_bp.route('/sim/joints', methods=['GET'])
def sim_get_joints():
    """Get current joint info."""
    engine = _get_engine()
    if not engine.is_running:
        return jsonify({'error': 'Simulation not running'}), 400

    return jsonify({
        'info': engine.get_joint_info(),
    })


@sim_bp.route('/sim/camera', methods=['POST'])
def sim_set_camera():
    """Update simulation camera parameters.

    Body: {distance, yaw, pitch, target, fov, width, height}
    """
    engine = _get_engine()
    if not engine.is_running:
        return jsonify({'error': 'Simulation not running'}), 400

    data = request.json or {}
    engine.set_camera(**data)
    return jsonify({'status': 'success'})


@sim_bp.route('/sim/stream', methods=['GET'])
def sim_stream():
    """MJPEG 스트리밍 — <img> 태그로 직접 사용 가능."""
    engine = _get_engine()

    def generate():
        while engine.is_running:
            frame = engine.render_frame()
            if frame is not None:
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            else:
                time.sleep(0.03)

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
