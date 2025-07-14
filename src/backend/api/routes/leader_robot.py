from flask import Blueprint, request, current_app
from flask_socketio import Namespace, emit
from ...database.models.leader_robot_preset_model import LeaderRobotPreset as LeaderRobotPresetModel
from ...env.dxl_controller import DxlController


# 1. Blueprint 생성
# 이 블루프린트는 카메라와 관련된 'HTTP' 라우트를 관리합니다.
leader_robot_bp = Blueprint('leader_robot', __name__)

reading_thread = None

@leader_robot_bp.route('/leader_robot:start', methods=['POST'])
def start_leader_robot():
    data = request.json
    serial_port = data.get('serial_port')
    
    current_app.pm.start_process(
        name='start_leader_robot',
        command=['roslaunch', 'dynamixel_ros', 'dynamixel_node.launch', f'device_port:={serial_port}'],
    )

    return {
        'status': 'success',
    }, 200


@leader_robot_bp.route('/leader_robot:stop', methods=['POST'])
def stop_leader_robot():
    current_app.pm.stop_process(name='start_leader_robot')
    return {'status': 'success', 'message': 'Leader robot stopped'}, 200


@leader_robot_bp.route('/leader_robot', methods=['POST'])
def save_leader_robot():
    data = request.json
    robot_id = data.get('robot_id')
    preset_form = data.get('preset')

    preset = LeaderRobotPresetModel.first_or_create(
        name = 'preset_' + str(robot_id),
        robot_id=robot_id,
    )

    preset.dxl_ids = preset_form['dxl_ids']
    preset.origin = preset_form['origin']
    preset.gripper_dxl_range = preset_form['gripper_dxl_range']
    preset.sign_corrector = preset_form['sign_corrector']
    preset.save()

    
    return {'status': 'success', 'message': 'Leader robot preset saved'}, 200


@leader_robot_bp.route('/leader_robot:tele_start', methods=['POST'])
def start_leader_teleoperation():
    data = request.json
    robot_id = data.get('robot_id')
    log_emit_id = data.get('log_emit_id', 'log_leader_teleoperation')

    preset = LeaderRobotPresetModel.where('robot_id', robot_id).first()

    if not preset:
        return {'status': 'error', 'message': 'Leader robot preset not found'}, 404

    current_app.pm.start_process(
        name='leader_teleoperation',
        command=['python3', '-u', '-m', 'backend.teleoperation.dynamixel_publisher', f'--leader_robot_preset_id={preset.id}'],
        log_emit_id=log_emit_id,
    )

    return {'status': 'success', 'message': 'Leader teleoperation started'}, 200



@leader_robot_bp.route('/leader_robot:tele_stop', methods=['POST'])
def stop_leader_teleoperation():    
    current_app.pm.stop_process(name='leader_teleoperation')
    return {'status': 'success', 'message': 'Leader teleoperation stopped'}, 200    