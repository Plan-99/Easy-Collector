from flask import Blueprint, request, current_app
from flask_socketio import Namespace, emit
from ...database.models.leader_robot_preset_model import LeaderRobotPreset as LeaderRobotPresetModel
from ..process.leader_teleoperation import Leader
from ...bridge.remote_agent import RemoteAgent as Agent
from ...database.models.robot_model import Robot as RobotModel


leader_robot_bp = Blueprint('leader_robot', __name__)

reading_thread = None

@leader_robot_bp.route('/leader_robot:start', methods=['POST'])
def start_leader_robot():
    data = request.json
    serial_port = data.get('serial_port')

    current_app.pm.start_process(
        name='start_leader_robot',
        command=['ros2', 'launch', 'dynamixel_ros', 'dynamixel_node.launch.py', f'device_port:={serial_port}'],
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
        name='preset_' + str(robot_id),
        robot_id=robot_id,
    )

    preset.dxl_ids = preset_form['dxl_ids']
    preset.origin = preset_form['origin']
    preset.gripper_dxl_range = preset_form['gripper_dxl_range']
    preset.gripper_dxl_ids = preset_form['gripper_dxl_ids']
    preset.sign_corrector = preset_form['sign_corrector']
    preset.port_name = preset_form['port_name']
    preset.ema = preset_form['ema']
    preset.save()


    return {'status': 'success', 'message': 'Leader robot preset saved'}, 200


@leader_robot_bp.route('/leader_robot:tele_start', methods=['POST'])
def start_leader_teleoperation():
    data = request.json
    log_emit_id = data.get('log_emit_id', 'leader_teleoperation')
    robot = data.get('robot')
    preset = data.get('preset')
    tool = data.get('tool', None)

    current_app.pm.stop_process(name='start_leader_robot')
    agent = current_app.agents[robot['id']]

    leader = Leader(current_app.node, agent, current_app.pm.socketio, preset)
    current_app.pm.start_function(
        name='leader_teleoperation',
        func=leader.leader_full_workflow,
        log_id=log_emit_id,
    )

    return {'status': 'success', 'message': 'Leader teleoperation started'}, 200



@leader_robot_bp.route('/leader_robot:tele_stop', methods=['POST'])
def stop_leader_teleoperation():
    current_app.pm.stop_function(name='leader_teleoperation')
    return {'status': 'success', 'message': 'Leader teleoperation stopped'}, 200
