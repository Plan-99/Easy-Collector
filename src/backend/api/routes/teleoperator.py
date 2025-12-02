from flask import Blueprint, request, current_app, Response
import json
from ...database.models.teleoperator_model import Teleoperator as TeleoperatorModel
from ...database.models.assembly_model import Assembly as AssemblyModel
from ...api.process.subscribe_dynamixel import subscribe_dynamixel
from ...api.process.leader_teleoperation import Leader


# 1. Blueprint 생성
# 이 블루프린트는 텔레오퍼레이터와 관련된 'HTTP' 라우트를 관리합니다.
teleoperator_bp = Blueprint('teleoperator', __name__)

@teleoperator_bp.route('/teleoperators', methods=['GET'])
def get_teleoperators():
    teleoperators = TeleoperatorModel.all()
    teleoperators = [teleop.to_dict() for teleop in teleoperators]
    return {
        'status': 'success',
        'teleoperators': teleoperators
    }, 200


@teleoperator_bp.route('/teleoperator:dxl_read', methods=['POST'])
def dxl_read():
    current_app.pm.start_function(
        f"subscribe_dxl",
        subscribe_dynamixel,
        socketio_instance=current_app.pm.socketio,
    )
    
    return {
        'status': 'success',
    }, 200

@teleoperator_bp.route('/teleoperator:stop_dxl_read', methods=['POST'])
def stop_dxl_read():
    current_app.pm.stop_function("subscribe_dxl")
    
    return {
        'status': 'success',
    }, 200

@teleoperator_bp.route('/teleoperator', methods=['POST'])
def create_teleoperator():
    data = request.json
    teleoperator = TeleoperatorModel.first_or_create(
        assembly_id=data.get('assembly_id'),
        type=data.get('type'),
    )
    teleoperator.settings = data.get('settings', {})
    teleoperator.save()
    return {
        'status': 'success',
        'teleoperator': teleoperator.to_dict()
    }, 201


@teleoperator_bp.route('/teleoperator:leader_start', methods=['POST'])
def start_leader_teleoperation():
    data = request.json
    log_emit_id = data.get('log_emit_id', 'leader_teleoperation')
    assembly_id = data.get('assembly_id')

    assembly = AssemblyModel.find(assembly_id).to_dict()

    robot_ids = list(set(v for v in {
        'left_arm_id': assembly.get('left_arm_id'),
        'right_arm_id': assembly.get('right_arm_id'),
        'left_tool_id': assembly.get('left_tool_id'),
        'right_tool_id': assembly.get('right_tool_id'),
        'mobile_base_id': assembly.get('mobile_base_id'),
    }.values() if v is not None))

    try:
        agents = [current_app.agents[robot_id] for robot_id in robot_ids]
    except KeyError as e:
        current_app.pm.socketio.emit(log_emit_id, {
            'log': f'[ERROR]: Robot is not running now.',
            'type': 'stderr'
        })
        return {'status': 'error', 'message': f'Robot with ID {str(e)} not found.'}, 404

    current_app.pm.stop_function(name='subscribe_dxl')

    teleoperator = TeleoperatorModel.where('assembly_id', assembly_id).where('type', 'leader').first()
    
    leader = Leader(agents, current_app.pm.socketio, teleop_setting=teleoperator.settings, log_emit_id=log_emit_id)
    leader.sync_leader_robot()
    current_app.pm.start_function(
        name='leader_teleoperation',
        func=leader.position_pub,
    )

    return {'status': 'success', 'message': 'Leader teleoperation started'}, 200