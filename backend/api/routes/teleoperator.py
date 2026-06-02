from flask import Blueprint, request, current_app, Response
import json
from ...database.models.teleoperator_model import Teleoperator as TeleoperatorModel
from ...database.models.assembly_model import Assembly as AssemblyModel
from ...database.models.robot_model import Robot as RobotModel
from ...api.process.subscribe_dynamixel import subscribe_dynamixel, scan_ids_on_port, get_available_ports
from ...api.process.leader_teleoperation import Leader


def _follower_serial_ports():
    """DB에 등록된 (팔로워) 로봇이 점유 중인 serial_port 목록.

    leader 스캔에서 이 포트들을 제외해야 — 이미 점유 중인 포트를 열려고 하면
    Dynamixel SDK가 무한 대기할 수 있음.
    """
    ports = set()
    try:
        for robot in RobotModel.select().where(RobotModel.deleted_at.is_null()):
            sp = robot.serial_port
            if sp:
                ports.add(sp)
    except Exception as e:
        print(f"[WARN] failed to collect follower serial ports: {e}")
    return list(ports)


teleoperator_bp = Blueprint('teleoperator', __name__)

@teleoperator_bp.route('/teleoperators', methods=['GET'])
def get_teleoperators():
    teleoperators = TeleoperatorModel.all()
    teleoperators = [teleop.to_dict() for teleop in teleoperators]
    return {
        'status': 'success',
        'teleoperators': teleoperators
    }, 200

@teleoperator_bp.route('teleoperator:dxl_check', methods=['GET'])
def dxl_check():
    ports = get_available_ports()
    skip = set(_follower_serial_ports())
    res = {}
    for port in ports:
        if port in skip:
            continue
        connected_ids = scan_ids_on_port(port)
        res[port] = connected_ids

    status = 'success'
    if len(res) == 0:
        status = 'error'

    return {
        'status': status,
        'data': res
    }, 200

@teleoperator_bp.route('/teleoperator:dxl_read', methods=['POST'])
def dxl_read():
    current_app.pm.start_function(
        f"subscribe_dxl",
        subscribe_dynamixel,
        socketio_instance=current_app.pm.socketio,
        skip_ports=_follower_serial_ports(),
        log_id="start_leader_robot",
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
        return {'status': 'error', 'message': f'Robot with ID {str(e)} not found.'}, 404

    current_app.pm.stop_function(name='subscribe_dxl')
    current_app.pm.stop_function(name='leader_teleoperation')

    teleoperator = TeleoperatorModel.select().where(
        TeleoperatorModel.assembly_id == assembly_id,
        TeleoperatorModel.type == 'leader',
        TeleoperatorModel.deleted_at.is_null()
    ).first()

    try:
        leader = Leader(current_app.node, agents, current_app.pm.socketio, teleop_setting=teleoperator._settings)

        current_app.pm.start_function(
            name='leader_teleoperation',
            func=leader.leader_teleop_workflow,
            log_id=log_emit_id,
        )

    except Exception as e:
        import traceback
        traceback_msg = traceback.format_exc()
        print(f"[ERROR] Error during episode recording:\n{traceback_msg}")
        return {'status': 'error', 'message': f'Init Failed: {str(traceback_msg)}'}, 500

    return {'status': 'success', 'message': 'Leader sync and teleop started in background'}, 200
