from flask import Blueprint, request, current_app, Response
import json
from ...database.models.teleoperator_model import Teleoperator as TeleoperatorModel
from ...database.models.assembly_model import Assembly as AssemblyModel
from ...api.process.subscribe_dynamixel import subscribe_dynamixel, scan_ids_on_port, get_available_ports
from ...api.process.leader_teleoperation import Leader
from ...api.process.xr_teleoperation import XRTeloperator
from ...env.agent import Agent
from ...database.models.robot_model import Robot as RobotModel


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

@teleoperator_bp.route('teleoperator:dxl_check', methods=['GET'])
def dxl_check():
    ports = get_available_ports()
    res = {}
    for port in ports:
        # 해당 포트의 다이나믹셀 ID 스캔
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

@teleoperator_bp.route('/leader_robot:tele_stop', methods=['POST'])
def stop_leader_teleoperation():    
    current_app.pm.stop_function(name='leader_teleoperation')
    return {'status': 'success', 'message': 'Leader teleoperation stopped'}, 200 


@teleoperator_bp.route('/teleoperator:xr_start', methods=['POST'])
def start_xr_teleoperation():
    data = request.json
    log_emit_id = data.get('log_emit_id', 'log_xr_teleoperation')
    assembly_id = data.get('assembly_id')

    assembly = AssemblyModel.find(assembly_id).to_dict()

    # left_tool_agent = current_app.agents.get(assembly.get('left_tool_id'))
    # right_tool_agent = current_app.agents.get(assembly.get('right_tool_id'))


    try:
        if assembly.get('left_arm_id') in current_app.agents:
            left_arm_agent = current_app.agents.get(assembly.get('left_arm_id'))
        else:
            left_arm_agent = Agent(node=current_app.node, robot=RobotModel.find(assembly.get('left_arm_id')).to_dict())
            current_app.agents[assembly.get('left_arm_id')] = left_arm_agent
        
        if assembly.get('right_arm_id') in current_app.agents:
            right_arm_agent = current_app.agents.get(assembly.get('right_arm_id'))
        else:
            right_arm_agent = Agent(node=current_app.node, robot=RobotModel.find(assembly.get('right_arm_id')).to_dict())
            current_app.agents[assembly.get('right_arm_id')] = right_arm_agent


        if assembly['id'] in current_app.teleops:
            xr = current_app.teleops[assembly['id']]
        else:
            xr = XRTeloperator(left_arm_agent=left_arm_agent, right_arm_agent=right_arm_agent)
            current_app.teleops[assembly['id']] = xr
            # agents = [current_app.agents[robot_id] for robot_id in robot_ids]
    except KeyError as e:
        current_app.pm.socketio.emit(log_emit_id, {
            'log': f'[ERROR]: Robot is not running now.',
            'type': 'stderr'
        })
        return {'status': 'error', 'message': f'Robot with ID {str(e)} not found.'}, 404

    # XR 텔레오퍼레이터 시작 로직 추가 예정
    current_app.pm.start_function(
        name='xr_teleoperation',
        func=xr.run,
        socketio_instance=current_app.pm.socketio,
        log_emit_id=log_emit_id,
    )
    
    return {'status': 'success', 'message': 'XR teleoperation started'}, 200


@teleoperator_bp.route('/teleoperator:xr_read', methods=['POST'])
def read_xr_teleoperation():
    current_app.pm.change_task_control(
        name='xr_teleoperation',
        key='read',
        value=True
    )
    return {'status': 'success', 'message': 'XR teleoperation read started'}, 200


@teleoperator_bp.route('/teleoperator:xr_stop', methods=['POST'])
def stop_xr_teleoperation():    
    current_app.pm.stop_function(name='xr_teleoperation')
    return {'status': 'success', 'message': 'XR teleoperation stopped'}, 200