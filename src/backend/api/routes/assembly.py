from flask import Blueprint, request, current_app, Response
import json
from ...database.models.assembly_model import Assembly as AssemblyModel
from ..utils.runtime import attach_robot_runtime

# 1. Blueprint 생성
# 이 블루프린트는 어셈블리와 관련된 'HTTP' 라우트를 관리합니다.
assembly_bp = Blueprint('assembly', __name__)
@assembly_bp.route('/assemblies', methods=['GET'])
def get_assemblies():
    assemblies = AssemblyModel.where('hide', False).with_(
        'teleoperators',
        'left_arm',
        'right_arm',
        'left_tool',
        'right_tool',
        'mobile_base',    
    ).get()
    assemblies = [assembly.to_dict() for assembly in assemblies]
    processes = set(current_app.pm.list_processes())
    # ROS2 컨테이너의 드라이버 프로세스 목록도 포함
    try:
        from ...bridge.client import get_bridge_client
        from ...bridge.generated import robot_bridge_pb2 as pb
        client = get_bridge_client()
        ros2_procs = client.driver.ListProcesses(pb.Empty())
        processes.update(ros2_procs.names)
    except Exception:
        pass
    for assembly in assemblies:
        if assembly.get('robots'):
            assembly['robots'] = [attach_robot_runtime(robot, processes) for robot in assembly['robots']]
    return {
        'status': 'success',
        'assemblies': assemblies
    }, 200


@assembly_bp.route('/assembly', methods=['POST'])
def create_assembly():
    data = request.json
    assembly = AssemblyModel.create(**data)
    return {
        'status': 'success',
    }, 201


@assembly_bp.route('/assembly/<id>', methods=['PUT'])
def update_assembly(id):
    data = request.json
    assembly = AssemblyModel.find(id)
    for key, value in data.items():
        setattr(assembly, key, value)
    assembly.save()
    return {
        'status': 'success',
        'assembly': assembly.to_dict()
    }, 200


@assembly_bp.route('/assembly/<id>', methods=['DELETE'])
def delete_assembly(id):
    assembly = AssemblyModel.find(id)
    assembly.hide = True
    assembly.save()
    return {
        'status': 'success',
        'message': 'Assembly Hidden'
    }, 200
