from flask import Blueprint, request, current_app, Response
import json
from ...database.models.assembly_model import Assembly as AssemblyModel
from ..utils.runtime import attach_robot_runtime

assembly_bp = Blueprint('assembly', __name__)

@assembly_bp.route('/assemblies', methods=['GET'])
def get_assemblies():
    assemblies = AssemblyModel.select().where(AssemblyModel.hide == False, AssemblyModel.deleted_at.is_null())
    assemblies = [assembly.to_dict() for assembly in assemblies]
    processes = set(current_app.pm.list_processes())
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
