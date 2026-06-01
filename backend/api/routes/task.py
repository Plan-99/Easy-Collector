from flask import Blueprint, request, current_app, Response
from ...database.models.task_model import Task as TaskModel
import json
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
from ...database.models.assembly_model import Assembly as AssemblyModel
from ..utils.runtime import attach_robot_runtime

from ..process.failure_detection import failure_detection

task_bp = Blueprint('task', __name__)

@task_bp.route('/tasks', methods=['GET'])
def get_tasks():
    tasks = TaskModel.select().where(TaskModel.deleted_at.is_null())
    tasks = [task.to_dict() for task in tasks]
    processes = set(current_app.pm.list_processes())
    try:
        from ...bridge.client import get_bridge_client
        from ...bridge.generated import robot_bridge_pb2 as pb
        client = get_bridge_client()
        # 짧은 timeout — ros2 gRPC worker pool 포화 시 route 가 매달리지 않도록.
        ros2_procs = client.driver.ListProcesses(pb.Empty(), timeout=2.0)
        processes.update(ros2_procs.names)
    except Exception:
        pass
    for task in tasks:
        if task.get('assembly') and task['assembly'].get('robots'):
            task['assembly']['robots'] = [
                attach_robot_runtime(robot, processes) for robot in task['assembly']['robots']
            ]
    return {
        'status': 'success',
        'tasks': tasks
    }, 200

@task_bp.route('/tasks/<id>', methods=['GET'])
def get_task(id):
    task = TaskModel.find(id)
    if not task:
        return {'status': 'error', 'message': 'Task not found'}, 404
    task_dict = task.to_dict()
    processes = set(current_app.pm.list_processes())
    try:
        from ...bridge.client import get_bridge_client
        from ...bridge.generated import robot_bridge_pb2 as pb
        client = get_bridge_client()
        # 짧은 timeout — ros2 gRPC worker pool 포화 시 route 가 매달리지 않도록.
        ros2_procs = client.driver.ListProcesses(pb.Empty(), timeout=2.0)
        processes.update(ros2_procs.names)
    except Exception:
        pass
    if task_dict.get('assembly') and task_dict['assembly'].get('robots'):
        task_dict['assembly']['robots'] = [
            attach_robot_runtime(robot, processes) for robot in task_dict['assembly']['robots']
        ]
    return {
        'status': 'success',
        'task': task_dict
    }, 200


@task_bp.route('/checkpoint', methods=['POST'])
def create_checkpoint():
    data = request.json
    task_id = data.get('task_id')
    policy_id = data.get('policy_id')
    dataset_info = data.get('dataset_info', {})
    load_model_id = data.get('load_model_id', None)
    checkpoint_name = data.get('name')
    train_settings = data.get('train_settings', {})

    new_checkpoint = CheckpointModel.create(
        name=checkpoint_name,
        task_id=task_id,
        policy_id=policy_id,
        dataset_info=dataset_info,
        status='waiting',
        train_settings=train_settings,
        load_model_id=load_model_id,
    )
    return {'status': 'success', 'message': 'Checkpoint Created', 'id': new_checkpoint.id}, 200


@task_bp.route('/task', methods=['POST'])
def create_task():
    data = request.json
    TaskModel.create(
        name=data.get('name'),
    )
    return {'status': 'success', 'message': 'Task Created'}, 200

@task_bp.route('/task/<id>', methods=['PUT'])
def update_task(id):
    data = request.json
    task = TaskModel.find(id)
    task.name = data.get('name', task.name)
    task.assembly_id = data.get('assembly_id', task.assembly_id)
    task.home_pose = data.get('home_pose', task.home_pose)
    task.image = data.get('image', task.image)
    task.episode_len = data.get('episode_len', task.episode_len)
    task.sensor_ids = data.get('sensor_ids', task.sensor_ids)
    task.settings = data.get('settings', task.settings)

    print("Updated Task Data:", task.to_dict())

    task.save()
    return {'status': 'success', 'message': 'Task Updated'}, 200

@task_bp.route('/task/<id>/device_settings', methods=['PUT'])
def update_task_device_settings(id):
    data = request.json
    task = TaskModel.find(id)

    key = data.get('key')
    setting = data.get('setting')
    device_type = data.get('device_type')

    original_settings = task._settings
    for device_id, value in setting.items():
        if device_id not in original_settings.get(device_type, {}):
            if device_type not in original_settings:
                original_settings[device_type] = {}
            original_settings[device_type][device_id] = {}

        original_settings[device_type][device_id][key] = value

    task.settings = original_settings

    print("Updated Device Settings:", task._settings)

    task.save()
    return {'status': 'success', 'message': 'Task Settings Updated'}, 200

@task_bp.route('/task/<id>', methods=['DELETE'])
def delete_task(id):
    task = TaskModel.find(id)
    task.delete_instance()
    return {'status': 'success', 'message': 'Task Deleted'}, 200
