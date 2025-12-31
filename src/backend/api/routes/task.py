from flask import Blueprint, request, current_app, Response
from ...database.models.task_model import Task as TaskModel
import json
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
from ...database.models.assembly_model import Assembly as AssemblyModel
from ..utils.runtime import attach_robot_runtime

from ..process.failure_detection import failure_detection

import os
import shutil

task_bp = Blueprint('task', __name__)

@task_bp.route('/tasks', methods=['GET'])
def get_tasks():
    tasks = TaskModel.with_('assembly').get()
    tasks = [task.to_dict() for task in tasks]
    processes = set(current_app.pm.list_processes())
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


@task_bp.route('/task:start_training', methods=['POST'])
def start_training():
    data = request.json
    checkpoint_id = data.get('checkpoint_id')

    command_list = ['python3', '-u', '-m', 'backend.scripts.train',
                    '--checkpoint_id', str(checkpoint_id)]

    process_id = f"train_task"

    current_app.pm.process_queue['train_task'].append({
        'checkpoint_id': checkpoint_id,
        'command': command_list
    })

    if process_id not in current_app.pm.processes:
        process = current_app.pm.start_process(process_id, command_list)
    
        return {
            'status': 'success',
            'message': f'Training started for checkpoint {checkpoint_id}',
            'process_id': process_id,
            'checkpoint_id': checkpoint_id,
            'pid': process.pid
        }, 200
    else:
        return {
            'status': 'success',
            'message': f'Training queued for checkpoint {checkpoint_id}',
            'process_id': process_id,
            'checkpoint_id': checkpoint_id,
            'pid': None
        }, 200


@task_bp.route('/task:stop_training', methods=['POST'])
def stop_training():
    checkpoint = CheckpointModel.where('status', 'training').first()
    if checkpoint:
        checkpoint.delete()  # Delete the checkpoint if it exists

    current_app.pm.stop_process('train_task')
    
    temp_dir = "/root/src/backend/datasets/tmp"
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
    return {'status': 'success', 'message': 'Training stopped'}, 200

@task_bp.route('/task:cancel_training', methods=['POST'])
def cancel_training():
    data = request.json
    checkpoint_id = data.get('checkpoint_id')
    checkpoint = CheckpointModel.find(checkpoint_id)
    if checkpoint:
        checkpoint.delete()  # Delete the checkpoint if it exists
    
    current_app.pm.process_queue['train_task'] = [proc for proc in current_app.pm.process_queue['train_task'] if proc['checkpoint_id'] != checkpoint_id]
    
    return {'status': 'success', 'message': 'Training cancelled'}, 200
    

    
@task_bp.route('/task', methods=['POST'])
def create_task():
    data = request.json
    TaskModel.create(
        name=data.get('name'),
        # robot_ids=data.get('robot_ids'),
        # sensor_ids=data.get('sensor_ids'),
        # home_pose=data.get('home_pose'),
        # end_pose=data.get('end_pose'),
        # prompt=data.get('prompt'),
        # image=data.get('image'),
        # episode_len=data.get('episode_len'),
        # dataset_dir=data.get('dataset_dir')
    )
    return {'status': 'success', 'message': 'Task Created'}, 200

@task_bp.route('/task/<id>', methods=['PUT'])
def update_task(id):
    data = request.json
    task = TaskModel.find(id)
    task.name = data.get('name', task.name)
    task.assembly_id = data.get('assembly_id', task.assembly_id)
    task.sensor_ids = data.get('sensor_ids', task.sensor_ids)
    task.home_pose = data.get('home_pose', task.home_pose)
    task.image = data.get('image', task.image)
    task.episode_len = data.get('episode_len', task.episode_len)
    task.sensor_img_size = data.get('sensor_img_size', task.sensor_img_size)
    task.settings = data.get('settings', task.settings)

    task.save()
    return {'status': 'success', 'message': 'Task Updated'}, 200

@task_bp.route('/task/<id>/device_settings', methods=['PUT'])  
def update_task_device_settings(id):
    data = request.json
    task = TaskModel.find(id)

    key = data.get('key')
    setting = data.get('setting')
    device_type = data.get('device_type')  # 'sensor' or 'robot'

    original_settings = task.settings
    for device_id, value in setting.items():
        if device_id not in task.settings.get(device_type):
            original_settings[device_type][device_id] = {}
        
        original_settings[device_type][device_id][key] = value

    task.settings = original_settings

    task.save()
    return {'status': 'success', 'message': 'Task Settings Updated'}, 200

    # device_id = data.get('device_id')
    # device_type = data.get('device_type')  # 'sensor' or 'robot'
    # key = data.get('key')
    # value = data.get('value')

    # setting = task.settings[device_type + 's']
    # if device_id not in setting:
    #     setting[device_id] = {}

    # setting[device_id][key] = value
    # task.settings[device_type + 's'] = setting

    # task.save()
    # return {'status': 'success', 'message': 'Task Settings Updated'}, 200   

@task_bp.route('/task/<id>', methods=['DELETE'])
def delete_task(id):
    task = TaskModel.find(id)
    task.delete()
    return {'status': 'success', 'message': 'Task Deleted'}, 200
