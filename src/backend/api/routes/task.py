from flask import Blueprint, request, current_app, Response
from ...database.models.task_model import Task as TaskModel
import json
import time

task_bp = Blueprint('task', __name__)

@task_bp.route('/tasks', methods=['GET'])
def get_tasks():
    tasks = TaskModel.all()
    tasks = [task.to_dict() for task in tasks]
    return {
        'status': 'success',
        'tasks': tasks
    }, 200

@task_bp.route('/tasks/<id>', methods=['GET'])
def get_task(id):
    task = TaskModel.find(id)
    if not task:
        return {'status': 'error', 'message': 'Task not found'}, 404
    return {
        'status': 'success',
        'task': task.to_dict()
    }, 200
    
@task_bp.route('/task:start_training', methods=['POST'])
def start_training():
    data = request.json
    task_id = data.get('task_id')
    policy_id = data.get('policy_id')
    load_model_id = data.get('load_model_id')
    dataset_ids = data.get('dataset_ids', [])
    checkpoint_id = data.get('checkpoint_id', None)
    num_epochs = data.get('num_epochs', 100)
    batch_size = data.get('batch_size', 32)
    
    print(data)

    if not task_id or not policy_id:
        return {'status': 'error', 'message': 'task_id and policy_id are required'}, 400

    command_list = ['python3', '-u', '-m', 'backend.scripts.train',
                    '--task_id', str(task_id), '--policy_id', str(policy_id),
                    '--dataset_ids', json.dumps(dataset_ids),
                    '--checkpoint_id', str(checkpoint_id),
                    '--num_epochs', str(num_epochs), '--batch_size', str(batch_size)]
    
    if load_model_id:
        command_list.extend(['--load_model_id', str(load_model_id)])

    process_id = f"train_task_{task_id}"
    process = current_app.pm.start_process(process_id, command_list)
    
    if process:
        return {
            'status': 'success',
            'message': f'Training started for task {task_id} with policy {policy_id}',
            'process_id': process_id,
            'pid': process.pid
        }, 200
    else:
        return {'status': 'error', 'message': f'Failed to train'}, 500
    
@task_bp.route('/task:stop_training', methods=['POST'])
def stop_training():
    task_id = request.json.get('task_id')
    if not task_id:
        return {'status': 'error', 'message': 'task_id is required'}, 400

    process_id = f"train_task_{task_id}"
    current_app.pm.stop_process(process_id)
    
    return {'status': 'success', 'message': 'Training stopped'}, 200
    
    
@task_bp.route('/task', methods=['POST'])
def create_task():
    data = request.json
    TaskModel.create(
        name=data.get('name'),
        robot_ids=data.get('robot_ids'),
        sensor_ids=data.get('sensor_ids'),
        home_pose=data.get('home_pose'),
        end_pose=data.get('end_pose'),
        prompt=data.get('prompt'),
        image=data.get('image'),
        episode_len=data.get('episode_len'),
        dataset_dir=data.get('dataset_dir')
    )
    return {'status': 'success', 'message': 'Task Created'}, 200

@task_bp.route('/task/<id>', methods=['PUT'])
def update_task(id):
    data = request.json
    task = TaskModel.find(id)
    task.name = data.get('name')
    task.robot_ids = data.get('robot_ids')
    task.sensor_ids = data.get('sensor_ids')
    task.home_pose = data.get('home_pose')
    task.end_pose = data.get('end_pose')
    task.image = data.get('image')
    task.episode_len = data.get('episode_len')
    task.sensor_img_size = data.get('sensor_img_size')
    task.save()
    return {'status': 'success', 'message': 'Task Updated'}, 200

@task_bp.route('/task/<id>', methods=['DELETE'])
def delete_task(id):
    task = TaskModel.find(id)
    task.delete()
    return {'status': 'success', 'message': 'Task Deleted'}, 200
