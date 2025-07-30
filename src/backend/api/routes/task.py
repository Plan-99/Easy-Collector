from flask import Blueprint, request, current_app, Response
from ...database.models.task_model import Task as TaskModel
import json
import time
from ..process.train import train_task
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

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
    dataset_info = data.get('dataset_info', {})
    dataset_ids = list(dataset_info.keys())
    load_model_id = data.get('load_model_id', None)
    num_epochs = data.get('num_epochs', 100)
    batch_size = data.get('batch_size', 32)
    learning_rate = data.get('learning_rate', 1e-5)
    lr_backbone = data.get('lr_backbone', 1e-6)

    new_checkpoint = CheckpointModel.create(
        name=data.get('name'),
        task_id=task_id,
        policy_id=data.get('policy_id'),
        dataset_info=dataset_info,
        is_training=True,
        num_epochs=num_epochs,
        batch_size=batch_size,
        learning_rate=learning_rate,
        lr_backbone=lr_backbone,
        load_model_id=load_model_id,
    )

    # current_app.pm.start_function(
    #     f'train_task_{new_checkpoint.id}',
    #     func=train_task,
    #     task_id=task_id,
    #     policy_id=policy_id,
    #     dataset_ids=dataset_ids,
    #     checkpoint_id=new_checkpoint.id,
    #     num_epochs=data.get('num_epochs', 100),
    #     batch_size=data.get('batch_size', 32),
    #     load_model_id=data.get('load_model_id', None),
    #     socketio_instance=current_app.pm.socketio,
    # )

    # return {'status': 'success', 'message': 'Task training started', 'process_id': f'train_task_{new_checkpoint.id}' }, 200 
    command_list = ['python3', '-u', '-m', 'backend.scripts.train',
                    '--task_id', str(task_id), '--policy_id', str(policy_id),
                    '--dataset_ids', json.dumps(dataset_ids),
                    '--checkpoint_id', str(new_checkpoint.id),
                    '--num_epochs', str(num_epochs), '--batch_size', str(batch_size),
                    '--lr_backbone', str(lr_backbone), '--learning_rate', str(learning_rate)
                    ]
    
    if load_model_id:
        command_list.extend(['--load_model_id', str(load_model_id)])

    process_id = f"train_task"
    print(command_list)
    process = current_app.pm.start_process(process_id, command_list)
    
    if process:
        return {
            'status': 'success',
            'message': f'Training started for task {task_id} with policy {policy_id}',
            'process_id': process_id,
            'checkpoint_id': new_checkpoint.id,
            'pid': process.pid
        }, 200
    else:
        return {'status': 'error', 'message': f'Failed to train'}, 500
    
@task_bp.route('/task:stop_training', methods=['POST'])
def stop_training():
    current_app.pm.stop_process('train_task')
    CheckpointModel.where('is_training', 1).first().delete()  # Delete the checkpoint if it exists
    
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
