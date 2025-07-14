from flask import Blueprint, request, current_app
from ...database.models.task_model import Task as TaskModel
import json

task_bp = Blueprint('task', __name__)

@task_bp.route('/tasks', methods=['GET'])
def get_tasks():
    tasks = TaskModel.with_('robot').get()
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

@task_bp.route('/task', methods=['POST'])
def create_task():
    data = request.json
    TaskModel.create(
        name=data.get('name'),
        robot_id=data.get('robot_id'),
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
    task.robot_id = data.get('robot_id')
    task.sensor_ids = data.get('sensor_ids')
    task.home_pose = data.get('home_pose')
    task.end_pose = data.get('end_pose')
    task.image = data.get('image')
    task.episode_len = data.get('episode_len')
    task.save()
    return {'status': 'success', 'message': 'Task Updated'}, 200

@task_bp.route('/task/<id>', methods=['DELETE'])
def delete_task(id):
    task = TaskModel.find(id)
    task.delete()
    return {'status': 'success', 'message': 'Task Deleted'}, 200
