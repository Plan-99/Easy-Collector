from flask import Blueprint, request, current_app
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
import os
import shutil

from ..process.checkpoint_test import checkpoint_test


checkpoint_bp = Blueprint('checkpoint_bp', __name__)

CHECKPOINT_DIR = '/root/src/backend/checkpoints'

@checkpoint_bp.route('/checkpoints', methods=['GET'])
def get_checkpoints():
    query = request.args.get('where', None)
    query = query.split('|') if query else []
    checkpointsQuery = CheckpointModel.with_('policy', 'task', 'load_model')
    for q in query:
        qarr = q.split(',')
        checkpointsQuery = checkpointsQuery.where(qarr[0], qarr[1], qarr[2])

    checkpoints = checkpointsQuery.get()    
    checkpoints = [checkpoint.to_dict() for checkpoint in checkpoints]
    return {
        'status': 'success', 'checkpoints': checkpoints}, 200


@checkpoint_bp.route('/checkpoints/<folder_name>', methods=['GET'])
def get_checkpoint_files(id):
    folder_path = os.path.join(CHECKPOINT_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    return {'status': 'success', 'files': files}, 200


@checkpoint_bp.route('/checkpoint', methods=['POST'])
def create_checkpoint():
    data = request.json
    new_checkpoint = CheckpointModel.create(
        name=data.get('name'),
        task_id=data.get('task_id'),
        policy_id=data.get('policy_id'),
        dataset_info=data.get('dataset_info', []),
        num_epochs=data.get('num_epochs'),
        batch_size=data.get('batch_size'),
    )
    return {'status': 'success', 'message': 'Checkpoint Created', 'id': new_checkpoint.id}, 200


@checkpoint_bp.route('/checkpoint/<id>/:check_create_successed', methods=['GET'])
def check_create_successed(id):
    checkpoint = CheckpointModel.find(id)
    if not checkpoint:
        return {'check_create_successed': False, 'message': 'Checkpoint creation failed'}, 200

    folder_path = os.path.join(CHECKPOINT_DIR, str(checkpoint.id))
    if os.path.exists(folder_path) and os.path.isdir(folder_path):
        return {'check_create_successed': True, 'message': 'Checkpoint created successfully'}, 200
    else:
        return {'check_create_successed': False, 'message': 'Checkpoint creation failed'}, 200
    
    
@checkpoint_bp.route('/checkpoint/<id>/:start_test', methods=['POST'])
def start_test(id):
    data = request.json
    current_app.pm.start_function(
        func=checkpoint_test,
        node=current_app.node,
        checkpoint_id=id,
        task=data.get('task'),
        policy_obj=data.get('policy'),
        robots=data.get('robots'),
        sensors=data.get('sensors'),
        socketio_instance=current_app.pm.socketio,
        name=f"checkpoint_test_{id}",
    )
    
    return {'status': 'success', 'message': 'Checkpoint test started'}, 200

@checkpoint_bp.route('/checkpoint/<id>/:stop_test', methods=['POST'])
def stop_test(id):
    current_app.pm.stop_function(
        name=f"checkpoint_test_{id}",
    )
    return {'status': 'success', 'message': 'Checkpoint test stopped'}, 200



@checkpoint_bp.route('/checkpoint/<id>', methods=['PUT'])
def update_checkpoint(id):
    data = request.json
    checkpoint = CheckpointModel.find(id)
    if not checkpoint:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404

    checkpoint.name = data.get('name', checkpoint.name)
    checkpoint.save()
    
    return {'status': 'success', 'message': 'Checkpoint Updated'}, 200


@checkpoint_bp.route('/checkpoint/<id>', methods=['DELETE'])
def delete_checkpoint(id):
    checkpoint = CheckpointModel.find(id)
    if not checkpoint:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404

    folder_path = os.path.join(CHECKPOINT_DIR, str(checkpoint.id))
    if os.path.exists(folder_path) and os.path.isdir(folder_path):
        shutil.rmtree(folder_path)

    checkpoint.delete()
    return {'status': 'success', 'message': 'Checkpoint Deleted'}, 200

