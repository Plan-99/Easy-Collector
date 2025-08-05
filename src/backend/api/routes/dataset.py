from flask import Blueprint, request, current_app
from ...database.models.dataset_model import Dataset as DatasetModel
import os
import shutil
from ..process.read_hdf5 import read_hdf5, add_config
from ..process.record_episode import record_episode
from ..process.augment_dataset import augment_dataset
import base64
from io import BytesIO
from PIL import Image
import h5py

dataset_bp = Blueprint('dataset_bp', __name__)

DATASET_DIR = '/root/src/backend/datasets'

@dataset_bp.route('/datasets', methods=['GET'])
def get_datasets():
    params = request.args
    print(params)
    task_id = params.get('task_id')
    datasets = DatasetModel.where('task_id', task_id).get() if task_id else DatasetModel.all()
    datasets = [dataset.to_dict() for dataset in datasets]
    return {
        'status': 'success', 'datasets': datasets}, 200


@dataset_bp.route('/datasets/<id>', methods=['GET'])
def get_dataset_files(id):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    files = [{ 'name': f } for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    files = sorted(
        files, key=lambda x: os.path.getmtime(os.path.join(folder_path, x['name'])), reverse=False
    )
    return {'status': 'success', 'files': files}, 200


@dataset_bp.route('/datasets/<id>/:get_one', methods=['GET'])
def get_dataset_file(id):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    files = [{ 'name': f } for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

    return {'status': 'success', 'file': files[0]}, 200



@dataset_bp.route('/dataset', methods=['POST'])
def create_dataset():
    data = request.json
    new_dataset = DatasetModel.create(
        name=data.get('name'),
        task_id=data.get('task_id'),
    )
    dataset_path = os.path.join(DATASET_DIR, str(new_dataset.id))
    os.makedirs(dataset_path, exist_ok=True)

    return {'status': 'success', 'message': 'Dataset Created'}, 200


@dataset_bp.route('/dataset/<id>', methods=['PUT'])
def update_dataset(id):
    data = request.json
    dataset = DatasetModel.find(id)
    if not dataset:
        return {'status': 'error', 'message': 'Dataset not found'}, 404

    dataset.name = data.get('name', dataset.name)
    # dataset.task_id = data.get('task_id', dataset.task_id)
    dataset.save()
    
    return {'status': 'success', 'message': 'Dataset Updated'}, 200


@dataset_bp.route('/dataset/<id>', methods=['DELETE'])
def delete_dataset(id):
    dataset = DatasetModel.find(id)
    if not dataset:
        return {'status': 'error', 'message': 'Dataset not found'}, 404

    folder_path = os.path.join(DATASET_DIR, str(dataset.id))
    if os.path.exists(folder_path) and os.path.isdir(folder_path):
        shutil.rmtree(folder_path)

    dataset.delete()
    return {'status': 'success', 'message': 'Dataset Deleted'}, 200

@dataset_bp.route('/dataset/<id>/<file_name>', methods=['DELETE'])
def delete_dataset_file(id, file_name):
    dataset_path = os.path.join(DATASET_DIR, id, file_name)
    if not os.path.exists(dataset_path) or not os.path.isfile(dataset_path):
        return {'status': 'error', 'message': 'File not found'}, 404

    os.remove(dataset_path)
    return {'status': 'success', 'message': 'File Deleted'}, 200
    

@dataset_bp.route('/dataset/<id>/<file_name>/:start_read_hdf5', methods=['POST'])
def start_read_hdf5(id, file_name):
    current_app.pm.start_function(
        func=read_hdf5,
        name=f"read_hdf5_{id}_{file_name}",
        hdf5_path=os.path.join(DATASET_DIR, id, file_name),
        socketio_instance=current_app.pm.socketio,
        sid=request.json.get('sid', None),  # Optional socket ID for real-time updates
    ),
    return {'status': 'success', 'message': 'HDF5 reading process started'}, 200


@dataset_bp.route('/dataset/<id>/<file_name>/:stop_read_hdf5', methods=['POST'])
def stop_read_hdf5(id, file_name):
    current_app.pm.stop_function(
        name=f"read_hdf5_{id}_{file_name}",
    ),
    return {'status': 'success', 'message': 'HDF5 reading process stopped'}, 200


@dataset_bp.route('/dataset/<id>/<file_name>/:read_hdf5_add_config', methods=['POST'])
def read_hdf5_add_config(id, file_name):
    add_config(request.json)
    return {'status': 'success', 'message': 'Configuration added to HDF5 reading process'}, 200

    


@dataset_bp.route('/dataset/<id>/:start_collection', methods=['POST'])
def start_collection(id):
    data = request.json
    current_app.pm.start_function(
        func=record_episode,
        node=current_app.node,
        dataset_id=id,
        robots=data.get('robots'),
        sensors=data.get('sensors'),
        task=data.get('task'),
        tele_type=data.get('tele_type', 'leader'),
        socketio_instance=current_app.pm.socketio,
        name=f"record_episode_dataset{id}",
    )

    return {'status': 'success', 'message': 'Data collection started'}, 200

@dataset_bp.route('/dataset/<id>/:stop_collection', methods=['POST'])
def stop_collection(id):
    current_app.pm.stop_function(
        name=f"record_episode_dataset{id}",
    )
    return {'status': 'success', 'message': 'Data collection stopped'}, 200

@dataset_bp.route('/dataset/<id>/augment', methods=['POST'])
def augment_dataset_route(id):
    data = request.json
    name = data.get('name')
    task_id = data.get('task_id')
    
    agumented_dataset = DatasetModel.create(
        name=name,
        task_id=task_id,
    )
    
    current_app.pm.start_function(
        func=augment_dataset,
        dataset_id = id,
        aug_dataset_id=agumented_dataset.id,
        lightness=data.get('lightness'),
        rectangles=data.get('rectangles'),
        salt_and_pepper=data.get('saltAndPepper'),
        gaussian=data.get('gaussian'),
        socketio_instance=current_app.pm.socketio,
        name=f"augment_dataset",
    )
    
    return {'status': 'success', 'message': 'Dataset augmentation started'}, 200