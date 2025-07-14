from flask import Blueprint, request
from ...database.models.dataset_model import Dataset as DatasetModel
import os

dataset_bp = Blueprint('dataset_bp', __name__)

DATASET_DIR = '/root/src/backend/datasets'

@dataset_bp.route('/datasets', methods=['GET'])
def get_datasets():
    datasets = DatasetModel.all()
    datasets = [dataset.to_dict() for dataset in datasets]
    return {
        'status': 'success', 'datasets': datasets}, 200


@dataset_bp.route('/datasets/<folder_name>', methods=['GET'])
def get_dataset_files(id):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    return {'status': 'success', 'files': files}, 200


@dataset_bp.route('/dataset', methods=['POST'])
def create_dataset():
    data = request.json
    new_dataset = DatasetModel.create(
        name=data.get('name'),
        task_id=data.get('task_id'),
    )
    os.mkdir(os.path.join(DATASET_DIR, str(new_dataset.id)))
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
        os.rmdir(folder_path)

    dataset.delete()
    return {'status': 'success', 'message': 'Dataset Deleted'}, 200
    
    
# def get_dataset(folder_name):
#     folder_path = os.path.join(DATASET_DIR, folder_name)
#     if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
#         return jsonify({'status': 'error', 'message': 'Folder not found'}), 404

#     files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
#     return jsonify({'status': 'success', 'files': files})
