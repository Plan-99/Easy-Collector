from flask import Blueprint, request, current_app
from ...database.models.dataset_model import Dataset as DatasetModel
import os
import shutil
from ..process.read_hdf5 import read_hdf5, add_config
from ..process.record_episode import record_episode
from ..process.augment_dataset import augment_dataset
from ..process.merge_dataset import merge_dataset
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


@dataset_bp.route('/dataset/<id>/:get_datasets_metadata', methods=['GET'])
def get_datasets_metadata(id):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    metadata = {}
    if os.listdir(folder_path):
        first_file = os.listdir(folder_path)[0]
        hdf5_path = os.path.join(folder_path, first_file)
        with h5py.File(hdf5_path, 'r') as f:
            sensor_names = [name for name in f["observations/images"].keys()]
            robot_names = [name for name in f["observations/qpos"].keys()]

            metadata['sensors'] = sensor_names
            metadata['robots'] = robot_names

    return {'status': 'success', 'metadata': metadata}, 200

@dataset_bp.route('/dataset/<id>/:edit_datasets_metadata', methods=['POST'])
def edit_datasets_metadata(id):
    data = request.json
    sensor_mappings = data.get('sensor_mappings', {})
    robot_mappings = data.get('robot_mappings', {})
    
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404
    
    for file_name in os.listdir(folder_path):
        hdf5_path = os.path.join(folder_path, file_name)
        with h5py.File(hdf5_path, 'a') as f:
            
            # 1. 센서 이름 변경 (Temporary renaming)
            # key: "sensor_1", value: 2 -> new_name: "sensor_2"
            for old_name, val in sensor_mappings.items():
                new_name = f"sensor_{val}"
                if old_name in f["observations/images"]:
                    # 바로 new_name으로 바꾸지 않고 임시 이름으로 변경
                    f["observations/images"].move(old_name, f"{new_name}_temp")

            # 2. 센서 이름 확정 (Finalize)
            for val in sensor_mappings.values():
                target = f"sensor_{val}"
                if f"{target}_temp" in f["observations/images"]:
                    f["observations/images"].move(f"{target}_temp", target)

            # 3. 로봇 관련 데이터셋 변경 (Temporary renaming)
            # 처리해야 할 경로 리스트
            robot_paths = [
                "observations/qpos", "observations/eepos", 
                "qaction", "qaction_delta", "eetarget", "eetarget_delta"
            ]

            for old_name, val in robot_mappings.items():
                new_name = f"robot_{val}"
                for path in robot_paths:
                    if path in f and old_name in f[path]:
                        f[path].move(old_name, f"{new_name}_temp")

            # 4. 로봇 이름 확정 (Finalize)
            for val in robot_mappings.values():
                target = f"robot_{val}"
                for path in robot_paths:
                    if path in f and f"{target}_temp" in f[path]:
                        f[path].move(f"{target}_temp", target)

    return {'status': 'success', 'message': 'Metadata updated'}, 200

@dataset_bp.route('/dataset/<id>/<file_name>/:start_read_hdf5', methods=['POST'])
def start_read_hdf5(id, file_name):
    current_app.pm.start_function(
        func=read_hdf5,
        node=current_app.node,
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

@dataset_bp.route('/dataset/<id>/<file_name>/:start_replay_episode', methods=['POST'])
def start_replay_episode(id, file_name):
    data = request.json
    hdf5_path = os.path.join(DATASET_DIR, id, file_name)
    if not os.path.exists(hdf5_path) or not os.path.isfile(hdf5_path):
        return {'status': 'error', 'message': 'HDF5 file not found'}, 404
    
    agents = [current_app.agents[agent_id] for agent_id in data.get('robot_ids', [])]

    current_app.pm.stop_function(
        name=f"read_hdf5_{id}_{file_name}",
    )

    current_app.pm.start_function(
        func=read_hdf5,
        node=current_app.node,
        name=f"replay_episode",
        hdf5_path=hdf5_path,
        socketio_instance=current_app.pm.socketio,
        agents=agents,
        task=data.get('task', {}),
        sensors=data.get('sensors', []),
        move_robot=True,
        sid=request.json.get('sid', None),  # Optional socket ID for real-time updates
    )
    return {'status': 'success', 'message': 'HDF5 replay process started'}, 200

@dataset_bp.route('/dataset/<id>/<file_name>/:stop_replay_episode', methods=['POST'])
def stop_replay_episode(id, file_name):
    current_app.pm.stop_function(
        name=f"replay_episode",
    )
    return {'status': 'success', 'message': 'HDF5 replay process stopped'}, 200

@dataset_bp.route('/dataset/<id>/<file_name>/:read_hdf5_add_config', methods=['POST'])
def read_hdf5_add_config(id, file_name):
    add_config(request.json)
    return {'status': 'success', 'message': 'Configuration added to HDF5 reading process'}, 200

    


@dataset_bp.route('/dataset/<id>/:start_collection', methods=['POST'])
def start_collection(id):
    data = request.json
    agents = [current_app.agents[robot['id']] for robot in data.get('robots', [])]

    current_app.pm.start_function(
        func=record_episode,
        node=current_app.node,
        dataset_id=id,
        agents=agents,
        move_homepose=data.get('move_homepose', False),
        assembly_id=data.get('assembly_id'),
        sensors=data.get('sensors'),
        task=data.get('task'),
        language_instruction=data.get('language_instruction'),
        tele_type=data.get('tele_type', 'leader'),
        socketio_instance=current_app.pm.socketio,
        iter=data.get('iter', 100000),
        name=f"record_episode",
    )

    return {'status': 'success', 'message': 'Data collection started'}, 200

@dataset_bp.route('/dataset/<id>/:stop_collection', methods=['POST'])
def stop_collection(id):
    current_app.pm.stop_function(
        name=f"record_episode",
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
        prospective=data.get('prospective'),
        hsv=data.get('hsv'),
        socketio_instance=current_app.pm.socketio,
        name=f"augment_dataset",
    )
    
    return {'status': 'success', 'message': 'Dataset augmentation started'}, 200

@dataset_bp.route('/dataset/:merge', methods=['POST'])
def merge_datasets():
    data = request.json
    source_dataset_id = data.get('source_dataset_id')
    target_dataset_ids = data.get('target_dataset_ids', [])
    
    source_dataset = DatasetModel.find(source_dataset_id)
    target_datasets = [DatasetModel.find(int(tid)) for tid in target_dataset_ids]

    merge_dataset(source_dataset, target_datasets)

    return {'status': 'success', 'message': 'Dataset merged successfully'}, 200