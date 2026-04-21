from flask import Blueprint, request, current_app
from ...database.models.dataset_model import Dataset as DatasetModel
from ...configs.global_configs import DATASET_DIR
import os
import shutil
from ..process.read_dataset import read_dataset, add_config
from ..process.record_episode import record_episode
from ..process.augment_dataset import augment_dataset
from ..process.merge_dataset import merge_dataset
from ..process.lerobot_io import (
    get_episodes_as_file_list, get_dataset_metadata, get_dataset_info,
    delete_episode as lerobot_delete_episode, list_episodes,
)

dataset_bp = Blueprint('dataset_bp', __name__)

@dataset_bp.route('/datasets', methods=['GET'])
def get_datasets():
    params = request.args
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

    files = get_episodes_as_file_list(folder_path)
    return {'status': 'success', 'files': files}, 200


@dataset_bp.route('/datasets/<id>/:get_one', methods=['GET'])
def get_dataset_file(id):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    files = get_episodes_as_file_list(folder_path)
    if not files:
        return {'status': 'error', 'message': 'No episodes found'}, 404

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

@dataset_bp.route('/dataset/<id>/<episode_name>', methods=['DELETE'])
def delete_dataset_file(id, episode_name):
    """Delete a single episode from LeRobot dataset."""
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path):
        return {'status': 'error', 'message': 'Dataset not found'}, 404

    # Parse episode index from name (episode_000000 or episode_0)
    ep_name_clean = episode_name.replace(".hdf5", "")
    ep_index_str = ep_name_clean.replace("episode_", "")
    episode_index = int(ep_index_str)

    try:
        lerobot_delete_episode(folder_path, episode_index)
        return {'status': 'success', 'message': 'Episode Deleted'}, 200
    except Exception as e:
        return {'status': 'error', 'message': str(e)}, 500


@dataset_bp.route('/dataset/<id>/:get_datasets_metadata', methods=['GET'])
def get_datasets_metadata(id):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    metadata = get_dataset_metadata(folder_path)
    return {'status': 'success', 'metadata': metadata}, 200

@dataset_bp.route('/dataset/<id>/:edit_datasets_metadata', methods=['POST'])
def edit_datasets_metadata(id):
    """Remap sensor/robot names in a LeRobot dataset.

    Expects JSON body:
        sensor_mappings: { "sensor_3": 5 }   → rename sensor_3 to sensor_5
        robot_mappings:  { "robot_1": 4 }     → rename robot_1 to robot_4
    """
    from ..process.lerobot_io import (
        _read_json, _write_json, _read_jsonl, _write_jsonl,
        PARQUET_PATH_TEMPLATE, INFO_PATH, EPISODES_PATH, EPISODES_STATS_PATH,
        DEFAULT_CHUNK_SIZE,
    )
    import pyarrow.parquet as pq
    import pyarrow as pa
    import pandas as pd

    data = request.json
    print(f"[edit_datasets_metadata] Raw request data: {data}")
    sensor_mappings = data.get('sensor_mappings', {})
    robot_mappings = data.get('robot_mappings', {})
    print(f"[edit_datasets_metadata] sensor_mappings={sensor_mappings}, robot_mappings={robot_mappings}")

    # Filter out None values
    sensor_mappings = {k: v for k, v in sensor_mappings.items() if v is not None}
    robot_mappings = {k: v for k, v in robot_mappings.items() if v is not None}
    print(f"[edit_datasets_metadata] After filter: sensor_mappings={sensor_mappings}, robot_mappings={robot_mappings}")

    if not sensor_mappings and not robot_mappings:
        print("[edit_datasets_metadata] No mappings to apply, returning early")
        return {'status': 'success', 'message': 'No mappings to apply'}, 200

    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    info = _read_json(os.path.join(folder_path, INFO_PATH))
    features = info.get("features", {})
    episodes = _read_jsonl(os.path.join(folder_path, EPISODES_PATH))

    # 1. Build sensor rename map (ALL mappings, even identity ones are needed for swap)
    sensor_col_remap = {}  # old_feature_key -> new_feature_key
    for old_name, new_id in sensor_mappings.items():
        old_col = f"observation.images.{old_name}"
        new_col = f"observation.images.sensor_{new_id}"
        if old_col in features:
            sensor_col_remap[old_col] = new_col

    # 2. Build robot rename map
    robot_name_remap = {}
    for old_name, new_id in robot_mappings.items():
        robot_name_remap[old_name] = f"robot_{new_id}"

    print(f"[edit_meta] sensor_col_remap={sensor_col_remap}")
    print(f"[edit_meta] robot_name_remap={robot_name_remap}")

    # 3. Update features in info.json (swap-safe: build entirely new dict)
    new_features = {}
    for key, feat in features.items():
        new_key = sensor_col_remap.get(key, key)
        new_feat = dict(feat)
        if "names" in new_feat and new_feat["names"]:
            new_names = []
            for name in new_feat["names"]:
                replaced = name
                for old_prefix, new_prefix in robot_name_remap.items():
                    if replaced.startswith(old_prefix + "_"):
                        replaced = new_prefix + replaced[len(old_prefix):]
                        break
                new_names.append(replaced)
            new_feat["names"] = new_names
        new_features[new_key] = new_feat
    info["features"] = new_features
    _write_json(info, os.path.join(folder_path, INFO_PATH))
    print(f"[edit_meta] Updated info.json features keys: {list(new_features.keys())}")

    # 4. Rename image/video directories (swap-safe via temp names)
    images_root = os.path.join(folder_path, "images")
    if os.path.isdir(images_root):
        # Image mode dirs: sensor_N/
        img_dir_remap = {}
        for old_name, new_id in sensor_mappings.items():
            new_name = f"sensor_{new_id}"
            if old_name != new_name:
                img_dir_remap[old_name] = new_name
        if img_dir_remap:
            print(f"[edit_meta] Renaming image dirs: {img_dir_remap}")
            _rename_dirs_safe(images_root, img_dir_remap)

    videos_root = os.path.join(folder_path, "videos")
    if os.path.isdir(videos_root):
        for chunk_dir_name in os.listdir(videos_root):
            chunk_path = os.path.join(videos_root, chunk_dir_name)
            if not os.path.isdir(chunk_path):
                continue
            # Video dirs: observation.images.{old_name} -> observation.images.sensor_{new_id}
            video_dir_remap = {}
            for old_col, new_col in sensor_col_remap.items():
                if old_col != new_col:
                    video_dir_remap[old_col] = new_col
            # Also check if dirs have original cam names (from convert_hdf5_package)
            existing_dirs = set(os.listdir(chunk_path))
            for old_name, new_id in sensor_mappings.items():
                old_feature_key = f"observation.images.{old_name}"
                new_feature_key = f"observation.images.sensor_{new_id}"
                if old_feature_key in existing_dirs and old_feature_key != new_feature_key:
                    video_dir_remap[old_feature_key] = new_feature_key
            if video_dir_remap:
                print(f"[edit_meta] Renaming video dirs in {chunk_dir_name}: {video_dir_remap}")
                _rename_dirs_safe(chunk_path, video_dir_remap)

    # 5. Update parquet files (rename columns if image-mode)
    for ep_entry in episodes:
        ep_idx = ep_entry["episode_index"]
        chunk = ep_idx // info.get("chunks_size", DEFAULT_CHUNK_SIZE)
        parquet_path = os.path.join(folder_path, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=ep_idx))
        if not os.path.exists(parquet_path):
            continue

        table = pq.read_table(parquet_path)
        df = table.to_pandas()

        actual_renames = {k: v for k, v in sensor_col_remap.items() if k in df.columns and k != v}
        if actual_renames:
            df = df.rename(columns=actual_renames)
            for old_col, new_col in actual_renames.items():
                if new_col in df.columns:
                    old_sensor = old_col.replace("observation.images.", "")
                    new_sensor = new_col.replace("observation.images.", "")
                    df[new_col] = df[new_col].str.replace(
                        f"images/{old_sensor}/", f"images/{new_sensor}/", regex=False
                    )
            pq.write_table(pa.Table.from_pandas(df), parquet_path)

    print(f"[edit_meta] Done. sensor_col_remap applied: {len(sensor_col_remap)}, robot_name_remap applied: {len(robot_name_remap)}")
    return {'status': 'success', 'message': 'Metadata updated'}, 200


def _rename_dirs_safe(parent_dir, name_map):
    """Rename directories under parent_dir using temp names to avoid conflicts."""
    for old_name, new_name in name_map.items():
        old_dir = os.path.join(parent_dir, old_name)
        if os.path.isdir(old_dir):
            shutil.move(old_dir, os.path.join(parent_dir, f"_tmp_{new_name}"))
    for new_name in name_map.values():
        tmp_dir = os.path.join(parent_dir, f"_tmp_{new_name}")
        if os.path.isdir(tmp_dir):
            final_dir = os.path.join(parent_dir, new_name)
            if os.path.isdir(final_dir):
                shutil.rmtree(final_dir)
            shutil.move(tmp_dir, final_dir)

@dataset_bp.route('/dataset/<id>/<episode_name>/:start_read_dataset', methods=['POST'])
def start_read_dataset(id, episode_name):
    episode_path = os.path.join(DATASET_DIR, id, episode_name)
    current_app.pm.start_function(
        func=read_dataset,
        node=current_app.node,
        name=f"read_dataset_{id}_{episode_name}",
        episode_path=episode_path,
        socketio_instance=current_app.pm.socketio,
        sid=request.json.get('sid', None),
    ),
    return {'status': 'success', 'message': 'Episode reading process started'}, 200


@dataset_bp.route('/dataset/<id>/<episode_name>/:stop_read_dataset', methods=['POST'])
def stop_read_dataset(id, episode_name):
    current_app.pm.stop_function(
        name=f"read_dataset_{id}_{episode_name}",
    ),
    return {'status': 'success', 'message': 'Episode reading process stopped'}, 200

@dataset_bp.route('/dataset/<id>/<episode_name>/:start_replay_episode', methods=['POST'])
def start_replay_episode(id, episode_name):
    data = request.json
    episode_path = os.path.join(DATASET_DIR, id, episode_name)

    agents = [current_app.agents[agent_id] for agent_id in data.get('robot_ids', [])]

    current_app.pm.stop_function(
        name=f"read_dataset_{id}_{episode_name}",
    )

    current_app.pm.start_function(
        func=read_dataset,
        node=current_app.node,
        name=f"replay_episode",
        episode_path=episode_path,
        socketio_instance=current_app.pm.socketio,
        agents=agents,
        task=data.get('task', {}),
        sensors=data.get('sensors', []),
        move_robot=True,
        action_key=data.get('action_type', 'qaction'),
        sid=request.json.get('sid', None),
        hz=data.get('hz', 5),
        capture_dataset_id=data.get('capture_dataset_id', None),
    )
    return {'status': 'success', 'message': 'Episode replay process started'}, 200

@dataset_bp.route('/dataset/<id>/<episode_name>/:stop_replay_episode', methods=['POST'])
def stop_replay_episode(id, episode_name):
    current_app.pm.stop_function(
        name=f"replay_episode",
    )
    return {'status': 'success', 'message': 'Episode replay process stopped'}, 200

@dataset_bp.route('/dataset/<id>/<episode_name>/:read_dataset_add_config', methods=['POST'])
def read_dataset_add_config(id, episode_name):
    add_config(request.json)
    return {'status': 'success', 'message': 'Configuration added to reading process'}, 200



@dataset_bp.route('/dataset/<id>/:start_collection', methods=['POST'])
def start_collection(id):
    data = request.json
    agents = [current_app.agents[robot['id']] for robot in data.get('robots', [])]
    tele_type = data.get('tele_type', 'leader')

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
        tele_type=tele_type,
        ros2_service=data.get('ros2_service', ''),
        socketio_instance=current_app.pm.socketio,
        iter=data.get('iter', 100000),
        hz=data.get('hz', 20),
        name=f"record_episode",
    )

    return {'status': 'success', 'message': 'Data collection started'}, 200

@dataset_bp.route('/dataset/<id>/:stop_collection', methods=['POST'])
def stop_collection(id):
    current_app.pm.stop_function(
        name=f"record_episode",
    )
    return {'status': 'success', 'message': 'Data collection stopped'}, 200

@dataset_bp.route('/dataset/<id>/:complete_episode', methods=['POST'])
def complete_episode(id):
    task = current_app.pm.processes.get('record_episode')
    if not task or task['type'] != 'function':
        return {'status': 'error', 'message': 'record_episode is not running'}, 404
    task['obj']['episode_complete'] = True
    return {'status': 'success', 'message': 'Episode complete signal sent'}, 200

@dataset_bp.route('/dataset/<id>/:set_succeed', methods=['POST'])
def set_succeed(id):
    task = current_app.pm.processes.get('record_episode')
    if not task or task['type'] != 'function':
        return {'status': 'error', 'message': 'record_episode is not running'}, 404
    task['obj']['succeed'] = True
    return {'status': 'success', 'message': 'Succeed flag set'}, 200

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
