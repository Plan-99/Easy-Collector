from flask import Blueprint, request, current_app, send_file, abort
from ...database.models.dataset_model import Dataset as DatasetModel
from ...configs.global_configs import DATASET_DIR
import os
import shutil
import re
import base64
import cv2
import numpy as np
import pyarrow.parquet as pq
from ..process.read_dataset import read_dataset, add_config
from ..process.record_episode import record_episode
from ..process.augment_dataset import augment_dataset
from ..process.merge_dataset import merge_dataset
from ..process.downsample_dataset import downsample_dataset
from ...utils.lerobot_io import (
    get_episodes_as_file_list, get_dataset_metadata, get_dataset_info,
    delete_episode as lerobot_delete_episode, list_episodes,
    set_episode_language, trim_episode, copy_episode_to,
    PARQUET_PATH_TEMPLATE, INFO_PATH, TASKS_PATH, EPISODES_PATH,
    _read_json, _read_jsonl,
)

dataset_bp = Blueprint('dataset_bp', __name__)

@dataset_bp.route('/datasets', methods=['GET'])
def get_datasets():
    params = request.args
    task_id = params.get('task_id')
    if task_id:
        datasets = DatasetModel.select().where(DatasetModel.task_id == task_id, DatasetModel.deleted_at.is_null())
    else:
        datasets = DatasetModel.all()
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

    return {'status': 'success', 'message': 'Dataset Created', 'dataset_id': new_dataset.id}, 200


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

    dataset.delete_instance()
    return {'status': 'success', 'message': 'Dataset Deleted'}, 200

@dataset_bp.route('/dataset/<id>/<episode_name>', methods=['DELETE'])
def delete_dataset_file(id, episode_name):
    """Delete a single episode from LeRobot dataset."""
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.exists(folder_path):
        return {'status': 'error', 'message': 'Dataset not found'}, 404

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
    """Remap sensor/robot names in a LeRobot dataset."""
    from ...utils.lerobot_io import (
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

    sensor_col_remap = {}
    for old_name, new_id in sensor_mappings.items():
        old_col = f"observation.images.{old_name}"
        new_col = f"observation.images.sensor_{new_id}"
        if old_col in features:
            sensor_col_remap[old_col] = new_col

    robot_name_remap = {}
    for old_name, new_id in robot_mappings.items():
        robot_name_remap[old_name] = f"robot_{new_id}"

    print(f"[edit_meta] sensor_col_remap={sensor_col_remap}")
    print(f"[edit_meta] robot_name_remap={robot_name_remap}")

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

    images_root = os.path.join(folder_path, "images")
    if os.path.isdir(images_root):
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
            video_dir_remap = {}
            for old_col, new_col in sensor_col_remap.items():
                if old_col != new_col:
                    video_dir_remap[old_col] = new_col
            existing_dirs = set(os.listdir(chunk_path))
            for old_name, new_id in sensor_mappings.items():
                old_feature_key = f"observation.images.{old_name}"
                new_feature_key = f"observation.images.sensor_{new_id}"
                if old_feature_key in existing_dirs and old_feature_key != new_feature_key:
                    video_dir_remap[old_feature_key] = new_feature_key
            if video_dir_remap:
                print(f"[edit_meta] Renaming video dirs in {chunk_dir_name}: {video_dir_remap}")
                _rename_dirs_safe(chunk_path, video_dir_remap)

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
    body = request.json or {}
    try:
        start_frame = int(body.get('start_frame') or 0)
    except (TypeError, ValueError):
        start_frame = 0
    current_app.pm.start_function(
        func=read_dataset,
        node=current_app.node,
        name=f"read_dataset_{id}_{episode_name}",
        episode_path=episode_path,
        socketio_instance=current_app.pm.socketio,
        sid=body.get('sid', None),
        start_frame=start_frame,
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
        move_homepose_duration=float(data.get('move_homepose_duration') or 5.0),
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
    return {'status': 'success', 'message': 'Episode finish signal sent'}, 200

@dataset_bp.route('/dataset/<id>/:set_succeed', methods=['POST'])
def set_succeed(id):
    task = current_app.pm.processes.get('record_episode')
    if not task or task['type'] != 'function':
        return {'status': 'error', 'message': 'record_episode is not running'}, 404
    task['obj']['succeed'] = True
    return {'status': 'success', 'message': 'Done flag set'}, 200

@dataset_bp.route('/dataset/<id>/:throw_episode', methods=['POST'])
def throw_episode(id):
    """현재 에피소드를 저장하지 않고 버리고 다음 에피소드로 진행."""
    task = current_app.pm.processes.get('record_episode')
    if not task or task['type'] != 'function':
        return {'status': 'error', 'message': 'record_episode is not running'}, 404
    # throw 플래그 + episode_complete 로 inner loop 즉시 break.
    task['obj']['throw'] = True
    task['obj']['episode_complete'] = True
    return {'status': 'success', 'message': 'Episode throw signal sent'}, 200

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

def _resolve_episode_index(episode_ref):
    """Accept 'episode_000003', 'episode_3', or '3' and return int."""
    s = str(episode_ref).replace('.hdf5', '')
    s = s.replace('episode_', '')
    return int(s)


@dataset_bp.route('/dataset/<id>/<episode_ref>/:get_data', methods=['GET'])
def get_episode_data(id, episode_ref):
    """Return raw episode data for graph plotting / scrubbing.

    Loads parquet only (no image decoding) and returns numeric arrays plus
    feature names so the UI can build per-channel plots.
    """
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Dataset not found'}, 404

    info = get_dataset_info(folder_path)
    if info is None:
        return {'status': 'error', 'message': 'Dataset info not found'}, 404

    try:
        episode_index = _resolve_episode_index(episode_ref)
    except Exception:
        return {'status': 'error', 'message': 'Invalid episode'}, 400

    chunk = episode_index // info["chunks_size"]
    parquet_path = os.path.join(
        folder_path, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_index)
    )
    if not os.path.exists(parquet_path):
        return {'status': 'error', 'message': 'Episode not found'}, 404

    df = pq.read_table(parquet_path).to_pandas()
    num_frames = len(df)
    features = info.get('features', {})

    out = {
        'episode_index': episode_index,
        'num_frames': num_frames,
        'fps': info.get('fps', 20),
    }

    # Numeric channels — frontend graphs build datasets from these.
    numeric_keys = [
        'observation.qpos', 'observation.state',  # 새/옛 schema
        'action', 'action.joint',
        'observation.qvel', 'observation.qeffort',
        'observation.eepos', 'observation.ee_delta', 'action.ee_delta',
    ]
    channels = {}
    for key in numeric_keys:
        if key not in features or key not in df.columns:
            continue
        arr = np.array(df[key].tolist(), dtype=np.float32)
        names = features[key].get('names') or [f'{key}_{i}' for i in range(arr.shape[1])]
        channels[key] = {
            'data': arr.tolist(),
            'names': list(names),
        }
    out['channels'] = channels

    # Sensor list (video features)
    sensors = []
    for k, f in features.items():
        if f.get('dtype') == 'video' and k.startswith('observation.images.'):
            sensors.append(k.replace('observation.images.', ''))
    out['sensors'] = sorted(sensors)

    # Language instruction
    out['language_instruction'] = ''
    if num_frames > 0 and 'task_index' in df.columns:
        task_index = int(df['task_index'].iloc[0])
        for t in _read_jsonl(os.path.join(folder_path, TASKS_PATH)):
            if t.get('task_index') == task_index:
                out['language_instruction'] = t.get('task', '')
                break

    return {'status': 'success', 'episode': out}, 200


@dataset_bp.route('/dataset/<id>/<episode_ref>/:frame', methods=['GET'])
def get_episode_frame(id, episode_ref):
    """Return a single frame (all sensors) at the given index as base64 JPEG.

    Used by the EpisodeViewer slider to seek without restarting the streaming
    process.
    """
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Dataset not found'}, 404

    info = get_dataset_info(folder_path)
    if info is None:
        return {'status': 'error', 'message': 'Dataset info not found'}, 404

    try:
        episode_index = _resolve_episode_index(episode_ref)
    except Exception:
        return {'status': 'error', 'message': 'Invalid episode'}, 400

    try:
        frame_index = int(request.args.get('index', 0))
    except (TypeError, ValueError):
        return {'status': 'error', 'message': 'Invalid frame index'}, 400

    chunk = episode_index // info['chunks_size']
    features = info.get('features', {})

    images = {}
    for k, f in features.items():
        if not k.startswith('observation.images.'):
            continue
        sensor_name = k.replace('observation.images.', '')
        if f.get('dtype') == 'video':
            video_path = os.path.join(
                folder_path, 'videos', f'chunk-{chunk:03d}', k,
                f'episode_{episode_index:06d}.mp4',
            )
            if not os.path.exists(video_path):
                continue
            cap = cv2.VideoCapture(video_path)
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, frame_index))
            ok, frame = cap.read()
            cap.release()
            if not ok or frame is None:
                continue
            ok2, buf = cv2.imencode('.jpg', frame)
            if not ok2:
                continue
            images[sensor_name] = (
                'data:image/jpeg;base64,' + base64.b64encode(buf).decode('utf-8')
            )

    return {'status': 'success', 'frame_index': frame_index, 'images': images}, 200


@dataset_bp.route('/dataset/<id>/<episode_ref>/:video', methods=['GET'])
def get_episode_video(id, episode_ref):
    """Stream the mp4 file for a given sensor of the episode."""
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.isdir(folder_path):
        return abort(404)

    info = get_dataset_info(folder_path)
    if info is None:
        return abort(404)

    try:
        episode_index = _resolve_episode_index(episode_ref)
    except Exception:
        return abort(400)

    sensor = request.args.get('sensor')
    if not sensor:
        return abort(400)

    chunk = episode_index // info['chunks_size']
    feature_key = f'observation.images.{sensor}'
    video_path = os.path.join(
        folder_path, 'videos', f'chunk-{chunk:03d}', feature_key,
        f'episode_{episode_index:06d}.mp4'
    )
    if not os.path.exists(video_path):
        return abort(404)
    return send_file(video_path, mimetype='video/mp4', conditional=True)


@dataset_bp.route('/dataset/<id>/<episode_ref>/:set_language', methods=['POST'])
def set_episode_language_route(id, episode_ref):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Dataset not found'}, 404
    try:
        episode_index = _resolve_episode_index(episode_ref)
    except Exception:
        return {'status': 'error', 'message': 'Invalid episode'}, 400
    language = (request.json or {}).get('language_instruction', '')
    try:
        set_episode_language(folder_path, episode_index, language)
    except Exception as e:
        return {'status': 'error', 'message': str(e)}, 500
    return {'status': 'success', 'message': 'Language updated'}, 200


@dataset_bp.route('/dataset/<id>/<episode_ref>/:trim', methods=['POST'])
def trim_episode_route(id, episode_ref):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Dataset not found'}, 404
    try:
        episode_index = _resolve_episode_index(episode_ref)
    except Exception:
        return {'status': 'error', 'message': 'Invalid episode'}, 400
    data = request.json or {}
    start = int(data.get('start', 0))
    end = int(data.get('end', -1))
    try:
        trim_episode(folder_path, episode_index, start, end)
    except Exception as e:
        return {'status': 'error', 'message': str(e)}, 500
    return {'status': 'success', 'message': 'Episode trimmed'}, 200


@dataset_bp.route('/dataset/<id>/:batch_delete', methods=['POST'])
def batch_delete_episodes(id):
    folder_path = os.path.join(DATASET_DIR, id)
    if not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Dataset not found'}, 404
    indices = (request.json or {}).get('episode_indices', []) or []
    parsed = []
    for ref in indices:
        try:
            parsed.append(_resolve_episode_index(ref))
        except Exception:
            continue
    # Delete in descending order so subsequent indices remain valid in metadata files.
    for idx in sorted(set(parsed), reverse=True):
        try:
            lerobot_delete_episode(folder_path, idx)
        except Exception as e:
            print(f'[batch_delete] failed idx={idx}: {e}')
    return {'status': 'success', 'deleted': len(parsed)}, 200


@dataset_bp.route('/dataset/:batch_copy', methods=['POST'])
def batch_copy_episodes():
    """Copy a list of episodes from source dataset to target dataset."""
    data = request.json or {}
    src_id = str(data.get('source_dataset_id'))
    dst_id = str(data.get('target_dataset_id'))
    indices = data.get('episode_indices', []) or []

    if not src_id or not dst_id:
        return {'status': 'error', 'message': 'source/target dataset_id required'}, 400

    src_dir = os.path.join(DATASET_DIR, src_id)
    dst_dir = os.path.join(DATASET_DIR, dst_id)
    if not os.path.isdir(src_dir) or not os.path.isdir(dst_dir):
        return {'status': 'error', 'message': 'Dataset not found'}, 404

    new_indices = []
    for ref in indices:
        try:
            src_idx = _resolve_episode_index(ref)
            new_idx = copy_episode_to(src_dir, src_idx, dst_dir)
            new_indices.append(new_idx)
        except Exception as e:
            print(f'[batch_copy] failed src_idx={ref}: {e}')
    return {'status': 'success', 'new_indices': new_indices}, 200


@dataset_bp.route('/dataset/:batch_move', methods=['POST'])
def batch_move_episodes():
    """Move episodes from source to target (copy then delete from source)."""
    data = request.json or {}
    src_id = str(data.get('source_dataset_id'))
    dst_id = str(data.get('target_dataset_id'))
    indices = data.get('episode_indices', []) or []

    if not src_id or not dst_id:
        return {'status': 'error', 'message': 'source/target dataset_id required'}, 400

    src_dir = os.path.join(DATASET_DIR, src_id)
    dst_dir = os.path.join(DATASET_DIR, dst_id)
    if not os.path.isdir(src_dir) or not os.path.isdir(dst_dir):
        return {'status': 'error', 'message': 'Dataset not found'}, 404

    moved = []
    parsed = []
    for ref in indices:
        try:
            parsed.append(_resolve_episode_index(ref))
        except Exception:
            continue
    parsed = sorted(set(parsed))

    # Copy first (preserves all sources even if a later copy fails)
    successful_src = []
    for src_idx in parsed:
        try:
            new_idx = copy_episode_to(src_dir, src_idx, dst_dir)
            moved.append({'src': src_idx, 'dst': new_idx})
            successful_src.append(src_idx)
        except Exception as e:
            print(f'[batch_move] copy failed src_idx={src_idx}: {e}')

    # Delete from source in descending order
    for src_idx in sorted(successful_src, reverse=True):
        try:
            lerobot_delete_episode(src_dir, src_idx)
        except Exception as e:
            print(f'[batch_move] delete failed src_idx={src_idx}: {e}')

    return {'status': 'success', 'moved': moved}, 200


@dataset_bp.route('/dataset/<id>/downsample', methods=['POST'])
def downsample_dataset_route(id):
    data = request.json
    name = data.get('name')
    task_id = data.get('task_id')
    keep = int(data.get('keep', 1))
    every = int(data.get('every', 2))

    if keep <= 0 or every <= 0 or keep >= every:
        return {'status': 'error', 'message': 'Invalid downsample ratio: keep must be < every, both > 0'}, 400

    new_dataset = DatasetModel.create(
        name=name,
        task_id=task_id,
    )

    current_app.pm.start_function(
        func=downsample_dataset,
        dataset_id=id,
        new_dataset_id=new_dataset.id,
        keep=keep,
        every=every,
        socketio_instance=current_app.pm.socketio,
        name="downsample_dataset",
    )

    return {'status': 'success', 'message': 'Dataset downsample started'}, 200


# ── Dataset import / export ─────────────────────────────────────────────────
# Symmetric round-trip via the web UI: Export streams a tar.gz of
# DATASET_DIR/<id>/ as an attachment; Import accepts the same archive shape
# (tar.gz / tgz / tar / zip) and unpacks it into DATASET_DIR/<new_id>/. Lets
# users move datasets between machines without dropping into the launcher.

def _extract_archive_to(archive_path: str, dest: str, archive_name: str):
    """Extract a tar.gz/tgz/tar/zip into ``dest``. If the archive has exactly
    one top-level entry that is a directory (the typical layout produced by our
    Export endpoint), its contents are unpacked directly into ``dest`` so the
    LeRobot layout (``meta/``, ``data/``, ``videos/``) lands at the dataset
    root. Rejects entries with absolute or parent-traversal paths.
    """
    import tarfile
    import zipfile
    import tempfile

    lower = (archive_name or '').lower()
    staging = tempfile.mkdtemp(prefix='dataset_import_', dir=os.path.dirname(dest) or None)

    try:
        if lower.endswith(('.tar.gz', '.tgz', '.tar')):
            mode = 'r:gz' if lower.endswith(('.tar.gz', '.tgz')) else 'r:'
            with tarfile.open(archive_path, mode) as tar:
                for member in tar.getmembers():
                    # Reject path traversal / absolute paths.
                    name = member.name.replace('\\', '/')
                    if name.startswith('/') or '..' in name.split('/'):
                        continue
                    tar.extract(member, staging)
        elif lower.endswith('.zip'):
            with zipfile.ZipFile(archive_path) as zf:
                for info in zf.infolist():
                    name = info.filename.replace('\\', '/')
                    if name.startswith('/') or '..' in name.split('/'):
                        continue
                    zf.extract(info, staging)
        else:
            raise ValueError(f'Unsupported archive type: {archive_name}')

        # If the archive has a single top-level directory, unwrap it.
        entries = [e for e in os.listdir(staging) if not e.startswith('.')]
        if len(entries) == 1 and os.path.isdir(os.path.join(staging, entries[0])):
            src_root = os.path.join(staging, entries[0])
        else:
            src_root = staging

        for name in os.listdir(src_root):
            shutil.move(os.path.join(src_root, name), os.path.join(dest, name))
    finally:
        shutil.rmtree(staging, ignore_errors=True)


@dataset_bp.route('/dataset/:import_upload', methods=['POST'])
def import_dataset_upload():
    """Import a dataset from an uploaded archive (``.tar.gz`` / ``.tgz`` /
    ``.tar`` / ``.zip``). Used by the web UI's native file picker — mirrors the
    Export endpoint's output format for a symmetric round-trip.

    Form fields:
        ``task_id``: workspace/task id (required).
        ``name`` (optional): dataset name; defaults to the archive's stem
            (with the ``.tar.gz``/``.zip`` extension stripped).
        ``file``: the archive.
    """
    import tempfile

    task_id = request.form.get('task_id')
    if not task_id:
        return {'status': 'error', 'message': 'task_id is required'}, 400

    archive = request.files.get('file')
    if archive is None:
        return {'status': 'error', 'message': 'no archive uploaded'}, 400

    filename = archive.filename or 'dataset.tar.gz'
    lower = filename.lower()
    if not lower.endswith(('.tar.gz', '.tgz', '.tar', '.zip')):
        return {
            'status': 'error',
            'message': 'Unsupported archive — use .tar.gz, .tgz, .tar, or .zip',
        }, 400

    # Default name: strip the archive suffix. Order matters — '.tar.gz' first.
    name = request.form.get('name')
    if not name:
        for ext in ('.tar.gz', '.tgz', '.tar', '.zip'):
            if lower.endswith(ext):
                name = filename[: -len(ext)]
                break
        name = name or filename

    new_dataset = DatasetModel.create(name=name, task_id=task_id)
    dest = os.path.join(DATASET_DIR, str(new_dataset.id))
    os.makedirs(dest, exist_ok=True)

    # Save the upload to a temp file, then extract — tarfile/zipfile both want
    # a seekable file rather than a stream.
    tmp = tempfile.NamedTemporaryFile(prefix='dataset_upload_', suffix=lower, delete=False)
    try:
        archive.save(tmp.name)
        tmp.close()
        _extract_archive_to(tmp.name, dest, filename)
    except Exception as e:
        shutil.rmtree(dest, ignore_errors=True)
        new_dataset.delete_instance()
        return {'status': 'error', 'message': f'Import failed: {e}'}, 500
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass

    return {
        'status': 'success',
        'message': 'Dataset imported',
        'dataset_id': new_dataset.id,
    }, 200


@dataset_bp.route('/dataset/<id>/:download', methods=['GET'])
def download_dataset(id):
    """Stream a dataset as a tar.gz attachment so the browser shows its native
    save-as dialog. Used by the web UI's Export Dataset menu."""
    import tempfile
    import tarfile

    dataset = DatasetModel.find(id)
    if not dataset:
        return {'status': 'error', 'message': 'Dataset not found'}, 404

    src = os.path.join(DATASET_DIR, str(dataset.id))
    if not os.path.isdir(src):
        return {'status': 'error', 'message': 'Dataset folder not found'}, 404

    safe_name = re.sub(r'[^A-Za-z0-9._-]+', '_', dataset.name or f'dataset_{dataset.id}')
    arcname = f'{safe_name}_{dataset.id}'

    tmp = tempfile.NamedTemporaryFile(prefix='dataset_export_', suffix='.tar.gz', delete=False)
    tmp.close()
    try:
        with tarfile.open(tmp.name, 'w:gz') as tar:
            tar.add(src, arcname=arcname)
    except Exception as e:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
        return {'status': 'error', 'message': f'Archive failed: {e}'}, 500

    return send_file(
        tmp.name,
        as_attachment=True,
        download_name=f'{arcname}.tar.gz',
        mimetype='application/gzip',
    )


