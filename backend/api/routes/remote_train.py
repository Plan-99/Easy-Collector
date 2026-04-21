"""
Remote Training Routes

Handles communication between the local Easy-Collector and a remote training server:
- Upload datasets and base models to training server
- Start/stop remote training
- Receive trained model back (callback)
- Proxy WebSocket events for progress monitoring
"""
import os
import io
import json
import uuid
import tarfile
import shutil
import threading

import requests
from flask import Blueprint, request, current_app

from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
from ...database.models.policy_model import Policy as PolicyModel
from ...database.models.task_model import Task as TaskModel
from ...configs.global_configs import DATASET_DIR

remote_train_bp = Blueprint('remote_train', __name__)

CHECKPOINT_DIR = '/root/backend/checkpoints'

# Track active remote training sessions
# checkpoint_id -> { job_id, server_url, ws_client }
_remote_sessions = {}
_sessions_lock = threading.Lock()


@remote_train_bp.route('/remote-train/health', methods=['POST'])
def check_server_health():
    """Check if a remote training server is reachable."""
    data = request.json
    server_url = data.get('server_url', '').rstrip('/')
    if not server_url:
        return {'status': 'error', 'message': 'server_url required'}, 400

    try:
        resp = requests.get(f'{server_url}/api/health', timeout=5)
        if resp.status_code == 200:
            return {'status': 'success', 'server': resp.json()}, 200
        return {'status': 'error', 'message': f'Server returned {resp.status_code}'}, 502
    except requests.exceptions.ConnectionError:
        return {'status': 'error', 'message': 'Cannot connect to training server'}, 502
    except requests.exceptions.Timeout:
        return {'status': 'error', 'message': 'Connection timed out'}, 504


@remote_train_bp.route('/remote-train/start', methods=['POST'])
def start_remote_training():
    """
    Upload dataset + optional base model to remote server and start training.

    Expected JSON body:
    {
        "server_url": "http://192.168.1.100:5100",
        "checkpoint_id": 123,
    }
    """
    data = request.json
    server_url = data.get('server_url', '').rstrip('/')
    checkpoint_id = data.get('checkpoint_id')

    if not server_url or not checkpoint_id:
        return {'status': 'error', 'message': 'server_url and checkpoint_id required'}, 400

    checkpoint = CheckpointModel.find(checkpoint_id)
    if not checkpoint:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404

    policy = PolicyModel.find(checkpoint.policy_id)
    task = TaskModel.find(checkpoint.task_id)
    load_model = CheckpointModel.find(checkpoint.load_model_id) if checkpoint.load_model_id else None

    job_id = str(uuid.uuid4())[:8] + f'_cp{checkpoint_id}'

    # Update checkpoint status
    checkpoint.update({'status': 'uploading'})

    # Build callback URL for auto model delivery
    # Use the local server's address that the training server can reach
    local_callback = data.get('callback_url')

    # Prepare sensor_ids from task settings
    sensor_settings = task.settings.get('sensors', {}) if task.settings else {}
    if sensor_settings:
        sensor_ids = [int(sid) for sid in task.sensor_ids if str(sid) in sensor_settings]
    else:
        sensor_ids = [int(sid) for sid in (task.sensor_ids or [])]

    def _upload_and_start():
        socketio = current_app.extensions.get('socketio') or current_app.pm.socketio

        try:
            # 1. Package and upload dataset
            socketio.emit('task_log', {
                'id': 'train_task',
                'message': f'[NOTICE] Uploading dataset to training server {server_url}...',
                'type': 'notice',
            })

            dataset_tar = _package_datasets(checkpoint.dataset_info)
            resp = requests.post(
                f'{server_url}/api/train/upload-dataset',
                files={'file': ('dataset.tar.gz', dataset_tar, 'application/gzip')},
                data={'job_id': job_id},
                timeout=600,
            )
            if resp.status_code != 200:
                raise Exception(f'Dataset upload failed: {resp.text}')

            socketio.emit('task_log', {
                'id': 'train_task',
                'message': '[SUCCESS] Dataset uploaded successfully.',
                'type': 'success',
            })

            # 2. Upload base model if finetuning
            if load_model:
                socketio.emit('task_log', {
                    'id': 'train_task',
                    'message': '[NOTICE] Uploading base model for finetuning...',
                    'type': 'notice',
                })

                model_tar = _package_checkpoint(load_model.id)
                resp = requests.post(
                    f'{server_url}/api/train/upload-model',
                    files={'file': ('model.tar.gz', model_tar, 'application/gzip')},
                    data={'job_id': job_id},
                    timeout=300,
                )
                if resp.status_code != 200:
                    raise Exception(f'Model upload failed: {resp.text}')

                socketio.emit('task_log', {
                    'id': 'train_task',
                    'message': '[SUCCESS] Base model uploaded successfully.',
                    'type': 'success',
                })

            # 3. Start training on remote server
            checkpoint.update({'status': 'training'})

            train_config = {
                'job_id': job_id,
                'policy': policy.to_dict(),
                'train_settings': checkpoint.train_settings,
                'dataset_info': checkpoint.dataset_info,
                'sensor_ids': sensor_ids,
                'callback_url': local_callback,
            }

            resp = requests.post(
                f'{server_url}/api/train/start',
                json=train_config,
                timeout=30,
            )
            if resp.status_code != 200:
                raise Exception(f'Training start failed: {resp.text}')

            # 4. Connect WebSocket to proxy training logs
            with _sessions_lock:
                _remote_sessions[str(checkpoint_id)] = {
                    'job_id': job_id,
                    'server_url': server_url,
                }

            socketio.emit('start_process', {'id': 'train_task'})

            # Start WebSocket proxy in background
            _start_ws_proxy(server_url, job_id, checkpoint_id, socketio)

        except Exception as e:
            import traceback
            error_msg = traceback.format_exc()
            print(f'[ERROR] Remote training failed: {error_msg}', flush=True)
            checkpoint.update({'status': 'waiting'})
            socketio.emit('task_log', {
                'id': 'train_task',
                'message': f'[ERROR] Remote training failed: {str(e)}',
                'type': 'error',
            })

    # Run upload/start in background so the API response returns immediately
    socketio = current_app.extensions.get('socketio') or current_app.pm.socketio
    socketio.start_background_task(target=_upload_and_start)

    return {
        'status': 'success',
        'message': 'Remote training initiated',
        'job_id': job_id,
        'checkpoint_id': checkpoint_id,
    }, 200


@remote_train_bp.route('/remote-train/stop', methods=['POST'])
def stop_remote_training():
    """Stop a remote training job."""
    data = request.json
    checkpoint_id = str(data.get('checkpoint_id'))

    with _sessions_lock:
        session = _remote_sessions.get(checkpoint_id)

    if not session:
        return {'status': 'error', 'message': 'No active remote session'}, 404

    try:
        resp = requests.post(
            f'{session["server_url"]}/api/train/stop/{session["job_id"]}',
            timeout=10,
        )

        checkpoint = CheckpointModel.find(int(checkpoint_id))
        if checkpoint:
            checkpoint.delete()

        with _sessions_lock:
            _remote_sessions.pop(checkpoint_id, None)

        return {'status': 'success', 'message': 'Remote training stopped'}, 200
    except Exception as e:
        return {'status': 'error', 'message': str(e)}, 500


@remote_train_bp.route('/remote-train/receive-model', methods=['POST'])
def receive_model():
    """Callback endpoint: training server pushes the trained model here."""
    job_id = request.form.get('job_id')
    if not job_id or 'file' not in request.files:
        return {'status': 'error', 'message': 'job_id and file required'}, 400

    # Find checkpoint_id from job_id
    checkpoint_id = None
    with _sessions_lock:
        for cp_id, session in _remote_sessions.items():
            if session['job_id'] == job_id:
                checkpoint_id = cp_id
                break

    if not checkpoint_id:
        # Try to extract from job_id format: xxxx_cpNN
        if '_cp' in job_id:
            checkpoint_id = job_id.split('_cp')[1]

    if not checkpoint_id:
        return {'status': 'error', 'message': 'Cannot determine checkpoint_id'}, 400

    # Save the model
    ckpt_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint_id))
    os.makedirs(ckpt_dir, exist_ok=True)

    f = request.files['file']
    tar_path = os.path.join(ckpt_dir, 'model.tar.gz')
    f.save(tar_path)

    # Extract
    os.system(f'tar -xzf {tar_path} -C {ckpt_dir}')
    os.remove(tar_path)

    # Read result.json if present
    result_path = os.path.join(ckpt_dir, 'result.json')
    best_epoch = 0
    loss = 0
    if os.path.exists(result_path):
        with open(result_path, 'r') as rf:
            result = json.load(rf)
            best_epoch = result.get('best_epoch', 0)
            loss = result.get('loss', 0)
        os.remove(result_path)

    # Update checkpoint in DB
    checkpoint = CheckpointModel.find(int(checkpoint_id))
    if checkpoint:
        checkpoint.update({
            'status': 'finished',
            'best_epoch': best_epoch,
            'loss': loss,
        })

    # Clean up session
    with _sessions_lock:
        _remote_sessions.pop(str(checkpoint_id), None)

    socketio = current_app.extensions.get('socketio') or current_app.pm.socketio
    socketio.emit('stop_process', {'id': 'train_task', 'return_code': 0})
    socketio.emit('task_log', {
        'id': 'train_task',
        'message': '[SUCCESS] Trained model received and saved.',
        'type': 'success',
    })

    print(f'[SUCCESS] Received trained model for checkpoint {checkpoint_id}', flush=True)
    return {'status': 'success', 'message': 'Model received'}, 200


def _package_datasets(dataset_info):
    """Package datasets into a tar.gz in-memory, merging multiple datasets."""
    # Use the same merging logic as train.py but package result as tar
    from ..process.lerobot_io import (
        list_episodes, get_dataset_info, _read_json, _write_json,
        _read_jsonl, _write_jsonl, _append_jsonl,
        PARQUET_PATH_TEMPLATE, IMAGE_PATH_TEMPLATE, INFO_PATH,
        EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH, DEFAULT_CHUNK_SIZE,
    )
    import pyarrow.parquet as pq
    import pyarrow as pa
    import numpy as np
    import tempfile

    temp_dir = tempfile.mkdtemp(prefix='remote_train_dataset_')

    try:
        dataset_ids = list(dataset_info.keys())
        episode_counter = 0

        # Initialize temp dataset from first source
        first_ds_id = dataset_ids[0]
        first_ds_path = os.path.join(DATASET_DIR, str(first_ds_id))
        first_info = get_dataset_info(first_ds_path)
        if first_info:
            tmp_info = dict(first_info)
            tmp_info["total_episodes"] = 0
            tmp_info["total_frames"] = 0
            tmp_info["total_chunks"] = 0
            tmp_info["total_tasks"] = 0
            tmp_info["splits"] = {}
            _write_json(tmp_info, os.path.join(temp_dir, INFO_PATH))
            for p in [EPISODES_PATH, TASKS_PATH, EPISODES_STATS_PATH]:
                fp = os.path.join(temp_dir, p)
                os.makedirs(os.path.dirname(fp), exist_ok=True)
                open(fp, "w").close()
            src_tasks = _read_jsonl(os.path.join(first_ds_path, TASKS_PATH))
            if src_tasks:
                _write_jsonl(src_tasks, os.path.join(temp_dir, TASKS_PATH))

        for ds_id in dataset_ids:
            dataset_path = os.path.join(DATASET_DIR, str(ds_id))
            ds_info = get_dataset_info(dataset_path)
            if ds_info is None:
                continue
            episodes = list_episodes(dataset_path)

            for ep_entry in episodes:
                ep_idx = ep_entry["episode_index"]
                chunk = ep_idx // ds_info.get("chunks_size", DEFAULT_CHUNK_SIZE)
                src_parquet = os.path.join(dataset_path, PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=ep_idx))
                if not os.path.exists(src_parquet):
                    continue

                table = pq.read_table(src_parquet)
                df = table.to_pandas()
                num_frames = len(df)

                tmp_info = _read_json(os.path.join(temp_dir, INFO_PATH))
                global_start = tmp_info["total_frames"]
                new_chunk = episode_counter // tmp_info.get("chunks_size", DEFAULT_CHUNK_SIZE)

                df["index"] = np.arange(global_start, global_start + num_frames, dtype=np.int64)
                df["episode_index"] = episode_counter

                # Copy videos/images
                features = ds_info.get("features", {})
                for feat_key, feat in features.items():
                    if not feat_key.startswith("observation.images."):
                        continue
                    if feat.get("dtype") == "video":
                        src_video = os.path.join(
                            dataset_path, "videos", f"chunk-{chunk:03d}", feat_key,
                            f"episode_{ep_idx:06d}.mp4"
                        )
                        dst_video = os.path.join(
                            temp_dir, "videos", f"chunk-{new_chunk:03d}", feat_key,
                            f"episode_{episode_counter:06d}.mp4"
                        )
                        os.makedirs(os.path.dirname(dst_video), exist_ok=True)
                        if os.path.exists(src_video):
                            shutil.copy2(src_video, dst_video)
                    elif feat.get("dtype") == "image" and feat_key in df.columns:
                        sensor_name = feat_key.replace("observation.images.", "")
                        s_id = sensor_name.replace("sensor_", "")
                        new_paths = []
                        for frame_idx, old_rel_path in enumerate(df[feat_key]):
                            old_abs = os.path.join(dataset_path, old_rel_path)
                            new_rel = IMAGE_PATH_TEMPLATE.format(sid=s_id, ep=episode_counter, frame=frame_idx)
                            new_abs = os.path.join(temp_dir, new_rel)
                            os.makedirs(os.path.dirname(new_abs), exist_ok=True)
                            if os.path.exists(old_abs):
                                shutil.copy2(old_abs, new_abs)
                            new_paths.append(new_rel)
                        df[feat_key] = new_paths

                dest_parquet = os.path.join(temp_dir, PARQUET_PATH_TEMPLATE.format(chunk=new_chunk, ep=episode_counter))
                os.makedirs(os.path.dirname(dest_parquet), exist_ok=True)
                pq.write_table(pa.Table.from_pandas(df), dest_parquet)

                src_stats = _read_jsonl(os.path.join(dataset_path, EPISODES_STATS_PATH))
                ep_stats = {}
                for s in src_stats:
                    if s.get("episode_index") == ep_idx:
                        ep_stats = s.get("stats", {})
                        break

                tmp_info["total_episodes"] = episode_counter + 1
                tmp_info["total_frames"] = global_start + num_frames
                tmp_info["total_chunks"] = new_chunk + 1
                tmp_info["splits"] = {"train": f"0:{episode_counter + 1}"}
                _write_json(tmp_info, os.path.join(temp_dir, INFO_PATH))

                _append_jsonl(
                    {"episode_index": episode_counter, "length": num_frames, "tasks": ep_entry.get("tasks", [""])},
                    os.path.join(temp_dir, EPISODES_PATH),
                )
                _append_jsonl(
                    {"episode_index": episode_counter, "stats": ep_stats},
                    os.path.join(temp_dir, EPISODES_STATS_PATH),
                )

                episode_counter += 1

        # Create tar.gz
        buf = io.BytesIO()
        with tarfile.open(fileobj=buf, mode='w:gz') as tar:
            tar.add(temp_dir, arcname='.')
        buf.seek(0)
        return buf

    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def _package_checkpoint(checkpoint_id):
    """Package a checkpoint directory as tar.gz."""
    ckpt_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint_id))
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode='w:gz') as tar:
        tar.add(ckpt_dir, arcname='.')
    buf.seek(0)
    return buf


def _start_ws_proxy(server_url, job_id, checkpoint_id, socketio):
    """Connect to the training server's WebSocket and proxy events to local clients."""
    import socketio as sio_client

    ws_url = server_url
    client = sio_client.Client()

    @client.on('task_log')
    def on_task_log(data):
        if data.get('id') == f'train_{job_id}':
            # Re-emit as local train_task event
            socketio.emit('task_log', {
                'id': 'train_task',
                'message': data.get('message', ''),
                'type': data.get('type', 'stdout'),
            })

    @client.on('train_status')
    def on_train_status(data):
        if data.get('job_id') == job_id:
            status = data.get('status')
            if status == 'finished':
                # Training done - if no auto-send, download manually
                socketio.emit('task_log', {
                    'id': 'train_task',
                    'message': '[NOTICE] Training completed on remote server. Downloading model...',
                    'type': 'notice',
                })
                _download_model(server_url, job_id, checkpoint_id, socketio)
            elif status == 'delivered':
                # Model was auto-sent via callback
                pass
            elif status == 'failed':
                error = data.get('error', 'Unknown error')
                socketio.emit('task_log', {
                    'id': 'train_task',
                    'message': f'[ERROR] Remote training failed: {error}',
                    'type': 'error',
                })
                checkpoint = CheckpointModel.find(int(checkpoint_id))
                if checkpoint:
                    checkpoint.delete()
                socketio.emit('stop_process', {'id': 'train_task', 'return_code': 1})

    try:
        client.connect(ws_url, wait_timeout=10)

        with _sessions_lock:
            if str(checkpoint_id) in _remote_sessions:
                _remote_sessions[str(checkpoint_id)]['ws_client'] = client

    except Exception as e:
        print(f'[WARNING] WebSocket proxy connection failed: {e}', flush=True)
        # Fall back to polling
        socketio.start_background_task(target=_poll_training_status,
                                        server_url=server_url, job_id=job_id,
                                        checkpoint_id=checkpoint_id, socketio=socketio)


def _poll_training_status(server_url, job_id, checkpoint_id, socketio):
    """Fallback: poll training status if WebSocket proxy fails."""
    import time

    while True:
        time.sleep(5)
        try:
            resp = requests.get(f'{server_url}/api/train/status/{job_id}', timeout=10)
            if resp.status_code != 200:
                continue

            data = resp.json().get('job', {})
            status = data.get('status')

            if status == 'finished':
                socketio.emit('task_log', {
                    'id': 'train_task',
                    'message': '[NOTICE] Training completed. Downloading model...',
                    'type': 'notice',
                })
                _download_model(server_url, job_id, checkpoint_id, socketio)
                break
            elif status in ('failed', 'stopped'):
                socketio.emit('task_log', {
                    'id': 'train_task',
                    'message': f'[ERROR] Remote training {status}.',
                    'type': 'error',
                })
                checkpoint = CheckpointModel.find(int(checkpoint_id))
                if checkpoint:
                    checkpoint.delete()
                socketio.emit('stop_process', {'id': 'train_task', 'return_code': 1})
                break

        except Exception as e:
            print(f'[WARNING] Polling failed: {e}', flush=True)

        with _sessions_lock:
            if str(checkpoint_id) not in _remote_sessions:
                break


def _download_model(server_url, job_id, checkpoint_id, socketio):
    """Download trained model from remote server."""
    try:
        resp = requests.get(f'{server_url}/api/train/download/{job_id}', timeout=300, stream=True)
        if resp.status_code != 200:
            raise Exception(f'Download failed: {resp.status_code}')

        ckpt_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint_id))
        os.makedirs(ckpt_dir, exist_ok=True)
        tar_path = os.path.join(ckpt_dir, 'model.tar.gz')

        with open(tar_path, 'wb') as f:
            for chunk in resp.iter_content(chunk_size=8192):
                f.write(chunk)

        os.system(f'tar -xzf {tar_path} -C {ckpt_dir}')
        os.remove(tar_path)

        # Read result
        result_path = os.path.join(ckpt_dir, 'result.json')
        best_epoch = 0
        loss = 0
        if os.path.exists(result_path):
            with open(result_path, 'r') as rf:
                result = json.load(rf)
                best_epoch = result.get('best_epoch', 0)
                loss = result.get('loss', 0)
            os.remove(result_path)

        checkpoint = CheckpointModel.find(int(checkpoint_id))
        if checkpoint:
            checkpoint.update({
                'status': 'finished',
                'best_epoch': best_epoch,
                'loss': loss,
            })

        with _sessions_lock:
            _remote_sessions.pop(str(checkpoint_id), None)

        socketio.emit('task_log', {
            'id': 'train_task',
            'message': '[SUCCESS] Model downloaded and saved locally.',
            'type': 'success',
        })
        socketio.emit('stop_process', {'id': 'train_task', 'return_code': 0})

    except Exception as e:
        print(f'[ERROR] Model download failed: {e}', flush=True)
        socketio.emit('task_log', {
            'id': 'train_task',
            'message': f'[ERROR] Model download failed: {e}. You can try manual download.',
            'type': 'error',
        })
