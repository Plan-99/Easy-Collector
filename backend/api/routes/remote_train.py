"""Routes proxying the remote training server."""
import json
import os
import shutil
import tarfile
import tempfile
import time
import traceback

import requests
from flask import Blueprint, current_app, request

from ...configs.global_configs import DATASET_DIR
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

remote_train_bp = Blueprint('remote_train', __name__)

CHECKPOINT_DIR = '/root/src/backend/checkpoints'
PROCESS_ID = 'train_task'


def _normalize_url(url: str) -> str:
    url = (url or '').strip().rstrip('/')
    if not url:
        return ''
    if not url.startswith(('http://', 'https://')):
        url = 'http://' + url
    return url


@remote_train_bp.route('/remote-train/health', methods=['POST'])
def remote_train_health():
    server_url = _normalize_url(request.json.get('server_url', ''))
    if not server_url:
        return {'status': 'error', 'message': 'server_url required'}, 400
    try:
        resp = requests.get(f'{server_url}/api/health', timeout=5)
        if resp.status_code == 200:
            return {'status': 'success', 'server': resp.json()}, 200
        return {'status': 'error', 'message': f'HTTP {resp.status_code}'}, 502
    except requests.exceptions.RequestException as e:
        return {'status': 'error', 'message': str(e)}, 502


@remote_train_bp.route('/remote-train/start', methods=['POST'])
def remote_train_start():
    data = request.json or {}
    server_url = _normalize_url(data.get('server_url', ''))
    checkpoint_id = data.get('checkpoint_id')
    callback_url = data.get('callback_url', '')

    if not server_url or not checkpoint_id:
        return {'status': 'error', 'message': 'server_url and checkpoint_id required'}, 400

    if PROCESS_ID in current_app.pm.processes:
        return {'status': 'error', 'message': 'Training already running'}, 409

    current_app.pm.start_function(
        name=PROCESS_ID,
        func=_remote_training_workflow,
        server_url=server_url,
        checkpoint_id=checkpoint_id,
        callback_url=callback_url,
        socketio_instance=current_app.pm.socketio,
    )

    return {
        'status': 'success',
        'message': 'Remote training started',
        'process_id': PROCESS_ID,
        'checkpoint_id': checkpoint_id,
    }, 200


@remote_train_bp.route('/remote-train/stop', methods=['POST'])
def remote_train_stop():
    """Stop the currently running training (local watcher + remote job)."""
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

    # Mark the training checkpoint as failed & remove it
    running = CheckpointModel.select().where(
        CheckpointModel.status == 'training',
        CheckpointModel.deleted_at.is_null(),
    ).first()
    if running:
        running.delete_instance()

    current_app.pm.stop_function(PROCESS_ID)
    return {'status': 'success', 'message': 'Training stopped'}, 200


@remote_train_bp.route('/remote-train/cancel', methods=['POST'])
def remote_train_cancel():
    """Remove a queued/unfinished checkpoint from the DB."""
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

    data = request.json or {}
    checkpoint_id = data.get('checkpoint_id')
    if checkpoint_id is None:
        return {'status': 'error', 'message': 'checkpoint_id required'}, 400
    checkpoint = CheckpointModel.find(checkpoint_id)
    if checkpoint:
        checkpoint.delete_instance()
    return {'status': 'success', 'message': 'Training cancelled'}, 200


@remote_train_bp.route('/remote-train/receive-model', methods=['POST'])
def remote_train_receive_model():
    """Training server uploads the trained checkpoint here after completion."""
    job_id = request.form.get('job_id', '')
    file = request.files.get('file')
    if not job_id or not file:
        return {'status': 'error', 'message': 'job_id and file required'}, 400

    checkpoint_id = _job_id_to_checkpoint_id(job_id)
    if checkpoint_id is None:
        return {'status': 'error', 'message': 'invalid job_id'}, 400

    ckpt_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint_id))
    os.makedirs(ckpt_dir, exist_ok=True)

    with tempfile.NamedTemporaryFile(suffix='.tar.gz', delete=False) as tmp:
        tmp_path = tmp.name
        file.save(tmp_path)

    try:
        with tarfile.open(tmp_path, 'r:gz') as tar:
            tar.extractall(ckpt_dir)
    finally:
        try:
            os.unlink(tmp_path)
        except Exception:
            pass

    ckpt = CheckpointModel.find(checkpoint_id)
    if ckpt is not None:
        ckpt.update({'status': 'finished'})

    return {'status': 'success', 'checkpoint_id': checkpoint_id}, 200


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _job_id_to_checkpoint_id(job_id: str):
    """Job IDs are formatted as 'ckpt_{checkpoint_id}'."""
    if not job_id.startswith('ckpt_'):
        return None
    try:
        return int(job_id[len('ckpt_'):])
    except ValueError:
        return None


def _emit_log(socketio_instance, log_id, message, msg_type='stdout'):
    socketio_instance.emit('task_log', {
        'id': log_id,
        'message': message,
        'type': msg_type,
    })


def _parse_json_field(value):
    """Accept dict/list or JSON string."""
    if value is None:
        return None
    if isinstance(value, (dict, list)):
        return value
    if isinstance(value, str):
        try:
            return json.loads(value)
        except Exception:
            return None
    return None


def _build_train_config(checkpoint):
    """Extract policy/train_settings/dataset_info/sensor_ids from checkpoint."""
    task = checkpoint.task
    policy = checkpoint.policy

    policy_settings = _parse_json_field(policy.settings) if policy else None
    policy_config = {
        'type': policy.type if policy else None,
        'settings': dict(policy_settings) if isinstance(policy_settings, dict) else {},
    }

    train_settings_parsed = _parse_json_field(checkpoint.train_settings)
    train_settings = dict(train_settings_parsed) if isinstance(train_settings_parsed, dict) else {}

    dataset_info_parsed = _parse_json_field(checkpoint.dataset_info)
    dataset_info = dataset_info_parsed if isinstance(dataset_info_parsed, dict) else {}

    sensor_ids = []
    if task:
        parsed = _parse_json_field(task.sensor_ids)
        if isinstance(parsed, list):
            sensor_ids = parsed

    return {
        'policy': policy_config,
        'train_settings': train_settings,
        'dataset_info': dataset_info,
        'sensor_ids': sensor_ids,
    }


def _tar_datasets(dataset_ids, tar_path):
    """Create a tar.gz bundle of the requested dataset directories.

    Single dataset: bundle contents at archive root (meta/, data/, videos/).
    Multiple datasets: each goes under its id directory — training server must
    merge them, which is not yet implemented.
    """
    dataset_ids = list(dataset_ids)
    for ds_id in dataset_ids:
        ds_dir = os.path.join(DATASET_DIR, str(ds_id))
        if not os.path.isdir(ds_dir):
            raise FileNotFoundError(f'Dataset directory not found: {ds_dir}')

    if len(dataset_ids) == 1:
        ds_dir = os.path.join(DATASET_DIR, str(dataset_ids[0]))
        with tarfile.open(tar_path, 'w:gz') as tar:
            tar.add(ds_dir, arcname='.')
        return

    # TODO: support multi-dataset merge on the training server side
    raise NotImplementedError(
        'Multi-dataset merge for remote training is not implemented yet. '
        'Please train on a single dataset.'
    )


def _tar_checkpoint(checkpoint_id, tar_path):
    ckpt_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint_id))
    if not os.path.isdir(ckpt_dir):
        raise FileNotFoundError(f'Checkpoint directory not found: {ckpt_dir}')
    with tarfile.open(tar_path, 'w:gz') as tar:
        tar.add(ckpt_dir, arcname='.')


def _download_and_install_model(server_url, job_id, checkpoint_id):
    """Pull the trained model from training_server and extract into the local checkpoint dir."""
    ckpt_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint_id))
    os.makedirs(ckpt_dir, exist_ok=True)

    with tempfile.NamedTemporaryFile(suffix='.tar.gz', delete=False) as tmp:
        tmp_path = tmp.name

    try:
        with requests.get(f'{server_url}/api/train/download/{job_id}',
                          stream=True, timeout=600) as resp:
            if resp.status_code != 200:
                raise RuntimeError(f'Download failed: HTTP {resp.status_code}')
            with open(tmp_path, 'wb') as f:
                for chunk in resp.iter_content(chunk_size=1024 * 1024):
                    if chunk:
                        f.write(chunk)
        with tarfile.open(tmp_path, 'r:gz') as tar:
            tar.extractall(ckpt_dir)
    finally:
        try:
            os.unlink(tmp_path)
        except Exception:
            pass


def _remote_training_workflow(server_url, checkpoint_id, callback_url,
                               socketio_instance, task_control):
    """Background workflow: upload dataset → start → poll status → mark finished."""
    job_id = f'ckpt_{checkpoint_id}'
    log_id = PROCESS_ID

    checkpoint = CheckpointModel.find(checkpoint_id)
    if checkpoint is None:
        _emit_log(socketio_instance, log_id, f'[ERROR] Checkpoint {checkpoint_id} not found', 'error')
        return

    try:
        checkpoint.update({'status': 'training'})

        config = _build_train_config(checkpoint)
        dataset_ids = list(config['dataset_info'].keys())
        if not dataset_ids:
            _emit_log(socketio_instance, log_id, '[ERROR] No datasets assigned to checkpoint', 'error')
            checkpoint.update({'status': 'failed'})
            return

        # 1) Upload dataset bundle
        _emit_log(socketio_instance, log_id, f'Bundling datasets {dataset_ids}...')
        with tempfile.NamedTemporaryFile(suffix='.tar.gz', delete=False) as tmp:
            tar_path = tmp.name
        try:
            _tar_datasets(dataset_ids, tar_path)
            size_mb = os.path.getsize(tar_path) / (1024 * 1024)
            _emit_log(socketio_instance, log_id, f'Uploading dataset ({size_mb:.1f} MB)...')
            with open(tar_path, 'rb') as f:
                resp = requests.post(
                    f'{server_url}/api/train/upload-dataset',
                    files={'file': ('dataset.tar.gz', f, 'application/gzip')},
                    data={'job_id': job_id},
                    timeout=600,
                )
            if resp.status_code != 200:
                raise RuntimeError(f'Dataset upload failed: {resp.status_code} {resp.text}')
        finally:
            try:
                os.unlink(tar_path)
            except Exception:
                pass

        # 2) Upload base model (for transfer learning)
        if checkpoint.load_model_id:
            _emit_log(socketio_instance, log_id,
                      f'Uploading base model from checkpoint {checkpoint.load_model_id}...')
            with tempfile.NamedTemporaryFile(suffix='.tar.gz', delete=False) as tmp:
                model_tar = tmp.name
            try:
                _tar_checkpoint(checkpoint.load_model_id, model_tar)
                with open(model_tar, 'rb') as f:
                    resp = requests.post(
                        f'{server_url}/api/train/upload-model',
                        files={'file': ('model.tar.gz', f, 'application/gzip')},
                        data={'job_id': job_id},
                        timeout=300,
                    )
                if resp.status_code != 200:
                    raise RuntimeError(f'Model upload failed: {resp.status_code} {resp.text}')
            finally:
                try:
                    os.unlink(model_tar)
                except Exception:
                    pass

        # 3) Start training
        _emit_log(socketio_instance, log_id, 'Requesting training start...')
        resp = requests.post(
            f'{server_url}/api/train/start',
            json={
                'job_id': job_id,
                'policy': config['policy'],
                'train_settings': config['train_settings'],
                'dataset_info': config['dataset_info'],
                'sensor_ids': config['sensor_ids'],
                # callback_url omitted: backend pulls the model after status==finished
                # (training_server's auto-send can't reach the backend container reliably).
            },
            timeout=30,
        )
        if resp.status_code != 200:
            raise RuntimeError(f'Training start failed: {resp.status_code} {resp.text}')

        # 4) Poll status until done / stopped
        _emit_log(socketio_instance, log_id, 'Training started. Polling status...')
        last_progress = -1.0
        while True:
            if task_control.get('stop'):
                _emit_log(socketio_instance, log_id, 'Stop requested. Notifying remote server...', 'warning')
                try:
                    requests.post(f'{server_url}/api/train/stop/{job_id}', timeout=10)
                except Exception:
                    pass
                checkpoint.update({'status': 'failed'})
                return

            try:
                resp = requests.get(f'{server_url}/api/train/status/{job_id}', timeout=10)
                if resp.status_code != 200:
                    time.sleep(3)
                    continue
                job = resp.json().get('job', {})
            except requests.exceptions.RequestException:
                time.sleep(3)
                continue

            status = job.get('status')
            progress = float(job.get('progress', 0) or 0)

            if abs(progress - last_progress) >= 0.01:
                socketio_instance.emit('train_progress', {
                    'checkpoint_id': checkpoint_id,
                    'progress': progress,
                    'status': status,
                })
                last_progress = progress

            if status == 'finished':
                _emit_log(socketio_instance, log_id, '[SUCCESS] Training finished. Downloading model...', 'success')
                try:
                    _download_and_install_model(server_url, job_id, checkpoint_id)
                    checkpoint.update({'status': 'finished'})
                    _emit_log(socketio_instance, log_id, '[SUCCESS] Model installed.', 'success')
                except Exception as e:
                    traceback.print_exc()
                    _emit_log(socketio_instance, log_id, f'[ERROR] Model download failed: {e}', 'error')
                    checkpoint.update({'status': 'failed'})
                return
            if status == 'failed':
                err = job.get('error', 'unknown')
                _emit_log(socketio_instance, log_id, f'[ERROR] Training failed: {err}', 'error')
                checkpoint.update({'status': 'failed'})
                return
            if status == 'stopped':
                _emit_log(socketio_instance, log_id, 'Training stopped.', 'warning')
                checkpoint.update({'status': 'failed'})
                return

            time.sleep(2)

    except Exception as e:
        traceback.print_exc()
        _emit_log(socketio_instance, log_id, f'[ERROR] {e}', 'error')
        try:
            checkpoint.update({'status': 'failed'})
        except Exception:
            pass
