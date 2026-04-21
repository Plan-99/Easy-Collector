"""
EasyTrainer Remote Training Server

A standalone Flask-SocketIO server that:
1. Receives datasets and model weights from the local Easy-Collector
2. Runs training jobs (ACT, Diffusion, PI05)
3. Streams training progress via WebSocket
4. Sends back trained model on completion
"""
import os
import json
import uuid
import shutil
import threading
import time
import traceback
import subprocess
import signal

from flask import Flask, request, jsonify, send_file
from flask_socketio import SocketIO
from flask_cors import CORS

DATA_DIR = os.environ.get('TRAINING_SERVER_DATA_DIR', '/data')
DATASETS_DIR = os.path.join(DATA_DIR, 'datasets')
CHECKPOINTS_DIR = os.path.join(DATA_DIR, 'checkpoints')
UPLOADS_DIR = os.path.join(DATA_DIR, 'uploads')

for d in [DATASETS_DIR, CHECKPOINTS_DIR, UPLOADS_DIR]:
    os.makedirs(d, exist_ok=True)

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet', max_http_buffer_size=500 * 1024 * 1024)

# In-memory job registry
# job_id -> { status, process, config, progress, error, callback_url }
jobs = {}
jobs_lock = threading.Lock()


@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok', 'gpu_available': _check_gpu()}), 200


@app.route('/api/train/upload-dataset', methods=['POST'])
def upload_dataset():
    """Receive a dataset tar.gz and extract it."""
    job_id = request.form.get('job_id')
    if not job_id:
        return jsonify({'status': 'error', 'message': 'job_id required'}), 400

    if 'file' not in request.files:
        return jsonify({'status': 'error', 'message': 'No file provided'}), 400

    f = request.files['file']
    job_dir = os.path.join(UPLOADS_DIR, job_id)
    os.makedirs(job_dir, exist_ok=True)

    tar_path = os.path.join(job_dir, 'dataset.tar.gz')
    f.save(tar_path)

    # Extract
    dataset_dir = os.path.join(job_dir, 'dataset')
    os.makedirs(dataset_dir, exist_ok=True)
    os.system(f'tar -xzf {tar_path} -C {dataset_dir}')
    os.remove(tar_path)

    return jsonify({'status': 'success', 'message': 'Dataset uploaded', 'job_id': job_id}), 200


@app.route('/api/train/upload-model', methods=['POST'])
def upload_model():
    """Receive base model weights for finetuning."""
    job_id = request.form.get('job_id')
    if not job_id:
        return jsonify({'status': 'error', 'message': 'job_id required'}), 400

    if 'file' not in request.files:
        return jsonify({'status': 'error', 'message': 'No file provided'}), 400

    f = request.files['file']
    job_dir = os.path.join(UPLOADS_DIR, job_id)
    os.makedirs(job_dir, exist_ok=True)

    tar_path = os.path.join(job_dir, 'model.tar.gz')
    f.save(tar_path)

    model_dir = os.path.join(job_dir, 'base_model')
    os.makedirs(model_dir, exist_ok=True)
    os.system(f'tar -xzf {tar_path} -C {model_dir}')
    os.remove(tar_path)

    return jsonify({'status': 'success', 'message': 'Model uploaded', 'job_id': job_id}), 200


@app.route('/api/train/start', methods=['POST'])
def start_training():
    """Start a training job."""
    data = request.json
    job_id = data.get('job_id')
    if not job_id:
        return jsonify({'status': 'error', 'message': 'job_id required'}), 400

    with jobs_lock:
        if job_id in jobs and jobs[job_id].get('status') == 'training':
            return jsonify({'status': 'error', 'message': 'Job already running'}), 409

    config = {
        'policy': data.get('policy'),
        'train_settings': data.get('train_settings'),
        'dataset_info': data.get('dataset_info'),
        'callback_url': data.get('callback_url'),
    }

    job_dir = os.path.join(UPLOADS_DIR, job_id)
    if not os.path.exists(os.path.join(job_dir, 'dataset')):
        return jsonify({'status': 'error', 'message': 'Dataset not uploaded yet'}), 400

    # Save config for the training subprocess
    config_path = os.path.join(job_dir, 'train_config.json')
    with open(config_path, 'w') as f:
        json.dump(config, f)

    with jobs_lock:
        jobs[job_id] = {
            'status': 'starting',
            'config': config,
            'progress': 0,
            'error': None,
        }

    # Start training in background
    socketio.start_background_task(target=_run_training, job_id=job_id, job_dir=job_dir, config=config)

    return jsonify({'status': 'success', 'message': 'Training started', 'job_id': job_id}), 200


@app.route('/api/train/status/<job_id>', methods=['GET'])
def get_status(job_id):
    with jobs_lock:
        job = jobs.get(job_id)
    if not job:
        return jsonify({'status': 'error', 'message': 'Job not found'}), 404
    return jsonify({
        'status': 'success',
        'job': {
            'job_id': job_id,
            'status': job['status'],
            'progress': job.get('progress', 0),
            'error': job.get('error'),
        }
    }), 200


@app.route('/api/train/stop/<job_id>', methods=['POST'])
def stop_training(job_id):
    with jobs_lock:
        job = jobs.get(job_id)
    if not job:
        return jsonify({'status': 'error', 'message': 'Job not found'}), 404

    process = job.get('process')
    if process and process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
        except Exception:
            process.kill()

    with jobs_lock:
        jobs[job_id]['status'] = 'stopped'

    return jsonify({'status': 'success', 'message': 'Training stopped'}), 200


@app.route('/api/train/download/<job_id>', methods=['GET'])
def download_model(job_id):
    """Download the trained model as tar.gz."""
    ckpt_dir = os.path.join(CHECKPOINTS_DIR, job_id)
    if not os.path.exists(ckpt_dir):
        return jsonify({'status': 'error', 'message': 'Checkpoint not found'}), 404

    tar_path = os.path.join(CHECKPOINTS_DIR, f'{job_id}.tar.gz')
    if not os.path.exists(tar_path):
        os.system(f'tar -czf {tar_path} -C {ckpt_dir} .')

    return send_file(tar_path, mimetype='application/gzip', as_attachment=True,
                     download_name=f'checkpoint_{job_id}.tar.gz')


@app.route('/api/train/jobs', methods=['GET'])
def list_jobs():
    with jobs_lock:
        result = []
        for jid, job in jobs.items():
            result.append({
                'job_id': jid,
                'status': job['status'],
                'progress': job.get('progress', 0),
            })
    return jsonify({'status': 'success', 'jobs': result}), 200


def _check_gpu():
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False


def _run_training(job_id, job_dir, config):
    """Run the training subprocess and stream logs via SocketIO."""
    with jobs_lock:
        jobs[job_id]['status'] = 'training'

    socketio.emit('train_status', {'job_id': job_id, 'status': 'training'})

    ckpt_dir = os.path.join(CHECKPOINTS_DIR, job_id)
    os.makedirs(ckpt_dir, exist_ok=True)

    try:
        # Build command
        cmd = [
            'python3', '-u', 'train_worker.py',
            '--job_id', job_id,
            '--job_dir', job_dir,
            '--checkpoint_dir', ckpt_dir,
        ]

        env = os.environ.copy()
        env['PYTHONPATH'] = '/app:/app/lerobot/src'

        popen_args = {
            'stdout': subprocess.PIPE,
            'stderr': subprocess.STDOUT,
            'text': True,
            'bufsize': 1,
            'env': env,
            'cwd': '/app',
        }
        if os.name != 'nt':
            popen_args['preexec_fn'] = os.setsid
            cmd = ['stdbuf', '-oL'] + cmd

        process = subprocess.Popen(cmd, **popen_args)

        with jobs_lock:
            jobs[job_id]['process'] = process

        # Stream stdout
        for line in process.stdout:
            stripped = line.strip()
            if not stripped:
                continue

            print(f'[{job_id}] {stripped}', flush=True)

            # Determine log type
            msg_type = 'stdout'
            if stripped.startswith('[ERROR]'):
                msg_type = 'error'
            elif stripped.startswith('[WARNING]'):
                msg_type = 'warning'

            # Emit to all connected clients
            socketio.emit('task_log', {
                'id': f'train_{job_id}',
                'message': stripped,
                'type': msg_type,
            })

            # Parse TRAIN_LOG for progress
            if '[TRAIN_LOG]' in stripped:
                try:
                    log_json = json.loads(stripped.split('[TRAIN_LOG] ')[1])
                    progress = log_json.get('epoch', 0) / max(log_json.get('total_epoch', 1), 1)
                    with jobs_lock:
                        jobs[job_id]['progress'] = progress
                except Exception:
                    pass

        return_code = process.wait()

        if return_code == 0:
            with jobs_lock:
                jobs[job_id]['status'] = 'finished'
                jobs[job_id]['progress'] = 1.0

            socketio.emit('train_status', {
                'job_id': job_id,
                'status': 'finished',
            })
            socketio.emit('task_log', {
                'id': f'train_{job_id}',
                'message': '[SUCCESS] Training completed',
                'type': 'success',
            })

            # Auto-send model back to local server
            _auto_send_model(job_id, config)
        else:
            with jobs_lock:
                jobs[job_id]['status'] = 'failed'
                jobs[job_id]['error'] = f'Process exited with code {return_code}'

            socketio.emit('train_status', {
                'job_id': job_id,
                'status': 'failed',
                'error': f'Process exited with code {return_code}',
            })

    except Exception as e:
        error_msg = traceback.format_exc()
        print(f'[ERROR] Training job {job_id} failed: {error_msg}', flush=True)
        with jobs_lock:
            jobs[job_id]['status'] = 'failed'
            jobs[job_id]['error'] = str(e)

        socketio.emit('train_status', {
            'job_id': job_id,
            'status': 'failed',
            'error': str(e),
        })


def _auto_send_model(job_id, config):
    """After training finishes, push the model back to the local server."""
    callback_url = config.get('callback_url')
    if not callback_url:
        print(f'[INFO] No callback_url for job {job_id}, model available for download.', flush=True)
        return

    ckpt_dir = os.path.join(CHECKPOINTS_DIR, job_id)
    tar_path = os.path.join(CHECKPOINTS_DIR, f'{job_id}.tar.gz')

    try:
        os.system(f'tar -czf {tar_path} -C {ckpt_dir} .')

        import requests
        with open(tar_path, 'rb') as f:
            resp = requests.post(
                callback_url,
                files={'file': (f'checkpoint_{job_id}.tar.gz', f, 'application/gzip')},
                data={'job_id': job_id},
                timeout=300,
            )
        if resp.status_code == 200:
            print(f'[SUCCESS] Model auto-sent to {callback_url} for job {job_id}', flush=True)
            socketio.emit('train_status', {
                'job_id': job_id,
                'status': 'delivered',
            })
        else:
            print(f'[WARNING] Auto-send failed ({resp.status_code}): {resp.text}', flush=True)
            # Model still available for manual download
    except Exception as e:
        print(f'[WARNING] Auto-send failed for job {job_id}: {e}', flush=True)
        # Model still available for manual download


if __name__ == '__main__':
    print('=' * 60)
    print('EasyTrainer Training Server')
    print(f'Data directory: {DATA_DIR}')
    print(f'GPU available: {_check_gpu()}')
    print('=' * 60)
    socketio.run(app, host='0.0.0.0', port=5100, debug=False)
