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
import collections

from flask import Flask, request, jsonify, send_file
from flask_socketio import SocketIO
from flask_cors import CORS

APP_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.environ.get('TRAINING_SERVER_DATA_DIR', '/data')
DATASETS_DIR = os.path.join(DATA_DIR, 'datasets')
CHECKPOINTS_DIR = os.path.join(DATA_DIR, 'checkpoints')
UPLOADS_DIR = os.path.join(DATA_DIR, 'uploads')

# 학습 자식 프로세스의 cwd / PYTHONPATH 베이스. 이 파일이 위치한 디렉터리를 기본
# 으로 쓴다 — training_server가 자체 컨테이너(WORKDIR=/app) 안이든, backend
# 컨테이너 안의 마운트 경로(/root/backend/training_server) 안이든 자동으로 잡힘.
# 외부에서 강제하려면 TRAINING_SERVER_APP_DIR 환경변수로 override 가능.
APP_DIR = os.environ.get(
    'TRAINING_SERVER_APP_DIR',
    os.path.dirname(os.path.abspath(__file__)),
)

for d in [DATASETS_DIR, CHECKPOINTS_DIR, UPLOADS_DIR]:
    os.makedirs(d, exist_ok=True)

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading', max_http_buffer_size=500 * 1024 * 1024)

LOG_BUFFER_SIZE = 10000


def _append_log(job_id, msg_type, message):
    with jobs_lock:
        job = jobs.get(job_id)
        if not job:
            return
        idx = job['log_next']
        job['log_buffer'].append({'idx': idx, 'type': msg_type, 'message': message})
        job['log_next'] = idx + 1

# In-memory job registry
# job_id -> { status, process, config, progress, error, callback_url }
jobs = {}
jobs_lock = threading.Lock()


@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok', 'gpu_available': _check_gpu()}), 200


def _safe_id(value: str) -> str:
    """Reject path-traversal characters in machine_id / dataset_id / checkpoint_id."""
    if not value:
        raise ValueError('id required')
    if '/' in value or '..' in value or value.startswith('.'):
        raise ValueError(f'invalid id: {value}')
    return value


def _dataset_path(machine_id: str, dataset_id: str) -> str:
    return os.path.join(DATASETS_DIR, _safe_id(machine_id), _safe_id(dataset_id))


def _checkpoint_path(machine_id: str, checkpoint_id: str) -> str:
    return os.path.join(CHECKPOINTS_DIR, _safe_id(machine_id), _safe_id(checkpoint_id))


@app.route('/api/train/upload-dataset', methods=['POST'])
def upload_dataset():
    """Receive a dataset tar.gz, extract to datasets/<machine_id>/<dataset_id>/."""
    machine_id = request.form.get('machine_id')
    dataset_id = request.form.get('dataset_id')
    if not machine_id or not dataset_id:
        return jsonify({'status': 'error', 'message': 'machine_id and dataset_id required'}), 400

    if 'file' not in request.files:
        return jsonify({'status': 'error', 'message': 'No file provided'}), 400

    try:
        target_dir = _dataset_path(machine_id, dataset_id)
    except ValueError as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

    # Replace any existing dataset for this id (re-upload).
    if os.path.isdir(target_dir):
        shutil.rmtree(target_dir, ignore_errors=True)
    os.makedirs(target_dir, exist_ok=True)

    f = request.files['file']
    tar_path = target_dir + '.tar.gz'
    f.save(tar_path)
    try:
        os.system(f'tar -xzf "{tar_path}" -C "{target_dir}"')
    finally:
        try:
            os.remove(tar_path)
        except Exception:
            pass

    return jsonify({
        'status': 'success', 'message': 'Dataset uploaded',
        'machine_id': machine_id, 'dataset_id': dataset_id,
    }), 200


@app.route('/api/train/upload-model', methods=['POST'])
def upload_model():
    """Receive base model weights, extract to checkpoints/<machine_id>/<checkpoint_id>/."""
    machine_id = request.form.get('machine_id')
    checkpoint_id = request.form.get('checkpoint_id')
    if not machine_id or not checkpoint_id:
        return jsonify({'status': 'error', 'message': 'machine_id and checkpoint_id required'}), 400

    if 'file' not in request.files:
        return jsonify({'status': 'error', 'message': 'No file provided'}), 400

    try:
        target_dir = _checkpoint_path(machine_id, checkpoint_id)
    except ValueError as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

    if os.path.isdir(target_dir):
        shutil.rmtree(target_dir, ignore_errors=True)
    os.makedirs(target_dir, exist_ok=True)

    f = request.files['file']
    tar_path = target_dir + '.tar.gz'
    f.save(tar_path)
    try:
        os.system(f'tar -xzf "{tar_path}" -C "{target_dir}"')
    finally:
        try:
            os.remove(tar_path)
        except Exception:
            pass

    return jsonify({
        'status': 'success', 'message': 'Model uploaded',
        'machine_id': machine_id, 'checkpoint_id': checkpoint_id,
    }), 200


@app.route('/api/train/start', methods=['POST'])
def start_training():
    """Start a training job.

    Body must include: job_id, machine_id, checkpoint_id, dataset_ids[], policy, train_settings.
    Optional: load_model_checkpoint_id (for transfer learning).
    """
    data = request.json or {}
    job_id = data.get('job_id')
    machine_id = data.get('machine_id')
    checkpoint_id = data.get('checkpoint_id')
    dataset_ids = data.get('dataset_ids') or []

    if not (job_id and machine_id and checkpoint_id and dataset_ids):
        return jsonify({
            'status': 'error',
            'message': 'job_id, machine_id, checkpoint_id and dataset_ids are required',
        }), 400

    if len(dataset_ids) != 1:
        return jsonify({
            'status': 'error',
            'message': 'Exactly one dataset_id is supported (multi-dataset merge is not yet implemented)',
        }), 400

    try:
        dataset_dir = _dataset_path(machine_id, dataset_ids[0])
        ckpt_out_dir = _checkpoint_path(machine_id, checkpoint_id)
    except ValueError as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

    if not os.path.isdir(dataset_dir):
        return jsonify({
            'status': 'error',
            'message': f'Dataset {dataset_ids[0]} not uploaded for this machine',
        }), 400

    with jobs_lock:
        if job_id in jobs and jobs[job_id].get('status') == 'training':
            return jsonify({'status': 'error', 'message': 'Job already running'}), 409

    # Build a transient job dir that train_worker can read (config + dataset symlink + base_model link).
    job_dir = os.path.join(UPLOADS_DIR, job_id)
    if os.path.isdir(job_dir):
        shutil.rmtree(job_dir, ignore_errors=True)
    os.makedirs(job_dir, exist_ok=True)

    ds_link = os.path.join(job_dir, 'dataset')
    try:
        os.symlink(dataset_dir, ds_link)
    except OSError:
        # Fallback for filesystems that don't allow symlinks.
        shutil.copytree(dataset_dir, ds_link)

    load_id = data.get('load_model_checkpoint_id')
    if load_id:
        try:
            base_model_src = _checkpoint_path(machine_id, load_id)
        except ValueError as e:
            return jsonify({'status': 'error', 'message': str(e)}), 400
        if os.path.isdir(base_model_src):
            base_link = os.path.join(job_dir, 'base_model')
            try:
                os.symlink(base_model_src, base_link)
            except OSError:
                shutil.copytree(base_model_src, base_link)

    config = {
        'policy': data.get('policy'),
        'train_settings': data.get('train_settings'),
        'dataset_info': data.get('dataset_info'),
        'sensor_ids': data.get('sensor_ids', []),
        'machine_id': machine_id,
        'checkpoint_id': checkpoint_id,
        'dataset_ids': dataset_ids,
    }
    with open(os.path.join(job_dir, 'train_config.json'), 'w') as f:
        json.dump(config, f)

    with jobs_lock:
        jobs[job_id] = {
            'status': 'starting',
            'config': config,
            'progress': 0,
            'error': None,
            'log_buffer': collections.deque(maxlen=LOG_BUFFER_SIZE),
            'log_next': 0,
            'machine_id': machine_id,
            'checkpoint_id': checkpoint_id,
            'ckpt_out_dir': ckpt_out_dir,
            'job_dir': job_dir,
        }

    socketio.start_background_task(
        target=_run_training, job_id=job_id, job_dir=job_dir,
        ckpt_out_dir=ckpt_out_dir, config=config,
    )
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
            pgid = os.getpgid(process.pid)
        except ProcessLookupError:
            pgid = None

        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Worker still alive (likely stuck in a CUDA call). Kill the
                # whole process group so child workers also die.
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
        else:
            process.kill()

    with jobs_lock:
        jobs[job_id]['status'] = 'stopped'

    return jsonify({'status': 'success', 'message': 'Training stopped'}), 200


@app.route('/api/train/logs/<job_id>', methods=['GET'])
def get_logs(job_id):
    """Return new log lines with index >= since."""
    try:
        since = int(request.args.get('since', 0))
    except (TypeError, ValueError):
        since = 0
    with jobs_lock:
        job = jobs.get(job_id)
        if not job:
            return jsonify({'status': 'error', 'message': 'Job not found'}), 404
        lines = [l for l in job['log_buffer'] if l['idx'] >= since]
        next_idx = job['log_next']
    return jsonify({'status': 'success', 'lines': lines, 'next': next_idx}), 200


@app.route('/api/train/download/<job_id>', methods=['GET'])
def download_model(job_id):
    """Download the trained model as tar.gz. Looks up the path from the job entry."""
    with jobs_lock:
        job = jobs.get(job_id)
    if not job:
        return jsonify({'status': 'error', 'message': 'Job not found'}), 404

    ckpt_dir = job.get('ckpt_out_dir')
    if not ckpt_dir or not os.path.isdir(ckpt_dir):
        return jsonify({'status': 'error', 'message': 'Checkpoint not found'}), 404

    tar_path = ckpt_dir.rstrip('/') + '.tar.gz'
    if not os.path.exists(tar_path):
        os.system(f'tar -czf "{tar_path}" -C "{ckpt_dir}" .')

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


def _run_training(job_id, job_dir, ckpt_out_dir, config):
    """Run the training subprocess and stream logs via SocketIO."""
    with jobs_lock:
        jobs[job_id]['status'] = 'training'

    ckpt_dir = ckpt_out_dir
    if os.path.isdir(ckpt_dir):
        # Wipe stale contents from a previous training run for this checkpoint id.
        shutil.rmtree(ckpt_dir, ignore_errors=True)
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
        # APP_DIR: training_server itself.
        # APP_DIR/lerobot/src: vendored lerobot.
        # parent of APP_DIR: in embedded mode (training_server inside backend container)
        #   this is /root/backend, which exposes `utils`, `api`, etc. as packages.
        parent_dir = os.path.dirname(APP_DIR)
        env['PYTHONPATH'] = ':'.join([
            APP_DIR,
            os.path.join(APP_DIR, 'lerobot', 'src'),
            parent_dir,
        ])

        popen_args = {
            'stdout': subprocess.PIPE,
            'stderr': subprocess.STDOUT,
            'text': True,
            'bufsize': 1,
            'env': env,
            'cwd': APP_DIR,
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

            # Append to log buffer (backend pulls via /api/train/logs/<id>?since=N)
            _append_log(job_id, msg_type, stripped)

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

        # If stop_training already set status to 'stopped', keep it.
        with jobs_lock:
            current_status = jobs[job_id].get('status')
        if current_status == 'stopped':
            _append_log(job_id, 'warning', '[WARNING] Training stopped')
        elif return_code == 0:
            with jobs_lock:
                jobs[job_id]['status'] = 'finished'
                jobs[job_id]['progress'] = 1.0
            _append_log(job_id, 'success', '[SUCCESS] Training completed')
        else:
            with jobs_lock:
                jobs[job_id]['status'] = 'failed'
                jobs[job_id]['error'] = f'Process exited with code {return_code}'
            _append_log(job_id, 'error', f'[ERROR] Process exited with code {return_code}')

    except Exception as e:
        error_msg = traceback.format_exc()
        print(f'[ERROR] Training job {job_id} failed: {error_msg}', flush=True)
        with jobs_lock:
            jobs[job_id]['status'] = 'failed'
            jobs[job_id]['error'] = str(e)
        _append_log(job_id, 'error', f'[ERROR] {e}')


if __name__ == '__main__':
    port = int(os.environ.get('TRAINING_SERVER_PORT', 5100))
    print('=' * 60)
    print('EasyTrainer Training Server')
    print(f'Data directory: {DATA_DIR}')
    print(f'Port: {port}')
    print(f'GPU available: {_check_gpu()}')
    print('=' * 60)
    socketio.run(app, host='0.0.0.0', port=port, debug=False, allow_unsafe_werkzeug=True)
