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

from ...configs.global_configs import DATASET_DIR, CHECKPOINT_DIR, get_checkpoint_dir, resolve_checkpoint_dir
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

remote_train_bp = Blueprint('remote_train', __name__)

PROCESS_ID = 'train_task'


def _normalize_url(url: str) -> str:
    url = (url or '').strip().rstrip('/')
    if not url:
        return ''
    if not url.startswith(('http://', 'https://')):
        url = 'http://' + url
    return url


REMOTE_DEFAULT_URL = 'http://easytrainer.training_server.com'
LOCAL_DEFAULT_URL = 'http://localhost:5100'


@remote_train_bp.route('/remote-train/default-url', methods=['GET'])
def remote_train_default_url():
    """Decide the default training-server URL for the frontend.

    런처가 같은 호스트에 training_server 컨테이너를 띄워뒀으면 로컬 URL을
    추천하고, 그렇지 않으면 공용 원격 서버 URL을 기본값으로 돌려준다.
    Backend는 host network 모드로 돌기 때문에 127.0.0.1:5100이 곧 launcher가
    띄운 컨테이너에 도달.
    """
    import socket
    local_running = False
    try:
        with socket.create_connection(('127.0.0.1', 5100), timeout=0.5):
            local_running = True
    except OSError:
        local_running = False
    return {
        'status': 'success',
        'local_running': local_running,
        'default_url': LOCAL_DEFAULT_URL if local_running else REMOTE_DEFAULT_URL,
    }, 200


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


@remote_train_bp.route('/train/queue', methods=['POST'])
def train_queue_enqueue():
    """체크포인트를 학습 큐에 추가. status가 active(queued/running)이면 no-op."""
    data = request.json or {}
    server_url = _normalize_url(data.get('server_url', ''))
    checkpoint_id = data.get('checkpoint_id')
    callback_url = data.get('callback_url', '')

    if not server_url or checkpoint_id is None:
        return {'status': 'error', 'message': 'server_url and checkpoint_id required'}, 400

    scheduler = current_app.training_scheduler
    enqueued = scheduler.enqueue(int(checkpoint_id), server_url, callback_url)
    return {
        'status': 'success',
        'enqueued': enqueued,
        'checkpoint_id': checkpoint_id,
    }, 200


@remote_train_bp.route('/train/queue/<int:checkpoint_id>', methods=['DELETE'])
def train_queue_cancel(checkpoint_id):
    """큐에서 제거 또는 실행 중이면 stop 신호.

    응답의 'result':
      - 'canceled': 큐 대기 중이었음, 즉시 status=canceled
      - 'stopping': 실행 중이었음, stop 신호 전달 (실제 종료는 스케줄러가 처리)
      - 'not_found': 존재하지 않거나 이미 종료된 상태
    """
    scheduler = current_app.training_scheduler
    result = scheduler.cancel(int(checkpoint_id))

    # 'not_found'는 두 가지 경우:
    #   (a) 정말로 row가 없음 — 진짜 404
    #   (b) row는 있는데 이미 finished/failed/canceled 상태 — 사용자가 dialog
    #       에서 "cancel" 누른 의도는 보통 "이 항목을 큐 패널에서 치우자"이므로
    #       idempotent하게 soft-delete + disk 정리 후 200으로 응답한다.
    if result == 'not_found':
        from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
        ckpt = CheckpointModel.find(int(checkpoint_id))
        if ckpt is None:
            return {'status': 'error', 'result': 'not_found'}, 404
        try:
            ckpt.delete_instance()
        except Exception as e:
            print(f'[train/queue cancel] DB soft-delete failed for ckpt {checkpoint_id}: {e}')
        ckpt_dir = get_checkpoint_dir(checkpoint_id)
        if os.path.isdir(ckpt_dir):
            try:
                shutil.rmtree(ckpt_dir)
            except Exception as e:
                print(f'[train/queue cancel] rmtree failed for {ckpt_dir}: {e}')
        return {'status': 'success', 'result': 'removed',
                'checkpoint_id': checkpoint_id}, 200

    # 부분 다운로드된 checkpoint 디렉토리 정리는 cancel 시점에 즉시.
    # (실행 중 → stopping의 경우, runner가 stop_event를 보고 빠져나오면 추가
    # cleanup이 필요하지만 그 시점엔 Scheduler가 status=canceled로 마감하므로
    # 다음 listing에서 자연스럽게 제외된다.)
    if result == 'canceled':
        ckpt_dir = get_checkpoint_dir(checkpoint_id)
        if os.path.isdir(ckpt_dir):
            try:
                shutil.rmtree(ckpt_dir)
            except Exception as e:
                print(f'[train/queue cancel] rmtree failed for {ckpt_dir}: {e}')

    return {'status': 'success', 'result': result, 'checkpoint_id': checkpoint_id}, 200


@remote_train_bp.route('/train/queue', methods=['GET'])
def train_queue_list():
    """현재 큐 스냅샷. {running, queued[], recent[]}."""
    scheduler = current_app.training_scheduler
    return {'status': 'success', **scheduler.snapshot()}, 200


# ---------------------------------------------------------------------------
# Legacy aliases — 기존 프론트엔드 호출처 호환용. 새 큐 라우트로 라우팅.
# (당분간 유지, 다음 단계에 프론트엔드에서 직접 큐 라우트로 이전.)
# ---------------------------------------------------------------------------
@remote_train_bp.route('/remote-train/start', methods=['POST'])
def remote_train_start_legacy():
    return train_queue_enqueue()


@remote_train_bp.route('/remote-train/stop', methods=['POST'])
def remote_train_stop_legacy():
    """과거 동작과 가장 가까운 동작: 현재 실행 중인 체크포인트를 cancel."""
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
    scheduler = current_app.training_scheduler
    snap = scheduler.snapshot()
    running = snap.get('running')
    if running and running.get('id') is not None:
        scheduler.cancel(int(running['id']))
        # legacy: 진행 중이던 체크포인트 row도 soft-delete.
        ckpt = CheckpointModel.find(int(running['id']))
        if ckpt is not None:
            try:
                ckpt.delete_instance()
            except Exception:
                pass
    return {'status': 'success', 'message': 'Stop signal sent (legacy alias)'}, 200


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

    ckpt_dir = get_checkpoint_dir(checkpoint_id)
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
        ckpt.status = 'finished'
        _apply_result_json(ckpt, ckpt_dir)
        ckpt.save()

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

    # Auto-detect has_succeed from the first dataset's first parquet — UI 가
    # 학습 시 이 플래그를 보내주지 않으므로 backend 에서 dataset 을 직접 보고
    # 결정한다. 결정된 값은 checkpoint.train_settings 에도 persist 시켜서 추후
    # 추론 (checkpoint_test) 에서 has_succeed 분기가 정상 동작하도록.
    #
    # NOTE: 이 함수는 학습 시작 직전에 호출되므로 dataset 의 'succeed' 컬럼 존재
    # = 새 모델이 succeed dim 으로 학습될 것 (training_server 가 같은 방식으로
    # 자동 감지). 옛 checkpoint (이미 학습 완료된 것) 는 disk 에 있는 model 의
    # action shape 로만 신뢰성 있게 판단 가능 — 이건 별도 백필 스크립트가 처리.
    if 'has_succeed' not in train_settings and isinstance(dataset_info, dict):
        try:
            import pyarrow.parquet as pq
            _first_ds_id = next(iter(dataset_info.keys()), None)
            if _first_ds_id is not None:
                _parquet_path = os.path.join(
                    DATASET_DIR, str(_first_ds_id),
                    'data', 'chunk-000', 'episode_000000.parquet',
                )
                if os.path.exists(_parquet_path):
                    _cols = pq.read_table(_parquet_path).column_names
                    if 'succeed' in _cols:
                        train_settings['has_succeed'] = True
                        # DB 에도 즉시 반영 (set & save)
                        checkpoint.train_settings = json.dumps(train_settings)
                        checkpoint.save()
                        print(
                            f'[checkpoint {checkpoint.id}] auto-detected '
                            f'has_succeed=True from dataset {_first_ds_id}',
                            flush=True,
                        )
        except Exception as _ex:
            print(f'[checkpoint {checkpoint.id}] has_succeed auto-detect failed: {_ex}', flush=True)

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
    Multi dataset: merge into a temp dir first (non-destructive), then tar that
    so training_server sees a single LeRobot-format dataset.
    """
    dataset_ids = [str(x) for x in dataset_ids]
    for ds_id in dataset_ids:
        ds_dir = os.path.join(DATASET_DIR, ds_id)
        if not os.path.isdir(ds_dir):
            raise FileNotFoundError(f'Dataset directory not found: {ds_dir}')

    if len(dataset_ids) == 1:
        ds_dir = os.path.join(DATASET_DIR, dataset_ids[0])
        with tarfile.open(tar_path, 'w:gz') as tar:
            tar.add(ds_dir, arcname='.')
        return

    merge_dir = tempfile.mkdtemp(prefix='et_merge_')
    try:
        _merge_datasets_to_dir(dataset_ids, merge_dir)
        with tarfile.open(tar_path, 'w:gz') as tar:
            tar.add(merge_dir, arcname='.')
    finally:
        shutil.rmtree(merge_dir, ignore_errors=True)


def _merge_datasets_to_dir(dataset_ids, output_dir):
    """Merge multiple LeRobot datasets into output_dir as a single dataset.

    First dataset becomes the base (full copy). Subsequent datasets get their
    episodes appended with re-indexed parquet/video/image paths and merged
    tasks.jsonl. Source datasets are not modified.
    """
    from ...utils.lerobot_io import (
        _read_json, _write_json, _read_jsonl, _write_jsonl, _append_jsonl,
        get_dataset_info, list_episodes,
        PARQUET_PATH_TEMPLATE, INFO_PATH, EPISODES_PATH, TASKS_PATH,
        EPISODES_STATS_PATH, DEFAULT_CHUNK_SIZE,
    )

    if not dataset_ids:
        raise ValueError('dataset_ids cannot be empty')

    base_id = str(dataset_ids[0])
    base_path = os.path.join(DATASET_DIR, base_id)
    if not os.path.isdir(base_path):
        raise FileNotFoundError(f'Dataset {base_id} not found at {base_path}')

    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    shutil.copytree(base_path, output_dir, symlinks=False)

    for tgt_id in dataset_ids[1:]:
        tgt_id = str(tgt_id)
        tgt_path = os.path.join(DATASET_DIR, tgt_id)
        if not os.path.isdir(tgt_path):
            raise FileNotFoundError(f'Dataset {tgt_id} not found at {tgt_path}')

        tgt_info = get_dataset_info(tgt_path)
        if tgt_info is None:
            raise RuntimeError(f'No info.json in dataset {tgt_id}')

        tgt_episodes = list_episodes(tgt_path)
        tgt_chunk_size = tgt_info.get('chunks_size', DEFAULT_CHUNK_SIZE)

        out_tasks = _read_jsonl(os.path.join(output_dir, TASKS_PATH))
        tgt_tasks = _read_jsonl(os.path.join(tgt_path, TASKS_PATH))
        existing = {t['task'] for t in out_tasks}
        for t in tgt_tasks:
            if t['task'] not in existing:
                out_tasks.append({'task_index': len(out_tasks), 'task': t['task']})
                existing.add(t['task'])
        _write_jsonl(out_tasks, os.path.join(output_dir, TASKS_PATH))

        for ep_entry in tgt_episodes:
            ep_idx = ep_entry['episode_index']
            tgt_chunk = ep_idx // tgt_chunk_size

            out_info = _read_json(os.path.join(output_dir, INFO_PATH))
            new_ep_idx = out_info['total_episodes']
            out_chunk_size = out_info.get('chunks_size', DEFAULT_CHUNK_SIZE)
            new_chunk = new_ep_idx // out_chunk_size
            num_frames = ep_entry.get('length', 0)

            tgt_parquet = os.path.join(tgt_path, PARQUET_PATH_TEMPLATE.format(chunk=tgt_chunk, ep=ep_idx))
            if not os.path.exists(tgt_parquet):
                continue
            new_parquet = os.path.join(output_dir, PARQUET_PATH_TEMPLATE.format(chunk=new_chunk, ep=new_ep_idx))
            os.makedirs(os.path.dirname(new_parquet), exist_ok=True)
            shutil.copy2(tgt_parquet, new_parquet)

            features = out_info.get('features', {})
            for key, feat in features.items():
                if not key.startswith('observation.images.'):
                    continue
                if feat.get('dtype') == 'image':
                    old = os.path.join(tgt_path, 'images', key, f'episode_{ep_idx:06d}')
                    new = os.path.join(output_dir, 'images', key, f'episode_{new_ep_idx:06d}')
                    if os.path.isdir(old):
                        if os.path.isdir(new):
                            shutil.rmtree(new)
                        shutil.copytree(old, new)
                elif feat.get('dtype') == 'video':
                    old = os.path.join(tgt_path, 'videos', f'chunk-{tgt_chunk:03d}', key, f'episode_{ep_idx:06d}.mp4')
                    new = os.path.join(output_dir, 'videos', f'chunk-{new_chunk:03d}', key, f'episode_{new_ep_idx:06d}.mp4')
                    if os.path.isfile(old):
                        os.makedirs(os.path.dirname(new), exist_ok=True)
                        shutil.copy2(old, new)

            tgt_stats_list = _read_jsonl(os.path.join(tgt_path, EPISODES_STATS_PATH))
            ep_stats = {}
            for s in tgt_stats_list:
                if s.get('episode_index') == ep_idx:
                    ep_stats = s.get('stats', {})
                    break

            out_info['total_episodes'] = new_ep_idx + 1
            out_info['total_frames'] = out_info.get('total_frames', 0) + num_frames
            out_info['total_chunks'] = new_chunk + 1
            out_info['total_tasks'] = len(out_tasks)
            out_info['splits'] = {'train': f'0:{new_ep_idx + 1}'}
            _write_json(out_info, os.path.join(output_dir, INFO_PATH))

            _append_jsonl(
                {'episode_index': new_ep_idx, 'length': num_frames, 'tasks': ep_entry.get('tasks', [''])},
                os.path.join(output_dir, EPISODES_PATH),
            )
            _append_jsonl(
                {'episode_index': new_ep_idx, 'stats': ep_stats},
                os.path.join(output_dir, EPISODES_STATS_PATH),
            )


def _tar_checkpoint(checkpoint_id, tar_path):
    ckpt_dir = get_checkpoint_dir(checkpoint_id)
    if not os.path.isdir(ckpt_dir):
        raise FileNotFoundError(f'Checkpoint directory not found: {ckpt_dir}')
    with tarfile.open(tar_path, 'w:gz') as tar:
        tar.add(ckpt_dir, arcname='.')


def _download_and_install_model(server_url, job_id, checkpoint_id):
    """Pull the trained model from training_server and extract into the local checkpoint dir.

    로컬 training_server는 backend와 같은 host volume(/opt/easytrainer/training_data)
    을 공유하고 동일한 <machine_id>/<checkpoint_id>/ 경로에 직접 저장하므로,
    이미 모델 파일이 있으면 HTTP 다운로드/재추출을 건너뛴다 (큰 모델은 GB 단위라
    같은 디스크 위에서 tar→post→untar를 반복하는 게 명백한 낭비).
    """
    ckpt_dir = get_checkpoint_dir(checkpoint_id)
    os.makedirs(ckpt_dir, exist_ok=True)

    # 학습 산출물의 핵심 파일이 이미 같은 자리에 있으면 로컬 학습으로 간주.
    # config.json은 모든 policy 타입이 공통으로 떨궈주는 sentinel.
    if os.path.exists(os.path.join(ckpt_dir, 'config.json')):
        return

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


def _apply_result_json(checkpoint, ckpt_dir):
    """Read result.json (best_epoch, loss) into the checkpoint row in-place.

    Caller is responsible for .save(). Returns True if values were applied.
    """
    result_path = os.path.join(ckpt_dir, 'result.json')
    if not os.path.exists(result_path):
        return False
    with open(result_path) as f:
        data = json.load(f)
    loss = data.get('loss')
    best_epoch = data.get('best_epoch')
    if loss is not None:
        checkpoint.loss = float(loss)
    if best_epoch is not None:
        checkpoint.best_epoch = int(best_epoch)
    return True


def _persist_train_result(checkpoint, socketio_instance, log_id):
    """학습 종료 후 result.json의 best_epoch/loss를 DB에 반영.

    실패해도 학습 결과 자체에는 영향을 주지 않는다 — 단순 메타 기록이므로
    경고만 emit하고 넘어간다.
    """
    ckpt_dir = get_checkpoint_dir(checkpoint.id)
    try:
        if _apply_result_json(checkpoint, ckpt_dir):
            checkpoint.save()
            _emit_log(socketio_instance, log_id,
                      f'[INFO] Saved result: loss={checkpoint.loss}, best_epoch={checkpoint.best_epoch}',
                      'success')
        else:
            _emit_log(socketio_instance, log_id,
                      '[WARNING] result.json not found in checkpoint dir', 'warning')
    except Exception as e:
        _emit_log(socketio_instance, log_id,
                  f'[WARNING] Failed to persist result.json: {e}', 'warning')


def _trigger_ood_features(checkpoint, socketio_instance, log_id):
    """학습 직후 OOD reference feature를 비동기로 생성.

    실패해도 학습 결과(STATUS_FINISHED)에는 영향을 주지 않는다. ORM row의
    lazy-loaded 관계(policy, task)는 메인 thread에서 미리 dict로 스냅샷한 뒤
    worker thread로 넘겨, scheduler가 row를 detach해도 안전하게 동작하게 한다.
    """
    import threading

    try:
        checkpoint_dict = checkpoint.to_dict()
        policy_dict = checkpoint.policy.to_dict() if checkpoint.policy else {}
        task_dict = checkpoint.task.to_dict() if checkpoint.task else {}
    except Exception as e:
        _emit_log(socketio_instance, log_id,
                  f'[OOD WARNING] Failed to snapshot checkpoint: {e}', 'warning')
        return

    def _worker():
        try:
            from ..process.generate_ood_features import generate_ood_features
            _emit_log(socketio_instance, log_id,
                      '[OOD] Generating reference features...', 'success')
            generate_ood_features(checkpoint_dict, policy_dict, task_dict)
            _emit_log(socketio_instance, log_id,
                      '[OOD] Reference features generated.', 'success')
        except Exception as e:
            _emit_log(socketio_instance, log_id,
                      f'[OOD WARNING] {e}', 'warning')

    threading.Thread(target=_worker, daemon=True, name='ood_features').start()


def run_training_job(checkpoint, server_url, callback_url,
                      stop_event, socketio_instance):
    """Pure execution function: dataset upload → start → poll → download model.

    Status 전이는 절대 건드리지 않는다 — TrainingScheduler가 호출 전후로
    queued→running, running→{finished,failed,canceled}를 일괄 관리.

    Args:
        checkpoint: Checkpoint ORM row
        server_url: training_server base URL (already normalized)
        callback_url: optional, currently unused (server pushes via download API)
        stop_event: threading.Event — set 시 즉시 정리하고 'canceled' 반환
        socketio_instance: 로그 emit용

    Returns:
        'finished' | 'failed' | 'canceled'
    """
    from ...utils.machine_id import machine_id as _machine_id_fn
    machine_id = _machine_id_fn()
    checkpoint_id = checkpoint.id
    job_id = f'{machine_id[:8]}_ckpt_{checkpoint_id}'
    log_id = PROCESS_ID

    try:
        config = _build_train_config(checkpoint)
        dataset_ids = [str(x) for x in config['dataset_info'].keys()]
        if not dataset_ids:
            _emit_log(socketio_instance, log_id, '[ERROR] No datasets assigned to checkpoint', 'error')
            return CheckpointModel.STATUS_FAILED

        # Multi-dataset: backend merges into a temp dir and uploads as a single
        # synthetic bundle so training_server stays single-dataset oriented.
        if len(dataset_ids) > 1:
            bundle_id = f'merged_{job_id}'
            _emit_log(socketio_instance, log_id,
                      f'Merging {len(dataset_ids)} datasets ({", ".join(dataset_ids)}) into bundle {bundle_id}...')
        else:
            bundle_id = dataset_ids[0]

        # 1) Upload dataset bundle (stored at datasets/<machine_id>/<bundle_id>/)
        _emit_log(socketio_instance, log_id, f'Bundling dataset {bundle_id}...')
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
                    data={'machine_id': machine_id, 'dataset_id': bundle_id},
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
                        data={'machine_id': machine_id, 'checkpoint_id': str(checkpoint.load_model_id)},
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
                'machine_id': machine_id,
                'checkpoint_id': str(checkpoint_id),
                'dataset_ids': [bundle_id],
                'load_model_checkpoint_id': str(checkpoint.load_model_id) if checkpoint.load_model_id else None,
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

        # 4) Poll status + logs until done / stopped
        _emit_log(socketio_instance, log_id, 'Training started. Polling status...')
        last_progress = -1.0
        last_log_cursor = 0
        while True:
            if stop_event.is_set():
                _emit_log(socketio_instance, log_id, 'Stop requested. Notifying remote server...', 'warning')
                try:
                    requests.post(f'{server_url}/api/train/stop/{job_id}', timeout=10)
                except Exception:
                    pass
                return CheckpointModel.STATUS_CANCELED

            # Pull new log lines and forward to the frontend.
            try:
                log_resp = requests.get(
                    f'{server_url}/api/train/logs/{job_id}',
                    params={'since': last_log_cursor},
                    timeout=10,
                )
                if log_resp.status_code == 200:
                    log_data = log_resp.json()
                    for line in log_data.get('lines', []):
                        socketio_instance.emit('task_log', {
                            'id': log_id,
                            'message': line.get('message', ''),
                            'type': line.get('type', 'stdout'),
                        })
                    last_log_cursor = log_data.get('next', last_log_cursor)
            except requests.exceptions.RequestException:
                pass

            try:
                resp = requests.get(f'{server_url}/api/train/status/{job_id}', timeout=10)
                if resp.status_code != 200:
                    time.sleep(2)
                    continue
                job = resp.json().get('job', {})
            except requests.exceptions.RequestException:
                time.sleep(2)
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
                    _emit_log(socketio_instance, log_id, '[SUCCESS] Model installed.', 'success')
                    _persist_train_result(checkpoint, socketio_instance, log_id)
                    _trigger_ood_features(checkpoint, socketio_instance, log_id)
                    return CheckpointModel.STATUS_FINISHED
                except Exception as e:
                    traceback.print_exc()
                    _emit_log(socketio_instance, log_id, f'[ERROR] Model download failed: {e}', 'error')
                    return CheckpointModel.STATUS_FAILED
            if status == 'failed':
                err = job.get('error', 'unknown')
                _emit_log(socketio_instance, log_id, f'[ERROR] Training failed: {err}', 'error')
                return CheckpointModel.STATUS_FAILED
            if status == 'stopped':
                _emit_log(socketio_instance, log_id, 'Training stopped.', 'warning')
                return CheckpointModel.STATUS_CANCELED

            time.sleep(2)

    except Exception as e:
        traceback.print_exc()
        _emit_log(socketio_instance, log_id, f'[ERROR] {e}', 'error')
        return CheckpointModel.STATUS_FAILED
