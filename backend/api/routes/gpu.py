"""GPU management API — status + large-model load/unload.

Backs the top-bar "GPU 관리" dialog:
  GET  /api/gpu/status                  → GPU(s) util/mem/temp + compute processes
                                          + per-model loaded state.
  POST /api/gpu/models/<id>:load        → eagerly load a big model into GPU.
  POST /api/gpu/models/<id>:unload      → free it from GPU.

Models live in the backend process (runner globals), so load/unload here affect
this Flask process — the same one that serves the wrist-reach detect + sensor
SAM3 overlay. Planner runs are separate subprocesses and still auto-load lazily
(unchanged), so "running a SAM3 process auto-loads it" still holds.
"""
import os
import subprocess

import requests
from flask import Blueprint

gpu_bp = Blueprint('gpu', __name__)


def _easytrainer_training_jobs():
    """로컬 training_server(5100)에서 진행 중인 학습 job 들을 조회.

    Returns list of {'checkpoint_id', 'gpu_mib'} for status in (starting/training).
    GPU 관리 다이얼로그가 nvidia-smi 의 '[Not Found]'(별도 PID namespace 라 컨테
    이너 /proc 에서 해석 불가) 프로세스를 EasyTrainer 학습으로 라벨링하는 데 쓴다.
    """
    port = os.environ.get('TRAINING_SERVER_PORT', '5100')
    try:
        resp = requests.get(f'http://localhost:{port}/api/train/jobs', timeout=3)
        if resp.status_code != 200:
            return []
        jobs = (resp.json() or {}).get('jobs', [])
    except (requests.exceptions.RequestException, ValueError, TypeError):
        return []
    out = []
    for j in jobs:
        if j.get('status') not in ('training', 'starting'):
            continue
        cp = j.get('checkpoint_id')
        if cp is None:
            # job_id 형식 '<mid>_ckpt_<cpid>' 에서 보조 추출.
            jid = j.get('job_id') or ''
            if '_ckpt_' in jid:
                try:
                    cp = int(jid.split('_ckpt_')[-1])
                except (TypeError, ValueError):
                    cp = None
        out.append({'checkpoint_id': cp, 'gpu_mib': j.get('gpu_mib')})
    return out


def _label_easytrainer_processes(processes):
    """name 이 해석 안 된 GPU 프로세스를, 진행 중인 EasyTrainer 학습 job 의 GPU
    메모리(gpu_mib)와 used_memory 로 근접 매칭해 라벨링한다. (호스트/컨테이너 PID
    namespace 가 달라 pid 매칭이 안 되므로 메모리로 매칭.)
    """
    jobs = _easytrainer_training_jobs()
    if not jobs:
        return
    UNRESOLVED = ('unknown', '', '-', '[Not Found]', '[Insufficient Permissions]')
    unresolved = [p for p in processes if p.get('name') in UNRESOLVED]
    if not unresolved:
        return

    # 1) gpu_mib 가 있는 job 부터 메모리 근접 매칭(그리디, job 1회 사용).
    remaining_jobs = list(jobs)
    for p in unresolved:
        mem = p.get('mem_mb') or 0
        best, best_d = None, None
        for j in remaining_jobs:
            g = j.get('gpu_mib')
            if g is None:
                continue
            d = abs(int(g) - int(mem))
            if best is None or d < best_d:
                best, best_d = j, d
        # 허용 오차: 25% 또는 1500MiB 중 큰 값(reserved vs nvidia-smi used 차이 흡수).
        tol = max(1500, int(mem * 0.25))
        if best is not None and best_d is not None and best_d <= tol:
            cp = best.get('checkpoint_id')
            p['name'] = f'EasyTrainer 학습 · cp{cp}' if cp is not None else 'EasyTrainer 학습'
            p['easytrainer'] = True
            p['checkpoint_id'] = cp
            remaining_jobs.remove(best)

    # 2) 아직 라벨 안 된 unresolved 가 있고 남은 학습 job 도 있으면 EasyTrainer
    #    학습으로 라벨. 단 메모리로 확정하지 못한 경우엔 cp 번호를 단정하면 오귀속
    #    위험이 있으므로, 남은 학습이 정확히 1개일 때만 cp 를 붙이고 그 외엔 일반
    #    라벨만 단다. (학습 직후 epoch 로그 전이라 gpu_mib 가 아직 없는 케이스.)
    still = [p for p in unresolved if not p.get('easytrainer')]
    if still and remaining_jobs:
        only_cp = remaining_jobs[0].get('checkpoint_id') if len(remaining_jobs) == 1 else None
        for p in still:
            p['name'] = f'EasyTrainer 학습 · cp{only_cp}' if only_cp is not None else 'EasyTrainer 학습'
            p['easytrainer'] = True
            p['checkpoint_id'] = only_cp

# Registry of large models the UI can manage. Each entry maps to a backend helper
# module exposing is_extension_installed / is_model_loaded / preload_model /
# unload_model. Add a model here and it appears in the GPU manager automatically.
_MODELS = [
    {'id': 'sam3', 'name': 'SAM 3 Segmentation'},
    {'id': 'yoloe', 'name': 'YOLOE Visual-Prompt Detection'},
]


def _helper(model_id):
    if model_id == 'sam3':
        from ...utils import sam3_helper
        return sam3_helper
    if model_id == 'yoloe':
        from ...utils import yoloe_helper
        return yoloe_helper
    return None


def _smi(query, fields):
    """Run nvidia-smi for a --query-<query> and return parsed rows (list of lists)."""
    out = subprocess.check_output(
        ['nvidia-smi', f'--query-{query}=' + ','.join(fields),
         '--format=csv,noheader,nounits'],
        stderr=subprocess.DEVNULL, timeout=6).decode('utf-8', 'replace')
    rows = []
    for line in out.splitlines():
        line = line.strip()
        if line:
            rows.append([c.strip() for c in line.split(',')])
    return rows


def _proc_name(pid):
    """Best-effort process name for a (host) pid via /proc; '' if unavailable."""
    for path in (f'/proc/{pid}/comm', f'/proc/{pid}/cmdline'):
        try:
            with open(path, 'rb') as f:
                raw = f.read().replace(b'\x00', b' ').strip()
                if raw:
                    return os.path.basename(raw.split(b' ')[0].decode('utf-8', 'replace')) \
                        if path.endswith('cmdline') else raw.decode('utf-8', 'replace')
        except Exception:
            continue
    return ''


def _int(v, default=0):
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return default


def _model_state(model_id):
    h = _helper(model_id)
    if h is None:
        return {'installed': False, 'loaded': False}
    try:
        return {'installed': bool(h.is_extension_installed()), 'loaded': bool(h.is_model_loaded())}
    except Exception:
        return {'installed': False, 'loaded': False}


@gpu_bp.route('/gpu/status', methods=['GET'])
def gpu_status():
    gpus, processes, available = [], [], True
    try:
        for p in _smi('gpu', ['index', 'name', 'memory.used', 'memory.total',
                              'utilization.gpu', 'temperature.gpu']):
            if len(p) < 6:
                continue
            gpus.append({
                'index': _int(p[0]), 'name': p[1],
                'mem_used_mb': _int(p[2]), 'mem_total_mb': _int(p[3]),
                'util_pct': _int(p[4]), 'temp_c': _int(p[5]),
            })
        for p in _smi('compute-apps', ['pid', 'process_name', 'used_memory']):
            if len(p) < 3:
                continue
            pid = _int(p[0])
            name = p[1]
            if name in ('', '-', '[Not Found]', '[Insufficient Permissions]'):
                name = _proc_name(pid) or 'unknown'
            processes.append({'pid': pid, 'name': name, 'mem_mb': _int(p[2]),
                              'easytrainer': False, 'checkpoint_id': None})
        # 해석 안 된 프로세스를 진행 중인 EasyTrainer 학습 job 과 매칭해 라벨링.
        _label_easytrainer_processes(processes)
    except Exception:
        available = False

    models = []
    for m in _MODELS:
        st = _model_state(m['id'])
        models.append({'id': m['id'], 'name': m['name'], **st})

    return {
        'status': 'success',
        'available': available,
        'gpus': gpus,
        'processes': processes,
        'models': models,
    }, 200


@gpu_bp.route('/gpu/models/<model_id>:load', methods=['POST'])
def load_model(model_id):
    h = _helper(model_id)
    if h is None:
        return {'status': 'error', 'message': f'unknown model: {model_id}'}, 404
    if not h.is_extension_installed():
        return {'status': 'error', 'message': f'{model_id} extension is not installed'}, 409
    res = h.preload_model()
    if not res.get('loaded'):
        return {'status': 'error', 'message': res.get('error') or 'load failed',
                'loaded': False}, 500
    return {'status': 'success', 'loaded': True}, 200


@gpu_bp.route('/gpu/models/<model_id>:unload', methods=['POST'])
def unload_model(model_id):
    h = _helper(model_id)
    if h is None:
        return {'status': 'error', 'message': f'unknown model: {model_id}'}, 404
    res = h.unload_model()
    return {'status': 'success', 'loaded': res.get('loaded', False)}, 200
