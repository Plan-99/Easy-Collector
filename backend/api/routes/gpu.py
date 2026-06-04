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

from flask import Blueprint

gpu_bp = Blueprint('gpu', __name__)

# Registry of large models the UI can manage. Each entry maps to helpers that
# report installed/loaded and perform load/unload. Currently SAM3 only.
_MODELS = [
    {'id': 'sam3', 'name': 'SAM 3 Segmentation'},
]


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


def _sam3_helper():
    from ...utils import sam3_helper
    return sam3_helper


def _model_state(model_id):
    h = _sam3_helper()
    if model_id == 'sam3':
        return {'installed': h.is_extension_installed(), 'loaded': h.is_model_loaded()}
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
            processes.append({'pid': pid, 'name': name, 'mem_mb': _int(p[2])})
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
    h = _sam3_helper()
    if model_id != 'sam3':
        return {'status': 'error', 'message': f'unknown model: {model_id}'}, 404
    if not h.is_extension_installed():
        return {'status': 'error', 'message': 'SAM3 extension is not installed'}, 409
    res = h.preload_model()
    if not res.get('loaded'):
        return {'status': 'error', 'message': res.get('error') or 'load failed',
                'loaded': False}, 500
    return {'status': 'success', 'loaded': True}, 200


@gpu_bp.route('/gpu/models/<model_id>:unload', methods=['POST'])
def unload_model(model_id):
    h = _sam3_helper()
    if model_id != 'sam3':
        return {'status': 'error', 'message': f'unknown model: {model_id}'}, 404
    res = h.unload_model()
    return {'status': 'success', 'loaded': res.get('loaded', False)}, 200
