"""Glue between core pipeline and the (optional) SAM3 extension.

The SAM3 extension installs to `backend/extensions/sam3/`. This helper:
  - Imports the runner lazily so the rest of EasyTrainer keeps working when
    SAM3 is not installed.
  - Manages per-(sensor_id) tracker instances for episode-scoped use.
  - Exposes `apply_sam3_to_image()` which fetch_image_with_config hooks into.

A "SAM3 config" dict (the value stored in task.settings.sensors[id].sam3) has
shape:
    {
        'enabled': bool,
        'text_prompts': [str, ...],
        'boxes': [[x1, y1, x2, y2], ...],
        'mode': 'off' | 'background' | 'object',
        'color': [r, g, b],
    }
"""
from __future__ import annotations

import importlib
import threading
from typing import Any, Dict, Optional

import numpy as np


_RUNNER = None
_RUNNER_IMPORT_TRIED = False
_TRACKERS: Dict[str, Any] = {}
_LOCK = threading.Lock()


def _load_runner():
    global _RUNNER, _RUNNER_IMPORT_TRIED
    if _RUNNER is not None or _RUNNER_IMPORT_TRIED:
        return _RUNNER
    _RUNNER_IMPORT_TRIED = True
    for mod_path in ('backend.extensions.sam3', 'extensions.sam3'):
        try:
            _RUNNER = importlib.import_module(mod_path)
            return _RUNNER
        except ImportError:
            continue
    return None


def is_extension_installed() -> bool:
    return _load_runner() is not None


# ---------------------------------------------------------------------------
# Explicit GPU load / unload (for the GPU manager UI). The model lives in this
# process's runner globals (_IMAGE_PROCESSOR / _VIDEO_PREDICTOR); these helpers
# preload it so the first detection is instant, or free it to reclaim VRAM.
# Detection paths still lazily auto-load via runner._ensure_image_model(), so a
# process that uses SAM3 keeps working even if nothing pre-loaded it.
# ---------------------------------------------------------------------------
def _model_module():
    """The submodule that actually holds the model globals/loaders. The package
    __init__ only re-exports a few names; `_ensure_image_model`, `_IMAGE_PROCESSOR`
    etc. live in `backend.extensions.sam3.runner`."""
    pkg = _load_runner()
    if pkg is None:
        return None
    rm = getattr(pkg, 'runner', None)
    if rm is not None:
        return rm
    try:
        return importlib.import_module(pkg.__name__ + '.runner')
    except Exception:
        return pkg  # last resort: maybe the globals are on the package itself


def is_model_loaded() -> bool:
    """True if the SAM3 model is resident in this process's GPU memory."""
    r = _model_module()
    if r is None:
        return False
    return getattr(r, '_IMAGE_PROCESSOR', None) is not None \
        or getattr(r, '_VIDEO_PREDICTOR', None) is not None


def model_load_error() -> Optional[str]:
    r = _model_module()
    return getattr(r, '_LOAD_ERROR', None) if r is not None else None


def preload_model() -> Dict[str, Any]:
    """Eagerly build + load the SAM3 image model into GPU. Blocks until ready.
    Returns {'loaded': bool, 'error': str|None}."""
    r = _model_module()
    if r is None:
        return {'loaded': False, 'error': 'SAM3 extension not installed'}
    ensure = getattr(r, '_ensure_image_model', None)
    if ensure is None:
        return {'loaded': False, 'error': 'SAM3 runner missing _ensure_image_model'}
    try:
        ensure()
    except Exception as e:  # pragma: no cover - surfaced to the UI
        return {'loaded': False, 'error': str(e)}
    if is_model_loaded():
        return {'loaded': True, 'error': None}
    return {'loaded': False, 'error': model_load_error() or 'model load failed'}


def unload_model() -> Dict[str, Any]:
    """Free the SAM3 model (image + video) from GPU memory and drop trackers."""
    import contextlib
    import gc
    r = _model_module()
    if r is None:
        return {'loaded': False, 'error': None}
    lock = getattr(r, '_MODEL_LOCK', None)
    with (lock if lock is not None else contextlib.nullcontext()):
        r._IMAGE_PROCESSOR = None
        r._VIDEO_PREDICTOR = None
        r._LOAD_ERROR = None
    with _LOCK:
        _TRACKERS.clear()
    gc.collect()
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.ipc_collect()
    except Exception:
        pass
    return {'loaded': is_model_loaded(), 'error': None}


def _is_active(cfg: Optional[dict]) -> bool:
    if not cfg or not isinstance(cfg, dict):
        return False
    if not cfg.get('enabled'):
        return False
    mode = cfg.get('mode', 'off')
    if mode == 'off':
        return False
    if not cfg.get('text_prompts') and not cfg.get('boxes'):
        return False
    return True


def start_episode(sensor_id, first_image: np.ndarray, cfg: Optional[dict]) -> bool:
    """Initialize a tracker for a sensor at the start of an episode.

    Idempotent — calling again resets the tracker. Returns True on success,
    False if disabled / unavailable / failed (caller should fall back to raw
    image)."""
    if not _is_active(cfg):
        return False
    runner = _load_runner()
    if runner is None or not runner.is_available():
        return False
    key = str(sensor_id)
    with _LOCK:
        old = _TRACKERS.pop(key, None)
        if old is not None:
            try:
                old.close()
            except Exception:
                pass
        try:
            tracker = runner.Sam3Tracker(
                prompts={
                    'text': cfg.get('text_prompts') or [],
                    'boxes': cfg.get('boxes') or [],
                },
                first_image=first_image,
            )
        except Exception as e:
            print(f'[sam3] tracker init failed for sensor {sensor_id}: {e}')
            return False
        _TRACKERS[key] = tracker
    return True


def end_episode(sensor_id=None):
    """Tear down trackers. With no arg, tears down everything (e.g. on shutdown)."""
    with _LOCK:
        if sensor_id is None:
            for t in _TRACKERS.values():
                try:
                    t.close()
                except Exception:
                    pass
            _TRACKERS.clear()
        else:
            t = _TRACKERS.pop(str(sensor_id), None)
            if t is not None:
                try:
                    t.close()
                except Exception:
                    pass


def apply_sam3_to_image(
    image: np.ndarray,
    sensor_id,
    cfg: Optional[dict],
) -> np.ndarray:
    """Apply SAM3 mask to a single frame using an already-initialized tracker.

    If no tracker was started for this sensor (e.g. SAM3 disabled, or first
    call), falls back to one-shot detection — slower, but means the function
    is safe to call from contexts that don't manage episode lifecycle (e.g.
    failure_detection, replay)."""
    if image is None or not _is_active(cfg):
        return image
    runner = _load_runner()
    if runner is None or not runner.is_available():
        return image

    key = str(sensor_id)
    tracker = _TRACKERS.get(key)
    try:
        if tracker is not None:
            mask = tracker.step(image)
        else:
            mask = runner.detect_one(
                image,
                {
                    'text': cfg.get('text_prompts') or [],
                    'boxes': cfg.get('boxes') or [],
                },
            )
    except Exception as e:
        print(f'[sam3] mask compute failed for sensor {sensor_id}: {e}')
        return image

    return runner.mask_apply(
        image,
        mask,
        mode=cfg.get('mode', 'background'),
        color=cfg.get('color'),
    )


def preview_mask(image: np.ndarray, cfg: Optional[dict]) -> Optional[np.ndarray]:
    """Single-shot mask + apply, used by the WorkspacePage Test button.

    Returns the masked image, or None when SAM3 is unavailable / disabled."""
    if image is None or not _is_active(cfg):
        return None
    runner = _load_runner()
    if runner is None or not runner.is_available():
        return None
    try:
        mask = runner.detect_one(
            image,
            {
                'text': cfg.get('text_prompts') or [],
                'boxes': cfg.get('boxes') or [],
            },
        )
    except Exception as e:
        print(f'[sam3] preview detect failed: {e}')
        return None
    return runner.mask_apply(
        image,
        mask,
        mode=cfg.get('mode', 'background'),
        color=cfg.get('color'),
    )
