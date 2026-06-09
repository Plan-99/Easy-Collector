"""Glue between the core pipeline and the (optional) YOLOE extension.

Mirrors sam3_helper: lazily imports backend.extensions.yoloe so EasyTrainer keeps
working when YOLOE is not installed, and exposes load/unload/status for the GPU
manager plus the cross-view detect used by Wrist View Reach.
"""
from __future__ import annotations

import importlib
import threading
from typing import Any, Dict, Optional

import numpy as np

_RUNNER = None
_RUNNER_IMPORT_TRIED = False
_LOCK = threading.Lock()


def _load_runner():
    global _RUNNER, _RUNNER_IMPORT_TRIED
    if _RUNNER is not None or _RUNNER_IMPORT_TRIED:
        return _RUNNER
    _RUNNER_IMPORT_TRIED = True
    for mod_path in ('backend.extensions.yoloe', 'extensions.yoloe'):
        try:
            _RUNNER = importlib.import_module(mod_path)
            return _RUNNER
        except ImportError:
            continue
    return None


def is_extension_installed() -> bool:
    """Installed AND its python dep (ultralytics) importable — i.e. actually usable."""
    r = _load_runner()
    if r is None:
        return False
    try:
        return bool(r.is_available())
    except Exception:
        return False


def is_model_loaded() -> bool:
    r = _load_runner()
    if r is None:
        return False
    try:
        return bool(r.is_loaded())
    except Exception:
        return False


def model_load_error() -> Optional[str]:
    r = _load_runner()
    try:
        return r.load_error() if r is not None else None
    except Exception:
        return None


def preload_model() -> Dict[str, Any]:
    r = _load_runner()
    if r is None or not is_extension_installed():
        return {'loaded': False, 'error': 'YOLOE extension not installed'}
    try:
        r.load_model()
    except Exception as e:  # pragma: no cover
        return {'loaded': False, 'error': str(e)}
    if is_model_loaded():
        return {'loaded': True, 'error': None}
    return {'loaded': False, 'error': model_load_error() or 'YOLOE load failed'}


def unload_model() -> Dict[str, Any]:
    r = _load_runner()
    if r is None:
        return {'loaded': False, 'error': None}
    try:
        r.unload_model()
    except Exception:
        pass
    return {'loaded': is_model_loaded(), 'error': None}


def detect_exemplar(target_rgb: np.ndarray, refer_rgb: np.ndarray, box,
                    conf: float = 0.25) -> np.ndarray:
    """Cross-view detect: find the object boxed in refer_rgb inside target_rgb.
    Raises if YOLOE is not installed (caller decides how to surface it)."""
    r = _load_runner()
    if r is None or not is_extension_installed():
        raise RuntimeError('YOLOE extension is not installed')
    return r.detect_exemplar(target_rgb, refer_rgb, box, conf=conf)


def detect_exemplar_multi(target_rgb: np.ndarray, refers, conf: float = 0.25) -> np.ndarray:
    """Multi-reference cross-view detect: average per-reference VPEs (여러 각도
    레퍼런스) for view-robust detection. ``refers`` = [(refer_rgb, box), ...].
    Raises if YOLOE not installed. Older extension w/o multi → single-exemplar fallback."""
    r = _load_runner()
    if r is None or not is_extension_installed():
        raise RuntimeError('YOLOE extension is not installed')
    refers = [(rr, bb) for (rr, bb) in (refers or []) if rr is not None and bb is not None]
    if hasattr(r, 'detect_exemplar_multi'):
        return r.detect_exemplar_multi(target_rgb, refers, conf=conf)
    if not refers:
        return np.zeros(target_rgb.shape[:2], bool)
    return r.detect_exemplar(target_rgb, refers[0][0], refers[0][1], conf=conf)


def detect_text(target_rgb: np.ndarray, texts, conf: float = 0.15) -> np.ndarray:
    """Open-vocabulary text-prompt detect via YOLOE. Raises if not installed."""
    r = _load_runner()
    if r is None or not is_extension_installed():
        raise RuntimeError('YOLOE extension is not installed')
    return r.detect_text(target_rgb, texts, conf=conf)
