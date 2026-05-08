"""Runtime copy of backend/utils/sam3_helper.py for exported checkpoints.

Imports the SAM3 extension at first use. The deployment host must have the
SAM3 module installed (e.g. via the EasyTrainer module installer) — if it's
not present, every call here becomes a no-op and the inference pipeline runs
without masking. No exception is raised so a model trained without SAM3 keeps
working unchanged.
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
    for mod_path in (
        'backend.extensions.sam3',
        'extensions.sam3',
        'easytrainer_runtime.sam3',
    ):
        try:
            _RUNNER = importlib.import_module(mod_path)
            return _RUNNER
        except ImportError:
            continue
    return None


def is_extension_installed() -> bool:
    return _load_runner() is not None


def _is_active(cfg: Optional[dict]) -> bool:
    if not cfg or not isinstance(cfg, dict):
        return False
    if not cfg.get('enabled'):
        return False
    if cfg.get('mode', 'off') == 'off':
        return False
    if not cfg.get('text_prompts') and not cfg.get('boxes'):
        return False
    return True


def start_episode(sensor_id, first_image: np.ndarray, cfg: Optional[dict]) -> bool:
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


def apply_sam3_to_image(image: np.ndarray, sensor_id, cfg: Optional[dict]) -> np.ndarray:
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
