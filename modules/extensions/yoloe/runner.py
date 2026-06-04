"""YOLOE visual-prompt runner — cross-view one-shot object detection.

Define the target by a box on ONE view (the "refer" image); later, in a DIFFERENT
view where the box no longer aligns, YOLOE's SAVPE encodes the boxed object and
finds it again by learned visual similarity. Loaded lazily so the rest of
EasyTrainer keeps working when ultralytics/YOLOE is not installed.

Public API (re-exported by __init__):
  is_available()  -> bool        # ultralytics importable
  load_model()    -> model|None  # eager load (GPU manager preload)
  unload_model()                 # free VRAM
  is_loaded()     -> bool
  load_error()    -> str|None
  detect_exemplar(target_rgb, refer_rgb, box, conf=0.25) -> bool mask (H,W)
"""
from __future__ import annotations

import os
import threading
from typing import Optional

import numpy as np

_LOCK = threading.Lock()
_MODEL = None
_LOAD_ERROR: Optional[str] = None
_CKPT = os.environ.get('YOLOE_CKPT', 'yoloe-11s-seg.pt')


def is_available() -> bool:
    try:
        import importlib.util as u
        return u.find_spec('ultralytics') is not None
    except Exception:
        return False


def _device():
    try:
        import torch
        return 0 if torch.cuda.is_available() else 'cpu'
    except Exception:
        return 'cpu'


def load_model():
    global _MODEL, _LOAD_ERROR
    if _MODEL is not None:
        return _MODEL
    with _LOCK:
        if _MODEL is not None:
            return _MODEL
        try:
            # Prefer weights cached under the data dir (install.sh prefetches there).
            data_dir = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
            cached = os.path.join(data_dir, 'models', 'yoloe', _CKPT)
            ckpt = cached if os.path.isfile(cached) else _CKPT
            from ultralytics import YOLOE
            _MODEL = YOLOE(ckpt)
            _LOAD_ERROR = None
        except Exception as e:  # pragma: no cover - surfaced to the UI
            _LOAD_ERROR = f'YOLOE load failed: {e}'
        return _MODEL


def unload_model():
    global _MODEL, _LOAD_ERROR
    with _LOCK:
        _MODEL = None
        _LOAD_ERROR = None
    try:
        import gc
        gc.collect()
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:
        pass


def is_loaded() -> bool:
    return _MODEL is not None


def load_error() -> Optional[str]:
    return _LOAD_ERROR


def detect_exemplar(target_rgb: np.ndarray, refer_rgb: np.ndarray, box,
                    conf: float = 0.25) -> np.ndarray:
    """Find the object boxed in `refer_rgb` inside `target_rgb`. Returns the best
    detection as a bool mask (H,W); all-False if nothing matched."""
    H, W = target_rgb.shape[:2]
    model = load_model()
    if model is None:
        raise RuntimeError(_LOAD_ERROR or 'YOLOE not available')
    from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor
    vp = dict(bboxes=np.array([list(box)[:4]], dtype=float), cls=np.array([0]))
    res = model.predict(
        np.ascontiguousarray(target_rgb[:, :, ::-1]),       # RGB -> BGR
        refer_image=np.ascontiguousarray(refer_rgb[:, :, ::-1]),
        visual_prompts=vp, predictor=YOLOEVPSegPredictor,
        conf=conf, verbose=False, device=_device(),
    )
    r0 = res[0]
    if r0.boxes is None or len(r0.boxes) == 0:
        return np.zeros((H, W), bool)
    confs = r0.boxes.conf.cpu().numpy()
    bi = int(confs.argmax())
    masks = getattr(r0, 'masks', None)
    if masks is not None and masks.data is not None and len(masks.data) > bi:
        md = masks.data[bi].cpu().numpy()
        m = md > 0.5
        if m.shape != (H, W):
            import cv2
            m = cv2.resize(m.astype(np.uint8), (W, H), interpolation=cv2.INTER_NEAREST) > 0
        return m
    # no seg mask → fill the detected box
    b = r0.boxes.xyxy.cpu().numpy()[bi]
    m = np.zeros((H, W), bool)
    x1, y1, x2, y2 = [int(round(v)) for v in b]
    m[max(0, y1):max(0, y2), max(0, x1):max(0, x2)] = True
    return m
