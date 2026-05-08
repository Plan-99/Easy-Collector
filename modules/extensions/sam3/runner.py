"""SAM 3 tracker runner.

Loaded lazily by backend/utils/sam3_helper.py only when a sensor's task config
contains active SAM3 prompts. Designed so the rest of the pipeline keeps
working even if `sam3` isn't installed.

Two operating modes:
  - Sam3Tracker(prompts, first_image): video predictor — initializes on the
    first frame of an episode (text + box prompts), then `step(frame)` returns
    the propagated mask. Used during recording and inference.
  - detect_one(image, prompts): single-image detector — runs the full prompt
    pipeline on one frame. Used for the WorkspacePage "Test mask" preview.

`mask_apply(image, mask, mode, color)` is a pure-numpy helper that takes a
boolean mask + the user-chosen visualization mode and returns the modified
image. Kept in this file so callers never have to import `sam3` directly.
"""
from __future__ import annotations

import os
import threading
from typing import Any, Optional

import numpy as np

# ---------------------------------------------------------------------------
# Lazy SAM3 model loader
# ---------------------------------------------------------------------------

_MODEL_LOCK = threading.Lock()
_IMAGE_PROCESSOR: Optional[Any] = None
_VIDEO_PREDICTOR: Optional[Any] = None
_LOAD_ERROR: Optional[str] = None


def _hf_token() -> Optional[str]:
    token = os.environ.get('HF_TOKEN')
    if token:
        return token
    data_dir = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
    token_file = os.path.join(data_dir, '.hf_token')
    if os.path.isfile(token_file):
        try:
            with open(token_file, 'r', encoding='utf-8') as f:
                return f.read().strip() or None
        except OSError:
            return None
    return None


def is_available() -> bool:
    """True if `sam3` package is importable (independent of weight download)."""
    try:
        import importlib.util
        return importlib.util.find_spec('sam3') is not None
    except Exception:
        return False


def _ensure_image_model():
    global _IMAGE_PROCESSOR, _LOAD_ERROR
    if _IMAGE_PROCESSOR is not None:
        return _IMAGE_PROCESSOR
    with _MODEL_LOCK:
        if _IMAGE_PROCESSOR is not None:
            return _IMAGE_PROCESSOR
        token = _hf_token()
        if token:
            os.environ.setdefault('HF_TOKEN', token)
            os.environ.setdefault('HUGGING_FACE_HUB_TOKEN', token)
        cache_root = os.path.join(
            os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer'),
            'models', 'sam3',
        )
        os.makedirs(cache_root, exist_ok=True)
        os.environ.setdefault('HF_HOME', cache_root)
        try:
            from sam3.model_builder import build_sam3_image_model
            from sam3.model.sam3_image_processor import Sam3Processor
            model = build_sam3_image_model()
            _IMAGE_PROCESSOR = Sam3Processor(model)
        except Exception as e:
            _LOAD_ERROR = f'SAM3 image model load failed: {e}'
            raise
        return _IMAGE_PROCESSOR


def _ensure_video_predictor():
    global _VIDEO_PREDICTOR, _LOAD_ERROR
    if _VIDEO_PREDICTOR is not None:
        return _VIDEO_PREDICTOR
    with _MODEL_LOCK:
        if _VIDEO_PREDICTOR is not None:
            return _VIDEO_PREDICTOR
        token = _hf_token()
        if token:
            os.environ.setdefault('HF_TOKEN', token)
            os.environ.setdefault('HUGGING_FACE_HUB_TOKEN', token)
        cache_root = os.path.join(
            os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer'),
            'models', 'sam3',
        )
        os.makedirs(cache_root, exist_ok=True)
        os.environ.setdefault('HF_HOME', cache_root)
        try:
            from sam3.model_builder import build_sam3_video_predictor
            _VIDEO_PREDICTOR = build_sam3_video_predictor()
        except Exception as e:
            _LOAD_ERROR = f'SAM3 video predictor load failed: {e}'
            raise
        return _VIDEO_PREDICTOR


def get_load_error() -> Optional[str]:
    return _LOAD_ERROR


# ---------------------------------------------------------------------------
# Single-image detection (used for preview)
# ---------------------------------------------------------------------------

def _to_pil(image: np.ndarray):
    from PIL import Image
    arr = image
    if arr.ndim == 3 and arr.shape[2] == 3:
        # OpenCV gives BGR — SAM3 expects RGB
        arr = arr[:, :, ::-1]
    return Image.fromarray(np.ascontiguousarray(arr))


def _normalize_prompts(prompts: dict) -> dict:
    """Validate / fill missing prompt slots."""
    out = {
        'text': [t for t in (prompts.get('text') or []) if isinstance(t, str) and t.strip()],
        'boxes': [b for b in (prompts.get('boxes') or []) if _is_box(b)],
    }
    return out


def _is_box(b) -> bool:
    return (
        isinstance(b, (list, tuple))
        and len(b) == 4
        and all(isinstance(v, (int, float)) for v in b)
    )


def detect_one(image: np.ndarray, prompts: dict) -> np.ndarray:
    """Single-image detect → boolean mask (H, W). Returns all-False on no match."""
    p = _normalize_prompts(prompts)
    if not p['text'] and not p['boxes']:
        return np.zeros(image.shape[:2], dtype=bool)
    proc = _ensure_image_model()
    pil = _to_pil(image)
    state = proc.set_image(pil)
    masks = []
    for text in p['text']:
        out = proc.set_text_prompt(state=state, prompt=text)
        m = out.get('masks')
        if m is not None:
            masks.append(_extract_mask_array(m, image.shape[:2]))
    for box in p['boxes']:
        out = proc.set_box_prompt(state=state, box=list(box))
        m = out.get('masks')
        if m is not None:
            masks.append(_extract_mask_array(m, image.shape[:2]))
    if not masks:
        return np.zeros(image.shape[:2], dtype=bool)
    union = np.zeros(image.shape[:2], dtype=bool)
    for m in masks:
        union |= m
    return union


def _extract_mask_array(masks_obj, target_shape) -> np.ndarray:
    """Coerce SAM3 mask output (torch.Tensor or np.ndarray, possibly with batch
    dims) into a single (H, W) boolean array matching `target_shape`."""
    arr = masks_obj
    try:
        import torch
        if isinstance(arr, torch.Tensor):
            arr = arr.detach().cpu().numpy()
    except ImportError:
        pass
    arr = np.asarray(arr)
    while arr.ndim > 2:
        # union over batch / instance dims
        arr = arr.any(axis=0) if arr.dtype == bool else arr.max(axis=0)
    if arr.shape != target_shape:
        # fallback: nearest-neighbor resize via cv2
        import cv2
        arr = cv2.resize(arr.astype(np.uint8), (target_shape[1], target_shape[0]),
                         interpolation=cv2.INTER_NEAREST)
    return arr.astype(bool)


# ---------------------------------------------------------------------------
# Video tracker (first-frame detect + propagate)
# ---------------------------------------------------------------------------

class Sam3Tracker:
    """Per-sensor tracker. Initialize once with the first frame + prompts,
    then call `step(frame)` for every subsequent frame in the episode."""

    def __init__(self, prompts: dict, first_image: np.ndarray):
        self._prompts = _normalize_prompts(prompts)
        self._predictor = _ensure_video_predictor()
        self._state = self._predictor.init_state(_to_pil(first_image))
        self._frame_shape = first_image.shape[:2]
        # Seed prompts on the first frame
        for text in self._prompts['text']:
            self._predictor.add_text_prompt(self._state, text)
        for box in self._prompts['boxes']:
            self._predictor.add_box_prompt(self._state, list(box))
        # First-frame mask (from detection step)
        self._last_mask = self._predict_current()

    def _predict_current(self) -> np.ndarray:
        try:
            out = self._predictor.track(self._state)
        except Exception:
            try:
                out = self._predictor.predict(self._state)
            except Exception:
                return np.zeros(self._frame_shape, dtype=bool)
        if isinstance(out, dict):
            m = out.get('masks')
        else:
            m = out
        if m is None:
            return np.zeros(self._frame_shape, dtype=bool)
        return _extract_mask_array(m, self._frame_shape)

    def step(self, frame: np.ndarray) -> np.ndarray:
        try:
            self._predictor.add_frame(self._state, _to_pil(frame))
        except Exception:
            try:
                self._predictor.append_frame(self._state, _to_pil(frame))
            except Exception:
                # Predictor doesn't support per-frame append → fallback to
                # per-frame one-shot detection (slower but always works).
                return detect_one(frame, {'text': self._prompts['text'],
                                          'boxes': self._prompts['boxes']})
        self._last_mask = self._predict_current()
        return self._last_mask

    @property
    def last_mask(self) -> np.ndarray:
        return self._last_mask

    def close(self):
        self._state = None
        self._predictor = None


# ---------------------------------------------------------------------------
# Mask visualization
# ---------------------------------------------------------------------------

def mask_apply(
    image: np.ndarray,
    mask: np.ndarray,
    mode: str = 'background',
    color: Optional[list] = None,
) -> np.ndarray:
    """Apply a boolean mask to an image.

    mode:
      - 'off'        → return image unchanged
      - 'background' → pixels OUTSIDE mask are replaced with `color`
      - 'object'     → pixels INSIDE mask are replaced with `color`

    color is an [R, G, B] triplet (0–255). When None, defaults to black.
    The image is assumed BGR (OpenCV convention).
    """
    if mode == 'off' or mask is None:
        return image
    if image is None:
        return image
    if mask.shape != image.shape[:2]:
        import cv2
        mask = cv2.resize(mask.astype(np.uint8), (image.shape[1], image.shape[0]),
                          interpolation=cv2.INTER_NEAREST).astype(bool)
    rgb = color if (color and len(color) == 3) else [0, 0, 0]
    bgr = np.array([rgb[2], rgb[1], rgb[0]], dtype=image.dtype)
    out = image.copy()
    if mode == 'background':
        out[~mask] = bgr
    elif mode == 'object':
        out[mask] = bgr
    return out
