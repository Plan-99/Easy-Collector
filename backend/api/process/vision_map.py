"""ACT vision saliency map computation.

Given a checkpoint + a single dataset frame, produces a per-camera heatmap
showing which image regions the policy attends to (attention mode) or which
pixels most influence the predicted action (gradcam mode). Heatmaps are
rendered as RGBA PNGs (transparent background, jet colormap with alpha =
intensity) so the frontend can overlay them on top of the camera image.

Two methods:
  - ``attention``  decoder cross-attention (action queries → encoder visual
                   tokens) from the last ACTDecoderLayer.multihead_attn.
                   Captured via forward_hook on the module; PyTorch's default
                   MultiheadAttention returns weights averaged over heads.
  - ``gradcam``    Grad-CAM on the backbone's last conv feature map. The
                   target scalar is the L2 norm of the predicted action chunk
                   (a generic "where does the action come from" signal). Per
                   camera, because the backbone runs once per camera image.

The loaded policy is cached in-process (single slot) so consecutive frame
requests on the same checkpoint don't pay the from_pretrained cost.
"""
from __future__ import annotations

import base64
import io
import os
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import pyarrow.parquet as pq
import torch

from ...configs.global_configs import DATASET_DIR, resolve_checkpoint_dir
from ...utils.lerobot_io import (
    INFO_PATH, PARQUET_PATH_TEMPLATE, _read_json,
)


# ---------------------------------------------------------------------------
# Policy cache (single-slot LRU). Loading ACTPolicy.from_pretrained is the
# expensive part — keep the most recently used one resident so the user can
# scrub frames smoothly.
# ---------------------------------------------------------------------------
_CACHE: Dict[str, Any] = {'ckpt_id': None, 'policy': None, 'preprocessor': None,
                          'postprocessor': None, 'image_resolution': None, 'device': None}


def _device() -> torch.device:
    return torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def _load_policy(checkpoint: dict, policy_obj: dict):
    """Return (policy, preprocessor, postprocessor, image_resolution) — cached per checkpoint.

    postprocessor 는 succeed 토큰을 추론과 동일하게 unnormalize 해 success score 를
    내기 위해 같이 반환한다(없으면 None).
    """
    ckpt_id = str(checkpoint['id'])
    if _CACHE['ckpt_id'] == ckpt_id and _CACHE['policy'] is not None:
        return (_CACHE['policy'], _CACHE['preprocessor'],
                _CACHE.get('postprocessor'), _CACHE['image_resolution'])

    ckpt_dir = resolve_checkpoint_dir(checkpoint['id'])
    if not os.path.isdir(ckpt_dir):
        raise FileNotFoundError(f'Checkpoint dir not found: {ckpt_dir}')

    # image_resolution sidecar (written by train_worker). Falls back to (224, 224).
    image_resolution = (224, 224)
    meta_path = os.path.join(ckpt_dir, 'train_meta.json')
    if os.path.exists(meta_path):
        try:
            import json
            with open(meta_path, 'r') as f:
                ir = (json.load(f) or {}).get('image_resolution')
            if isinstance(ir, (list, tuple)) and len(ir) == 2:
                image_resolution = (int(ir[0]), int(ir[1]))
        except Exception:
            pass

    if policy_obj.get('type') != 'ACT':
        raise ValueError(f"Vision map only supports ACT policies for now "
                         f"(got {policy_obj.get('type')})")

    from lerobot.policies.act.modeling_act import ACTPolicy
    from ...policies.utils import make_easytrainer_processors

    device = _device()
    policy = ACTPolicy.from_pretrained(ckpt_dir).to(device)
    policy.eval()
    preprocessor, postprocessor = make_easytrainer_processors(
        policy_type='ACT', cfg=policy.config, pretrained_path=ckpt_dir,
    )

    _CACHE.update({
        'ckpt_id': ckpt_id, 'policy': policy, 'preprocessor': preprocessor,
        'postprocessor': postprocessor,
        'image_resolution': image_resolution, 'device': device,
    })
    return policy, preprocessor, postprocessor, image_resolution


# ---------------------------------------------------------------------------
# Frame loading — read one frame (all cameras + state) from a LeRobot dataset.
# ---------------------------------------------------------------------------
def _read_single_frame(dataset_id: str, episode_idx: int, frame_idx: int
                       ) -> Tuple[Dict[str, np.ndarray], Optional[np.ndarray], Dict[str, Any]]:
    """Returns (camera_frames_bgr, state_vector_or_None, info)."""
    folder = os.path.join(DATASET_DIR, dataset_id)
    if not os.path.isdir(folder):
        raise FileNotFoundError(f'Dataset not found: {folder}')

    info = _read_json(os.path.join(folder, INFO_PATH))
    chunk = episode_idx // info['chunks_size']

    # Cameras: decode the requested frame from each sensor's mp4.
    images: Dict[str, np.ndarray] = {}
    for key, feat in info.get('features', {}).items():
        if not key.startswith('observation.images.'):
            continue
        if feat.get('dtype') != 'video':
            continue
        sensor_name = key.replace('observation.images.', '')
        video_path = os.path.join(
            folder, 'videos', f'chunk-{chunk:03d}', key,
            f'episode_{episode_idx:06d}.mp4',
        )
        if not os.path.exists(video_path):
            continue
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, int(frame_idx)))
        ok, frame = cap.read()
        cap.release()
        if ok and frame is not None:
            images[sensor_name] = frame  # BGR HxWx3 uint8

    # State (qpos) at this frame. Used to populate observation.state when the
    # policy was trained with robot_state_feature; without it the attention
    # map would be misleading.
    state = None
    parquet_path = os.path.join(folder,
                                PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_idx))
    if os.path.exists(parquet_path):
        df = pq.read_table(parquet_path).to_pandas()
        qpos_col = 'observation.qpos' if 'observation.qpos' in df.columns else \
                   ('observation.state' if 'observation.state' in df.columns else None)
        if qpos_col is not None and 0 <= frame_idx < len(df):
            state = np.array(df[qpos_col].iloc[int(frame_idx)], dtype=np.float32)

    return images, state, info


# ---------------------------------------------------------------------------
# Batch construction matching the policy's input schema.
# ---------------------------------------------------------------------------
def _build_batch(policy, frames_bgr: Dict[str, np.ndarray],
                 state_np: Optional[np.ndarray],
                 image_resolution: Tuple[int, int],
                 device: torch.device,
                 ) -> Tuple[dict, List[str]]:
    """Build a single-batch dict in the order the policy expects.

    Returns (batch, ordered_sensor_names) where ordered_sensor_names matches
    the iteration order of policy.config.image_features so per-camera outputs
    can be paired back to sensor names.
    """
    cfg = policy.config
    h, w = image_resolution
    ordered: List[str] = []
    batch: Dict[str, torch.Tensor] = {}

    for image_key in cfg.image_features:  # dict order — matches modeling_act.py
        sensor_name = image_key.replace('observation.images.', '')
        ordered.append(sensor_name)
        if sensor_name not in frames_bgr:
            raise ValueError(
                f"Sensor '{sensor_name}' required by checkpoint but not in dataset frame"
            )
        frame = frames_bgr[sensor_name]
        # BGR -> RGB, resize to training resolution, to CHW float32 [0, 1].
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
        tensor = torch.from_numpy(resized).permute(2, 0, 1).float() / 255.0
        batch[image_key] = tensor.unsqueeze(0).to(device)

    if cfg.robot_state_feature is not None:
        state_dim = cfg.robot_state_feature.shape[-1]
        if state_np is not None and state_np.size >= state_dim:
            state = torch.from_numpy(state_np[:state_dim].astype(np.float32))
        else:
            state = torch.zeros(state_dim, dtype=torch.float32)
        batch['observation.state'] = state.unsqueeze(0).to(device)

    if cfg.env_state_feature is not None:
        env_dim = cfg.env_state_feature.shape[-1]
        batch['observation.environment_state'] = torch.zeros(1, env_dim, device=device)

    # task description placeholder — ACT doesn't use it but keep schema-compat.
    batch['task'] = ['']
    return batch, ordered


# ---------------------------------------------------------------------------
# Attention extraction — hook last decoder cross-attention.
# ---------------------------------------------------------------------------
def _compute_attention_maps(policy, batch: dict, preprocessor
                            ) -> Tuple[torch.Tensor, List[Tuple[int, int]]]:
    """Run policy forward with hooks; return (attn_weights, per_cam_HW).

    attn_weights: (num_queries, num_kv_tokens) — averaged over heads, batch=1.
    per_cam_HW: feature map (H, W) for each camera in order.
    """
    cam_shapes: List[Tuple[int, int]] = []
    attn_holder: Dict[str, torch.Tensor] = {}

    def _backbone_hook(_module, _inp, out):
        # out is dict-like with 'feature_map' key; shape (B, C, H, W).
        fmap = out['feature_map'] if isinstance(out, dict) else out
        cam_shapes.append((int(fmap.shape[-2]), int(fmap.shape[-1])))

    def _attn_hook(_module, _inp, out):
        # nn.MultiheadAttention returns (attn_output, attn_weights). Weights are
        # averaged over heads by default → shape (B, L, S).
        if isinstance(out, tuple) and len(out) > 1 and out[1] is not None:
            attn_holder['w'] = out[1].detach()

    h1 = policy.model.backbone.register_forward_hook(_backbone_hook)
    h2 = policy.model.decoder.layers[-1].multihead_attn.register_forward_hook(_attn_hook)
    try:
        b = preprocessor(batch) if preprocessor is not None else batch
        with torch.no_grad():
            policy.predict_action_chunk(b)
    finally:
        h1.remove()
        h2.remove()

    if 'w' not in attn_holder:
        raise RuntimeError('Decoder cross-attention weights were not captured')
    return attn_holder['w'][0], cam_shapes  # (L, S)


def _split_attention_per_camera(attn: torch.Tensor, cam_shapes: List[Tuple[int, int]],
                                policy) -> List[np.ndarray]:
    """attn: (num_queries, num_kv_tokens). Slice off prefix tokens (latent +
    robot_state + env_state) then split the visual block per camera into
    (H, W) heatmaps. Aggregates across action queries by mean — gives a single
    stable heatmap per camera.
    """
    cfg = policy.config
    n_prefix = 1  # latent token always present
    if cfg.robot_state_feature is not None:
        n_prefix += 1
    if cfg.env_state_feature is not None:
        n_prefix += 1

    # Aggregate across decoder queries — mean over action chunk.
    per_token = attn.mean(dim=0)  # (num_kv_tokens,)
    visual = per_token[n_prefix:]  # drop prefix
    expected = sum(h * w for h, w in cam_shapes)
    if visual.numel() != expected:
        raise RuntimeError(
            f'Visual token count mismatch: got {visual.numel()}, expected {expected} '
            f'(prefix={n_prefix}, cam_shapes={cam_shapes})'
        )
    maps: List[np.ndarray] = []
    offset = 0
    for (h, w) in cam_shapes:
        block = visual[offset:offset + h * w].reshape(h, w).cpu().numpy()
        maps.append(block)
        offset += h * w
    return maps


# ---------------------------------------------------------------------------
# Grad-CAM extraction.
# ---------------------------------------------------------------------------
def _compute_gradcam_maps(policy, batch: dict, preprocessor
                          ) -> List[np.ndarray]:
    """Per-camera Grad-CAM on the backbone's last conv feature map.

    Hooks the backbone to capture (activations, gradients) per camera call.
    Backward target = L2 norm of predicted action chunk — a generic "where
    does the predicted motion come from" signal.
    """
    activations: List[torch.Tensor] = []

    def _fwd(_m, _i, out):
        fmap = out['feature_map'] if isinstance(out, dict) else out
        # retain_grad() 는 requires_grad=True 인 텐서에만 호출 가능. 추론 경로의
        # batch 는 보통 torch.no_grad() 안에서 만들어져서 fmap.requires_grad=False
        # 가 자주 발생 → 그 때 에러 던지지 말고 그냥 capture 만 함. 아래 backward
        # 후 fmap.grad 가 None 으로 남으면 activation magnitude fallback 으로 처리.
        if fmap.requires_grad:
            try:
                fmap.retain_grad()
            except RuntimeError:
                pass
        activations.append(fmap)

    backbone = policy.model.backbone
    h_fwd = backbone.register_forward_hook(_fwd)

    try:
        b = preprocessor(batch) if preprocessor is not None else batch
        # Run model directly (not predict_action_chunk — that wraps in no_grad).
        policy.eval()
        # Need to mimic ACTPolicy.predict_action_chunk's image-list build.
        if policy.config.image_features:
            b = dict(b)
            from lerobot.utils.constants import OBS_IMAGES
            # Gradcam 은 image 입력에 grad 가 흘러야 backbone 활성화의 .grad 가
            # 채워짐. 호출자가 넘긴 텐서는 no_grad 컨텍스트에서 만들어졌을 수
            # 있으니 여기서 clone + requires_grad_(True) 로 leaf 화해 grad 트래킹.
            for _k in list(b.keys()):
                _v = b[_k]
                if (
                    isinstance(_v, torch.Tensor)
                    and _v.is_floating_point()
                    and _k.startswith('observation.images.')
                ):
                    b[_k] = _v.detach().clone().requires_grad_(True)
            b[OBS_IMAGES] = [b[key] for key in policy.config.image_features]

        # Enable grads even though eval() — by default torch.no_grad isn't on.
        with torch.enable_grad():
            actions = policy.model(b)[0]  # (B, chunk, action_dim)
            scalar = actions.norm()
            policy.model.zero_grad(set_to_none=True)
            scalar.backward()
    finally:
        h_fwd.remove()

    if not activations:
        raise RuntimeError('Backbone activations were not captured for Grad-CAM')

    maps: List[np.ndarray] = []
    for fmap in activations:
        if fmap.grad is None:
            # ResNet IntermediateLayerGetter wraps output — grad may be on the
            # dict value. Fall back to using the activation magnitude itself as
            # a coarse saliency.
            cam = fmap[0].abs().mean(dim=0)
        else:
            # weights: spatial mean of gradients per channel.
            grads = fmap.grad[0]                  # (C, H, W)
            acts = fmap[0]                        # (C, H, W)
            w = grads.mean(dim=(1, 2), keepdim=True)  # (C, 1, 1)
            cam = (w * acts).sum(dim=0)               # (H, W)
            cam = torch.relu(cam)
        maps.append(cam.detach().cpu().numpy())
    return maps


# ---------------------------------------------------------------------------
# Rendering — heatmap → RGBA PNG (jet colormap, alpha = intensity).
# ---------------------------------------------------------------------------
def _heatmap_to_rgba_png(heatmap: np.ndarray, target_hw: Tuple[int, int],
                         alpha_gain: float = 0.7) -> str:
    """Normalize heatmap to [0, 1], upscale to target size, apply jet colormap
    with alpha = intensity * alpha_gain, encode as base64 PNG.
    """
    th, tw = target_hw
    # Normalize.
    hm = heatmap.astype(np.float32)
    lo, hi = float(hm.min()), float(hm.max())
    if hi - lo < 1e-8:
        hm = np.zeros_like(hm)
    else:
        hm = (hm - lo) / (hi - lo)
    # Upscale (bilinear) to original frame size.
    hm_up = cv2.resize(hm, (tw, th), interpolation=cv2.INTER_LINEAR)
    # Jet colormap via cv2 (returns BGR uint8).
    color = cv2.applyColorMap((hm_up * 255).astype(np.uint8), cv2.COLORMAP_JET)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    alpha = np.clip(hm_up * alpha_gain * 255, 0, 255).astype(np.uint8)
    rgba = np.dstack([color, alpha])
    ok, buf = cv2.imencode('.png', cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
    if not ok:
        raise RuntimeError('PNG encoding failed')
    return base64.b64encode(buf).decode('utf-8')


# ---------------------------------------------------------------------------
# Public entry point.
# ---------------------------------------------------------------------------
def compute_vision_map(checkpoint: dict, policy_obj: dict,
                       dataset_id: str, episode_idx: int, frame_idx: int,
                       method: str = 'attention') -> Dict[str, str]:
    """Returns {sensor_name: 'data:image/png;base64,...'}."""
    method = (method or 'attention').lower()
    if method not in ('attention', 'gradcam'):
        raise ValueError(f'Unknown vision map method: {method}')

    policy, preprocessor, _postprocessor, image_resolution = _load_policy(checkpoint, policy_obj)
    frames, state, _info = _read_single_frame(dataset_id, episode_idx, frame_idx)
    if not frames:
        raise RuntimeError('No camera frames decoded at this index')

    device = _device()
    batch, ordered_sensors = _build_batch(policy, frames, state, image_resolution, device)

    if method == 'attention':
        attn, cam_shapes = _compute_attention_maps(policy, batch, preprocessor)
        per_cam = _split_attention_per_camera(attn, cam_shapes, policy)
    else:  # gradcam
        per_cam = _compute_gradcam_maps(policy, batch, preprocessor)

    out: Dict[str, str] = {}
    for sensor_name, heatmap in zip(ordered_sensors, per_cam):
        orig = frames[sensor_name]
        png_b64 = _heatmap_to_rgba_png(heatmap, (orig.shape[0], orig.shape[1]))
        out[sensor_name] = 'data:image/png;base64,' + png_b64
    return out


# ---------------------------------------------------------------------------
# Episode streaming — precompute heatmaps for all frames in an episode and
# stream them out via socketio so the frontend can cache per-frame and show
# the matching overlay during playback.
# ---------------------------------------------------------------------------
def _episode_total_frames(dataset_id: str, episode_idx: int) -> Tuple[int, dict, int]:
    """Returns (total_frames, info_dict, chunk_index)."""
    folder = os.path.join(DATASET_DIR, dataset_id)
    if not os.path.isdir(folder):
        raise FileNotFoundError(f'Dataset not found: {folder}')
    info = _read_json(os.path.join(folder, INFO_PATH))
    chunk = episode_idx // info['chunks_size']
    parquet_path = os.path.join(folder,
                                PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_idx))
    if not os.path.exists(parquet_path):
        raise FileNotFoundError(f'Episode parquet not found: {parquet_path}')
    return pq.read_metadata(parquet_path).num_rows, info, chunk


def _open_episode_videos(dataset_id: str, episode_idx: int, info: dict, chunk: int
                         ) -> Dict[str, Any]:
    """Open one VideoCapture per camera. Caller must release them all."""
    folder = os.path.join(DATASET_DIR, dataset_id)
    caps: Dict[str, Any] = {}
    for key, feat in info.get('features', {}).items():
        if not key.startswith('observation.images.') or feat.get('dtype') != 'video':
            continue
        sensor_name = key.replace('observation.images.', '')
        video_path = os.path.join(
            folder, 'videos', f'chunk-{chunk:03d}', key,
            f'episode_{episode_idx:06d}.mp4',
        )
        if not os.path.exists(video_path):
            continue
        cap = cv2.VideoCapture(video_path)
        if cap.isOpened():
            caps[sensor_name] = cap
    return caps


def _load_episode_states(dataset_id: str, episode_idx: int, info: dict, chunk: int
                         ) -> Optional[np.ndarray]:
    """Returns (N, state_dim) array of qpos for every frame, or None."""
    folder = os.path.join(DATASET_DIR, dataset_id)
    parquet_path = os.path.join(folder,
                                PARQUET_PATH_TEMPLATE.format(chunk=chunk, ep=episode_idx))
    if not os.path.exists(parquet_path):
        return None
    df = pq.read_table(parquet_path).to_pandas()
    qpos_col = ('observation.qpos' if 'observation.qpos' in df.columns
                else ('observation.state' if 'observation.state' in df.columns else None))
    if qpos_col is None:
        return None
    return np.stack([np.asarray(v, dtype=np.float32) for v in df[qpos_col].values], axis=0)


# ---------------------------------------------------------------------------
# Live-inference helpers: persistent attention hooks so the heatmap can be
# extracted "for free" from the same forward pass the inference loop already
# runs, instead of doing an extra predict_action_chunk per step.
# ---------------------------------------------------------------------------
def make_attention_holder(policy: Any) -> dict:
    """Register persistent forward hooks that capture the last forward pass's
    decoder cross-attention weights and per-camera backbone feature-map shapes.

    Returns a holder dict updated in place on every forward through ``policy``.
    Call :func:`release_attention_holder` before unloading the policy.
    """
    holder: dict = {
        'w': None,                 # (B, L, S) attn weights, latest forward
        'cam_shapes': [],          # [(H, W), ...] per camera, latest forward
        '_pending_shapes': [],     # accumulated within a single forward
        '_hooks': [],
    }

    def _backbone_hook(_m, _i, out):
        fmap = out['feature_map'] if isinstance(out, dict) else out
        holder['_pending_shapes'].append((int(fmap.shape[-2]), int(fmap.shape[-1])))

    def _attn_hook(_m, _i, out):
        # Decoder cross-attention fires once per forward (after all backbones).
        # Snapshot the accumulated cam_shapes at that moment as "this forward".
        if isinstance(out, tuple) and len(out) > 1 and out[1] is not None:
            holder['w'] = out[1].detach()
            holder['cam_shapes'] = list(holder['_pending_shapes'])
            holder['_pending_shapes'].clear()

    h1 = policy.model.backbone.register_forward_hook(_backbone_hook)
    h2 = policy.model.decoder.layers[-1].multihead_attn.register_forward_hook(_attn_hook)
    holder['_hooks'] = [h1, h2]
    return holder


def release_attention_holder(holder: dict) -> None:
    for h in holder.get('_hooks', []):
        try:
            h.remove()
        except Exception:
            pass
    holder['_hooks'] = []


def render_attention_heatmaps_from_holder(
    holder: dict, policy: Any,
    ordered_sensors: List[str],
    target_hw_per_sensor: Dict[str, Tuple[int, int]],
) -> Dict[str, str]:
    """Render per-camera attention heatmaps using the holder's captured weights
    (from the inference loop's own forward pass — no extra forward needed)."""
    if holder.get('w') is None or not holder.get('cam_shapes'):
        return {}
    attn = holder['w'][0]
    per_cam = _split_attention_per_camera(attn, holder['cam_shapes'], policy)
    out: Dict[str, str] = {}
    for sensor_name, hm in zip(ordered_sensors, per_cam):
        th, tw = target_hw_per_sensor.get(sensor_name, (240, 320))
        out[sensor_name] = 'data:image/png;base64,' + _heatmap_to_rgba_png(hm, (th, tw))
    return out


def compute_vision_map_from_preprocessed_batch(
    policy: Any,
    batch: dict,
    ordered_sensors: List[str],
    target_hw_per_sensor: Dict[str, Tuple[int, int]],
    method: str = 'attention',
) -> Dict[str, str]:
    """Live-inference variant: compute heatmaps from an already-preprocessed
    batch dict (the one the inference loop just passed to predict_action_chunk).

    ``ordered_sensors`` must match the iteration order of policy.config.image_features
    so per-camera maps can be paired back to sensor names. ``target_hw_per_sensor``
    is the (H, W) to upscale each heatmap to for overlay rendering (typically the
    raw camera resolution so the PNG matches the live WebRTC feed).

    Returns ``{sensor_name: 'data:image/png;base64,...'}``.
    """
    method = (method or 'attention').lower()
    if method not in ('attention', 'gradcam'):
        raise ValueError(f'Unknown vision map method: {method}')

    # The batch is already preprocessed — pass preprocessor=None so the
    # internal helpers don't re-normalize.
    if method == 'attention':
        attn, cam_shapes = _compute_attention_maps(policy, batch, preprocessor=None)
        per_cam = _split_attention_per_camera(attn, cam_shapes, policy)
    else:
        per_cam = _compute_gradcam_maps(policy, batch, preprocessor=None)

    out: Dict[str, str] = {}
    for sensor_name, hm in zip(ordered_sensors, per_cam):
        th, tw = target_hw_per_sensor.get(sensor_name, (240, 320))
        out[sensor_name] = 'data:image/png;base64,' + _heatmap_to_rgba_png(hm, (th, tw))
    return out


def _load_ood_image_ref(ckpt_dir: str, device: torch.device) -> Optional[dict]:
    """checkpoint 의 ``ood_features.npz`` 에서 image OOD reference 를 로드.

    반환 {'feats': (N, D) cuda tensor, 'dist_sorted': sorted 1D np array | None} 또는
    파일/키 없으면 None (→ OOD score 표시 안 함). 추론(checkpoint_test)과 동일 포맷.
    """
    try:
        path = os.path.join(ckpt_dir, 'ood_features.npz')
        if not os.path.exists(path):
            return None
        data = np.load(path)
        if 'image_features' not in data:
            return None
        feats = torch.from_numpy(data['image_features']).float().to(device)
        dist_sorted = data['image_dist_sorted'] if 'image_dist_sorted' in data else None
        return {'feats': feats, 'dist_sorted': dist_sorted}
    except Exception as e:
        print(f'[vision_map] OOD ref load failed: {e}')
        return None


def _score_frame(policy, preprocessed_batch: dict, postprocessor,
                 has_succeed: bool, ood_ref: Optional[dict],
                 img_keys: List[str]):
    """이미 preprocess 된 batch 로 (success_score, ood_score) 계산. 둘 다 실패/미가용
    시 None 반환 — heatmap 흐름을 절대 깨지 않도록 전부 graceful.

    - success: 정책 action chunk 의 succeed 토큰(마지막 dim, chunk[0]) 을 추론과 동일
      하게 postprocessor 로 unnormalize 한 값 [0~1 근처]. has_succeed 인 경우만.
    - ood: 현재 image backbone latent vs 학습 reference 의 5-NN 거리 → reference 분포
      백분위 [0(in-dist)~1(far OOD)]. ood_features.npz 있을 때만.
    """
    success = None
    ood = None
    # success score (succeed 토큰)
    if has_succeed:
        try:
            with torch.no_grad():
                act = policy.predict_action_chunk(preprocessed_batch)
                if postprocessor is not None:
                    try:
                        act = postprocessor(act)
                    except Exception:
                        pass  # unnormalize 실패 시 정규화 값 그대로 사용
                a = act.detach().float().cpu().numpy()
                a = a.reshape(-1, a.shape[-1])  # (chunk, dim)
                success = round(float(a[0, -1]), 4)  # chunk[0] 의 succeed 토큰
        except Exception as e:
            print(f'[vision_map] success score skipped: {e}')
    # ood score (image latent percentile)
    if ood_ref is not None:
        try:
            from .checkpoint_test import compute_image_latent
            with torch.no_grad():
                lat = compute_image_latent(policy, preprocessed_batch, img_keys)
            if lat is not None and lat.shape[1] == ood_ref['feats'].shape[1]:
                lat = lat.float().to(ood_ref['feats'].device)
                kk = min(5, ood_ref['feats'].shape[0])
                rd = float(torch.cdist(lat, ood_ref['feats']).topk(kk, largest=False).values.mean())
                ds = ood_ref.get('dist_sorted')
                if ds is not None and len(ds) > 0:
                    ii = int(np.searchsorted(ds, rd))
                    ood = round(min(ii / len(ds), 1.0), 3)
                else:
                    ood = round(rd, 4)
        except Exception as e:
            print(f'[vision_map] ood score skipped: {e}')
    return success, ood


def _compute_one_frame(policy, preprocessor, postprocessor, image_resolution, device,
                       frames_bgr: Dict[str, np.ndarray],
                       state_np: Optional[np.ndarray],
                       method: str,
                       has_succeed: bool = False,
                       ood_ref: Optional[dict] = None,
                       img_keys: Optional[List[str]] = None):
    """Compute (heatmap PNGs, success_score, ood_score) for a single frame.

    batch 를 한 번만 preprocess 해서 heatmap 과 score 계산에 함께 쓴다 (generate 와
    동일 경로 보장 + 중복 전처리 회피). 반환: (heatmaps, success, ood).
    """
    batch, ordered = _build_batch(policy, frames_bgr, state_np, image_resolution, device)
    # preprocess once — heatmap helper 와 scoring 이 같은 normalized batch 를 공유.
    b = preprocessor(batch) if preprocessor is not None else batch
    if method == 'attention':
        attn, cam_shapes = _compute_attention_maps(policy, b, preprocessor=None)
        per_cam = _split_attention_per_camera(attn, cam_shapes, policy)
    else:
        per_cam = _compute_gradcam_maps(policy, b, preprocessor=None)
    out: Dict[str, str] = {}
    for sensor_name, hm in zip(ordered, per_cam):
        orig = frames_bgr[sensor_name]
        png_b64 = _heatmap_to_rgba_png(hm, (orig.shape[0], orig.shape[1]))
        out[sensor_name] = 'data:image/png;base64,' + png_b64
    # compute_image_latent 는 full 'observation.images.*' 키를 기대(센서명 아님).
    _ik = img_keys if img_keys else list(policy.config.image_features)
    success, ood = _score_frame(policy, b, postprocessor, has_succeed, ood_ref, _ik)
    return out, success, ood


def compute_vision_map_episode_stream(
    checkpoint: dict,
    policy_obj: dict,
    dataset_id: str,
    episode_idx: int,
    method: str,
    socketio_instance: Any,
    session_id: str,
    task_control: Optional[dict] = None,
) -> None:
    """Stream per-frame heatmaps for an entire episode over socketio.

    Emits (all payloads include ``session_id`` so the frontend can ignore
    events from a prior/other request):
      - vision_map_episode_start: {session_id, total_frames}
      - vision_map_episode_frame: {session_id, frame_idx, heatmaps}
      - vision_map_episode_done:  {session_id, computed, total}
      - vision_map_episode_error: {session_id, message}

    Cancellation: checks ``task_control['stop']`` between frames.
    Performance: opens video captures once and reads sequentially (cv2 seek
    per-frame is ~10× slower than sequential .read()).
    """
    method = (method or 'attention').lower()

    def emit(event, payload):
        if socketio_instance is None:
            return
        try:
            socketio_instance.emit(event, {'session_id': session_id, **payload})
        except Exception as e:
            print(f'[vision_map_episode] emit failed ({event}): {e}')

    caps: Dict[str, Any] = {}
    try:
        policy, preprocessor, postprocessor, image_resolution = _load_policy(checkpoint, policy_obj)
        device = _device()
        # success/ood score 준비(1회). has_succeed 는 train_settings 에서, OOD reference 는
        # checkpoint dir 의 ood_features.npz 에서. 둘 다 없으면 해당 score 는 null 로 나가
        # heatmap 만 표시된다(기존 동작 보존).
        ts = checkpoint.get('train_settings') or {}
        if isinstance(ts, str):
            try:
                import json as _json
                ts = _json.loads(ts)
            except (ValueError, TypeError):
                ts = {}
        has_succeed = bool(ts.get('has_succeed', False))
        ood_ref = _load_ood_image_ref(resolve_checkpoint_dir(checkpoint['id']), device)
        img_keys = list(policy.config.image_features)
        total, info, chunk = _episode_total_frames(dataset_id, episode_idx)
        emit('vision_map_episode_start', {
            'total_frames': int(total),
            # 프론트가 점수 컬럼을 띄울지 미리 알 수 있게 가용 여부 전달.
            'has_success': has_succeed,
            'has_ood': ood_ref is not None,
        })

        caps = _open_episode_videos(dataset_id, episode_idx, info, chunk)
        if not caps:
            raise RuntimeError('No videos found for this episode')
        states = _load_episode_states(dataset_id, episode_idx, info, chunk)

        computed = 0
        for frame_idx in range(int(total)):
            if task_control is not None and task_control.get('stop'):
                print(f'[vision_map_episode] cancelled at frame {frame_idx}/{total}')
                break

            # Read one frame from every open capture (sequential decode).
            frames_bgr: Dict[str, np.ndarray] = {}
            for sensor_name, cap in caps.items():
                ok, frame = cap.read()
                if ok and frame is not None:
                    frames_bgr[sensor_name] = frame
            if not frames_bgr:
                # End of stream for every camera — stop early.
                break

            state_vec = states[frame_idx] if (states is not None and frame_idx < len(states)) else None
            try:
                heatmaps, success, ood = _compute_one_frame(
                    policy, preprocessor, postprocessor, image_resolution, device,
                    frames_bgr, state_vec, method,
                    has_succeed=has_succeed, ood_ref=ood_ref, img_keys=img_keys,
                )
            except Exception as e:
                print(f'[vision_map_episode] frame {frame_idx} failed: {e}')
                continue

            emit('vision_map_episode_frame', {
                'frame_idx': int(frame_idx), 'heatmaps': heatmaps,
                'success': success, 'ood': ood,
            })
            computed += 1

        emit('vision_map_episode_done', {'computed': int(computed), 'total': int(total)})
    except Exception as e:
        import traceback; traceback.print_exc()
        emit('vision_map_episode_error', {'message': str(e)})
    finally:
        for cap in caps.values():
            try:
                cap.release()
            except Exception:
                pass
