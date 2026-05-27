"""Standalone inference for an exported EasyTrainer checkpoint.

This file is bundled with every checkpoint export. It loads the policy + the
preprocessor/postprocessor pipelines that were saved alongside the model and
exposes a simple ``CheckpointInference`` class plus a small CLI demo.

Layout expected (this file lives at the root of the unzipped bundle)::

    bundle/
    ├── inference.py        ← this file
    ├── export_meta.json
    ├── model/              ← all the checkpoint files
    ├── lerobot/            ← vendored lerobot src

Usage as a Python module
------------------------
::

    from inference import CheckpointInference
    import numpy as np
    from PIL import Image

    inf = CheckpointInference("./model", "./export_meta.json")
    inf.reset()                                              # at episode start

    state = np.array([0., 0.1, -0.6, 0., 1., 0., 0.05], dtype=np.float32)
    images = {
        "sensor_1": np.array(Image.open("cam1.png").convert("RGB")),
        "sensor_2": np.array(Image.open("cam2.png").convert("RGB")),
        "sensor_3": np.array(Image.open("cam3.png").convert("RGB")),
    }
    action = inf.infer(state, images)        # (action_dim,) numpy float32
    print(action)

Usage as a CLI demo
-------------------
::

    python inference.py \
        --state "0,0.1,-0.6,0,1,0,0.05" \
        --image sensor_1=cam1.png \
        --image sensor_2=cam2.png \
        --image sensor_3=cam3.png

Limitations
-----------
- vision_backbone must be ``resnet18`` for this MVP exporter. dinov2/dinov3 are
  not yet supported because they require HuggingFace Hub access at load time.
- relative_ee_pos / ee_delta_action action modes are not converted here. The
  raw action vector returned matches what the model was trained to predict.
- ``has_succeed=true`` checkpoints return the trailing succeed bit as the last
  element of the action vector. The caller decides what to do with it.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

# ── Bundled lerobot isolation ────────────────────────────────────────────────
# We need ``import lerobot`` to resolve to the copy that ships inside this
# bundle, regardless of the user's PYTHONPATH, pip-installed packages, or any
# pre-imported lerobot in sys.modules. The bundle's lerobot is patched for
# Python 3.10 compatibility (PEP 695 workarounds, lazy imports, etc.) — any
# other lerobot will fail with ``ModuleNotFoundError: lerobot.datasets`` or a
# SyntaxError.
#
# Step 1: locate the bundled package directory.
# Step 2: drop any pre-loaded lerobot* modules.
# Step 3: rewrite sys.path so the bundled directory is the ONLY place Python
#         can find a top-level ``lerobot``. Any other entry that contains a
#         lerobot package directory is removed.
_HERE = Path(__file__).resolve().parent
_BUNDLED_LEROBOT_PARENT = _HERE / "lerobot"   # directory that *contains* the
                                              # ``lerobot/`` package directory
_BUNDLED_LEROBOT_PKG = _BUNDLED_LEROBOT_PARENT / "lerobot"  # the package itself

if _BUNDLED_LEROBOT_PKG.is_dir():
    # 1. Purge any half-imported lerobot from sys.modules.
    for _name in list(sys.modules.keys()):
        if _name == "lerobot" or _name.startswith("lerobot."):
            del sys.modules[_name]

    # 2. Remove any sys.path entries that would resolve ``import lerobot`` to
    #    something other than our bundled copy.
    _bundled_parent_resolved = _BUNDLED_LEROBOT_PARENT.resolve()
    _filtered = []
    for _entry in sys.path:
        if not _entry:
            _filtered.append(_entry)
            continue
        try:
            _entry_resolved = Path(_entry).resolve()
        except (OSError, RuntimeError):
            _filtered.append(_entry)
            continue
        if _entry_resolved == _bundled_parent_resolved:
            continue  # we'll re-add this at position 0
        # Drop any path that has a top-level ``lerobot/__init__.py`` or
        # ``lerobot/`` namespace dir under it, since that would shadow ours.
        if (_entry_resolved / "lerobot" / "__init__.py").is_file():
            continue
        # Drop entries that ARE inside the lerobot package (mis-configured
        # PYTHONPATH that points at the package internals).
        if _entry_resolved.name == "lerobot" and (_entry_resolved / "__init__.py").is_file():
            continue
        _filtered.append(_entry)

    # 3. Bundled path wins. Insert at position 0.
    sys.path[:] = [str(_bundled_parent_resolved)] + _filtered

    # 4. Make sure Python re-scans sys.path on the next import.
    import importlib as _importlib
    _importlib.invalidate_caches()

import numpy as np
import torch
from PIL import Image
from torchvision import transforms


# ──────────────────────────────────────────────────────────────────────────────
# Policy loading dispatch
# ──────────────────────────────────────────────────────────────────────────────
def _load_policy(policy_type: str, model_dir: str):
    """Load a policy from a local directory based on the saved type."""
    if policy_type == "ACT":
        from lerobot.policies.act.modeling_act import ACTPolicy
        return ACTPolicy.from_pretrained(model_dir)
    if policy_type == "Diffusion":
        from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
        return DiffusionPolicy.from_pretrained(model_dir)
    if policy_type == "PI05":
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy
        return PI05Policy.from_pretrained(model_dir)
    raise ValueError(
        f"Unknown policy_type {policy_type!r}. Expected one of ACT/Diffusion/PI05."
    )


# ──────────────────────────────────────────────────────────────────────────────
# Pre/post processor loading (mirrors make_easytrainer_processors)
# ──────────────────────────────────────────────────────────────────────────────
def _load_processors(model_dir: str):
    """Load Normalize/Unnormalize processor pipelines saved with the checkpoint.

    This bypasses ``lerobot.policies.factory.make_pre_post_processors`` (which
    eagerly imports envs/robots/motors that aren't needed for inference) and
    builds the same PolicyProcessorPipeline directly from the saved JSON.
    """
    from lerobot.processor.pipeline import PolicyProcessorPipeline
    from lerobot.processor.converters import (
        batch_to_transition,
        transition_to_batch,
        policy_action_to_transition,
        transition_to_policy_action,
    )
    from lerobot.utils.constants import (
        POLICY_PREPROCESSOR_DEFAULT_NAME,
        POLICY_POSTPROCESSOR_DEFAULT_NAME,
    )
    pre = PolicyProcessorPipeline.from_pretrained(
        pretrained_model_name_or_path=model_dir,
        config_filename=f"{POLICY_PREPROCESSOR_DEFAULT_NAME}.json",
        to_transition=batch_to_transition,
        to_output=transition_to_batch,
    )
    post = PolicyProcessorPipeline.from_pretrained(
        pretrained_model_name_or_path=model_dir,
        config_filename=f"{POLICY_POSTPROCESSOR_DEFAULT_NAME}.json",
        to_transition=policy_action_to_transition,
        to_output=transition_to_policy_action,
    )
    return pre, post


# ──────────────────────────────────────────────────────────────────────────────
# Image preprocessing — mirrors src/backend/policies/utils.py process_image()
# resnet18 path only.
# ──────────────────────────────────────────────────────────────────────────────
_RESNET18_TRANSFORM = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
])


def _process_image(img: np.ndarray) -> torch.Tensor:
    """uint8 HxWx3 numpy → float32 (3, 224, 224) tensor in [0, 1]."""
    if not isinstance(img, np.ndarray):
        raise TypeError(f"image must be a numpy array, got {type(img)}")
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError(f"image must be HxWx3, got shape {img.shape}")
    pil = Image.fromarray(img.astype(np.uint8))
    return _RESNET18_TRANSFORM(pil)


# ──────────────────────────────────────────────────────────────────────────────
# Public inference class
# ──────────────────────────────────────────────────────────────────────────────
class CheckpointInference:
    """One-call inference wrapper around an exported EasyTrainer checkpoint.

    Parameters
    ----------
    model_dir:
        Path to the ``model/`` subdirectory of the bundle (containing
        ``config.json``, ``model.safetensors``, ``policy_preprocessor*``, etc.).
    meta_path:
        Path to ``export_meta.json`` (sensor list, vision backbone, etc.).
    device:
        ``"cuda"`` (default) falls back to CPU automatically when CUDA is
        unavailable.
    """

    def __init__(self, model_dir: str, meta_path: str, device: str = "cuda"):
        self.device = torch.device(
            device if (device == "cpu" or torch.cuda.is_available()) else "cpu"
        )

        with open(meta_path) as f:
            self.meta = json.load(f)

        backbone = self.meta.get("vision_backbone", "resnet18")
        if backbone != "resnet18":
            raise NotImplementedError(
                f"Exported inference currently supports vision_backbone='resnet18' "
                f"only. This checkpoint uses {backbone!r}. See the README for the "
                f"reason and a workaround."
            )

        self.policy = _load_policy(self.meta["policy_type"], model_dir)
        self.policy.to(self.device)
        self.policy.eval()

        self.pre, self.post = _load_processors(model_dir)

        self.sensor_ids = list(self.meta.get("sensor_ids", []))
        if not self.sensor_ids:
            raise ValueError(
                "export_meta.json has no sensor_ids — cannot map images to inputs."
            )

    # ──────────────────────────────────────────────────────────────────────
    def reset(self) -> None:
        """Call at the start of each episode.

        Clears the temporal ensembler state / action queue inside the policy
        so the next ``infer`` call is treated as a fresh trajectory.
        """
        if hasattr(self.policy, "reset"):
            self.policy.reset()

    # ──────────────────────────────────────────────────────────────────────
    @torch.no_grad()
    def infer(self, state: np.ndarray, images: dict) -> np.ndarray:
        """Run a single inference step.

        Parameters
        ----------
        state:
            ``(state_dim,)`` numpy float32 array. Raw units, the same as
            ``observation.state`` in the training dataset.
        images:
            Dict with one entry per sensor declared in ``export_meta.json``.
            Keys must be ``"sensor_<id>"``. Values are ``(H, W, 3)`` uint8
            numpy arrays in RGB order. Resolution doesn't matter — the
            preprocessor resizes to 224x224.

        Returns
        -------
        action:
            ``(action_dim,)`` numpy float32 array in raw units (already
            unnormalized via the postprocessor).
        """
        # State → tensor
        if not isinstance(state, np.ndarray):
            state = np.asarray(state, dtype=np.float32)
        state_t = torch.from_numpy(state.astype(np.float32)).to(self.device).unsqueeze(0)

        policy_input = {"observation.state": state_t}

        # Images → tensors.
        # Multi-view: sensor_ids 가 같은 sensor_id 를 N 번 포함하면 그만큼
        # view_key 가 펼쳐진다. caller 의 ``images`` dict 는 view_key 기준
        # (예: "sensor_5", "sensor_5_2") 으로 받는다.
        # Sensor_id stable sort → 학습/inference 의 image_features 순서와 정합.
        _seen_occ = {}
        _view_pairs = []
        for sid in self.sensor_ids:
            sid_int = int(sid)
            occ = _seen_occ.get(sid_int, 0)
            _seen_occ[sid_int] = occ + 1
            vkey = str(sid_int) if occ == 0 else f"{sid_int}_{occ + 1}"
            _view_pairs.append((sid_int, occ, vkey))
        _view_pairs.sort(key=lambda t: (t[0], t[1]))
        for _sid_int, _occ, vkey in _view_pairs:
            user_key = f"sensor_{vkey}"
            if user_key not in images:
                raise KeyError(
                    f"Missing image for {user_key!r}. "
                    f"Required view keys derived from sensor_ids "
                    f"{self.sensor_ids}: views=[{', '.join(self._view_keys())}]"
                )
            img_t = _process_image(images[user_key]).to(self.device).unsqueeze(0)
            policy_input[f"observation.images.sensor_{vkey}"] = img_t

        # Preprocessor: rename → batch dim → device → normalize state/images
        policy_input = self.pre(policy_input)

        # Policy. select_action returns a single (B, action_dim) tensor for
        # ACT (with temporal ensembling) and Diffusion/PI05 (action queue).
        raw_action = self.policy.select_action(policy_input)

        # Postprocessor: unnormalize back to raw units, move to CPU
        unnorm = self.post(raw_action)
        return unnorm.squeeze(0).detach().cpu().numpy()

    def _view_keys(self):
        """Diagnostic helper: list of view_keys derived from sensor_ids,
        sorted by (sensor_id, occurrence) — same ordering as infer() loop."""
        seen = {}
        items = []
        for sid in self.sensor_ids:
            sid_int = int(sid)
            occ = seen.get(sid_int, 0)
            seen[sid_int] = occ + 1
            vkey = str(sid_int) if occ == 0 else f"{sid_int}_{occ + 1}"
            items.append((sid_int, occ, vkey))
        items.sort(key=lambda t: (t[0], t[1]))
        return [vkey for _sid, _occ, vkey in items]


# ──────────────────────────────────────────────────────────────────────────────
# CLI demo
# ──────────────────────────────────────────────────────────────────────────────
def _cli() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description="Run a single inference step against an exported checkpoint."
    )
    parser.add_argument(
        "--state",
        required=True,
        help="Comma-separated state floats, e.g. '0,0.1,-0.6,0,1,0,0.05'",
    )
    parser.add_argument(
        "--image",
        action="append",
        required=True,
        help="Image spec sensor_<id>=<path>. Repeat for multiple sensors.",
    )
    parser.add_argument(
        "--model-dir",
        default=str(_HERE / "model"),
        help="Path to the model/ directory (default: ./model)",
    )
    parser.add_argument(
        "--meta",
        default=str(_HERE / "export_meta.json"),
        help="Path to export_meta.json (default: ./export_meta.json)",
    )
    parser.add_argument(
        "--device",
        default="cuda",
        help="cuda (default) or cpu",
    )
    args = parser.parse_args()

    state = np.array([float(x) for x in args.state.split(",")], dtype=np.float32)

    images = {}
    for spec in args.image:
        if "=" not in spec:
            raise SystemExit(f"--image expects sensor_<id>=path, got: {spec!r}")
        key, path = spec.split("=", 1)
        images[key] = np.array(Image.open(path).convert("RGB"))

    inf = CheckpointInference(args.model_dir, args.meta, device=args.device)
    inf.reset()
    action = inf.infer(state, images)
    print("action:", action.tolist())


if __name__ == "__main__":
    _cli()
