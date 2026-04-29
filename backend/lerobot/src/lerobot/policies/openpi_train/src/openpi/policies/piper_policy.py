# one piper + 3 cameras
import dataclasses
from typing import ClassVar

import einops
import numpy as np

from src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi import transforms


def make_piper_example(action_horizon: int = 50) -> dict:
    """Creates a random input example for the Piper policy (single-arm, 3-cam)."""
    return {
        "state": np.ones((7,), dtype=np.float32),
        "images": {
            "cam_high": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
            "cam_left_wrist": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
            "cam_right_wrist": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
        },
        "actions": np.zeros((action_horizon, 7), dtype=np.float32),
        "prompt": "Stack red block on blue block",
    }


def _to_hwc_uint8(img: np.ndarray) -> np.ndarray:
    """Accept CHW or HWC (float or uint8) and return HWC uint8."""
    img = np.asarray(img)

    # float -> uint8
    if np.issubdtype(img.dtype, np.floating):
        img = (255.0 * img).clip(0, 255).astype(np.uint8)

    # If CHW -> HWC
    # Heuristic: CHW commonly shaped (3, H, W)
    if img.ndim == 3 and img.shape[0] in (1, 3) and img.shape[-1] not in (1, 3):
        img = einops.rearrange(img, "c h w -> h w c")

    # Ensure HWC
    if img.ndim != 3 or img.shape[-1] not in (1, 3, 4):
        raise ValueError(f"Expected image with shape CHW or HWC, got {img.shape}")

    # If grayscale HWC with channel=1 -> keep as is (ResizeImages should still handle)
    return img


@dataclasses.dataclass(frozen=True)
class PiperInputs(transforms.DataTransformFn):
    """Inputs transform for a single-arm Piper-like setup.

    Expected incoming keys (after repack):
    - images: dict[name, img], img is CHW or HWC. (uint8 or float)
    - state: [7]
    - actions: [action_horizon, 7] (only during training)
    - prompt: str (optional)

    Produces keys expected by downstream transforms / model:
    - image: dict[str, HWC uint8]  (NOTE: key is 'image', not 'images')
    - image_mask: dict[str, bool]
    - state: [7]
    - actions: [action_horizon, 7] (if present)
    - prompt: (if present)
    """

    # Leave False unless you REALLY know you need a robot-specific conversion.
    adapt_to_pi: bool = False
    # Raw action dimension (7 = 6 joints + gripper, 8 = 6 joints + gripper + done).
    action_dim: int = 7

    # Cameras that are allowed to appear in `data["images"]`.
    EXPECTED_CAMERAS: ClassVar[tuple[str, ...]] = ("cam_high", "cam_left_wrist", "cam_right_wrist")

    def __call__(self, data: dict) -> dict:
        if "images" not in data:
            raise KeyError("PiperInputs expected key 'images' in input data.")
        if "state" not in data:
            raise KeyError("PiperInputs expected key 'state' in input data.")

        in_images = data["images"]
        if not isinstance(in_images, dict):
            raise ValueError(f"'images' must be a dict, got {type(in_images)}")

        # Optional strictness: fail on unexpected camera names
        unexpected = set(in_images) - set(self.EXPECTED_CAMERAS)
        if unexpected:
            raise ValueError(f"Unexpected camera(s) {tuple(unexpected)}. Expected subset of {self.EXPECTED_CAMERAS}")

        # Base image must exist
        if "cam_high" not in in_images:
            raise KeyError("PiperInputs requires 'cam_high' in images (base camera).")

        # Convert base image
        base_image = _to_hwc_uint8(in_images["cam_high"])

        # Build canonical image dict expected by OpenPI pipelines (matches aloha_policy convention)
        images = {"base_0_rgb": base_image}
        image_masks = {"base_0_rgb": np.True_}

        # Optional extra views
        extra_image_names = {
            "left_wrist_0_rgb": "cam_left_wrist",
            "right_wrist_0_rgb": "cam_right_wrist",
        }
        for dest, source in extra_image_names.items():
            if source in in_images:
                images[dest] = _to_hwc_uint8(in_images[source])
                image_masks[dest] = np.True_
            else:
                images[dest] = np.zeros_like(base_image)
                image_masks[dest] = np.False_

        out = {
            # IMPORTANT: downstream ResizeImages expects 'image', not 'images'
            "image": images,
            "image_mask": image_masks,
            "state": np.asarray(data["state"], dtype=np.float32),
        }

        # Actions only during training
        if "actions" in data:
            actions = np.asarray(data["actions"], dtype=np.float32)
            if actions.ndim != 2 or actions.shape[-1] != self.action_dim:
                raise ValueError(f"Expected actions shape [T,{self.action_dim}], got {actions.shape}")
            out["actions"] = actions

        if "prompt" in data:
            out["prompt"] = data["prompt"]

        return out


@dataclasses.dataclass(frozen=True)
class PiperOutputs(transforms.DataTransformFn):
    """Outputs transform (model -> environment/action space)."""
    adapt_to_pi: bool = False
    # Raw action dimension (must match PiperInputs.action_dim).
    action_dim: int = 7

    def __call__(self, data: dict) -> dict:
        actions = np.asarray(data["actions"], dtype=np.float32)
        actions = actions[:, :self.action_dim]
        return {"actions": actions}