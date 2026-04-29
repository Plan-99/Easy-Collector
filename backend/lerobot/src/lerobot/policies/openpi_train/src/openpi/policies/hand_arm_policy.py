# hand + arm: 1 camera + 14DOF (ee 6 + hand 8)
import dataclasses
from typing import ClassVar

import einops
import numpy as np

from src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi import transforms


def make_hand_arm_example(action_horizon: int = 50) -> dict:
    return {
        "state": np.ones((14,), dtype=np.float32),
        "images": {
            "cam_wrist": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
        },
        "actions": np.zeros((action_horizon, 14), dtype=np.float32),
        "prompt": "place the ball on the white bowl",
    }


def _to_hwc_uint8(img: np.ndarray) -> np.ndarray:
    img = np.asarray(img)
    if np.issubdtype(img.dtype, np.floating):
        img = (255.0 * img).clip(0, 255).astype(np.uint8)
    if img.ndim == 3 and img.shape[0] in (1, 3) and img.shape[-1] not in (1, 3):
        img = einops.rearrange(img, "c h w -> h w c")
    if img.ndim != 3 or img.shape[-1] not in (1, 3, 4):
        raise ValueError(f"Expected image with shape CHW or HWC, got {img.shape}")
    return img


@dataclasses.dataclass(frozen=True)
class HandArmInputs(transforms.DataTransformFn):
    """Inputs transform for hand+arm (14DOF, 1 camera)."""

    adapt_to_pi: bool = False
    action_dim: int = 14

    EXPECTED_CAMERAS: ClassVar[tuple[str, ...]] = ("cam_wrist",)

    def __call__(self, data: dict) -> dict:
        in_images = data["images"]

        if "cam_wrist" not in in_images:
            raise KeyError("HandArmInputs requires 'cam_wrist' in images.")

        base_image = _to_hwc_uint8(in_images["cam_wrist"])

        images = {"base_0_rgb": base_image}
        image_masks = {"base_0_rgb": np.True_}

        # 카메라 1개이므로 나머지는 빈 이미지
        for dest in ("left_wrist_0_rgb", "right_wrist_0_rgb"):
            images[dest] = np.zeros_like(base_image)
            image_masks[dest] = np.False_

        out = {
            "image": images,
            "image_mask": image_masks,
            "state": np.asarray(data["state"], dtype=np.float32),
        }

        if "actions" in data:
            actions = np.asarray(data["actions"], dtype=np.float32)
            if actions.ndim != 2 or actions.shape[-1] != self.action_dim:
                raise ValueError(f"Expected actions shape [T,{self.action_dim}], got {actions.shape}")
            out["actions"] = actions

        if "prompt" in data:
            out["prompt"] = data["prompt"]

        return out


@dataclasses.dataclass(frozen=True)
class HandArmOutputs(transforms.DataTransformFn):
    adapt_to_pi: bool = False
    action_dim: int = 14

    def __call__(self, data: dict) -> dict:
        actions = np.asarray(data["actions"], dtype=np.float32)
        actions = actions[:, :self.action_dim]
        return {"actions": actions}
