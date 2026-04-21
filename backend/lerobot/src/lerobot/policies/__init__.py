# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from .act.configuration_act import ACTConfig as ACTConfig
from .diffusion.configuration_diffusion import DiffusionConfig as DiffusionConfig
from .pi05.configuration_pi05 import PI05Config as PI05Config

# Lazy imports for policies that may not be compatible with Python 3.10
# or are not used in EasyTrainer
import sys as _sys

def __getattr__(name):
    """Lazy-load optional policy configs to avoid import errors on Python 3.10."""
    _lazy_map = {
        "GrootConfig": ".groot.configuration_groot",
        "MultiTaskDiTConfig": ".multi_task_dit.configuration_multi_task_dit",
        "PI0Config": ".pi0.configuration_pi0",
        "PI0FastConfig": ".pi0_fast.configuration_pi0_fast",
        "SmolVLAConfig": ".smolvla.configuration_smolvla",
        "SmolVLANewLineProcessor": ".smolvla.processor_smolvla",
        "TDMPCConfig": ".tdmpc.configuration_tdmpc",
        "VQBeTConfig": ".vqbet.configuration_vqbet",
        "WallXConfig": ".wall_x.configuration_wall_x",
        "XVLAConfig": ".xvla.configuration_xvla",
    }
    if name in _lazy_map:
        import importlib
        mod = importlib.import_module(_lazy_map[name], package=__package__)
        return getattr(mod, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "ACTConfig",
    "DiffusionConfig",
    "PI05Config",
]
