#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team.
# All rights reserved.
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

from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.multi_dataset import MultiLeRobotDataset
from lerobot.datasets.sampler import EpisodeAwareSampler
from lerobot.datasets.transforms import ImageTransforms, ImageTransformsConfig

# StreamingLeRobotDataset uses Python 3.12+ generics syntax (class Foo[T]:)
# Lazy-load to avoid SyntaxError on Python 3.10
import sys as _sys
if _sys.version_info >= (3, 12):
    from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset
else:
    StreamingLeRobotDataset = None

__all__ = [
    "EpisodeAwareSampler",
    "ImageTransforms",
    "ImageTransformsConfig",
    "LeRobotDataset",
    "LeRobotDatasetMetadata",
    "MultiLeRobotDataset",
    "StreamingLeRobotDataset",
]
