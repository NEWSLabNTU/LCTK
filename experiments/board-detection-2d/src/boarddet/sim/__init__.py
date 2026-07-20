"""Ray-based VLP-32C LiDAR simulator (Phase 7, Stage 9 continued / Task 29).

Casts rays along the REAL VLP-32C beam angles (vendored
`VeloView-VLP-32C.yaml`) instead of grid-sampling scene surfaces in object
space (see `boarddet.synth`, which aliases when re-binned into a range
image). numpy-only; no ROS, no PyTorch.
"""
from __future__ import annotations

from .dataset import (
    BoardLabel,
    DatasetConfig,
    SceneSample,
    generate_dataset,
    load_scene,
    render_labeled_scene,
)
from .primitives import Box, Cylinder, Rect
from .range_image import (
    RangeImageGrid,
    sim_frame_to_range_image,
    to_range_image,
)
from .raycast import SimFrame, render
from .scenegen import (
    BoardMeta,
    ClutterMeta,
    Scene,
    SceneGenConfig,
    make_diamond_board,
    random_scene,
)
from .sensor import BeamGrid, Vlp32cSensor

__all__ = [
    "Box",
    "Cylinder",
    "Rect",
    "SimFrame",
    "render",
    "BeamGrid",
    "Vlp32cSensor",
    "RangeImageGrid",
    "to_range_image",
    "sim_frame_to_range_image",
    "Scene",
    "SceneGenConfig",
    "BoardMeta",
    "ClutterMeta",
    "make_diamond_board",
    "random_scene",
    "DatasetConfig",
    "SceneSample",
    "BoardLabel",
    "render_labeled_scene",
    "generate_dataset",
    "load_scene",
]
