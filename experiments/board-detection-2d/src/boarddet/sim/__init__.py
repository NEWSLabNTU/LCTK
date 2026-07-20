"""Ray-based VLP-32C LiDAR simulator (Phase 7, Stage 9 continued / Task 29).

Casts rays along the REAL VLP-32C beam angles (vendored
`VeloView-VLP-32C.yaml`) instead of grid-sampling scene surfaces in object
space (see `boarddet.synth`, which aliases when re-binned into a range
image). numpy-only; no ROS, no PyTorch.
"""
from __future__ import annotations

from .primitives import Box, Cylinder, Rect
from .raycast import SimFrame, render
from .sensor import BeamGrid, Vlp32cSensor

__all__ = [
    "Box",
    "Cylinder",
    "Rect",
    "SimFrame",
    "render",
    "BeamGrid",
    "Vlp32cSensor",
]
