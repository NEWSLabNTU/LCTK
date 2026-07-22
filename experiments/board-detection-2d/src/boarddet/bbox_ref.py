"""The true-board reference box, read from a `bbox.json5`.

Benchmarks classify each accepted detection as true-board or clutter by
where its centre falls. Stages 3-8 and Method E did that against one
hardcoded box -- the pcap rig's -- which is wrong for any other rig. Each
recording rig supplies its own reference file in the same schema the
detector's crop box already uses:

    {
      "pose": {"translation": [x, y, z],
               "rotation": [x, y, z, w]},   // quaternion, w LAST
      "size_xyz": [x, y, z]                 // FULL extent, not half
    }

The rotation is nalgebra's serde order (w last) -- the same trap
`bbox.json5`'s own comment documents, where [1,0,0,0] looks like identity
but is a 180 deg rotation about x.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import json5
import numpy as np


def _quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix (box -> world)."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@dataclass
class BoxRef:
    center: np.ndarray  # (3,) box centre in world coords
    half: np.ndarray    # (3,) half extents along the box's own axes
    rot: np.ndarray     # (3,3) box -> world

    def contains(self, point: np.ndarray) -> bool:
        """Is `point` inside the box? Tested in the BOX's frame, so a
        rotated reference is handled correctly rather than being treated as
        its axis-aligned bounding box."""
        local = self.rot.T @ (np.asarray(point, dtype=np.float64) - self.center)
        return bool(np.all(np.abs(local) <= self.half))


def load_bbox(path: str | Path) -> BoxRef:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"bbox reference not found: {path}")
    raw = json5.loads(path.read_text())
    pose = raw["pose"]
    return BoxRef(
        center=np.asarray(pose["translation"], dtype=np.float64),
        half=np.asarray(raw["size_xyz"], dtype=np.float64) / 2.0,
        rot=_quat_xyzw_to_matrix(
            np.asarray(pose["rotation"], dtype=np.float64)),
    )
