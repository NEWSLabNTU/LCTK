"""Board pose from a scored quad + its plane."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import PlaneModel, unproject
from .scorer import ScoreResult


@dataclass
class BoardDetection:
    center: np.ndarray     # (3,)
    rotation: np.ndarray   # (3,3): cols = board x, board y, normal
    corners_3d: np.ndarray  # (4,3)
    score: float
    result: ScoreResult


def board_pose(plane: PlaneModel, result: ScoreResult,
               up: np.ndarray = (0.0, 0.0, 1.0)) -> BoardDetection:
    up = np.asarray(up, dtype=float)
    up = up / np.linalg.norm(up)
    corners_3d = unproject(result.corners_2d, plane)
    center = corners_3d.mean(axis=0)

    # Orient the plane normal toward the sensor at the origin. SVD fixes the
    # normal only up to sign; a calibration board's normal must face the
    # sensor for a consistent optical-frame convention.
    n = plane.normal / np.linalg.norm(plane.normal)
    if n @ center > 0.0:          # points away from origin -> flip toward it
        n = -n

    # Board X axis: center -> up-most corner (the diamond "top"), projected
    # in-plane. Uses the caller's `up` (world up in the sensor frame), NOT
    # raw world-Z, so a z-forward rig (Falcon) is handled correctly.
    top = corners_3d[np.argmax(corners_3d @ up)]
    x = top - center
    x = x - (x @ n) * n
    x = x / np.linalg.norm(x)
    y = np.cross(n, x)            # right-handed: (x, y, n)

    # Canonical winding: sort corners CCW about n in the (x, y) basis so both
    # the ICP and non-ICP paths emit one consistent ordering for ArUco
    # correspondence. atan2 starts near the +x (up-most) corner.
    rel = corners_3d - center
    ang = np.arctan2(rel @ y, rel @ x)
    order = np.argsort(ang)
    corners_3d = corners_3d[order]

    rotation = np.stack([x, y, n], axis=1)
    return BoardDetection(center=center, rotation=rotation,
                          corners_3d=corners_3d,
                          score=result.score, result=result)
