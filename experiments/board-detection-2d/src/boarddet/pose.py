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


def board_pose(plane: PlaneModel, result: ScoreResult) -> BoardDetection:
    corners_3d = unproject(result.corners_2d, plane)
    center = corners_3d.mean(axis=0)
    # board x axis: center -> highest corner (diamond "top"), projected in-plane
    top = corners_3d[np.argmax(corners_3d[:, 2])]
    x = top - center
    x = x - (x @ plane.normal) * plane.normal
    x = x / np.linalg.norm(x)
    n = plane.normal
    y = np.cross(n, x)
    rotation = np.stack([x, y, n], axis=1)
    return BoardDetection(center=center, rotation=rotation,
                          corners_3d=corners_3d,
                          score=result.score, result=result)
