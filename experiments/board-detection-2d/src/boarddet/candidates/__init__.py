"""Candidate generators: full scene -> plausible board plane patches."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..board_config import BoardConfig
from ..geometry import PlaneModel, extent_2d, fit_plane, plane_rms, \
    project_to_plane

_FLATNESS_RMS_MAX = 0.03  # m; above VLP-32C noise floor, below "not a plane"
_MIN_PATCH_POINTS = 60


@dataclass
class Candidate:
    points: np.ndarray  # (N,3)
    plane: PlaneModel


def plausible_board_patch(points_3d: np.ndarray,
                          board: BoardConfig) -> Candidate | None:
    """Gate a 3D patch: enough points, flat, board-sized. None if implausible."""
    if len(points_3d) < _MIN_PATCH_POINTS:
        return None
    plane = fit_plane(points_3d)
    if plane_rms(points_3d, plane) > _FLATNESS_RMS_MAX:
        return None
    ext = extent_2d(project_to_plane(points_3d, plane))
    diag = board.side_m * np.sqrt(2.0)
    if not (0.5 * board.side_m <= ext <= 1.8 * diag):
        return None
    return Candidate(points=points_3d, plane=plane)
