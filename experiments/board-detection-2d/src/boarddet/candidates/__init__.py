"""Candidate generators: full scene -> plausible board plane patches."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..board_config import BoardConfig
from ..geometry import PlaneModel, extent_2d, fit_plane, plane_rms, \
    project_to_plane

# m. CLAUDE.md / board_detector.json5's icp_good_fit_threshold documents the
# VLP-32C's own range noise as the reason ICP loss asymptotes at 0.026-0.029 m
# on this same recorded board -- that is the noise floor, not a bad fit, and
# icp_good_fit_threshold sits at 0.035 to clear it with margin. This gate's
# plane-fit RMS lands in exactly that band on real board clusters (measured
# 0.029-0.031 m on dataset 3), so the previous 0.03 m was *at* the noise
# floor rather than above it and intermittently rejected genuine board
# patches. Match icp_good_fit_threshold's margin instead of re-deriving one.
_FLATNESS_RMS_MAX = 0.035  # m; above VLP-32C noise floor, below "not a plane"
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
