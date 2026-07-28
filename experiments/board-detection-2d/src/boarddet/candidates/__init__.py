"""Candidate generators: full scene -> plausible board plane patches."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..board_config import BoardConfig
from ..geometry import PlaneModel, extent_2d, fit_plane, plane_rms, \
    project_to_plane
from ..reject import RejectReason, Stage, band, upper

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


def lower_points(n: int) -> RejectReason:
    """PATCH_POINTS reject: structural count gate, no tunable param, margin 0."""
    return RejectReason(Stage.PATCH_POINTS, "patch_points", None,
                        float(n), float(_MIN_PATCH_POINTS), 0.0)


@dataclass
class Candidate:
    points: np.ndarray  # (N,3)
    plane: PlaneModel


def plausible_board_patch(points_3d: np.ndarray, board: BoardConfig,
                          flatness_rms_max: float | None = None,
                          rejects: list[RejectReason] | None = None
                          ) -> Candidate | None:
    """Gate a 3D patch: enough points, flat, board-sized. None if implausible.

    flatness_rms_max=None (default) reads board.flatness_rms_max (Task 20),
    falling back to the module constant if board lacks that attribute --
    board.flatness_rms_max itself defaults to _FLATNESS_RMS_MAX, so the
    default call path is byte-identical to pre-Task-20 behavior.

    rejects, when given, collects a RejectReason at each gate that fires
    (side channel; does not change the accept/reject decision or return type).
    """
    if len(points_3d) < _MIN_PATCH_POINTS:
        if rejects is not None:
            rejects.append(lower_points(len(points_3d)))
        return None
    threshold = flatness_rms_max
    if threshold is None:
        threshold = getattr(board, "flatness_rms_max", _FLATNESS_RMS_MAX)
    plane = fit_plane(points_3d)
    rms = plane_rms(points_3d, plane)
    if rms > threshold:
        if rejects is not None:
            rejects.append(upper(Stage.PATCH_FLATNESS, "flatness",
                                 "flatness_rms_max", rms, threshold))
        return None
    ext = extent_2d(project_to_plane(points_3d, plane))
    diag = board.side_m * np.sqrt(2.0)
    lo, hi = 0.5 * board.side_m, 1.8 * diag
    if not (lo <= ext <= hi):
        if rejects is not None:
            rejects.append(band(Stage.PATCH_EXTENT, "extent", None, ext, lo, hi))
        return None
    return Candidate(points=points_3d, plane=plane)
