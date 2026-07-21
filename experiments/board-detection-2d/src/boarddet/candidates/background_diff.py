"""Approach E (roadmap Method E): background / motion subtraction.

Diffs the frame against a caller-supplied, already-finalized
`BackgroundModel`, clusters the surviving foreground with generator B's
range-scaled anisotropic DBSCAN, and gates each cluster through the same
`plausible_board_patch` every other generator uses.

There is deliberately NO `_remove_big_planes` stage: ground and walls are
background by construction and are already gone before clustering runs,
which also drops generator B's most expensive stage. The anisotropic
scaling IS still needed -- a revealed board is sampled through the same
VLP-32C rings, so ring-gap fragmentation survives the diff untouched.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d

from . import Candidate, plausible_board_patch
from ..background import BackgroundModel
from ..board_config import BoardConfig
from .cluster_after_ground import _anisotropic_scaled


def generate_background_diff(points: np.ndarray, board: BoardConfig, *,
                             background: BackgroundModel,
                             cluster_eps: float = 0.15,
                             cluster_min_points: int = 30,
                             vertical_gap_deg: float = 3.0
                             ) -> list[Candidate]:
    fg = background.foreground_points(points)
    if len(fg) < cluster_min_points:
        return []
    scaled = _anisotropic_scaled(fg.astype(np.float64), cluster_eps,
                                 vertical_gap_deg)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scaled))
    labels = np.asarray(pc.cluster_dbscan(eps=cluster_eps,
                                          min_points=cluster_min_points))
    out: list[Candidate] = []
    for lbl in np.unique(labels):
        if lbl < 0:
            continue
        cand = plausible_board_patch(fg[labels == lbl], board)
        if cand is not None:
            out.append(cand)
    return out
