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

from . import Candidate
from ..background import BackgroundModel
from ..board_config import BoardConfig
from ..reject import RejectReason


def generate_background_diff(points: np.ndarray, board: BoardConfig, *,
                             background: BackgroundModel,
                             cluster_eps: float = 0.15,
                             cluster_min_points: int = 30,
                             vertical_gap_deg: float = 3.0,
                             rejects: list[RejectReason] | None = None
                             ) -> list[Candidate]:
    fg = background.foreground_points(points)
    from .cluster_after_ground import _cluster_and_gate
    return _cluster_and_gate(
        fg, board, cluster_eps=cluster_eps,
        cluster_min_points=cluster_min_points,
        vertical_gap_deg=vertical_gap_deg, rejects=rejects)
