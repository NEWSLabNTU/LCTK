"""Approach A: iterative RANSAC plane extraction (velo2cam style)."""
from __future__ import annotations

import numpy as np
import open3d as o3d

from . import Candidate, plausible_board_patch
from ..board_config import BoardConfig


def generate_ransac_iterative(points: np.ndarray, board: BoardConfig,
                              max_planes: int = 8,
                              dist_thresh: float = 0.02,
                              min_inliers: int = 60,
                              component_eps: float = 0.10) -> list[Candidate]:
    remaining = points.astype(np.float64)
    out: list[Candidate] = []
    for _ in range(max_planes):
        if len(remaining) < min_inliers:
            break
        pc = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(remaining))
        _, inlier_idx = pc.segment_plane(
            distance_threshold=dist_thresh, ransac_n=3, num_iterations=500)
        if len(inlier_idx) < min_inliers:
            break
        inliers = remaining[inlier_idx].astype(np.float32)
        # A RANSAC plane can span board + coplanar clutter (the extended board
        # plane grazes ground/walls). Gate each spatially connected component,
        # not the whole inlier set (velo2cam-style clustering).
        pc_in = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(inliers.astype(np.float64)))
        labels = np.asarray(
            pc_in.cluster_dbscan(eps=component_eps, min_points=10))
        for lbl in np.unique(labels):
            if lbl < 0:
                continue
            cand = plausible_board_patch(inliers[labels == lbl], board)
            if cand is not None:
                out.append(cand)
        mask = np.ones(len(remaining), dtype=bool)
        mask[inlier_idx] = False
        remaining = remaining[mask]
    return out
