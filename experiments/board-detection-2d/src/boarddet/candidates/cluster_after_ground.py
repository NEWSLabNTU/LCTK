"""Approach B: remove large planes, Euclidean-cluster the rest, gate clusters."""
from __future__ import annotations

import numpy as np
import open3d as o3d

from . import Candidate, plausible_board_patch
from ..board_config import BoardConfig
from ..geometry import extent_2d, fit_plane, project_to_plane


def _remove_big_planes(points: np.ndarray, board: BoardConfig,
                       dist: float, min_frac: float) -> np.ndarray:
    """Iteratively strip planes whose inlier patch is far larger than a board."""
    diag = board.side_m * np.sqrt(2.0)
    remaining = points.astype(np.float64)
    for _ in range(6):
        if len(remaining) < 100:
            break
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(remaining))
        _, idx = pc.segment_plane(distance_threshold=dist, ransac_n=3,
                                  num_iterations=300)
        if len(idx) < max(100, int(min_frac * len(remaining))):
            break
        inliers = remaining[idx].astype(np.float32)
        # Design amendment (controller-authorized): judge big-vs-board-scale on
        # the largest connected component of the inliers, not the raw inlier
        # set — a few stray coplanar clutter points otherwise inflate the raw
        # extent past the cutoff and the board itself gets stripped.
        pc_in = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(inliers.astype(np.float64)))
        labels = np.asarray(pc_in.cluster_dbscan(eps=0.10, min_points=10))
        valid = labels[labels >= 0]
        if len(valid) == 0:
            break
        biggest = inliers[labels == np.bincount(valid).argmax()]
        ext = extent_2d(project_to_plane(biggest, fit_plane(biggest)))
        if ext <= 2.0 * diag:
            break  # largest remaining coherent plane patch is board-scale: stop stripping
        mask = np.ones(len(remaining), dtype=bool)
        mask[idx] = False
        remaining = remaining[mask]
    return remaining.astype(np.float32)


def generate_cluster_after_ground(points: np.ndarray, board: BoardConfig,
                                  big_plane_dist: float = 0.05,
                                  big_plane_min_frac: float = 0.15,
                                  cluster_eps: float = 0.10,
                                  cluster_min_points: int = 30
                                  ) -> list[Candidate]:
    rest = _remove_big_planes(points, board, big_plane_dist,
                              big_plane_min_frac)
    if len(rest) < cluster_min_points:
        return []
    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(rest.astype(np.float64)))
    labels = np.asarray(pc.cluster_dbscan(eps=cluster_eps,
                                          min_points=cluster_min_points))
    out: list[Candidate] = []
    for lbl in np.unique(labels):
        if lbl < 0:
            continue
        cand = plausible_board_patch(rest[labels == lbl], board)
        if cand is not None:
            out.append(cand)
    return out
