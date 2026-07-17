"""Approach C: grow regions of coherent normals (custom BFS; o3d lacks one)."""
from __future__ import annotations

from collections import deque

import numpy as np
import open3d as o3d

from . import Candidate, plausible_board_patch
from ..board_config import BoardConfig


def generate_region_growing(points: np.ndarray, board: BoardConfig,
                            knn: int = 16, angle_deg: float = 12.0,
                            min_region: int = 40) -> list[Candidate]:
    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(points.astype(np.float64)))
    pc.estimate_normals(
        o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    normals = np.asarray(pc.normals)
    tree = o3d.geometry.KDTreeFlann(pc)
    n_pts = len(points)
    cos_thresh = np.cos(np.radians(angle_deg))

    # precompute neighbor lists once
    neighbors = []
    for i in range(n_pts):
        _, idx, _ = tree.search_knn_vector_3d(pc.points[i], knn)
        neighbors.append(np.asarray(idx[1:]))

    visited = np.zeros(n_pts, dtype=bool)
    out: list[Candidate] = []
    for seed in range(n_pts):
        if visited[seed]:
            continue
        region = [seed]
        visited[seed] = True
        queue = deque([seed])
        while queue:
            cur = queue.popleft()
            for nb in neighbors[cur]:
                if visited[nb]:
                    continue
                if abs(normals[cur] @ normals[nb]) >= cos_thresh:
                    visited[nb] = True
                    region.append(int(nb))
                    queue.append(int(nb))
        if len(region) >= min_region:
            cand = plausible_board_patch(points[np.array(region)], board)
            if cand is not None:
                out.append(cand)
    return out
