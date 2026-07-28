"""Approach C: grow regions of coherent normals (custom BFS; o3d lacks one)."""
from __future__ import annotations

from collections import deque

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from . import Candidate, plausible_board_patch
from ..board_config import BoardConfig
from ..reject import RejectReason


def generate_region_growing(points: np.ndarray, board: BoardConfig,
                            knn: int = 16, angle_deg: float = 12.0,
                            min_region: int = 40,
                            rejects: list[RejectReason] | None = None) -> list[Candidate]:
    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(points.astype(np.float64)))
    pc.estimate_normals(
        o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    normals = np.asarray(pc.normals)
    n_pts = len(points)
    cos_thresh = np.cos(np.radians(angle_deg))

    # Precompute neighbour lists once, in a single vectorized batch query.
    #
    # scipy's cKDTree rather than open3d's KDTreeFlann: the latter's
    # search_knn_vector_3d segfaults outright when open3d is running against
    # numpy 2.x (open3d 0.18 declares `numpy>=1.18.0` but is compiled against
    # the numpy 1.x C ABI, which numpy 2 changed). Every other open3d call
    # this package makes -- voxel_down_sample, segment_plane, cluster_dbscan,
    # estimate_normals -- is unaffected; only the KD-tree search is. Both
    # trees do exact KNN, so neighbour sets are equivalent.
    #
    # Column 0 of a self-query is the point itself, hence [:, 1:].
    tree = cKDTree(points.astype(np.float64))
    _, idx = tree.query(points.astype(np.float64), k=knn)
    neighbors = [row for row in np.atleast_2d(idx)[:, 1:]]

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
            cand = plausible_board_patch(points[np.array(region)], board,
                                         rejects=rejects)
            if cand is not None:
                out.append(cand)
    return out
