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
    # open3d's RANSAC plane segmentation draws from a global RNG with no
    # per-call seed argument; leaving it unseeded made this generator's
    # candidate set (and therefore detection_rate) change from run to run on
    # identical input, which makes the benchmark numbers unreproducible.
    # Fix the seed once per call for deterministic, repeatable results.
    o3d.utility.random.seed(0)
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
        # Real VLP-32C returns ring-striped point clouds: a large flat plane
        # (e.g. the ground) is not spatially contiguous at eps=0.10 because
        # the gaps *between* rings exceed that at a few metres' range, so it
        # fragments into many small islands and the "biggest island" looks
        # board-scale even though the plane itself is huge. Use a looser eps
        # here (bridges ring gaps) purely to judge big-vs-board-scale; the
        # final candidate clustering below stays at the tighter cluster_eps.
        pc_in = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(inliers.astype(np.float64)))
        labels = np.asarray(pc_in.cluster_dbscan(eps=0.20, min_points=10))
        valid = labels[labels >= 0]
        if len(valid) == 0:
            # This plane's inliers DBSCAN entirely to noise (no component
            # has >= 10 points at eps=0.20) -- an unmeasurable, fragmented
            # patch is not the board, so it's safe to strip and keep looking
            # for other big planes rather than aborting the whole strip loop
            # (a `break` here left every later big plane unstripped).
            mask = np.ones(len(remaining), dtype=bool)
            mask[idx] = False
            remaining = remaining[mask]
            continue
        biggest = inliers[labels == np.bincount(valid).argmax()]
        ext = extent_2d(project_to_plane(biggest, fit_plane(biggest)))
        if ext <= 2.0 * diag:
            break  # largest remaining coherent plane patch is board-scale: stop stripping
        mask = np.ones(len(remaining), dtype=bool)
        mask[idx] = False
        remaining = remaining[mask]
    return remaining.astype(np.float32)


def _merge_coplanar_clusters(points: np.ndarray, labels: np.ndarray,
                             board: BoardConfig,
                             seed_min_points: int = 40,
                             offset_tol: float = 0.02,
                             merge_dist_factor: float = 1.6
                             ) -> list[np.ndarray]:
    """Greedily grow each sizeable DBSCAN cluster with nearby coplanar ones.

    A VLP-32C's ring gaps also cross the board face itself at a few metres'
    range (not just the ground): a single physical 1 m board frequently comes
    back as several separate horizontal-stripe DBSCAN clusters, each
    individually too small/too thin (near-1D, a line not a patch) to pass
    plausible_board_patch. Fixing this by widening cluster_eps globally was
    tried and made things *worse*: it started merging unrelated nearby
    clutter into board-shaped noise instead of just bridging stripe gaps.

    A pairwise normal-similarity merge (an earlier attempt) does not work
    either: a lone ring stripe is itself so close to a 1D line that its own
    SVD-fitted normal is essentially arbitrary (little variance to fix the
    third axis), so two genuinely coplanar stripes routinely fail a
    normal-agreement test against each other. Instead, grow outward from
    each cluster large enough to have a *reliable* plane fit (>= 3 rings
    worth of points) and test candidate stripes by point-to-plane distance
    against that reliable anchor plane -- which sidesteps the small
    cluster's own unstable normal entirely.
    """
    diag = board.side_m * np.sqrt(2.0)
    label_ids = [lbl for lbl in np.unique(labels) if lbl >= 0]
    clusters = {lbl: points[labels == lbl] for lbl in label_ids}
    order = sorted(clusters, key=lambda lbl: -len(clusters[lbl]))
    used: set = set()
    groups: list[np.ndarray] = []
    for seed in order:
        if seed in used:
            continue
        used.add(seed)
        group_pts = clusters[seed]
        if len(group_pts) >= seed_min_points:
            plane = fit_plane(group_pts)
            center = group_pts.mean(axis=0)
            grew = True
            while grew:
                grew = False
                for lbl in order:
                    if lbl in used:
                        continue
                    pts = clusters[lbl]
                    if (np.linalg.norm(pts.mean(axis=0) - center)
                            > merge_dist_factor * diag):
                        continue
                    if np.abs((pts - plane.center) @ plane.normal).mean() \
                            > offset_tol:
                        continue
                    used.add(lbl)
                    group_pts = np.concatenate([group_pts, pts], axis=0)
                    plane = fit_plane(group_pts)  # refit: more points, more
                                                  # stable normal as we grow
                    center = group_pts.mean(axis=0)
                    grew = True
        groups.append(group_pts)
    return groups


def generate_cluster_after_ground(points: np.ndarray, board: BoardConfig,
                                  big_plane_dist: float = 0.05,
                                  big_plane_min_frac: float = 0.08,
                                  cluster_eps: float = 0.15,
                                  cluster_min_points: int = 30
                                  ) -> list[Candidate]:
    # big_plane_min_frac was 0.15 in the synthetic-only design; real VLP-32C
    # scenes carry far more non-ground clutter (background structures, poles,
    # distant returns) than the synthetic ground+wall+blob scene, so the
    # ground plane's share of total points drops to ~0.13-0.14 even though it
    # is still clearly the dominant flat surface. 0.08 keeps stripping real
    # ground/wall planes while still well above what a board-sized patch
    # could ever contribute.
    rest = _remove_big_planes(points, board, big_plane_dist,
                              big_plane_min_frac)
    if len(rest) < cluster_min_points:
        return []
    # cluster_eps was 0.10 in the synthetic-only design. Real VLP-32C rings
    # leave gaps on the board face itself at ~2 m range (not just on the
    # ground), which fragmented a single 1 m board into 2-3 DBSCAN pieces
    # below the plausible-patch extent floor. 0.15 bridges that without
    # merging the board into unrelated nearby clutter in the observed frames.
    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(rest.astype(np.float64)))
    labels = np.asarray(pc.cluster_dbscan(eps=cluster_eps,
                                          min_points=cluster_min_points))
    out: list[Candidate] = []
    for group_pts in _merge_coplanar_clusters(rest, labels, board):
        cand = plausible_board_patch(group_pts, board)
        if cand is not None:
            out.append(cand)
    return out
