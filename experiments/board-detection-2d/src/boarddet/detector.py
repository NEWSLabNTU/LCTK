"""Glue: downsample -> candidate generator -> shared scorer -> best pose."""
from __future__ import annotations

import dataclasses
import time
from dataclasses import dataclass

import numpy as np

from .board_config import BoardConfig
from .candidates.cluster_after_ground import generate_cluster_after_ground
from .candidates.ransac_iterative import generate_ransac_iterative
from .candidates.region_growing import generate_region_growing
from .geometry import PlaneModel, downsample, project_to_plane
from .pose import BoardDetection, board_pose
from .scorer import score_candidate

GENERATORS = {
    "a": generate_ransac_iterative,
    "b": generate_cluster_after_ground,
    "c": generate_region_growing,
}


@dataclass
class DetectOutcome:
    detection: BoardDetection | None
    timings_ms: dict[str, float]
    n_candidates: int
    best_rejected: BoardDetection | None = None


_UP = np.array([0.0, 0.0, 1.0])
_MIN_UP_PROJ = 0.2  # below this, the plane is near-horizontal: no privileged
                    # "up" stripe direction, fall back to the isotropic kernel


def _up_2d(plane: PlaneModel) -> np.ndarray | None:
    """Direction in plane (u, v) coords along which horizontal ring stripes
    are separated: the projection of world +z onto the plane's in-plane
    basis, normalized. None when the plane is near-horizontal (u, v then
    span a ~horizontal patch, so +z projects to near-zero in-plane and there
    is no meaningful "up" direction to rotate stripes onto)."""
    proj = np.array([_UP @ plane.u, _UP @ plane.v])
    norm = np.linalg.norm(proj)
    if norm < _MIN_UP_PROJ:
        return None
    return proj / norm


def _close_height_m(cand_points: np.ndarray, board: BoardConfig) -> float:
    """Physical vertical closing reach: twice the mean horizontal range of
    the candidate's points times the worst-case adjacent-channel vertical
    angular gap (board.vertical_gap_deg)."""
    horiz_range = np.hypot(cand_points[:, 0], cand_points[:, 1])
    return float(2.0 * horiz_range.mean()
                * np.tan(np.radians(board.vertical_gap_deg)))


def _stance(corners_3d: np.ndarray) -> float:
    """Diamond-stance score: how gravity-aligned is either diagonal.

    corners_3d is CCW-ordered (see pose.board_pose), so corners[2]-corners[0]
    and corners[3]-corners[1] are the two diagonals. A board standing on a
    corner has one diagonal ~vertical (stance ~1); an axis-aligned flat
    panel has both diagonals at ~45 deg off vertical (stance ~0.71).
    """
    d1 = corners_3d[2] - corners_3d[0]
    d2 = corners_3d[3] - corners_3d[1]
    d1 = d1 / np.linalg.norm(d1)
    d2 = d2 / np.linalg.norm(d2)
    return float(max(abs(d1 @ _UP), abs(d2 @ _UP)))


def detect(points: np.ndarray, board: BoardConfig, generator: str,
           voxel: float = 0.03) -> DetectOutcome:
    gen = GENERATORS[generator]
    t0 = time.perf_counter()
    dn = downsample(points, voxel)
    t1 = time.perf_counter()
    # Generator B alone takes anisotropic clustering tolerance; A and C keep
    # their stage-1 signatures (gen(points, board)) unchanged this stage, so
    # this is an explicit special-case rather than a shared kwarg.
    if generator == "b":
        cands = gen(dn, board, vertical_gap_deg=board.vertical_gap_deg)
    else:
        cands = gen(dn, board)
    t2 = time.perf_counter()
    best: BoardDetection | None = None
    best_rejected: BoardDetection | None = None
    for cand in cands:
        up_2d = None
        close_height_m = None
        if board.vertical_gap_deg > 0:
            up_2d = _up_2d(cand.plane)
            if up_2d is not None:
                close_height_m = _close_height_m(cand.points, board)
        res = score_candidate(project_to_plane(cand.points, cand.plane),
                              board, up_2d=up_2d,
                              close_height_m=close_height_m)
        if res is None:
            continue
        det = board_pose(cand.plane, res)
        if board.stance_weight > 0:
            stance = _stance(det.corners_3d)
            w = board.stance_weight
            blended = res.score * ((1 - w) + w * stance)
            det = dataclasses.replace(det, score=blended)
        if det.score < board.min_score:
            if best_rejected is None or det.score > best_rejected.score:
                best_rejected = det
            continue
        if best is None or det.score > best.score:
            best = det
    if best is not None:
        best_rejected = None
    t3 = time.perf_counter()
    return DetectOutcome(
        detection=best,
        timings_ms={
            "downsample": (t1 - t0) * 1e3,
            "candidates": (t2 - t1) * 1e3,
            "scoring": (t3 - t2) * 1e3,
            "total": (t3 - t0) * 1e3,
        },
        n_candidates=len(cands),
        best_rejected=best_rejected,
    )
