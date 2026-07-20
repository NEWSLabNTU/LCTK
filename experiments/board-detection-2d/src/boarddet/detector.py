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
from .isolation import isolation_density
from .pose import BoardDetection, board_pose
from .scorer import ScoreResult, score_candidate
from .square_fit import fit_fixed_square

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


def _quad_center(res: ScoreResult) -> np.ndarray:
    """Center seed for `fit_fixed_square` read off an already-scored 2D
    quad's corners (ORIGINAL plane coords -- see `ScoreResult`). Center only:
    the quad's angle is NOT used as a theta seed (see `square_icp` handling
    in `detect` -- the raw-point quad's angle is untrustworthy on sparse
    frames regardless of whether the quad itself was accepted or rejected,
    so `fit_fixed_square` always gets `init_theta=None` and does its own
    full mod-90 sweep instead); the quad's center is still a reasonable
    localization seed since `fit_fixed_square`'s center estimate is
    per-theta closed-form and comparatively robust."""
    return res.corners_2d.mean(axis=0)


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
    best_residual = np.inf
    for cand in cands:
        up_2d = None
        close_height_m = None
        if board.vertical_gap_deg > 0:
            up_2d = _up_2d(cand.plane)
            if up_2d is not None:
                close_height_m = _close_height_m(cand.points, board)
        coords = project_to_plane(cand.points, cand.plane)
        res = score_candidate(coords, board, up_2d=up_2d,
                              close_height_m=close_height_m)

        if board.square_icp:
            # Refine-after-quad (Task 23, corrected): the raw-point quad's
            # angle is untrustworthy on sparse frames PERIOD -- whether the
            # quad was accepted by score_candidate (refine) or rejected
            # outright, including by its own internal stance gate on that
            # same untrustworthy angle (rescue; see
            # stage7-stance-cause.md) -- so it is never used to seed theta.
            # `init_theta=None` always, so `fit_fixed_square` does its own
            # coarse-to-fine full [0, 90 deg) sweep (mod-90 square symmetry,
            # so that range is the whole period): the one mechanism that
            # actually covers the range a bad quad seed can land in. This is
            # only ~37 coarse evals per candidate -- negligible against the
            # ~60ms budget -- and is strictly more robust than any narrower
            # window while not regressing an already-good quad (the sweep
            # still finds the true theta there too).
            # The quad's CENTER (when available) is still used to localize
            # the fit -- center is a robust closed-form estimate per theta,
            # unlike the angle, so seeding it costs nothing.
            seed_center = _quad_center(res) if res is not None \
                else coords.mean(axis=0)
            fit = fit_fixed_square(
                coords, board.side_m, init_center=seed_center,
                init_theta=None)
            if fit is None or fit.residual >= board.square_icp_residual_max:
                continue
            refined_score = 1.0 / (1.0 + fit.residual)
            if res is not None:
                refined_res = dataclasses.replace(
                    res, corners_2d=fit.corners_2d, score=refined_score)
            else:
                # No quad-derived ScoreResult to refine -- build a minimal
                # one; board_pose only reads corners_2d/score off it, and
                # the rest (raster, fill_ratio, ...) are debug-only fields
                # with no real value to report on a rescued candidate.
                refined_res = ScoreResult(
                    score=refined_score, corners_2d=fit.corners_2d,
                    side_lengths=np.full(4, board.side_m), fill_ratio=0.0,
                    angle_err_deg=0.0,
                    raster=np.zeros((1, 1), dtype=np.uint8),
                    origin=np.zeros(2), cell_m=board.cell_m)
            det = board_pose(cand.plane, refined_res)
            det = dataclasses.replace(det, score=refined_score)
            if board.stance_floor > 0:
                if _stance(det.corners_3d) <= board.stance_floor:
                    continue
            if board.isolation:
                density = isolation_density(dn, cand.plane,
                                           det.result.corners_2d)
                if density > board.isolation_max_density:
                    continue
            if fit.residual < best_residual:
                best_residual = fit.residual
                best = det
            continue

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
        if board.isolation:
            density = isolation_density(dn, cand.plane, det.result.corners_2d)
            if density > board.isolation_max_density:
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
