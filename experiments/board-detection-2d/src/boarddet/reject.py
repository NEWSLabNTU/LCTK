"""Structured reject reasons for the board detector.

Zero intra-package imports on purpose: this module is imported by scorer,
candidates, and detector, and must not create an import cycle. A gate records
*why* it rejected without changing its accept/reject decision or return type;
the collector is a side channel threaded in as an optional `rejects` list.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class Stage(IntEnum):
    # generation band (cluster -> candidate), gapped below the scorer band
    NO_CLUSTERS = 0        # generator emitted zero clusters at all
    PATCH_POINTS = 1       # patch < _MIN_PATCH_POINTS
    PATCH_FLATNESS = 2     # plane_rms > flatness_rms_max
    PATCH_EXTENT = 3       # extent outside [0.5*side, 1.8*diag]
    # scorer band
    MIN_POINTS = 11        # coords < _MIN_POINTS
    RASTER_SIZE = 12       # raster > 4000 px
    MINAREA_SIZE = 13      # minAreaRect side too small / no contour
    SIZE_GATE = 14         # coarse mean side out of 2*side_tol band
    STRICT_SQUARENESS = 15  # max corner angle dev > 8 deg
    STANCE_2D = 16         # 2D diamond stance <= stance_floor
    EDGE_SUPPORT = 17      # min side support < edge_support_min
    SIDE_ERR = 18          # |mean side - side_m| > side_tol*side_m
    # detector band
    SQUARE_FIT = 21        # icp: fit None or residual >= square_icp_residual_max
    MIN_SCORE = 22         # non-icp: det.score < min_score
    STANCE_3D = 23         # icp: 3D stance <= stance_floor
    ISOLATION = 24         # both paths: density > isolation_max_density


@dataclass(frozen=True)
class RejectReason:
    stage: Stage
    gate: str
    param: str | None
    value: float | None
    threshold: float | tuple[float, float] | None
    margin: float


def _safe_div(num: float, den: float) -> float:
    return 0.0 if den == 0 else num / den


def upper(stage: Stage, gate: str, param: str | None,
          value: float, thr: float) -> RejectReason:
    """Gate that rejects when value > thr."""
    return RejectReason(stage, gate, param, float(value), float(thr),
                        max(0.0, _safe_div(float(value) - float(thr), float(thr))))


def lower(stage: Stage, gate: str, param: str | None,
          value: float, thr: float) -> RejectReason:
    """Gate that rejects when value < thr."""
    return RejectReason(stage, gate, param, float(value), float(thr),
                        max(0.0, _safe_div(float(thr) - float(value), float(thr))))


def band(stage: Stage, gate: str, param: str | None,
         value: float, lo: float, hi: float) -> RejectReason:
    """Gate that rejects when value is outside (lo, hi)."""
    v, lo, hi = float(value), float(lo), float(hi)
    dist = (lo - v) if v < lo else (v - hi)
    half = (hi - lo) / 2.0
    return RejectReason(stage, gate, param, v, (lo, hi),
                        max(0.0, _safe_div(dist, half)))


def furthest(rejects: list[RejectReason]) -> RejectReason | None:
    """The reject that reached the highest stage; ties keep the first seen."""
    best: RejectReason | None = None
    for r in rejects:
        if best is None or r.stage > best.stage:
            best = r
    return best
