"""Geometric diversity of the board placements — the primary quality gate.

Measured on the real field capture, this is the only statistic that separates a degenerate capture
from a good one without inverting or going flat:

    scene 1 (1 placement)   normal span   3.0 deg
    scene 2 (1 placement)   normal span   1.7 deg
    both    (2 placements)  normal span  41.4 deg

Reprojection RMSE inverts (the degenerate capture scores *better*), subset resampling inverts (a
single placement filmed nine times reports +/-0.22 deg), and cond(JtJ) separates by only ~2x. The
normal span separates by 20x.

It is also the only metric that tells the operator what to DO. "cond(JtJ) = 4.4e4" is not
actionable. "Your board normals span 3 degrees; tilt the board" is.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .placements import Placement

# Collection targets, from the literature (Tsai et al., ITSC 2021; ACFR cam_lidar_calibration):
# 10-20 distinct placements, >= 1-2 m depth range, spread across the FoV, maximum yaw/pitch
# variation. These are targets, not hard limits -- nothing here rejects.
MIN_PLACEMENTS = 10
MIN_NORMAL_SPAN_DEG = 20.0
MIN_DEPTH_RANGE_M = 1.5
MIN_LATERAL_SPAN_M = 1.0


@dataclass(frozen=True)
class Diversity:
    n_placements: int
    normal_span_deg: float
    depth_range_m: float
    lateral_span_m: float

    @property
    def is_degenerate(self) -> bool:
        """True when the geometry cannot constrain the extrinsic, whatever the residuals say."""
        return (
            self.n_placements < 2
            or self.normal_span_deg < MIN_NORMAL_SPAN_DEG
            or self.depth_range_m < MIN_DEPTH_RANGE_M
        )

    def shortfalls(self) -> List[str]:
        """Actionable guidance: what is missing, and what to do about it."""
        out: List[str] = []
        if self.n_placements < 2:
            out.append(
                f"only {self.n_placements} distinct board placement(s); a single placement cannot "
                "constrain the extrinsic no matter how many frames are buffered"
            )
        elif self.n_placements < MIN_PLACEMENTS:
            out.append(
                f"{self.n_placements} distinct placements (aim for {MIN_PLACEMENTS}+); "
                "move the board to a new spot and re-capture"
            )
        if self.normal_span_deg < MIN_NORMAL_SPAN_DEG:
            out.append(
                f"board normals span only {self.normal_span_deg:.0f} deg "
                f"(aim for {MIN_NORMAL_SPAN_DEG:.0f}+); vary the board's yaw and pitch"
            )
        if self.depth_range_m < MIN_DEPTH_RANGE_M:
            out.append(
                f"depth range is {self.depth_range_m:.2f} m "
                f"(aim for {MIN_DEPTH_RANGE_M:.1f}+); move the board nearer and farther"
            )
        if self.lateral_span_m < MIN_LATERAL_SPAN_M:
            out.append(
                f"lateral spread is {self.lateral_span_m:.2f} m "
                f"(aim for {MIN_LATERAL_SPAN_M:.1f}+); work the sides of the field of view"
            )
        return out


def _max_pairwise_normal_angle(normals: np.ndarray) -> float:
    """Largest angle between any two board normals, in degrees.

    abs() on the dot product: a plane's normal and its negation are the same orientation, so
    without it a board flipped end-for-end would read as 180 deg of "diversity".
    """
    if len(normals) < 2:
        return 0.0
    worst = 0.0
    for a, b in itertools.combinations(normals, 2):
        cos = abs(float(np.dot(a, b)))
        worst = max(worst, float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))))
    return worst


def compute_diversity(placements: Sequence[Placement]) -> Optional[Diversity]:
    if not placements:
        return None

    positions = np.array([p.position for p in placements], dtype=float)
    normals = np.array([p.normal for p in placements], dtype=float)
    ranges = np.linalg.norm(positions, axis=1)

    return Diversity(
        n_placements=len(placements),
        normal_span_deg=_max_pairwise_normal_angle(normals),
        depth_range_m=float(ranges.max() - ranges.min()),
        # Spread perpendicular to the sensor's forward axis. The board is placed ahead of the
        # LiDAR (x forward), so lateral spread is the y extent.
        lateral_span_m=float(np.ptp(positions[:, 1])) if len(positions) > 1 else 0.0,
    )
