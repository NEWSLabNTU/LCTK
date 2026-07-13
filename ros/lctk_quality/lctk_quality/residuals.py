"""Reprojection residuals.

**Report these. Never rank on them, and never show them alone.**

Reprojection error does not merely fail to detect a degenerate calibration — it *inverts*. Measured:

    synthetic, degenerate (board held still)   8.77 px   true error 5.07 deg / 230 mm
    synthetic, well-spread                    10.88 px   true error 0.38 deg /  19 mm
    real, scene 2 (1 placement)                3.46 px   degenerate
    real, both scenes (2 placements)           8.12 px   the only usable set

The degenerate capture scores the *better* RMSE in both cases, and the single-pose solve that
`just demo` produces scores the best of all (0.125 px) while being the worst-conditioned thing the
pipeline can make. A low reprojection error is a necessary condition for a good calibration and
nothing more; used as a quality score it points the wrong way.

It is still worth computing: it catches gross blunders (a permuted corner order, a mis-solved pose),
and a *large* value is genuinely informative. It is only the small values that lie.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import cv2
import numpy as np

NO_DISTORTION = np.zeros(5)


@dataclass(frozen=True)
class Residuals:
    rms_px: float
    max_px: float
    per_pose_rms_px: List[float]

    #: The PnP Jacobian, 2N x 6, d(projection)/d(rvec, tvec). Kept because `conditioning` needs it
    #: and `cv2.projectPoints` hands it to us for free.
    jacobian: np.ndarray


def compute_residuals(
    object_points_per_pose: Sequence[np.ndarray],
    image_points_per_pose: Sequence[np.ndarray],
    camera_matrix: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> Residuals:
    """Per-corner and per-pose reprojection error for a solved extrinsic.

    Per-pose is the meaningful unit: the 16 corners of one pose are all generated from a single
    rigid board pose, so their errors are correlated. They stand or fall together.
    """
    object_points = np.vstack(object_points_per_pose).astype(np.float64)
    image_points = np.vstack(image_points_per_pose).astype(np.float64)

    projected, jacobian = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, NO_DISTORTION
    )
    err = projected.reshape(-1, 2) - image_points
    dist = np.linalg.norm(err, axis=1)

    per_pose: List[float] = []
    start = 0
    for pose_obj in object_points_per_pose:
        stop = start + len(pose_obj)
        per_pose.append(float(np.sqrt((dist[start:stop] ** 2).mean())))
        start = stop

    return Residuals(
        rms_px=float(np.sqrt((dist**2).mean())),
        max_px=float(dist.max()),
        per_pose_rms_px=per_pose,
        # Columns 0..5 are d/d(rvec) and d/d(tvec); the rest are the intrinsics, which are fixed.
        jacobian=jacobian[:, :6],
    )
