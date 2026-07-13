"""Assemble the quality report, and say something useful about it.

Order matters, and is enforced here:

    frames -> distinct placements -> diversity (the gate) -> residuals -> conditioning -> spread

Diversity is evaluated first and decides the verdict. Everything else is supporting evidence,
because everything else either inverts or goes flat on real data. See the module docstrings.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .conditioning import Conditioning, compute_conditioning
from .diversity import Diversity, compute_diversity
from .placements import Placement, distinct_placements, representative_frames
from .residuals import Residuals, compute_residuals
from .resampling import Spread, compute_spread

#: The project's own accuracy target, from `calibration_judge`'s ground-truth scoring config.
TARGET_ROT_DEG = 2.0
TARGET_TRANS_MM = 50.0

NO_DISTORTION = np.zeros(5)


@dataclass(frozen=True)
class QualityReport:
    placements: List[Placement]
    diversity: Diversity
    residuals: Residuals
    conditioning: Conditioning
    spread: Optional[Spread]
    n_frames: int

    @property
    def n_placements(self) -> int:
        return len(self.placements)

    @property
    def exceeds_target(self) -> bool:
        """Uncertainty is measurably worse than the project's 2 deg / 50 mm target.

        Only meaningful when a spread exists; a missing spread is not evidence of quality.
        """
        if self.spread is None:
            return False
        return (
            self.spread.rot_deg > TARGET_ROT_DEG
            or self.spread.trans_mm > TARGET_TRANS_MM
        )

    @property
    def is_degenerate(self) -> bool:
        return self.diversity.is_degenerate or self.exceeds_target

    def status_line(self) -> str:
        """One line, for `last_solve_status`. No message change needed -- it is already a string."""
        verdict = "DEGENERATE" if self.is_degenerate else "OK"
        parts = [
            verdict,
            f"{self.n_placements} placements ({self.n_frames} frames)",
        ]
        if self.spread is not None:
            parts.append(
                f"uncert +/-{self.spread.rot_deg:.1f}deg +/-{self.spread.trans_mm:.0f}mm"
            )
        else:
            parts.append("uncert n/a (too few placements)")
        parts.append(f"normals {self.diversity.normal_span_deg:.0f}deg")
        parts.append(f"cond {self.conditioning.cond:.0e}")
        parts.append(f"rms {self.residuals.rms_px:.2f}px")
        return " | ".join(parts)

    def warnings(self) -> List[str]:
        """What is wrong and what to do about it. Empty when the capture is sound."""
        if not self.is_degenerate:
            return []

        out = ["Calibration is under-constrained."]

        if self.spread is not None and self.exceeds_target:
            out.append(
                f"  Uncertainty across placement subsets is "
                f"+/-{self.spread.rot_deg:.1f} deg / +/-{self.spread.trans_mm:.0f} mm, "
                f"against a {TARGET_ROT_DEG:.0f} deg / {TARGET_TRANS_MM:.0f} mm target."
            )
        elif self.spread is None:
            out.append(
                f"  Uncertainty cannot be estimated from {self.n_placements} distinct "
                "placement(s); resampling needs at least 4. Note this is NOT a good sign -- "
                "it means there is too little data to even measure the problem."
            )

        for shortfall in self.diversity.shortfalls():
            out.append(f"  {shortfall}")

        out.append(
            f"  Reprojection error ({self.residuals.rms_px:.2f} px) is NOT evidence of quality "
            "here -- a degenerate capture typically scores a BETTER reprojection error than a "
            "good one."
        )
        return out


def build_report(
    object_points_per_frame: Sequence[np.ndarray],
    image_points_per_frame: Sequence[np.ndarray],
    board_poses: Sequence[Tuple[Sequence[float], Sequence[float]]],
    camera_matrix: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> Optional[QualityReport]:
    """Compute the full quality report for a solved extrinsic.

    `board_poses` is one `(position, quaternion)` per frame, parallel to the point arrays. It is
    what lets us collapse frames into distinct placements -- the step without which every downstream
    number is a lie about how much information the capture actually contains.
    """
    if not object_points_per_frame:
        return None

    placements = distinct_placements(board_poses)
    diversity = compute_diversity(placements)
    if diversity is None:
        return None

    residuals = compute_residuals(
        object_points_per_frame, image_points_per_frame, camera_matrix, rvec, tvec
    )
    conditioning = compute_conditioning(residuals.jacobian, residuals.rms_px)

    # Resampling sees ONE frame per distinct placement. Handing it the raw frames is precisely the
    # bug this design exists to avoid: nine frames of a static board would report +/-0.22 deg.
    reps = representative_frames(placements)
    spread = compute_spread(
        [object_points_per_frame[i] for i in reps],
        [image_points_per_frame[i] for i in reps],
        camera_matrix,
    )

    return QualityReport(
        placements=placements,
        diversity=diversity,
        residuals=residuals,
        conditioning=conditioning,
        spread=spread,
        n_frames=len(object_points_per_frame),
    )


def solve_pnp(
    object_points: np.ndarray, image_points: np.ndarray, camera_matrix: np.ndarray
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """SQPnP with zero distortion -- the corners on the wire are already rectified (C-03, H-08)."""
    ok, rvec, tvec = cv2.solvePnP(
        np.asarray(object_points, np.float64),
        np.asarray(image_points, np.float64),
        camera_matrix,
        NO_DISTORTION,
        flags=cv2.SOLVEPNP_SQPNP,
    )
    return (rvec, tvec) if ok else (None, None)
