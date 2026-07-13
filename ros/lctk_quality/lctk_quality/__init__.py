"""Quality metrics for the LiDAR-camera extrinsic solve (H-09).

The pipeline could not tell you whether a calibration was any good. A degenerate solve and a sound
one both reported ``"Calibration successful"``. This package produces the numbers that distinguish
them.

The headline finding, which the design was twice rebuilt around: **the obvious metrics lie.**

- **Reprojection error inverts.** A degenerate capture scores a *better* RMSE than a good one, and
  the single-pose solve `just demo` produces scores the best of all while being the worst-
  conditioned thing the pipeline can make. Report it; never rank on it.
- **Resampled uncertainty inverts too, if you feed it frames.** Nine frames of a *static* board
  report +/-0.22 deg / +/-9 mm -- the most confident number in the suite -- for a capture that
  cannot constrain the extrinsic at all. Repeated frames of one placement carry correlated error,
  so every subset agrees. Resampling measures variance; a degenerate capture has low variance and
  high bias.

So the order is fixed, and the module boundaries enforce it:

    frames -> DISTINCT PLACEMENTS -> diversity (the gate) -> residuals -> conditioning -> spread

`N` is the number of distinct board placements, never the frame count. Geometric diversity is the
primary signal -- it is the only one that separates cleanly on real field data (board-normal span
1.7-3.0 deg degenerate vs 41.4 deg diverse, a 20x gap) and the only one that tells the operator what
to do next.

Nothing here rejects a calibration. It reports, and it warns.
"""

from .conditioning import Conditioning, compute_conditioning
from .diversity import Diversity, compute_diversity
from .placements import Placement, distinct_placements, representative_frames
from .report import QualityReport, build_report, solve_pnp
from .resampling import Spread, compute_spread
from .residuals import Residuals, compute_residuals

__all__ = [
    "Conditioning",
    "Diversity",
    "Placement",
    "QualityReport",
    "Residuals",
    "Spread",
    "build_report",
    "compute_conditioning",
    "compute_diversity",
    "compute_residuals",
    "compute_spread",
    "distinct_placements",
    "representative_frames",
    "solve_pnp",
]
