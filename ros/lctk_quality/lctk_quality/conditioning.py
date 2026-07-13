"""Conditioning of the PnP normal equations.

`cond(JtJ)` collapses the near-null-direction story of H-07 into one number: a rotation about the
correspondence centroid barely changes the reprojection cost, so `JtJ` is nearly singular along it.

The Jacobian is free -- `cv2.projectPoints` already returns it, and its first six columns are
exactly d(projection)/d(rvec, tvec) (verified against finite differences to ~1e-6).

**Two honest caveats, both measured.**

1. `cond(JtJ)` separates cleanly in simulation (4.6e4 degenerate vs 2.4e2 well-spread, a 190x gap)
   but only by ~2x on the real field capture (4.4e4 vs 2.2e4). It is a supporting signal, not the
   discriminator. The discriminator is geometric diversity -- see `diversity.py`.

2. The per-DoF sigma from `Sigma ~ sigma^2 (JtJ)^-1` is the Cramer-Rao bound *under the assumption
   that all noise is in the pixels*. It is not: the 3D points are model corners pushed through an
   ICP board pose carrying cm-level error (M-13). Measured, it under-reports by ~4x -- it claimed
   1.22 deg where the true error was 5.07 deg. Useful as a relative signal; do not present it to an
   operator as "your calibration is accurate to X".
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Above this, the normal equations are ill-conditioned enough to be worth mentioning. Calibrated
#: against the measurements above; it is a reporting threshold, not a rejection threshold.
ILL_CONDITIONED = 1e4


@dataclass(frozen=True)
class Conditioning:
    cond: float

    #: Per-DoF standard deviation. OPTIMISTIC -- see the module docstring.
    sigma_rot_deg: np.ndarray
    sigma_trans_mm: np.ndarray

    @property
    def is_ill_conditioned(self) -> bool:
        return self.cond > ILL_CONDITIONED

    @property
    def worst_sigma_rot_deg(self) -> float:
        return float(self.sigma_rot_deg.max())

    @property
    def worst_sigma_trans_mm(self) -> float:
        return float(self.sigma_trans_mm.max())


def compute_conditioning(jacobian: np.ndarray, rms_px: float) -> Conditioning:
    """`cond(JtJ)` and the (optimistic) Cramer-Rao per-DoF sigma."""
    n_residuals = jacobian.shape[0]  # 2 per corner
    JtJ = jacobian.T @ jacobian

    cond = float(np.linalg.cond(JtJ))

    # sigma^2 estimated from the residuals, with the 6 solved DoF removed.
    dof = max(n_residuals - 6, 1)
    sigma_sq = (rms_px**2) * n_residuals / dof

    try:
        covariance = sigma_sq * np.linalg.inv(JtJ)
        sd = np.sqrt(np.abs(np.diag(covariance)))
    except np.linalg.LinAlgError:
        # Exactly singular: the extrinsic is not determined at all.
        sd = np.full(6, np.inf)

    return Conditioning(
        cond=cond,
        sigma_rot_deg=np.degrees(sd[:3]),
        sigma_trans_mm=sd[3:] * 1000.0,
    )
