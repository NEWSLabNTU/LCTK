"""Uncertainty by resampling over DISTINCT board placements.

Re-solve the extrinsic from every C(N, k) subset of placements and take the spread of the answers.
This is Tsai et al.'s construction (ITSC 2021), and it yields a covariance with **no ground truth**.
At the N of a real capture it is free: C(10, 3) = 120 solves, milliseconds.

**This module must be handed DISTINCT placements, never raw frames**, and it refuses to produce a
number below `MIN_PLACEMENTS_FOR_SPREAD`. That refusal is the whole point.

Measured on the real field capture: scene 2 is one board placement filmed nine times. Handed those
nine frames as if they were nine poses, this computation returns **+/-0.22 deg / +/-9 mm** -- the
most confident number in the entire metric suite -- for a capture that cannot constrain the
extrinsic at all. Repeated frames of a static board carry correlated error, so every subset returns
nearly the same answer.

Resampling measures **variance**. A degenerate capture has low variance and high **bias**. It is
therefore silent about exactly the failure it looks like it should catch, and it will happily
manufacture confidence from duplication. Hence: dedupe first, and refuse when N is too small.
"""

from __future__ import annotations

import itertools
from collections.abc import Sequence
from dataclasses import dataclass

import cv2
import numpy as np

NO_DISTORTION = np.zeros(5)

#: Subsets of this many placements. 3 is the minimum for a non-degenerate PnP over planar targets.
SUBSET_SIZE = 3

#: Below this many DISTINCT placements, resampling is not merely noisy -- it is misleading, because
#: the subsets overlap almost completely. Refuse rather than mislead.
MIN_PLACEMENTS_FOR_SPREAD = 4

#: Cap the enumeration. C(20,3) = 1140; beyond that, sample. Real captures never come close.
MAX_SUBSETS = 2000


@dataclass(frozen=True)
class Spread:
    """Empirical spread of the solved extrinsic across placement subsets."""

    rot_deg: float
    trans_mm: float
    n_subsets: int
    n_placements: int


def compute_spread(
    object_points_per_placement: Sequence[np.ndarray],
    image_points_per_placement: Sequence[np.ndarray],
    camera_matrix: np.ndarray,
    subset_size: int = SUBSET_SIZE,
) -> Spread | None:
    """Spread of the extrinsic over all C(N, k) subsets of DISTINCT placements.

    Returns `None` -- deliberately, rather than a falsely confident number -- when there are too few
    distinct placements for the result to mean anything.
    """
    n = len(object_points_per_placement)
    if n < MIN_PLACEMENTS_FOR_SPREAD or n <= subset_size:
        return None

    combos = list(itertools.combinations(range(n), subset_size))
    if len(combos) > MAX_SUBSETS:
        step = len(combos) // MAX_SUBSETS + 1
        combos = combos[::step]

    rvecs, tvecs = [], []
    for subset in combos:
        obj = np.vstack([object_points_per_placement[i] for i in subset]).astype(
            np.float64
        )
        img = np.vstack([image_points_per_placement[i] for i in subset]).astype(
            np.float64
        )
        if len(obj) < 4:
            continue
        ok, rvec, tvec = cv2.solvePnP(
            obj, img, camera_matrix, NO_DISTORTION, flags=cv2.SOLVEPNP_SQPNP
        )
        if ok:
            rvecs.append(np.degrees(rvec.ravel()))
            tvecs.append(tvec.ravel() * 1000.0)

    if len(rvecs) < 2:
        return None

    return Spread(
        rot_deg=float(np.std(np.array(rvecs), axis=0).max()),
        trans_mm=float(np.std(np.array(tvecs), axis=0).max()),
        n_subsets=len(rvecs),
        n_placements=n,
    )
