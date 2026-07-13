"""Collapse frames into DISTINCT board placements.

This runs before every other metric, and it is not an optimisation — it is a correctness
requirement, learned the hard way.

Measured on the real field capture (`data/2022-10-14-otobrite-calibration`): scene 2 is a single
board placement filmed nine times. Treated as N = 9 independent poses, subset resampling reports an
uncertainty of **±0.22 deg / ±9 mm** — the most confident number the metric can produce — for a
capture that is completely degenerate.

The reason is that repeated frames of a *static* board carry highly *correlated* error: the same
points, the same systematic ICP bias. Every subset therefore returns nearly the same answer.
Resampling measures **variance**, and a degenerate capture has low variance and high **bias**.

So: N is the number of distinct board placements, never the number of frames. Buffering the same
board a thousand times is not more information, and any metric that believes otherwise will be most
confident exactly when the calibration is worst.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

# Two placements closer than this in position AND orientation are the same placement.
# 5 cm is well above the LiDAR's ~3 cm range noise, so it does not split a static board into
# several placements; 5 deg likewise for orientation.
DEFAULT_POSITION_TOL_M = 0.05
DEFAULT_ORIENTATION_TOL_DEG = 5.0


@dataclass(frozen=True)
class Placement:
    """One distinct board placement, and the frames that observed it."""

    position: Tuple[float, float, float]
    normal: Tuple[float, float, float]
    frame_indices: Tuple[int, ...]

    @property
    def n_frames(self) -> int:
        return len(self.frame_indices)

    @property
    def range_m(self) -> float:
        return float(np.linalg.norm(self.position))


def board_normal(quaternion: Sequence[float]) -> np.ndarray:
    """The board plane normal: the local z axis of the board pose. Quaternion is (x, y, z, w)."""
    from scipy.spatial.transform import Rotation

    return Rotation.from_quat(np.asarray(quaternion, dtype=float)).as_matrix()[:, 2]


def _same_placement(
    pos_a: np.ndarray,
    nrm_a: np.ndarray,
    pos_b: np.ndarray,
    nrm_b: np.ndarray,
    position_tol_m: float,
    orientation_tol_deg: float,
) -> bool:
    if float(np.linalg.norm(pos_a - pos_b)) > position_tol_m:
        return False
    # abs(): the board is a plane, so a normal and its negation describe the same orientation.
    cos = abs(float(np.dot(nrm_a, nrm_b)))
    angle = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))
    return bool(angle <= orientation_tol_deg)


def distinct_placements(
    board_poses: Sequence[Tuple[Sequence[float], Sequence[float]]],
    position_tol_m: float = DEFAULT_POSITION_TOL_M,
    orientation_tol_deg: float = DEFAULT_ORIENTATION_TOL_DEG,
) -> List[Placement]:
    """Group `(position, quaternion)` board poses into distinct placements, in first-seen order.

    Greedy single-pass clustering against placement representatives. The clusters are tiny (a
    handful of placements) so nothing cleverer is warranted.
    """
    reps: List[Tuple[np.ndarray, np.ndarray, List[int]]] = []

    for i, (position, quaternion) in enumerate(board_poses):
        pos = np.asarray(position, dtype=float)
        nrm = board_normal(quaternion)

        for rep_pos, rep_nrm, members in reps:
            if _same_placement(
                pos, nrm, rep_pos, rep_nrm, position_tol_m, orientation_tol_deg
            ):
                members.append(i)
                break
        else:
            reps.append((pos, nrm, [i]))

    return [
        Placement(
            position=(float(pos[0]), float(pos[1]), float(pos[2])),
            normal=(float(nrm[0]), float(nrm[1]), float(nrm[2])),
            frame_indices=tuple(members),
        )
        for pos, nrm, members in reps
    ]


def representative_frames(placements: Sequence[Placement]) -> List[int]:
    """One frame index per distinct placement — the first observed.

    Deliberately *not* an average over the placement's frames: averaging would suppress the
    per-frame noise and make the resampling look even more confident, which is the trap this module
    exists to close.
    """
    return [p.frame_indices[0] for p in placements]
