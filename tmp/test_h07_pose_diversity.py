#!/usr/bin/env python3
"""H-07 verification: pose-diversity metric separates degenerate vs good buffers.

Reproduces the exact geometry of advanced_extrinsic_solver._compute_pose_diversity
and checks:
1. A single-placement buffer (same orientation, same depth) reads ~0 deg spread and
   ~0 m depth range -> below the gate thresholds (would warn/refuse).
2. A spread buffer (varied board orientations + depths) reads a large normal spread
   and a wide depth range -> passes the gate.
"""
import sys
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation as R


def compute_pose_diversity(board_detections):
    """Copy of the node's _compute_pose_diversity geometry."""
    if len(board_detections) < 2:
        return 0.0, 0.0
    normals, depths = [], []
    for det in board_detections:
        rot = R.from_quat(det.orientation).as_matrix()
        normals.append(rot @ np.array([0.0, 0.0, 1.0]))
        depths.append(float(np.linalg.norm(det.position)))
    max_angle = 0.0
    for i in range(len(normals)):
        for j in range(i + 1, len(normals)):
            c = min(1.0, max(-1.0, abs(float(np.dot(normals[i], normals[j])))))
            max_angle = max(max_angle, float(np.degrees(np.arccos(c))))
    return max_angle, max(depths) - min(depths)


def board(pos, rpy_deg):
    q = R.from_euler("xyz", rpy_deg, degrees=True).as_quat()  # (x,y,z,w)
    return SimpleNamespace(position=tuple(pos), orientation=tuple(q))


def main():
    MIN_SPREAD, MIN_DEPTH = 20.0, 1.0

    # 1. Degenerate: operator holds the board still, adds 20 frames.
    degenerate = [board([2.6, 0.0, 0.35], [0, 0, 0]) for _ in range(20)]
    s, d = compute_pose_diversity(degenerate)
    assert s < MIN_SPREAD and d < MIN_DEPTH, f"degenerate not caught: spread={s}, depth={d}"
    print(f"[1] degenerate buffer: spread={s:.1f} deg, depth range={d:.2f} m -> GATED")

    # 2. Good: board at varied yaw/pitch and depths across the FoV.
    good = [
        board([2.0, -0.8, 0.3], [0, 0, 0]),
        board([2.6, 0.0, 0.35], [0, 25, 0]),
        board([3.2, 0.7, 0.4], [0, -20, 15]),
        board([4.0, -0.3, 0.5], [10, 30, -10]),
    ]
    s, d = compute_pose_diversity(good)
    assert s >= MIN_SPREAD and d >= MIN_DEPTH, f"good buffer rejected: spread={s}, depth={d}"
    print(f"[2] good buffer:       spread={s:.1f} deg, depth range={d:.2f} m -> PASSES")

    # Sanity: a flipped normal is treated as the same plane (unsigned).
    flip = [board([2.6, 0, 0.35], [0, 0, 0]), board([2.6, 0, 0.35], [180, 0, 0])]
    s, _ = compute_pose_diversity(flip)
    assert s < 1.0, f"flipped normal should read ~0 spread, got {s}"
    print(f"[3] flipped normal:    spread={s:.2f} deg -> treated as same plane")

    print("\nH-07 PASS: diversity metric separates degenerate from good buffers")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # noqa: BLE001
        print(f"H-07 FAIL: {e}")
        sys.exit(1)
