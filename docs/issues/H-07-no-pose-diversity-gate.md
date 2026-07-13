# H-07 · Degenerate pose sets are accepted silently; extrinsic is under-constrained

- **Severity:** High
- **Area:** advanced_extrinsic_solver
- **Status:** Partially fixed (2026-07-13) — geometric diversity gate landed; conditioning metric is [H-09](./H-09-no-extrinsic-quality-metric.md)
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:**
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:109` (`min_poses_required` default `2`)
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:474-476` (buffer append, no similarity check)
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1008-1090` (`_solve_from_buffer`)
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1290-1331` (`_create_point_correspondences`)

## Problem

This is the root cause of the "overlay is correct on the board but the background points are
tilted" symptom.

### What the correspondences actually are

The 3D points fed to PnP are **not** LiDAR returns. `_create_point_correspondences` takes the
*ideal* ArUco corner coordinates from the config, and rigidly transforms them by the board pose
that ICP fitted to the point cloud:

```python
# main.py:1298-1323
board_rotation = R.from_quat(board_detection.orientation).as_matrix().astype(np.float32)
board_position = np.array(board_detection.position, dtype=np.float32)
board_frame_corners = self._compute_multi_marker_corners()   # ideal model corners, z = 0
...
world_corners = (board_rotation @ local_corners.T).T + board_position
```

So every object point lies on a 500 mm coplanar ArUco patch, wherever the board happened to be.
All 16 corners of one pose derive from a single rigid `T_board`, so their errors are perfectly
correlated: **one buffered pose carries ~6 DoF of information, not 32 independent residuals.**

### Why the background tilts

Perturb the extrinsic by `(δθ, δt)`. A point `p` moves by `δθ × p + δt`. Write `p = p̄ + Δ`,
where `p̄` is the centroid of *all* accumulated correspondences:

```
motion(p) = (δθ × p̄ + δt) + δθ × Δ
```

Pick `δt = −δθ × p̄`. The first term vanishes identically. The residual is `δθ × Δ` —
proportional only to how far the correspondences spread from **their own centroid**.

**A rotation about the correspondence centroid is a near-null direction of the reprojection
cost, damped only by the spread of the correspondence set.** Reprojection error on the board
barely moves; a background point at distance `Δ` from that centroid swings by
`f·|δθ × Δ|/Z` pixels. Board stays glued, background tilts. This is geometry, not a coding
error — but the solver has no defence against it.

Corollary for how data is collected: **buffering more frames of the same board placement
reduces noise variance but does not change `Δ`** — the conditioning is unchanged. Only *new
board placements at new depths and orientations* shrink the null direction.

### What the code does about it

Nothing. The buffer accepts anything:

```python
# main.py:474-476
# Add to buffer (no similarity check - allow multiple detections to average out)
with self.lock:
    self.detection_buffer.append((aruco_msg, board_msg))
```

and the only gate before solving is a count, defaulting to **2**
(`main.py:109`, not overridden in `calibrate.launch.py`).

## Failure scenario

Operator holds the board still, hits *Add* twenty times in the interactive controller, and the
solver reports `"Calibration successful"` with 320 correspondences — from a correspondence set
that is effectively a single coplanar 500 mm patch. The extrinsic is determined only up to a
rotation about that patch, and the resulting transform tilts the entire background. Nothing in
the pipeline warns, because nothing measures conditioning ([H-09](./H-09-no-extrinsic-quality-metric.md)).

## Suggested fix

Gate the solve on the *geometry* of the accumulated set, not its cardinality:

- **Board-normal spread** — require ≥3 board plane normals mutually separated by >20–30°.
  A set of near-parallel normals cannot constrain rotation.
- **Depth spread** — require the board centroids to span ≥1.5–2 m in range.
- **Image coverage** — require the ArUco patches to cover a meaningful fraction of the image
  area, not just the centre. (Blocked on [C-03](./C-03-double-undistortion.md): today, border
  poses carry the largest systematic bias.)
- **Conditioning** — compute `cond(JᵀJ)` for the PnP Jacobian over the accumulated set and
  refuse / loudly warn above a threshold. See [H-09](./H-09-no-extrinsic-quality-metric.md).
- Raise `min_poses_required` and expose the diversity state in `GetBufferStatus` /
  the interactive controller, so the operator is told *where to put the board next*.

Collection guidance in the literature converges on 10–20 poses, ≥1–2 m depth range, spread
across the FoV width, with maximum variation in board yaw/pitch (ACFR `cam_lidar_calibration`;
Tsai et al., ITSC 2021). Full design in
[docs/roadmap/phase-5-stable-extrinsic-solution.md](../roadmap/phase-5-stable-extrinsic-solution.md).

## Resolution (2026-07-13) — geometric diversity gate

`advanced_extrinsic_solver` now computes the geometric diversity of the accumulated
buffer before solving (`_compute_pose_diversity`) and gates on it:

- **Board-normal spread** — the largest angle between any two board plane normals
  (unsigned, so a normal and its flip are one plane). Near-parallel normals cannot
  constrain the extrinsic rotation.
- **Depth range** — the span of board-centroid ranges from the sensor.

`_solve_from_buffer` logs both numbers every solve and, when either is below its
threshold (`min_normal_spread_deg` default 20°, `min_depth_range_m` default 1.0 m),
emits a loud, actionable warning explaining that the poses are near-coplanar / at one
depth so the background will tilt, and that *more frames of the same placement will
not help*. Set `enforce_pose_diversity:=true` to refuse the solve outright instead of
warning (default is warn-only so the shipped single-placement sample demo still runs).

This resolves the "accepted **silently**" part of the finding and gives the operator
the "where to put the board next" signal. The conditioning number `cond(JᵀJ)` and the
reprojection/cross-validation metrics remain [H-09](./H-09-no-extrinsic-quality-metric.md);
raising `min_poses_required` beyond a warning and surfacing diversity in
`GetBufferStatus`/the controller is left to the H-09 metric surface to avoid two
changes to the same service.

Verified: `tmp/test_h07_pose_diversity.py` shows a still-board 20-add buffer reads
0° / 0 m (gated) while a 4-pose spread buffer reads 47.6° / 1.87 m (passes); full
`just build` + `just test` (273 Rust, 48 Python) green.
