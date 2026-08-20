# H-13 · LiDAR-to-LiDAR calibration overwrites the extrinsic from one board-pose pair

- **Severity:** High
- **Area:** lidar_to_lidar_solver
- **Status:** Open
- **Reported:** 2026-08-19, observed during two-LiDAR field validation
- **Related:** [M-16](./M-16-l2l-pipeline-untested.md), [H-12](./H-12-continuous-solver-forgets-prior-placements.md), [M-13](./archive/M-13-icp-quality-not-propagated.md)

## What happened

LiDAR-to-LiDAR calibration computes a new sensor extrinsic from each synchronized pair of board
poses. The newest result immediately replaces and publishes over the previous transform. Earlier
observations contribute nothing to the current result; the count of synchronized pairs is only a
runtime statistic.

In field use, a single imperfect board-pose pair produces a skewed extrinsic. Moving the board does
not add another constraint to one static calibration. Instead, each new placement produces another
independent transform and becomes the complete published answer.

This path performs direct pose composition:

```text
T_lidar1←lidar2 = T_lidar1←board · inverse(T_lidar2←board)
```

It does not optimize across observations. It has no retained capture collection, covariance
weighting, robust loss, placement-diversity requirement, or Quality Verdict. Each board detector
fits its board model to its own cropped cloud, but the solver does not register one LiDAR's scene
cloud against the other's.

## What I expected

Calibration should estimate one static LiDAR-to-LiDAR extrinsic from multiple independent scene or
Board Placement constraints. A noisy board fit at one instant must not become the complete answer,
and moving the board should improve observability rather than overwrite earlier evidence.

The published result should remain geometrically consistent across the overlapping field of view,
not merely align the most recent board observation.

## Steps to reproduce

1. Launch the two-LiDAR calibration pipeline and visualize both clouds under the published
   LiDAR-to-LiDAR transform.
2. Place the shared calibration board where both LiDARs can detect it and wait for a synchronized
   board-pose pair.
3. Observe the published transform and cloud alignment away from the board; the result can be
   visibly skewed.
4. Move or tilt the board to a distinct placement and wait for the next synchronized pair.
5. Observe that the published transform changes to the new single-pair result instead of jointly
   satisfying both placements.

## Additional context

### Current estimator

For every synchronized non-empty Detection Pair, the solver reads one board pose from each LiDAR,
composes the relative transform algebraically, overwrites its current transform, and publishes it
immediately. A timer republishes that latest value but performs no filtering or estimation.

Synchronization proves only that the two board observations are close in time. It does not prove
that either ICP board fit is accurate, that the placement constrains all six extrinsic degrees of
freedom, or that the derived transform agrees with previous observations.

Operator documentation describes collecting multiple positions and mentions a minimum-detection
gate, but the running solver has no such accumulation or gate. There is also no deterministic
LiDAR-to-LiDAR estimator test that would fail when a later pair erases earlier constraints.

### Target-based multi-placement estimation

Minimum correction can retain synchronized board-pose pairs and optimize one constant transform
against all retained observations:

```text
board pair A ─┐
board pair B ─┼─ weighted robust SE(3) objective ─→ one static extrinsic
board pair C ─┘
```

Both board-pose covariances should influence the observation weight. Repeated captures from one
stationary placement must not dominate through frame count alone; the retained set should be
balanced by distinct Board Placement. Robust pose-grouped residual handling is needed so one bad
board fit cannot poison the complete calibration.

This keeps the calibration target workflow and is the smallest conceptual change. It still aligns
ideal board-pose estimates rather than directly checking the surrounding scene.

### Whole-point-cloud registration

Other LiDAR-to-LiDAR calibration systems instead build a map or accumulated cloud from one LiDAR and
register the other LiDAR's clouds against it. ICP, GICP, FastGICP, and NDT are candidate registration
families. This uses geometry across the overlapping scene rather than one fitted board pose and can
directly penalize the observed whole-cloud skew.

That is a broader workflow, not a drop-in replacement for pose composition. It requires:

- adequate static geometry and field-of-view overlap;
- accurate time synchronization and motion compensation when sensors or scene move;
- dynamic-object rejection;
- an initial transform within the registration method's convergence basin;
- degeneracy detection for scenes dominated by one plane or repeated structure;
- a held-out whole-cloud alignment metric rather than optimizer convergence alone.

The board-derived transform may remain useful as an initializer for full-cloud registration. The
implementation decision should compare placement-balanced target estimation, full-cloud
registration, and a staged board-initialized registration pipeline on the same two-LiDAR recordings
rather than selecting an algorithm name without measurements.

## Acceptance evidence

- Multiple synchronized observations constrain one current static extrinsic; the newest pair cannot
  independently overwrite it.
- Repeated observations from one Board Placement cannot dominate distinct placements through frame
  count alone.
- Board-fit covariance and a robust pose-grouped loss prevent one low-quality fit from controlling
  the result, if the target-based path is retained.
- A deterministic regression test injects one biased pose pair and fails if the published extrinsic
  follows that pair instead of the consistent multi-observation solution.
- Evaluation on the two-LiDAR recordings compares target-based joint estimation with at least one
  whole-cloud registration baseline, including initialization, runtime, convergence failures, and
  held-out cloud alignment.
- Final field validation shows both clouds remain aligned across the overlapping scene and across
  all exercised board placements; alignment at only the calibration board is insufficient.
- Solver reports an explicit Quality Verdict and withholds publication when geometry is
  underconstrained or inconsistent.
