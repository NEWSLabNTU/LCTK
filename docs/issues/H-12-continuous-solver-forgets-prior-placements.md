# H-12 · Continuous LiDAR-camera calibration forgets prior board placements

- **Severity:** High
- **Area:** lidar_to_camera_solver
- **Status:** Open
- **Reported:** 2026-08-19, observed during live visual overlay validation
- **Related:** [H-07](./archive/H-07-no-pose-diversity-gate.md), [H-09](./archive/H-09-no-extrinsic-quality-metric.md), [M-12](./archive/M-12-no-robust-estimation-or-refinement.md)

## What happened

Continuous LiDAR-camera calibration derives each published extrinsic from only the newest
synchronized Detection Pair. When the board moves to a new Board Placement, that observation
replaces all prior calibration evidence.

The point-cloud overlay consequently fits the board at the newest placement, while alignment at
previously observed placements is lost. A new placement can completely dominate the published
calibration because it is the complete solve dataset rather than one constraint on a shared static
extrinsic.

This is especially unsafe because `continuous` is the default operator path and a successful
single-placement PnP solve can still be geometrically degenerate.

## What I expected

Continuous mode should automatically retain useful evidence from multiple distinct Board
Placements and estimate one camera–LiDAR extrinsic jointly from that evidence. Moving the board
should constrain the same static transform more strongly, not replace the transform with a result
that fits only the newest placement.

Repeated frames from one stationary placement must not dominate merely because the board remained
there longer. Retention or weighting therefore needs to be bounded or balanced by Board Placement,
with the Quality Verdict describing the exact retained evidence.

## Steps to reproduce

1. Launch LiDAR-camera calibration with `solver_mode=continuous` and the point-cloud image overlay.
2. Hold the calibration board at placement A and observe the projected cloud alignment there.
3. Move or tilt the board to a geometrically distinct placement B and wait for a new solution.
4. Observe that the overlay now fits placement B.
5. Return the board to placement A, or inspect scene geometry constrained by placement A, and
   observe that the earlier alignment is no longer preserved.

## Additional context

PnP itself is memoryless; changing from `SOLVEPNP_ITERATIVE` to SQPnP followed by LM refinement does
not add temporal information. Both optimize only the correspondences supplied to that solve.

This is static extrinsic calibration, not tracking a moving camera pose. An EKF, particle filter,
pose graph, or SLAM trajectory would smooth or relate time-varying pose estimates, but would not by
itself create the required joint constraints on one fixed camera–LiDAR transform. The relevant
calibration model is a joint or bounded-window solve over retained Detection Pairs from multiple
Board Placements.

### What “temporal information” means here

The observations arrive at different times, but the quantity being estimated is not meant to move.
There is one fixed extrinsic `T_camera←lidar`; each Detection Pair is another noisy constraint on that
same transform:

```text
Detection Pair A ─┐
Detection Pair B ─┼─ raw 3D–2D constraints ─→ one static camera–LiDAR extrinsic
Detection Pair C ─┘
```

Classical PnP is memoryless. It only optimizes the correspondences supplied to the current call.
Temporal accumulation therefore has to live around or above PnP. Several formulations can do that,
provided they preserve the static-extrinsic model:

- **Batch optimization / calibration bundle adjustment:** retain multiple Detection Pairs and jointly
  minimize their reprojection residuals against one extrinsic. This is the most direct formulation.
- **Placement-balanced sliding window:** retain a bounded set of captures from recent distinct Board
  Placements and jointly re-solve. This bounds memory and computation while preserving multi-pose
  constraints.
- **EKF or information filter:** model the extrinsic as a constant state and treat each Detection Pair
  as a new nonlinear reprojection measurement. This can recursively accumulate information, but it
  requires credible measurement covariance and careful handling of rotation and translation.
- **Factor graph:** represent one persistent extrinsic variable connected to observation factors from
  each Board Placement. This is valid but likely more infrastructure than the current solver needs.

Filtering already-solved single-pair PnP transforms is not equivalent to jointly solving the raw
observations:

```text
single-pair PnP → transform sample → pose averaging/filtering
```

That approach can make the published transform look smoother, but every input sample still comes
from a coplanar, weakly conditioned solve. Its rotation and translation errors are coupled; a useful
6×6 uncertainty for each complete PnP result is not currently available; and systematic ArUco or
LiDAR board-pose bias survives averaging. A smooth transform can therefore remain wrong.

The existing multi-capture calibration model already supplies the better seam: preserve raw
Detection Pairs, balance their contribution by Board Placement, then solve one joint reprojection
objective. An incremental filter or factor graph is a possible later implementation if full joint
re-solving becomes too expensive, but it must consume equivalent raw observation constraints rather
than merely smooth independent PnP outputs.

The superseded continuous solver also solved one Detection Pair at a time, so this weakness predates
the unified solver. Stage 2 preserved it while changing the numerical estimator. Existing focused
coverage treats latest-pair replacement as desired behavior; regression coverage must instead test
that adding a distinct placement preserves prior calibration constraints.

M-12 fixed estimator/refinement asymmetry, but did not add pose-grouped outlier rejection or
multi-placement robustness. Its final resolution must not be read as closing this behavior defect.

## Acceptance evidence

- Sequential distinct Board Placements contribute to one current Solved Estimate in continuous
  mode.
- Repeated captures from one placement cannot overwhelm other placements through frame count alone.
- Any recursive or windowed implementation estimates one constant extrinsic from raw observation
  constraints; smoothing completed single-pair PnP transforms alone does not satisfy this issue.
- A deterministic multi-placement test measures error at earlier and later placements and fails if
  the newest placement erases earlier accuracy.
- Dataset 3 visual validation shows the projected cloud remains aligned across all exercised board
  placements; single-pose reprojection RMS is not accepted as the validation gate.
- Publication is withheld, or clearly marked untrustworthy, until configured placement-diversity
  requirements are met.
