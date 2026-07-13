# L-13 · A fourth piece of dead quality scaffolding: `CalibrationMetrics.msg`

- **Severity:** Low
- **Area:** lctk_interfaces
- **Status:** Open
- **Verified:** Yes (confirmed against live source, 2026-07-13)
- **Location:**
  - `ros/lctk_interfaces/msg/CalibrationMetrics.msg`
  - `ros/lctk_interfaces/CMakeLists.txt:27` (it is registered and built)

## Problem

`lctk_interfaces/msg/CalibrationMetrics.msg` is built into the interface package and has **zero
users** — no publisher, no subscriber, no import anywhere in `ros/` or `rust/`.

Worse, its name promises something it does not deliver. It is an **IoU / coverage** message —
fields along the lines of `iou`, `coverage`, `precision`, `projected_point_count`,
`inlier_point_count`, `ground_truth_area` — i.e. a *segmentation-overlap* score that requires a
ground-truth board region in the image. It is **not** a reprojection-residual message, and it
cannot be used as one.

This is now the **fourth** piece of unused quality scaffolding in the repo:

| | what it is | why it does not help |
|---|---|---|
| `lctk_interfaces/msg/CalibrationMetrics.msg` | IoU/coverage message | wrong shape; needs ground truth; zero users |
| `rust/calibration-quality/` | `CalibrationMetrics` struct | its `reprojection_error` is a **3D-3D** residual in metres, not pixels ([L-12](./L-12-dead-solver-crates.md)) |
| `rust/dynamic-calibration/` | the only consumer of the above | itself has zero consumers |
| `ros/calibration_judge/` | scores against a supplied ground-truth transform | benchmark harness; useless in the field, where there is no ground truth |

## Failure scenario

Not a runtime failure — a **research hazard**, and it has a cost. Anyone (human or agent) starting
[H-09](./H-09-no-extrinsic-quality-metric.md) will grep for "metrics", find four plausible-looking
homes for a quality number, and have to read all four to discover that none of them measures what
H-09 needs. Two of them are actively misleading: one is named `reprojection_error` but returns
metres, and this one is named `CalibrationMetrics` but computes image-overlap against a ground
truth you do not have.

The repo currently advertises four answers to "how good is this calibration?" and implements none.

## Suggested fix

Decide as part of H-09, not before it:

- If H-09 publishes a metrics topic, **reuse this message name** — but redefine its fields to the
  residual/covariance/conditioning quantities H-09 actually produces, and delete the IoU fields.
- If H-09 stays inside the existing services (`last_solve_status`, `ListDetectionBuffer`, the
  `dump_detections` JSON), **delete this message** along with `rust/calibration-quality/` and
  `rust/dynamic-calibration/`.

Either way, do not leave four dead answers in the tree. Whatever H-09 lands should be the only
thing in the repo called "calibration metrics".
