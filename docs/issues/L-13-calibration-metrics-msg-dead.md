# L-13 · A fourth piece of dead quality scaffolding: `CalibrationMetrics.msg`

- **Severity:** Low
- **Area:** lctk_interfaces
- **Status:** Fixed (2026-07-14) — deleted
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
[H-09](./archive/H-09-no-extrinsic-quality-metric.md) will grep for "metrics", find four plausible-looking
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

## Disposition (2026-07-13) — deferred to H-09

Confirmed: `lctk_interfaces/msg/CalibrationMetrics.msg` is built (CMakeLists.txt) and
has zero publishers/subscribers/imports across `ros/` and `rust/`. Its fields are
IoU/coverage, not residual/covariance/conditioning.

The suggested fix is explicitly "decide as part of H-09, not before it": if H-09
publishes a metrics topic, reuse this message name and redefine its fields; otherwise
delete it together with `rust/calibration-quality` and `rust/dynamic-calibration`
([L-12](./L-12-dead-solver-crates.md)). H-09 is in progress by another agent, so this
message is left untouched to avoid pre-empting or colliding with that decision.

## Resolution (2026-07-14) — deleted

H-09 landed inside the existing services (`last_solve_status`, the logs, the `dump_detections`
JSON) and needed no new topic, so per this issue's own suggested fix the message goes.

Removed `ros/lctk_interfaces/msg/CalibrationMetrics.msg` and its line in
`ros/lctk_interfaces/CMakeLists.txt` (the only live reference). It was IoU/coverage-shaped and
required a ground-truth board region, so it could not have carried H-09's residual/covariance
numbers even if someone had wired it up.

The tree no longer offers four answers to "how good is this calibration?". `lctk_quality` is the
only one, and `ros/calibration_judge/` remains what it always was: a benchmark harness that scores
against a supplied ground truth, which is precisely what you do not have in the field.
