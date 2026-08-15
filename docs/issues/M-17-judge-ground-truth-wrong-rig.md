# M-17 · The checked-in judge ground truth does not describe the shipped sample data

- **Severity:** Medium
- **Area:** calibration_judge / lctk_launch config
- **Status:** Open
- **Verified:** Measured against a live `just demo` solve on dataset 3 (2026-08-16)
- **Location:** `ros/lctk_launch/config/judge/ground_truth_config.yaml`

## Problem

`ground_truth_config.yaml` is the default reference for `calibration_judge`, and it is what
`just demo enable_judge=true` scores against. Its translation does not correspond to the rig that
recorded `lctk_sample_data` dataset 3.

Measured against the transform the solver actually publishes on that data, after the
[M-01](./archive/M-01-transform-direction-inverted.md) direction fix:

| quantity | value |
|----------|-------|
| rotation error | 5.80° |
| translation error | 0.745 m |
| `\|t\|` in this file | 0.213 m |
| `\|t\|` solved | 0.889 m |

The rotation agrees to within a few degrees. The translation is off by 0.68 m, and — this is the
part that rules out a convention mistake — **`|t|` is invariant under inversion**. Both the old
and new conventions give 0.213 m, so no change of direction can reconcile 0.213 m with 0.889 m.
The discrepancy predates M-01.

The solved 0.889 m baseline is self-consistent and physically sensible: it places the camera
0.89 m in front of the LiDAR, and the same solve reproduces across the run. The reference does
not.

## Failure scenario

`just demo enable_judge=true` reports **0.0/15.0** on sample data and always will, regardless of
how good the calibration is. A scoring gate that cannot be passed is the same defect class as
[C-04](./archive/C-04-board-detector-gate-unreachable.md) (an ICP threshold below the sensor noise
floor, so nothing could ever pass) — it reads as "the calibration is bad" when it means "the
reference is wrong".

It also burns the judge's credibility: an operator who sees 0/15 on the shipped demo learns to
ignore the judge, which is precisely the instrument
[H-09](./archive/H-09-no-extrinsic-quality-metric.md) added to make quality visible.

## Why it was not simply corrected

The true camera-to-LiDAR offset for that recording is not known here. Overwriting the reference
with the solver's own output would make the judge score itself — it would report a perfect 15/15
by construction and measure nothing at all. That converts a visibly wrong reference into an
invisibly wrong one, which is worse.

## Suggested fix

1. Establish the actual extrinsic for the dataset-3 rig — from the rig drawing, a tape measure, or
   a calibration the team trusts — and record it, with a note saying where the number came from.
2. If no ground truth exists for the sample data, ship the judge disabled by default for it and
   say so, rather than shipping a reference that guarantees 0/15.
3. Consider a startup sanity check: if `|t_groundtruth|` and `|t_estimate|` differ by more than
   the `max_error_m` threshold on the very first message, log that the reference may be for a
   different rig instead of silently scoring zero forever.
