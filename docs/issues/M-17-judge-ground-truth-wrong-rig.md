# M-17 · The checked-in judge ground truth does not describe the shipped sample data

- **Severity:** Medium
- **Area:** calibration_judge / lctk_launch config
- **Status:** Partially fixed (2026-08-16) — the silence is fixed; the reference still needs rig geometry
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

## Partial resolution (2026-08-16)

Suggestions 2 and 3 are done. Suggestion 1 — record the true extrinsic — is **not**, and cannot be
from inside this repo.

**The failure is no longer silent.** `CalibrationJudgeNode.check_reference_plausibility` compares
the reference's sensor baseline against the solved one and, when they disagree by more than 3x the
`max_error_m` threshold, logs an error naming both. On the shipped demo it now says:

```
Ground truth may describe a DIFFERENT RIG than the data being judged: its sensor baseline is
0.21 m but the calibration solves to 0.89 m, a gap of 0.68 m. Note that ||t|| is unchanged by
inverting a transform, so this is NOT a frame-direction mistake -- the two describe different
geometry. Scores will stay near zero regardless of calibration quality until the reference is
replaced with one recorded for this rig. See docs/issues/M-17.
```

Logged once on the first estimate, not per message: it cannot change, and repeating it would bury
the scores it exists to explain.

The check keys on `||t||` deliberately, because that quantity is **invariant under inversion**. A
direction mistake — the [M-01](./archive/M-01-transform-direction-inverted.md) class of error —
leaves the baseline untouched, so a large gap is positive evidence of different *geometry* rather
than a convention mix-up. That is what makes it safe to state the diagnosis rather than hedge, and
a test pins it: feeding the check an inverted-but-correct reference must produce **no** complaint,
so it can never send someone hunting for a hardware discrepancy that does not exist.

On suggestion 2: `enable_judge` already defaults to `false`, so the demo does not run the judge
unless asked. With the diagnostic in place, an operator who does opt in now gets the reason rather
than a bare zero.

Regression coverage: `ros/calibration_judge/test/test_reference_plausibility.py` — five tests,
including the real 0.21-vs-0.89 case and the inverted-reference case that must stay quiet. The
package had no tests at all before this; the suite is now wired into `just test`.

## Still open

**A ground truth actually recorded for `lctk_sample_data` dataset 3.** The matrix was added in
October 2025 (`e8dd049`) with no provenance, and the repo holds no rig drawing, dimensions, or
trusted prior calibration — only the pcap, the avi, and a topic README. Establishing the real
camera-to-LiDAR offset needs someone with access to the rig or to the records from that capture.

It is deliberately not being replaced with the solver's own output: that would make the judge
score itself, reporting a perfect 15/15 by construction while measuring nothing.
