# 0009. Evaluate a fixed extrinsic against held-out observations

- **Date:** 2026-10-04
- **Status:** accepted

## Context

Calibration residuals reuse the observations that selected an estimate. Operators need
an independent geometric check without ground-truth extrinsics. Repeated observations
at one Board Placement are correlated and can dominate a pooled error.

## Decision

Evaluate the exact saved Candidate Transform against Detection Archives captured from
separate reserved recordings with unchanged mounting and camera settings. Project target
corners from observed LiDAR board poses and compare them with undistorted image corners.
Use placement-balanced RMS as the headline metric and report coverage with residuals.
Any unscorable retained Capture makes full-dataset metrics unavailable; label usable-subset
diagnostics explicitly. Evaluation never solves, refines, or rejects large residuals.

The [design specification](../superpowers/specs/2026-10-04-extrinsic-validation-design.md)
defines the student implementation contract. The local reference validator package stays
on a separate feature branch; shared pipeline contracts and documentation are published.

## Consequences

Held-out residuals assess predictive geometric agreement but cannot establish absolute
translation or rotation accuracy. Shared biases can remain invisible. Exact overlap
checking catches reused observations; recording separation remains the operator's job.
The initial contract accepts archives with identical projection settings and one validation
target per invocation, avoiding another playback or transform-input subsystem.
