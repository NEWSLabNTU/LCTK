# Extrinsic Validation

Extrinsic validation checks a saved LiDAR-camera candidate against independently
captured observations. It projects the observed Calibration Target from LiDAR
coordinates through the saved transform into the camera image, then reports
reprojection error and placement coverage. It does not solve, refine, or change the
candidate transform, and it does not define a universal pass threshold.

The `lctk_extrinsic_validation` package is a student exercise and is not shipped on the
published `main` branch. This guide documents the CLI contract for implementations; a
local reference implementation is retained separately for comparison.

## Prepare held-out observations

First finish the normal calibration and save its Detection Archive. Keep the sensor
mounting and camera settings fixed. Record separate observations with the same sensors
at varied Board Placements, then retain them with the existing manual or assisted
capture workflow and save one or more validation Detection Archives. See
[Calibration Sessions](./sessions.md) and [Assisted Capture](./assisted-capture.md).

Use one Target Definition that matches every validation archive. The candidate archive
may have a different Target Identity: its saved transform relates the sensor frames,
while the supplied Target Definition describes the target seen in the held-out data.
The evaluator ignores the validation archives' solved transforms and Quality Verdicts.

## Run the evaluator

In a workspace that includes a built implementation, source ROS and the workspace, then
pass the candidate archive, validation archives, Target Definition, and output report
path:

```bash
ros2 run lctk_extrinsic_validation validate \
  --extrinsic ~/calib/calibration.json \
  --validation ~/calib/held-out-left.json ~/calib/held-out-right.json \
  --target-config /path/to/target.json5 \
  --output ~/calib/validation-report.json
```

An implementation following this contract reads each Detection Archive directly and
runs without a ROS graph. It uses the stored `transform.rvec` and `transform.tvec`
exactly as saved. The transform maps LiDAR coordinates to camera optical coordinates,
with translation in metres and rotation in radians. Do not invert or edit it for this
command.

All archives must use format version 6. The source and validation archives must have
camera matrices that are exactly equal after numeric normalization, along with matching
image dimensions, camera optical frame, LiDAR frame, and projection model
(`undistorted_pixels_using_k`). This version requires unchanged camera settings. Camera
frame labels must match the projection metadata and the saved camera headers; LiDAR
headers must be consistent within each archive. Exact reuse of a source calibration
Capture is rejected. Exact duplicates among validation inputs are retained once and
counted in the report.

## Read the report

The report identifies its candidate and validation inputs, hashes the exact archive
bytes it read, records the supplied and archived Target Identities, and lists results
for every retained Capture. Its headline metric is placement-balanced RMS in pixels:
each Board Placement has equal weight, so repeated observations at one static placement
do not dominate. The report also includes pooled corner RMS, per-Capture error
summaries, and coverage measurements. Coverage uses board positions in LiDAR
coordinates; depth means radial distance from the LiDAR and lateral span means the
LiDAR Y extent.

A report is complete when it contains at least one retained validation Capture and every
unique retained Capture is scorable. A large residual does not make it incomplete. If
any Capture cannot be scored, full-dataset aggregate metrics are `null`; metrics for the
scorable Captures appear separately as usable-subset diagnostics. An empty validation
dataset is incomplete. The command writes the report before returning: exit status 0
means complete, and a nonzero status means the written report is incomplete. Invalid or
incompatible archive inputs are rejected before scoring and do not require a report.

Use the report to assess error and coverage against acceptance limits established for
the actual sensors and application. It is a held-out geometric check, not a guarantee
that publisher frame labels or physical mounting remained unchanged. For field capture
and review, see the [Field Validation Runbook](./field-validation.md). If the downstream
system is Autoware, see [Exporting to Autoware](./autoware-export.md).
