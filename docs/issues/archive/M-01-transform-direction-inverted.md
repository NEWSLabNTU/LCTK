# M-01 · Transform frame labels are inverted vs ROS TF semantics

- **Severity:** Medium
- **Area:** extrinsic solvers / TF output
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review
- **Location:**
  - `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:753-768`
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1357-1370` — this package no
    longer exists at this path; renamed to `ros/lidar_to_camera_solver` in `ecba23c`. The equivalent
    construction is now `ros/lidar_to_camera_solver/lidar_to_camera_solver/main.py:1053-1071`
    (`_create_transform_message`) — see the 2026-08-28 note at the bottom of this file
  - `ros/lctk_launch/lctk_launch/tf_tree_broadcaster.py`

## Problem

`cv2.solvePnP` returns `(R, t)` mapping LiDAR-frame points into the camera frame (`p_cam = R·P_lidar + t`). That raw rvec/tvec is placed into a `TransformStamped` labeled `header.frame_id = lidar`, `child_frame_id = camera`. In ROS TF, a transform labeled lidar→camera must express the camera's pose in lidar coordinates — the inverse of what is stored. The in-repo overlay node bypasses TF and uses the transform directly as rvec/tvec for `projectPoints`, so topic-consumers and TF-consumers require opposite interpretations.

## Failure scenario

Any standard `tf2` consumer — the natural Autoware ingestion route via `/tf_static` — looks up the transform and gets it pointing the wrong way. The success log even prints "LiDAR → Camera", reinforcing the ambiguity.

## Suggested fix

Publish the transform with frame labels matching TF semantics (invert `(R, t)` before building the `TransformStamped`, or relabel parent/child), and keep the overlay node's direct-rvec/tvec usage consistent with that choice. Document the convention explicitly.

## Status note (2026-07-11)

Deferred, not because it isn't real but because it cannot be verified without a
visual check. The published transform is self-consistent within LCTK (the overlay
consumes it directly as `rvec`/`tvec` for `projectPoints`), so changing the TF
labels to the correct direction requires a coordinated change in the overlay
(invert it back) and the only correctness signal is whether point clouds still
project onto the image correctly — a visual result this environment can't produce.
Recommended fix, to be done with sample data + the overlay:

1. Publish the `TransformStamped` with correct TF semantics (invert the solvePnP
   `T_cam_lidar` so `frame_id=lidar, child=camera` really is the camera pose in
   the lidar frame).
2. Update `pointcloud_image_overlay` to invert the looked-up transform back into
   the LiDAR→camera-points mapping it feeds to `projectPoints`.
3. Verify the overlay alignment is unchanged on `just demo`, then confirm a
   `tf2` lookup gives the Autoware-correct direction.

Until then this is the one blocker between the solver output and a correct
Autoware `sensor_kit_calibration.yaml` (see [gap-autoware-export.md](./gap-autoware-export.md)).

## Resolution (2026-08-15)

Fixed. Both solvers invert before building the `TransformStamped`, and `pointcloud_image_overlay`
inverts back -- it consumes the topic directly as `rvec`/`tvec` for `projectPoints` rather than
going through tf2, which is why the two had to move together. The pair is a no-op end to end: the
projected pixels are unchanged, while the published transform now means what its labels say.

**The deferral reason is gone.** This sat open since 2026-07-11 because "the only correctness
signal is whether point clouds still project onto the image correctly -- a visual result this
environment can't produce". That signal is now arithmetic, and stricter than a visual one:

- the published matrix equals `inv(T_camera<-lidar)`,
- it is *not* the raw solve, so a future simplification cannot quietly undo this,
- the overlay recovers exactly the original `rvec`/`tvec`,
- projecting real points through both paths gives identical pixels to 1e-6.

Deliberately unchanged: the dump JSON keeps the raw `rvec`/`tvec`, because that is what
`lctk_autoware_export` consumes and its frame handling is already correct and tested. This closes
the last blocker between the solver output and a correct Autoware `sensor_kit_calibration.yaml`.

**One consumer needs operator attention.** `calibration_judge` compares the topic against a
user-supplied `ground_truth_file` whose convention was implicitly "whatever the solver published".
A file written against the old behaviour now holds the inverse and will score as badly wrong. The
judge's module docstring states the convention and the one-line fix (`T_new = inv(T_old)`). This is
documented rather than silently migrated, because only the operator knows which convention their
file was recorded in.


## Update (2026-08-28) — pointer repaired; finding confirmed unchanged on the renamed package

`ros/advanced_extrinsic_solver` was renamed to `ros/lidar_to_camera_solver` in `ecba23c`, ahead of
this phase. Reading the renamed package's current `main.py`, the same pattern this issue describes
is still present: `_create_transform_message` (`lidar_to_camera_solver/main.py:1053-1071`) builds a
`TransformStamped` directly from the raw solver `rvec`/`tvec` — `message.header.frame_id =
self.parent_frame` (the lidar), `message.child_frame_id = self.child_frame` (the camera), with no
inversion — the same labeling this issue calls backwards against ROS TF semantics.

This is a pointer repair only, per this packet's scope (owned by in-progress work elsewhere, per the
W6-A packet brief). Status remains **Deferred**; nothing here changes the verification blocker (a
visual overlay check) or the recommended fix.
