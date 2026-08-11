# M-17 · Shared initial-pose rewrite leaves the bbox path's "unchanged" guarantee unproven

- **Severity:** Medium
- **Area:** lidar_board_detector / Stage-3b initial pose
- **Status:** Open
- **Verified:** By code review (2026-08-11, spec axis) against the sub-project-2 design's global constraint
- **Related:** [M-14](./M-14-corner-order-brittle.md), [M-16](./M-16-l2l-pipeline-untested.md)

## Problem

The bbox-free integration design states as a global constraint that *"the bbox path must remain
byte-identical to today"* — the crop-box-free work was meant to add a Stage-1 alternative and leave
Stage 2 and Stage 3 untouched ("selected points flow unchanged into the existing Stage-2 → Stage-3
ICP").

Restoring the Seyond sensor-up-axis handling (commit `162a28e`) rewrote
`compute_initial_pose_from_plane`, which is a **single shared call site used by both detection
modes**. The old construction built the initial rotation from a fixed lifting rotation
(`Ry(-90°)·Rz(-45°)`) aligned against the plane normal's XY projection. The new construction builds
the board frame from the configured up axis projected onto the board plane
(`Z = plane normal`, `Y = up projected`, `X = Y × Z`) plus an optional in-plane offset.

With `sensor_up_axis = "z"` and `initial_inplane_rotation_deg = 0` the new frame is *intended* to be
equivalent, but it is not obviously so, and equivalence was never demonstrated. The same commit also
generalised the post-fixup lowest-corner pick from a raw `.z` comparison to a projection onto the up
axis — same reasoning, same lack of proof.

This matters because the initial pose is the ICP **seed**: a different seed can converge to a
different local minimum, so a legacy bbox-mode calibration could shift without anything reporting an
error. No test covers the bbox path's initial pose.

## Suggested fix

Pick one:

1. **Prove it.** Add a unit test asserting the new frame construction equals the old one for
   `up = +Z`, `inplane = 0`, over a spread of plane normals — then the shared call site is safe.
2. **Or measure it.** Run a golden bbox-mode calibration (sample data, dataset 3) before and after
   `162a28e` and compare the solved extrinsics within sensor noise.

If neither holds, gate the new frame construction so the legacy path keeps the old math and only
`bbox_free` (or a non-Z `sensor_up_axis`) takes the new one.

Note that the rewrite itself is *correct and necessary* for the Seyond rig — this issue is about the
unproven blast radius on the legacy path, not about reverting it.
