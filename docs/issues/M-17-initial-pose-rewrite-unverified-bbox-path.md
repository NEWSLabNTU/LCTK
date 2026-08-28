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

## Update (2026-08-13) — the template's seed is now the *correct* seed, still unmeasured

Phase 1 of the corner-aligned board-frame work
([M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md), spec
[`2026-08-13-corner-aligned-board-frame.md`](../superpowers/specs/2026-08-13-corner-aligned-board-frame.md))
redefines the board model's canonical frame so its in-plane axes run corner to corner. Under the new
convention the base rotation the node builds — up axis projected into the board plane — is already
the right one for a diamond-mounted board, so `initial_inplane_rotation_deg: 0.0` becomes correct
rather than 45° off. `board_detector.json5` (deleted W5-E1; successor
`ros/lctk_launch/config/board/hollow_1000/velodyne_bbox.json5`, the bbox-mode preset this issue is
about) has always shipped `0.0`, so the template's seed (and
with it `sample_data.yaml` and `vehicle.yaml`, the concrete case this issue tracks) should now be
right by construction.

**This issue stays open: "should now be right" is not a measurement.** The bbox path was not
exercised and its extrinsic was not compared before and after, which is exactly the gap this issue
exists to record. Both suggested fixes still apply — the equivalence unit test, or a golden bbox-mode
calibration on sample data dataset 3 before and after.

Note also that the "unchanged" claim now has to be read against the frame change, not just against
commit `162a28e`: the seed *rotation* is unchanged by the flip (the conjugation cancels), but the
frame's origin moves from a plate corner to the plate centre, so any before/after comparison must
account for that half-diagonal offset rather than reading it as drift. `compute_initial_pose_from_plane`
collapses accordingly: its translation is now the plane-inlier centroid directly, and its
`board_width` argument is gone — which is a compiler-enforced migration, not a silent one.

## Update (2026-08-14) — `bbox_free` is now field-validated; `bbox` still is not

The corner-aligned frame was run against the real two-LiDAR rig (Seyond + Velodyne) using the
`TWO_LIDAR_*` recordings: detection works at `initial_inplane_rotation_deg: 0.0` on both rigs, the
board outline coincides with the real plate's LiDAR returns, the `+Y` arrow points at the physically
up-most corner (ruling out the `−45°` conjugation), and the LiDAR-to-LiDAR extrinsic publishes plausible
relative poses. See [M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md), now fixed.

**That run exercised the `bbox_free` path only.** The crop-box (`bbox`) path was not run, so
everything this issue tracks is still open on it:

- the shared `compute_initial_pose_from_plane` rewrite remains unproven for `bbox` mode — no
  before/after extrinsic comparison, no equivalence unit test;
- the two rig crop-box configs `ros/lctk_launch/config/board/bbox_2_lidar_seyond.json5` and
  `bbox_2_lidar_vlp32.json5` carry recently corrected rotation values that **no run has touched**.

Validating `bbox_free` is not evidence about `bbox`. Both suggested fixes above still stand.
