# M-28 · The ArUco generator and the target geometry bind cells with opposite handedness

- **Severity:** Medium
- **Area:** rust/aruco-config, rust/calibration-target, ros/lctk_target
- **Status:** Open
- **Verified:** By code trace (2026-08-31), found while root-causing the `np.roll` corner hack in `3e6b873`
- **Related:** [M-14 (archived)](./archive/M-14-corner-order-brittle.md), [2026-08-12 initial-board-pose spec](../superpowers/specs/2026-08-12-initial-board-pose-inplane-rotation.md)

## Problem

Two implementations decide which ArUco marker ID sits in which cell of a multi-cell target,
and they disagree by a **reflection** — not a rotation.

The renderer lays cells out row-major in the produced image
(`rust/aruco-config/src/multi_aruco.rs:117-118`):

```rust
row = index / n
col = index % n
```

The geometry maps the same index onto the plate's paper axes as
`u_cell = index % n`, `v_cell = index // n`, with `u × v = +z`
(`rust/calibration-target/src/lib.rs:144-179`, mirrored in
`ros/lctk_target/lctk_target/target.py:417-444`).

A printed sheet mounted face-out always satisfies `image_x × image_y_down = −z`. The geometry's
`u × v = +z` therefore has the opposite handedness to the rendered image, and **no physical
mounting of the printed sheet reconciles the two** for a multi-cell target. A rotation can permute
cells cyclically; this needs a mirror, which gluing the sheet on differently cannot produce.

## Why it has not bitten

- `solid_600_aruco_1` is `cells_per_side: 1`. A single cell has nothing to permute, so the
  handedness is unobservable there. This is the target the recent field work used.
- `hollow_1000_aruco_4` is 2×2 and therefore *is* affected — but its physical sheet predates the
  current renderer (`rust/aruco-detector/tests/rectify_contract.rs:186` calls it the "legacy hollow
  renderer"), so the board in the lab was not produced by the code that now disagrees.
- No recording in this repository exercises the hollow board's **camera** path end to end
  (`ros/lctk_sample_data/bags/README.md`: the `TWO_LIDAR_*` bags carry no camera stream), so
  nothing in CI or in a demo would surface it.

The consequence, if someone prints a fresh hollow board from `aruco_generator_node` and calibrates
against it, is a silent permutation of marker identity across cells. That is precisely M-14's
failure mode: the symmetric 2×2 grid absorbs it with a plausible reprojection error, so it shows up
as a wrong extrinsic rather than as a detection failure.

## Why the existing tests do not catch it

`rectify_contract.rs:176-247` asserts the rendered-image ID layout and the object-model ID layout
**separately**. Each is self-consistent. Nothing asserts the two against each other, which is the
only place the disagreement lives. This is the same gap M-14 part 2 describes:

> Corner order is defined twice, in two languages, and never verified … Nothing checks that the
> two orders agree.

## Suggested fix

Not attempted here, deliberately: choosing which side is wrong requires knowing how the existing
physical hollow 1000 board is actually printed, and nobody working from this repository can
determine that. A "fix" chosen without that measurement could just move the error onto the board
that exists.

What is needed:

1. Someone with the physical hollow 1000 board in hand records which marker ID sits in which cell,
   viewed face-on with the board's `+Y` toward the up-most plate corner.
2. Whichever of the renderer or the geometry disagrees with the board is corrected — and that
   choice is recorded in the target manifest, not in a consumer.
3. A **cross-domain** test is added: render the target, detect it, build correspondences, reproject,
   and assert the residual. That is the check `rectify_contract.rs` stops one step short of, and it
   is what would have caught both this and the `3e6b873` corner hack.

M-14 part 1b — camera cross-validation of the LiDAR-derived in-plane orientation against the ArUco
IDs — is the runtime half of the same protection, and is still open.
