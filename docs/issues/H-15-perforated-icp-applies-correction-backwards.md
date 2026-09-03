# H-15 · Perforated ICP applied its Kabsch correction backwards — every iteration moved the pose away from the fit

- **Severity:** High
- **Area:** `rust/calibration-target-detector` / perforated (hollow) ICP
- **Status:** 🟢 Fixed
- **Verified:** 2026-08-28, by argument-order analysis against the pre-migration implementation plus
  six migrated convergence tests that fail before the fix and pass after it
- **Related:** [M-21](./M-21-icp-stable-pose-exit-unreachable.md) (ICP termination),
  [M-17](./M-17-initial-pose-rewrite-unverified-bbox-path.md) (the other unverified initial-pose
  path), [M-27](./archive/M-27-solid-600-handheld-topics-alias-sample-data.md)

> **Numbering note.** This was filed as H-14 and is referred to by that number in commit
> `fcf9f06`, which carries the fix. A later `git fetch` showed `origin/main` had already allocated
> H-14 (and M-23, M-24, L-26, L-27) to unrelated conflux work, so this branch's five new issues were
> renumbered before merge. The commit message text could not be changed; read "H-14" there as this
> issue.

## Problem

`PerforatedBoardIcpIterator::step` in `rust/calibration-target-detector/src/perforated.rs` computed
its pose update as

```rust
let align_pose = kabsch_transform(model_points, input_points);   // maps model -> sensor
let new_pose = align_pose.inverse() * current.board_pose;        // WRONG: applies sensor -> model
```

`kabsch_transform(input, target)` returns the transform `T` with `T * input ≈ target` (its
translation is `target_centroid - R * input_centroid`). Called with `(model, input)` it therefore
returns the model→sensor correction — which is exactly the correction wanted. Inverting it applies
the correction in reverse, so **each iteration moved the board pose further from the observed
points rather than onto them.**

The code carried a comment asserting the opposite:

> This inversion is intentional and matches the legacy iterator's input/model ordering exactly.

That comment was false.

## How the migration introduced it

The pre-Phase-8 implementation (`rust/hollow-board-detector/src/algo.rs`, deleted in W5-E2) was
correct, but by a route that is easy to misread. `BoardModel::find_correspondences` returned
`Vec<(InputPoint, Point3)>` — **(sensor point, model point)**. The old step then did:

```rust
let (good_corresponding_points, good_inlier_points) = good_correspondences.into_iter().unzip();
let align_pose = compute_kabsch_transform(&good_corresponding_points, &good_inlier_points)?.inverse();
let new_pose = align_pose * current_state.board_pose;
```

The two variable names are the reverse of their contents: `good_corresponding_points` holds the
**input** points and `good_inlier_points` holds the **model** points. So the old call was
`kabsch(input, model)` → sensor→model, and `.inverse()` turned it into model→sensor before it was
applied. Correct.

W3-C's migration read the *names* rather than the types, swapped the argument order to
`kabsch(model, input)` — and kept the `.inverse()`. Swapping the arguments already inverts the
transform, so inverting again returns it to the wrong direction.

## Why no test caught it

Every ICP test that existed before this fix seeds the iterator **at or extremely near the true
pose**, where the Kabsch correction is the identity — and the identity is its own inverse. The
quadrant test (`asymmetric_cutout_evidence_selects_the_correct_quadrant`) generates its samples
*from* the expected pose and starts its hypothesis *at* that pose. The characterization golden
(`manifest_icp_step_keeps_the_legacy_hollow_characterization_golden`) pins per-step metrics —
`avg_loss`, correspondence counts — that are computed **before** the pose update, so it is blind to
the update's direction and still passes either way.

Nothing asserted that ICP converges to the right answer **from a perturbed seed**. That is the
single property this class of bug cannot survive, and it was the one property no test had.

It was also invisible in the field because, per
`docs/roadmap/phase-8-single-source-target-definition.md` ("Outstanding items no packet owns"), the
new detection path has never been run against real data — every Phase 8 gate is headless. The
throughput and ICP-quality figures quoted in CLAUDE.md were measured in January 2026 against the
pre-migration implementation.

## Fix

```rust
let new_pose = align_pose * current.board_pose;
```

Applied together with `rust/calibration-target-detector/tests/perforated_convergence.rs`, six tests
migrated from the deleted crate that assert convergence from a perturbed seed:

- corners land on the true corners from a 7.1 cm seed error (measured 8.9e-5 m)
- a pose seeded exactly at truth does not drift
- a 3.74 cm translation offset recovers to 2.8e-5 m, and must improve by at least 10x
- a 5-degree rotation about the board normal recovers to 1.0e-4 rad
- the fixture self-guard: generated samples really do lie on the physical board
- the stable-pose exit is reachable (~1809 iterations on the 1 m manifest), corroborating M-21

All four convergence tests fail before the fix and pass after it. No previously passing test
changes behaviour: the full suite is 336 Rust tests green with the fix in place.

## Consequences worth noting

Any hollow-target LiDAR detection produced by a build between the W3-C migration and this fix
should be treated as suspect. No calibration result in the repository is known to derive from one —
the path was never run on real data — but a locally solved extrinsic from that window should be
re-derived rather than trusted.
