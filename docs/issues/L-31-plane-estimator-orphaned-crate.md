# L-31 · `rust/plane-estimator` is a live workspace member with zero consumers

- **Severity:** Low
- **Area:** plane-estimator (workspace membership / dead code)
- **Status:** Open
- **Verified:** 2026-08-28 — `grep -rn "plane-estimator" -- '*/Cargo.toml'` and
  `grep -rln plane_estimator -- '*.rs'` across the repository; details below

## Problem

`rust/plane-estimator` builds a RANSAC plane fit (`PlaneEstimator`, via the `sample-consensus`
crate) and is a member of the workspace through the root `Cargo.toml`'s `members = ["rust/*", ...]`
glob — it is not excluded, and it is not behind any feature flag.

It has no consumers:

- No `Cargo.toml` anywhere under `ros/` or `rust/` lists `plane-estimator` as a dependency — the
  only file naming it is the crate's own `rust/plane-estimator/Cargo.toml` (`[package] name =
  "plane-estimator"`).
- No `.rs` file outside `rust/plane-estimator/` itself references the `plane_estimator` crate. The
  only two hits for `plane_estimator` in the whole tree are `rust/plane-estimator/src/lib.rs` and
  its own `rust/plane-estimator/tests/simple.rs`.

Its only historical dependent was `rust/hollow-board-detector`, deleted in W5-E2 (`21142ac`,
alongside `rust/hollow-board-config` and `fixtures/board/`, superseded by `rust/calibration-target`
and `rust/calibration-target-detector`). That deletion is what makes `plane-estimator` fully
orphaned rather than merely quiet.

`ros/lidar_board_detector` — the node that actually needs a plane fit today — has its own RANSAC
plane fit, reached from `src/main.rs` via `board_cluster_detector::geometry::fit_plane` (see
`main.rs:1877`, `Ok(board_cluster_detector::geometry::fit_plane(points))`), a different crate. So
this is not a case of "not wired up yet" — the live detection path already has a superseding
implementation elsewhere, and `plane-estimator` was not it even before the deletion.

## What's still true about it

- **It is a live workspace member**, not excluded like the packages listed in root `Cargo.toml`'s
  `exclude = [...]`. `just build` and `cargo build --workspace` still compile it.
- **It still carries a test.** `rust/plane-estimator/tests/simple.rs` (`#[test] fn simple()`) fits a
  plane through five hardcoded points and asserts the fit succeeds — a real, passing test, not a
  stub.
- **It is advertised as a project library in `book/`.** The architecture doc
  (`book/src/developer-guide/architecture.md`, "Project Structure" tree) lists `plane-estimator/`
  alongside `calibration-target/` and `calibration-target-detector/` as one of the `rust/` core
  libraries. Note this is a *correction* to this issue's own filing brief: `CLAUDE.md`'s
  "Project Structure" section listed `plane-estimator` in its `rust/` library summary until W5-E2
  (`21142ac`) dropped it — apparently incidentally, as part of swapping `hollow-board-detector` for
  `calibration-target-detector` in the same parenthetical list — so as of this filing, `CLAUDE.md`
  itself no longer names it, but `book/`'s architecture doc still does. Either way, at least one
  maintained document currently presents it as part of the project's library surface.

## Why this is filed now, and why it doesn't recommend deletion

This crate became fully orphaned as a side effect of W5-E2's deletion of its one consumer.
W5-E3 (this packet) is a verification/reference-repair pass — its brief is repairing dangling
evidence pointers in the issue tracker, not removing crates — so the decision to delete
`plane-estimator`, keep it as a documented-but-unused library, or find it a new consumer was
deliberately left out of scope here rather than decided unilaterally in a pointer-repair packet.

This is presented as a decision for the maintainer, with the evidence above, not as a settled
recommendation. Arguments exist on both sides that a future packet (or the maintainer) should
weigh, not this issue:

- *For deletion:* it duplicates functionality (`board-cluster-detector::geometry::fit_plane`) that
  the live detection path already uses instead; an unused crate with no consumers is exactly the
  shape L-12 (`archive/L-12-dead-solver-crates.md`) already flagged as worse-than-nothing for
  `pnp-solver` and `calibration-quality`.
- *For keeping it:* unlike L-12's crates, this one is small (29 lines of library code), still
  compiles, still has a passing test, and is presented in `book/` as part of the intentional public
  library surface — it may be meant as a standalone reusable utility rather than pipeline
  plumbing, in which case "no consumer in this repo" is not the same claim as "dead code."

## Suggested next step

Whoever owns this decision should either: (a) delete it and its `book/` mention, following the
L-12 precedent, or (b) explicitly document it as a standalone library with no in-repo consumer (and
fix `CLAUDE.md`'s now-inconsistent omission one way or the other). Either is a legitimate choice;
what should not happen is the crate continuing to exist by accident, uncounted by any decision.
