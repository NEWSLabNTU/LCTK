# M-19 · Every `debug_assert!` is compiled out of both `just build` and `just test`

- **Severity:** Medium
- **Area:** build profiles / test coverage
- **Status:** Open — the `hollow-board-config` half is resolved (see [Update](#update-2026-08-13--the-51-assertions-are-gone)); the build-profile half stands
- **Verified:** 2026-08-13 — read from `Cargo.toml:90-92`, `justfile:48`, `justfile:150`
- **Related:** [L-14](./archive/L-14-lint-red-on-main.md), [M-18](./archive/M-18-root-cargo-config-missing-rust-tests-unrunnable.md), [M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md)

## Problem

`Cargo.toml:90-92` defines the profile both sanctioned commands use:

```toml
[profile.test-release]
inherits = "release"
debug = true
```

`inherits = "release"` brings `debug-assertions = false`. `debug = true` adds debug *symbols*, which
is a different thing and is easy to misread as enabling assertions.

Both entry points use that profile:

- `just build` → `--cargo-args --profile=test-release` (`justfile:48`)
- `just test` → `cargo nextest run --cargo-profile test-release` (`justfile:150`)

**So no `debug_assert!` in this workspace has ever executed under either command.**

The largest concentration is in `rust/hollow-board-config/src/lib.rs`: **51 assertions** — 22 in the
preamble of each of the two `find_correspondences` copies (`:191-300`, `:435-544`) plus 3 more inside
each closure (`:323`, `:369`, `:374`). They are intended as the board model's geometry contract.

## Why it matters

Two compounding failures, not one:

1. **They never run.** Any invariant expressed as a `debug_assert!` anywhere in the workspace is
   decoration.
2. **Even if they ran, they would catch nothing about the frame.** Every one of the 51 is stated as a
   world-frame norm or dot product — `(top_corner - left_corner).norm() == board_width`,
   `(left_circle_center - board_center).norm() == hole_center_shift * sqrt(2)`, and so on. Those are
   **rotation-invariant**, so they hold identically under any in-plane relabelling of the local axes.

That is precisely why the 45° convention mismatch documented in
[`docs/superpowers/specs/2026-08-12-initial-board-pose-inplane-rotation.md`](../superpowers/specs/2026-08-12-initial-board-pose-inplane-rotation.md)
was invisible to the assertions that were supposed to guard the geometry.

`EPS_F64 = 0.3` (`lib.rs:11`) compounds it — a **30 cm** tolerance on a 1 m board. Even executing,
most of these would tolerate a badly wrong model.

## Suggested fix

1. Convert the geometry contract from `debug_assert!` to real `#[test]`s, and make them
   **frame-sensitive**: dot each accessor against the model's *own* `board_x_axis()` / `board_y_axis()`
   and assert the expected local coordinates, under several randomised poses. Only convention-sensitive
   assertions can catch a convention error.
2. Split `EPS_F64`: keep a tight `1e-9` for the "is in-plane" checks (that quantity is exactly zero by
   construction — the code just subtracted the normal component) and drop the geometric-identity
   asserts once tests cover them.
3. Separately, decide whether `[profile.test-release]` should set `debug-assertions = true`. This may
   surface latent violations elsewhere in the workspace, so land it as an isolated commit — or skip it
   entirely if the invariants have become real tests, which is strictly better.

## Notes

Found while planning the board-frame change; the plan's step for this is deliberately isolated so a
newly-surfaced violation is attributable.

## Update (2026-08-13) — the 51 assertions are gone

Suggested fixes 1 and 2 landed with Phase 1 of the corner-aligned board-frame work
([M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md), spec
[`2026-08-13-corner-aligned-board-frame.md`](../superpowers/specs/2026-08-13-corner-aligned-board-frame.md)).

All 51 `debug_assert!`s in `rust/hollow-board-config/src/lib.rs` are deleted. In their place are real
integration tests that are **convention-sensitive** — they dot each accessor against the model's own
`board_x_axis()`/`board_y_axis()` and assert the expected local coordinates, under randomised poses,
so an in-plane relabelling can no longer pass:

- `tests/board_frame.rs` — the frame contract: accessor coordinates in the board's own frame, the
  pose translation being the plate centre, round-tripping through the board's axes,
  right-handedness, and the three-hole asymmetry.
- `tests/boundary_projection.rs` — the L¹-ball projection against a brute-force nearest-point
  reference over many random points, plus corner/edge/hole-rim cases and a property test asserting
  the new frame's projection is an exact re-parameterisation of the old frame's.
- `tests/marker_layout_golden.rs` + `tests/fixtures/` — marker corners in world coordinates, keyed by
  ArUco marker id, with an independent Python generator.

That is the "strictly better" outcome suggested fix 3 anticipated, but only for this crate.

**Still open:** every other `debug_assert!` in the workspace remains compiled out of both `just build`
and `just test`, because `[profile.test-release]` still inherits `release`. Deciding whether to set
`debug-assertions = true` there — and absorbing whatever that surfaces — is the remaining work, and
was gated on [M-18](./archive/M-18-root-cargo-config-missing-rust-tests-unrunnable.md), since the Rust
suite must be runnable before turning assertions on is meaningful. M-18 was fixed on 2026-08-14 —
`just test` now runs the whole suite from the workspace root — so this work is unblocked.
