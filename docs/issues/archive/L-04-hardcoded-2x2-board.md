# L-04 · `multi_marker_corners` hardcodes 2×2 board; unknown config fields silently ignored

- **Severity:** Low (no production caller of `multi_marker_corners` today)
- **Area:** hollow-board-config / aruco-config
- **Status:** Fixed (2026-07-14)
- **Verified:** Static review
- **Location:**
  - `rust/hollow-board-config/src/lib.rs:130` (literal `/ 2.0`)
  - `rust/aruco-config/src/multi_aruco.rs` (no `deny_unknown_fields`)

## Problem

`multi_marker_corners` computes `square_size = (marker_paper_size − 2·board_border_size) / 2.0` with a literal `2.0` and emits exactly 4 marker tiles, whereas `MultiArucoPattern::square_size()` divides by `num_squares_per_side`. For any `num_squares_per_side != 2` the 3-D marker object points are wrong. Separately, the aruco JSON5 has no `deny_unknown_fields`, so `permute_axis` and any typo'd key are silently dropped.

## Failure scenario

A user builds a non-2×2 board (silently wrong correspondences) or expects `permute_axis` to have an effect (silently ignored).

## Suggested fix

Use `num_squares_per_side` instead of the literal, and add `#[serde(deny_unknown_fields)]` (or warn on unknown keys) to the aruco config struct.

## Partial resolution (2026-07-11)
Removed the silently-ignored `permute_axis` field from the shipped
`aruco_pattern.json5` (it has no effect). The `multi_marker_corners` 2×2 hardcode
in `hollow-board-config` is left as-is: it has no production caller (the Python
solvers compute object points independently), and `deny_unknown_fields` was NOT
added because several shipped configs carry extra keys and would break. Tracked as
remaining work if that library entry point is ever wired in.

## Resolution (2026-07-14)
The live path (both Python `_compute_multi_marker_corners`) already derives the
square size from `num_squares_per_side`, so it was never affected. The dead Rust
`multi_marker_corners` (test-only caller) is 2x2 by construction — it places exactly
four markers — so instead of a bare `/ 2.0` it now derives the divisor from
`num_squares_per_side` and `debug_assert!`s the 2x2 grid, turning a would-be silent
wrong-corner set into a loud failure if it is ever wired in with a different grid.
`permute_axis` was already removed from the shipped config.

