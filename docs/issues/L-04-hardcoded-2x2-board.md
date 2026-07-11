# L-04 · `multi_marker_corners` hardcodes 2×2 board; unknown config fields silently ignored

- **Severity:** Low (no production caller of `multi_marker_corners` today)
- **Area:** hollow-board-config / aruco-config
- **Status:** Open
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
