# L-02 · Pure-Rust panics on empty / NaN point sets in shared helpers

- **Severity:** Low (ROS node guards emptiness first; mostly library-only)
- **Area:** hollow-board-detector
- **Status:** Open
- **Verified:** Static review
- **Location:**
  - `rust/hollow-board-detector/src/algo.rs:263, 545, 643` (`centroid_of_points(...).unwrap()`)
  - `rust/hollow-board-detector/src/algo.rs:572, 669` (`eigen_pairs.sort_by(... partial_cmp().unwrap())`)
  - `rust/hollow-board-detector/src/detector.rs:104, 208, 370` (`min_by_key(|(_,p)| r64(p.z))`)

## Problem

`centroid_of_points(...).unwrap()` panics on an empty inlier set. The eigen-sort `partial_cmp().unwrap()` panics if a NaN/Inf point propagates into the covariance matrix (PCA init). `r64()` asserts non-NaN and panics if a corner z is NaN (e.g. after a degenerate ICP pose). The ROS node guards emptiness before calling, so these are mostly reachable via the library API or adversarial sensor data.

## Failure scenario

A bad LiDAR return (NaN/Inf) reaches PCA init, or the library API is called with an empty set → hard panic instead of a graceful skip.

## Suggested fix

Return `Result`/`Option` from these helpers and handle the empty/NaN cases; use `total_cmp` for sorting.
