# L-01 · Library `fit_board_icp` reports non-converged and zero-correspondence fits as successful

- **Severity:** Low (library-only; production path uses a correct test)
- **Area:** hollow-board-detector
- **Status:** Fixed (2026-07-11)
- **Verified:** Static review
- **Location:** `rust/hollow-board-detector/src/algo.rs:504-505`, `rust/hollow-board-detector/src/detector.rs:313-314`

## Problem

`successful = !reason.contains("failed") && !reason.contains("Insufficient")`. The reasons `"Max iterations reached: N"` and `"No correspondences found"` contain neither substring, so a fit that exhausted `max_icp_iterations` with arbitrarily high loss — or found zero correspondences — is returned as a successful board detection. The production ROS node uses a correct `avg_loss < icp_good_fit_threshold` test, so only library / test callers are affected, but the two entry points disagree on what "success" means.

## Failure scenario

If this library API is wired into a new pipeline, garbage poses are returned as valid detections.

## Suggested fix

Base `successful` on the actual loss/convergence state (as the iterator-based path and the ROS node do), not on substring matching of a human-readable reason string.

## Resolution (2026-07-11)
Both `fit_board_icp` (algo.rs) and the iterator path (detector.rs) now also exclude
"Max iterations reached" and "No correspondences found" from `successful`, so a
non-converged / empty-correspondence run is no longer reported as a valid fit.
