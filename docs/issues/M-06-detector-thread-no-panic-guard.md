# M-06 · Board-detector processing thread has no panic guard → silent dead node

- **Severity:** Medium
- **Area:** lidar_board_detector
- **Status:** Open
- **Verified:** Static review
- **Location:** `ros/lidar_board_detector/src/main.rs:569-613` (detached thread); unwrap-on-NaN at `930, 1349, 1634`

## Problem

Processing runs in a detached `std::thread`. A `panic!` / `unwrap` — e.g. `partial_cmp().unwrap()` on a NaN LiDAR return — unwinds and kills only that thread. The executor keeps spinning and overwriting `latest_msg`, so the node still looks alive but never publishes again.

## Failure scenario

One bad point (NaN/Inf) arrives; the processing thread dies. The node reports no error, RViz shows nothing new, and the user has no indication the detector is dead.

## Suggested fix

Wrap the processing loop body in `std::panic::catch_unwind` and log/recover, and replace `partial_cmp().unwrap()` with `total_cmp` (or filter NaN points at ingest).
