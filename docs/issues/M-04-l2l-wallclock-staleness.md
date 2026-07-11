# M-04 · L2L staleness check uses wall-clock vs sensor stamp → drops all pairs on rosbag

- **Severity:** Medium
- **Area:** lidar_to_lidar_solver
- **Status:** Open
- **Verified:** Static review
- **Location:**
  - `ros/lidar_to_lidar_solver/lidar_to_lidar_solver/main.py:169-181`
  - `ros/lidar_board_detector/src/main.rs:1017` (output stamped with input-cloud time)

## Problem

The solver computes `age = get_clock().now() - msg.header.stamp` and drops the pair if `age > max_message_age_ms` (default 500 ms). Board detections are stamped with the input cloud's recorded time. Without `use_sim_time`, `now()` is wall clock.

## Failure scenario

In offline / rosbag playback without `use_sim_time`, the wall clock is far ahead of the recorded stamps, so `age` is huge and every pair is dropped — zero output, only debug logs. Compounds the already-untested L2L pipeline.

## Suggested fix

Respect `use_sim_time`, or make the staleness check relative to the latest received stamp rather than wall-clock `now()`, or disable it in offline mode.
