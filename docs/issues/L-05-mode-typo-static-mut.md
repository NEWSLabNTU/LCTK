# L-05 · `mode` typo silently falls back to offline; `static mut` counters race

- **Severity:** Low
- **Area:** lctk_launch / aruco_locator_node
- **Status:** Open
- **Verified:** Static review
- **Location:**
  - `ros/lctk_launch/launch/calibrate.launch.py:56` (`== "realtime"`)
  - `ros/aruco_locator_node/src/main.rs:605-687` (`static mut` counters)

## Problem

`mode` is only compared `== "realtime"`; any other value (including a typo) silently selects offline QoS with no warning. Separately, the aruco image callback reads/writes `static mut` counters (`NO_DETECTOR_COUNT`, etc.) — a data race under any multithreaded executor, and a hard error under Rust edition 2024.

## Failure scenario

`mode=realtim` ships to a live sensor and gets the wrong QoS with no indication; or the node is run multithreaded and races on the counters.

## Suggested fix

Validate `mode` against the known set and error on unknown values. Replace `static mut` with `AtomicU*` or per-node state.
