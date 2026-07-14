# L-05 · `mode` typo silently falls back to offline; `static mut` counters race

- **Severity:** Low
- **Area:** lctk_launch / aruco_locator_node
- **Status:** Fixed (2026-07-11)
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

## Resolution (2026-07-11)
`calibrate.launch.py` now raises on an unknown `mode` instead of silently using
offline QoS. The four `static mut` counters in `aruco_locator_node` are now
`AtomicU32`/`AtomicUsize` (no data race, edition-2024 safe).
