# C-02 · Conflux realtime mode leaks a message object for every dropped message

- **Severity:** Critical
- **Area:** conflux synchronizer (Python + C++ FFI)
- **Status:** Open
- **Verified:** Static review
- **Location:**
  - `ros/conflux/conflux_py/conflux_py/_ffi.py:239`
  - `ros/conflux/crates/conflux-core/src/state.rs:149, 315`
  - `ros/conflux/conflux_cpp/src/synchronizer.cpp:62, 70-74, 97-98`

## Problem

Every pushed message is stored in `_message_refs` (to keep the Python object alive across the FFI boundary) and is removed only on a push-error or when returned by poll. But the Rust core silently discards messages during DropOldest eviction (`state.rs:315`) and finite-window pruning (`state.rs:149`), returning `Ok` — so the corresponding Python ref is never freed. The C++ binding has the same defect.

## Failure scenario

In `mode=realtime` (finite 50 ms window + DropOldest), every evicted message leaks a full `Image` / `PointCloud2` Python object. On live sensors, RSS grows without bound until OOM. Offline mode avoids it only by accident (RejectNew frees the ref on error; the infinite window never prunes).

## Suggested fix

Have the Rust core report evicted/pruned message IDs back across the FFI (e.g. via the poll callback or a dedicated "released IDs" out-param) so the binding can drop the ref. Alternatively, free refs on a periodic reconciliation against the core's live ID set.
