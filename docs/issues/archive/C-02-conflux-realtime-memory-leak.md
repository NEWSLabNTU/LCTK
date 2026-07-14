# C-02 · Conflux realtime mode leaks a message object for every dropped message

- **Severity:** Critical
- **Area:** conflux synchronizer (Python + C++ FFI)
- **Status:** Fixed (2026-07-11, conflux submodule)
- **Verified:** Static review + leak regression test
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

## Resolution (2026-07-11)

Fixed in the conflux submodule (`jerry73204/conflux`@`a9bbcbc`; LCTK pins it) using
the reconciliation approach. Added `conflux_for_each_live()` to the FFI (backed by
a new `Buffer::iter`) that enumerates the `user_data` of every still-buffered
message; the Python binding periodically reconciles `_message_refs` against that
live set and frees references for messages no longer buffered (i.e. silently
evicted or pruned). Verified with a leak test (`tmp/test_c02_leak.py`): 3000
messages that all get evicted leave the reference table bounded at ~10 entries
instead of growing to 3000. This required no change to the core `State`/`push`
API. Should be upstreamed to conflux `main` (done — it is on conflux main).
