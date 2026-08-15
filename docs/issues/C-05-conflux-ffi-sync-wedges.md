# C-05 · Conflux FFI synchronizer wedges permanently after a stream divergence

- **Severity:** Critical
- **Area:** conflux-core / conflux FFI (all LCTK solver nodes)
- **Status:** Open
- **Verified:** Reproduced against the built `libconflux_ffi.so` via `conflux_py` (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/state.rs:130-186` (`try_match`),
  `ros/conflux/conflux_cpp/rust/src/lib.rs:293-330` (`conflux_poll`),
  `ros/conflux/crates/conflux-core/src/sync.rs:188-205` (the escape hatch the FFI lacks)

## Problem

`State::try_match` refuses to emit a group when the buffered spread is narrower than the
sync window:

```rust
if !self.all_one() && inf_ts + window_size > sup_ts {
    return None;
}
```

The only escape from that branch is `all_one()` — every buffer drained down to exactly one
message. The pure-Rust pipeline provides a second escape: `sync()`'s poll loop detects
`is_full()` and calls `drop_min()` to force progress (`sync.rs:195-205`).

**The FFI exposes no equivalent.** `conflux_poll` calls `try_match()` and nothing else; there
is no `drop_min`, no feedback, no eviction entry point in the C ABI. Once every buffer holds
≥ 2 messages and the spread stays under the window, the synchronizer can never emit again.

## Failure scenario

Reproduced with `window_size_ms=50, buffer_size=2` (the shipped **realtime** preset), two
streams briefly diverged in time, then 40 perfectly aligned fresh messages pushed with a full
drain loop after each push:

| drop policy | pushes accepted | groups emitted | final buffers |
|-------------|-----------------|----------------|---------------|
| `RejectNew` | 0 / 40 | 0 | A=2, B=2 |
| `DropOldest` | 40 / 40 | 0 | A=2, B=2 |

- Under `RejectNew` the buffers are full, so every subsequent push returns `BufferFull`
  forever. The node stops calibrating and never recovers.
- Under `DropOldest` the failure is worse to diagnose: every push is *accepted*, statistics
  show zero rejections and zero overflows, and still nothing is emitted. With
  `buffer_size = 2` the retained spread is one message period (~33 ms at 30 Hz), permanently
  below the 50 ms window, so `all_one()` never becomes true again.

Both states are unrecoverable short of destroying and rebuilding the synchronizer.

Normal operation survives only by accident: `ROS2Synchronizer._poll`
(`conflux_py/conflux_py/synchronizer.py:201`) drains to empty after every message, which keeps
`all_one()` true. Any transient divergence — a sensor hiccup, a slow detector, clock skew —
pushes both buffers to 2 and latches the fault.

## Suggested fix

Move the forced-progress rule into the shared core so both pipelines get it, rather than
patching the FFI separately:

1. Give `try_match` (or a new `State::advance`) the `is_full → drop_min` escape that
   `sync.rs` already implements, so a full, unmatchable state always makes progress.
2. Failing that, export `conflux_drop_min` across the C ABI and call it from the C++/Python
   wrappers when `is_full() && poll() == None`.
3. Add a regression test that drives the realtime preset through a divergence and asserts
   that group emission resumes.

Related: H-12 (the two pipelines diverge in the first place), M-23 (the wedge is invisible
from the outside).
