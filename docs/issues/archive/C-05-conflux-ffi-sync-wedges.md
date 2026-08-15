# C-05 · Conflux FFI synchronizer wedges permanently after a stream divergence

- **Severity:** Critical
- **Area:** conflux-core / conflux FFI (all LCTK solver nodes)
- **Status:** Fixed (2026-08-15)
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

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@bb490d9; LCTK pins it). The forced-progress rule moved
into the shared core as `State::advance`, which both `conflux_poll` and `sync()`'s poll loop now
call (H-12), so the FFI can no longer lack an escape the Rust pipeline has.

Two conditions in `advance` are load-bearing and took iteration:

- The trigger is `any_full`, not `is_full`. A buffer at capacity cannot accept another message,
  so waiting on that stream is futile even while other buffers still have room. Using `is_full`
  (every buffer full) left the wedge intact, since the wedge state has one buffer full and one not.
- It must **not** fire while any buffer is empty. Forcing progress there lets `drop_min` take a
  slow stream's only message along with the fast stream's oldest, which silently destroyed
  matches for mixed-frequency inputs (caught by `realistic_timing_tests`).

Before dropping anything, `advance` retries through a new `try_match_relaxed`, which skips the
wait-for-spread rule while keeping the window validity check — emitting the earliest genuinely
valid group instead of holding out for a better one that can no longer form.

Regression coverage: `test_recovers_from_divergence_reject_new` and
`test_recovers_from_divergence_drop_oldest` in the `conflux-ffi` crate, plus
`pipeline_parity_tests.rs` in conflux-core.

Verified end-to-end through the built `libconflux_ffi.so`: the original reproduction now yields
19 groups (RejectNew) and 20 groups (DropOldest) where it previously yielded 0, with buffers
draining to empty. Clean aligned input is unchanged at 12/12 groups.
