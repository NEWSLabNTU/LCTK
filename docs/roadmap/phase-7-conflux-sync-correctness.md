# Phase 7: Conflux Synchronizer Correctness

## Overview

This phase repairs the message synchronizer that every LCTK solver node depends on. It covers
the 2026-08-15 conflux audit's correctness findings: the permanent wedge in the FFI matching
path (C-05), the structural duplication that allows it (H-12), the unrecoverable clock-reset
state (M-22), and the missing observability that makes all of them invisible (M-23).

Staleness is deliberately **out of scope** here — it is unreachable from LCTK today and is
handled separately in [Phase 8](./phase-8-conflux-staleness-subsystem.md). API ergonomics and
test tooling are in [Phase 9](./phase-9-conflux-api-and-tooling.md).

Work lands in the `jerry73204/conflux` submodule, then LCTK bumps the pin.

## Problem Statement

conflux ships two implementations of the pipeline over one shared `State`:

| | `sync()` (pure Rust) | FFI (`conflux_cpp/rust`) |
|---|---|---|
| Driver | `poll()` stream state machine | `conflux_poll` → `try_match()` only |
| Forced progress when full | `is_full → drop_min` | **none** |
| Staleness | configurable | hardcoded `None` |
| Feedback channel | `watch` sender wired | `feedback_tx: None` |
| Emission gate | `is_ready()` (≥2 per buffer) | `try_match` directly (`all_one` emits) |

Every solver node uses the FFI path. The 156 core tests almost entirely exercise `sync()`.

`State::try_match` refuses to emit when the buffered spread is narrower than the window:

```rust
if !self.all_one() && inf_ts + window_size > sup_ts {
    return None;
}
```

The only escape is `all_one()` — every buffer down to one message. `sync()` has a second
escape (`is_full → drop_min`); the FFI has none. Once every buffer holds ≥2 messages with the
spread under the window, the FFI synchronizer never emits again.

Measured against the shipped realtime preset (`window=50ms, buffer=2`), after a brief stream
divergence, with 40 perfectly aligned fresh pushes and a full drain loop after each:

| drop policy | pushes accepted | groups emitted | final buffers |
|-------------|-----------------|----------------|---------------|
| `RejectNew` | 0 / 40 | 0 | A=2, B=2 |
| `DropOldest` | 40 / 40 | 0 | A=2, B=2 |

Normal operation survives only because `ROS2Synchronizer._poll` drains to empty after every
message, keeping `all_one()` true. Any transient divergence latches the fault.

## Scope

| Issue | Sev | Summary |
|-------|-----|---------|
| [C-05](../issues/C-05-conflux-ffi-sync-wedges.md) | Critical | FFI synchronizer wedges permanently after a stream divergence |
| [H-12](../issues/H-12-conflux-two-divergent-pipelines.md) | High | Two divergent pipelines; tests cover the unshipped one |
| [M-22](../issues/M-22-conflux-last-ts-never-resets.md) | Medium | Clock reset kills a stream permanently (`last_ts` never resets) |
| [M-23](../issues/M-23-conflux-stall-is-unobservable.md) | Medium | A stalled synchronizer is unobservable |
| [L-17](../issues/L-17-conflux-is-empty-means-any-empty.md) | Low | `is_empty()` means "any buffer empty" |
| [L-24](../issues/L-24-conflux-sync-is-ready-latency.md) | Low | `sync()` withholds a matched pair until every stream has 2 messages |
| [L-23](../issues/L-23-conflux-core-dead-code.md) | Low | Half-built feedback path, dead assert, commented-out blocks |

## Stages

### Stage 1 — Reproduce and pin the behaviour (no fixes)

Before changing anything, lock the current semantics into tests so the refactor in Stage 2 is
provably behaviour-preserving where intended and behaviour-changing only where meant.

1. Add an FFI-level integration suite in `conflux_cpp/rust/tests/` driving
   `conflux_push_message` / `conflux_poll` directly. Today the FFI crate has three smoke tests.
2. Port the C-05 reproduction into it as a `#[should_panic]`-style **expected-failure** test,
   or an `#[ignore]`d test with a comment, so the wedge is recorded before it is fixed.
3. Add a matrix test over `{RejectNew, DropOldest} × {finite, infinite window} ×
   {buffer 2, 8, 100}` asserting group counts for clean aligned input. This is the regression
   net for Stage 2.

**Exit:** the wedge is reproducible from `cargo test` in CI, not just by hand.

### Stage 2 — Unify the pipeline (H-12)

The fix for C-05 belongs in the shared core, not bolted onto the FFI, or the two paths drift
again.

1. Lift the poll loop's rules out of `sync.rs` into `State`: forced progress when full,
   emission gating, and the ready/all-one decision. Introduce something like
   `State::advance() -> Option<IndexMap<K,T>>` that encapsulates "match if possible, otherwise
   make progress".
2. Reduce `sync()`'s `poll()` to a thin adapter over `advance()`.
3. Reduce `conflux_poll` to the same adapter.
4. Decide `is_ready()`'s fate (L-24) explicitly: if the two-message gate is meant to establish
   ordering, apply it once at startup; if it is vestigial, drop it. Either way both paths get
   the same answer, and the chosen latency characteristic is documented.

**Exit:** `sync()` and the FFI produce identical group sequences for identical input. Add a
differential test that asserts exactly this over randomized schedules.

**Risk:** this changes `sync()`'s emission timing for existing pure-Rust consumers. conflux
is pre-1.0 and LCTK is the only known consumer, but the change belongs in release notes.

### Stage 3 — Recovery from unrecoverable states (C-05, M-22)

1. With Stage 2 in place, C-05 is closed by construction: `advance()` always makes progress
   when full. Flip the Stage 1 expected-failure test to a passing regression test.
2. Add `Buffer::reset()` clearing the deque **and** `last_ts`, plumbed through `State`, a new
   `conflux_synchronizer_reset` in the C ABI, and `conflux_py`.
3. Detect the clock-reset case: a push whose stamp precedes `last_ts` by more than a
   configurable margin is a reset, not a late message. Log it distinctly; offer auto-reset as
   an opt-in policy rather than a default.
4. Have `ROS2Synchronizer` reset on a detected time jump when `use_sim_time` is set, so
   `ros2 bag play --loop` and sample-data restarts stop killing the solver.

**Exit:** a synchronizer survives a replay restart and a divergence without operator action.

### Stage 4 — Make the matcher observable (M-23)

The C-05 investigation needed information the library does not expose. Close that gap.

1. Export the matcher's own view across the FFI: `inf_ts`, `sup_ts`, current spread, and the
   shortfall against the window (`inf + window - sup`).
2. Add a `blocked_because` enum — `WaitingForData`, `SpreadTooNarrow`, `WindowExceeded`,
   `BufferFullNoMatch` — set on every `advance()` that declines to emit.
3. Add "time since last emitted group" and surface all of it through `conflux_py`.
4. Have `ROS2Synchronizer` warn, rate-limited, when it has accepted messages on every topic
   but emitted nothing for N seconds. This one check would have turned C-05 into a log line.
5. Rename `is_empty()` → `has_empty_buffer()` across core/FFI/Python, keeping a deprecated
   alias for one release (L-17).

**Exit:** an operator can answer "why is it not matching?" from logs alone.

### Stage 5 — Cleanup (L-23)

1. Finish or delete the feedback path. `accepted_max_timestamp` is hardcoded `None` with its
   computation commented out; the FFI never reads feedback at all. Deleting is the default
   unless a consumer is identified.
2. Delete the dead assert in `try_match` — it cannot fire, and a live assert on an
   `extern "C"` path aborts the ROS node.
3. Delete the commented-out blocks in `state.rs`, `sync.rs`, `buffer.rs`. Git history keeps
   them.

**Exit:** `just lint` clean; no commented-out logic adjacent to live logic.

## Verification

- `just test` in conflux: core + FFI + Python suites all green, with the new FFI integration
  and differential suites included.
- The C-05 matrix reproduction emits groups under every policy/buffer combination.
- End-to-end on sample data: `just demo mode=realtime` and `mode=offline`, confirming group
  counts and that a mid-run sensor stall recovers rather than latching.
- A replay restart (`just sample-data` restarted under a running solver) resumes calibration.

## Sequencing

Stage 1 → 2 → 3 gates the critical fix. Stage 4 is independent of 2–3 and can run in parallel;
it is the highest-value work per hour for field debugging. Stage 5 is cleanup and can land
anytime after Stage 2.

Phase 8 (staleness) should start only after Stage 2, since unifying the pipeline determines
where staleness ticking belongs.
