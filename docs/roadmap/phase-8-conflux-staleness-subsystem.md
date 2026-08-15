# Phase 8: Conflux Staleness Subsystem — Repair or Remove

## Overview

The conflux staleness subsystem (`crates/conflux-core/src/staleness.rs`, 726 lines) is closer
to a prototype than a working feature. The 2026-08-15 audit found five defects, one of which
makes every tracked message expire immediately, and one of which is an acknowledged
placeholder in the source.

**This phase opens with a decision, not an implementation.** The subsystem is unreachable from
LCTK today — `conflux_cpp/rust/src/lib.rs:179` hardcodes `staleness_detector: None` — so
"remove it" is a legitimate and much cheaper outcome than "repair it". Stage 0 makes that call
before any code is written.

Prerequisite: [Phase 7](./phase-7-conflux-sync-correctness.md) Stage 2, which determines where
staleness ticking would live in a unified pipeline.

## Status

**H-11 is fixed** (`jerry73204/conflux`@bb490d9): expiry is anchored to message
arrival, with regression coverage in `staleness_anchor_tests.rs`. It was taken
ahead of the Stage 0 decision because it is a small, self-contained correctness
fix that is worth having under either path — and because leaving a known-wrong
expiry rule in place while the decision is pending is the worse option.

**Stage 0 is still open.** M-17 through M-21 remain, and the repair-vs-remove
question is unchanged: the subsystem is still unreachable from LCTK, still built
on the wrong clock for offline playback, and B1 is now the only step already done.

## Problem Statement

### The defects

| Issue | Sev | Summary |
|-------|-----|---------|
| [H-11](../issues/archive/H-11-conflux-staleness-anchored-to-construction.md) | High | Expiry anchored to construction time, not message arrival |
| [M-17](../issues/M-17-conflux-timer-wheel-loses-messages.md) | Medium | Timer wheel drains one slot per call; insertion double-counts elapsed time |
| [M-18](../issues/M-18-conflux-immediate-expiration-is-a-stub.md) | Medium | `enable_immediate_expiration` spawns a task that does nothing |
| [M-19](../issues/M-19-conflux-staleness-tracks-rejected-messages.md) | Medium | Messages are tracked before the ordering check → ghost entries |
| [M-20](../issues/M-20-conflux-expiration-only-removes-front.md) | Medium | Expired messages removed only if at a buffer front |
| [M-21](../issues/M-21-conflux-two-time-bases-for-expiry.md) | Medium | Wall clock vs message stamp used interchangeably |

### Why it never surfaced

Two reasons, both worth recording.

First, nothing reaches it: the FFI disables staleness, and LCTK only ever uses the FFI.

Second, the tests that covered it **had not compiled for an unknown period**
([H-13](../issues/archive/H-13-conflux-tokio-tests-never-compiled.md)). `staleness_tokio_tests.rs`
was never updated when `Config` gained an `Option<Duration>` window and a `drop_policy`
parameter — 20 call sites, 20 compile errors — and `just test-rust` omitted `--features tokio`,
so the file compiled to nothing and the suite reported green. Repaired 2026-08-15.

So the subsystem has effectively never been exercised, by tests or in production. Treat any
claim about its behaviour as unverified.

### The headline defect

```rust
reference_time: Instant::now(),                                   // staleness.rs:127, set once
let expiration_time = self.reference_time + staleness_timeout;    // staleness.rs:139
```

Expiry is measured from **synchronizer construction**. Any message arriving more than
`staleness_timeout` after construction is born expired — i.e. everything, within the first
second of a real run. The horizon guard does not catch it: `saturating_duration_since` returns
zero for a past instant, which passes the `> heap_time_horizon` check, so the entry is admitted
and drained as expired on the next tick.

Probe (10 s horizon, 300 ms sleep after construction, fresh message with a 200 ms timeout):

```
expired immediately: 1 (expected 0 for a fresh message)
```

## Stage 0 — Decide: repair or remove

Answer these before writing code:

1. **Is there a consumer?** Does any planned LCTK work need per-message expiry that the
   window and buffer bounds do not already provide? The realtime preset's job — bound latency,
   prefer newest data — is already served by `DropOldest` + a small buffer.
2. **Which clock?** (M-21.) For recorded playback — the LCTK default — wall-clock expiry is
   meaningless. If the answer is "message time", the heap/timer-wheel machinery built on
   `Instant` is the wrong structure and repair means a rewrite, not a patch.
3. **Is the complexity earned?** A constrained heap with coalescing, plus a timer wheel, plus
   a background tokio task, is a lot of machinery for "drop messages older than X". A sorted
   deque scan at push time would cover the same requirement in a fraction of the code, given
   buffers are bounded at 100 entries.

**Recommendation:** remove, unless Stage 0 identifies a concrete consumer. `Buffer::drop_expired`
+ `WithTimestamp::timeout` already provide message-time expiry through
`State::drop_expired_messages`, which is the semantics recorded data actually needs. That path
stays; the `Instant`-based subsystem goes.

**Exit:** a written decision in this document, with the consumer named or the removal
justified. Everything below is conditional on it.

## Path A — Remove (recommended default)

1. Delete `ConstrainedHeap`, `TimerWheel`, `StalenessDetector`, `ExpirationCommand` and the
   background task. Closes M-17, M-18, M-19, M-20 and H-11 outright.
2. Keep and document `Buffer::drop_expired` / `State::drop_expired_messages` as *the*
   expiration mechanism, explicitly message-time based (M-21).
3. Remove `staleness_config` from `Config`; keep `Config::with_staleness` as a deprecated
   shim for one release, or drop it — conflux is pre-1.0 and LCTK is the only consumer.
4. Rewrite `staleness_tokio_tests.rs` against the retained message-time path. Most of its 20
   tests describe behaviour worth keeping; the machinery under them is what goes.
5. Update the conflux CLAUDE.md and any docs referencing staleness presets.

**Exit:** `just test` green; `staleness.rs` gone or reduced to the message-time helpers;
line count down ~700.

## Path B — Repair

Only if Stage 0 names a consumer. In dependency order:

### B1 — Fix the anchor (H-11)

`let expiration_time = Instant::now() + staleness_timeout;` and delete `reference_time`, which
has no other use. Make the horizon check operate on a genuinely future instant so overflow
delegation to the wheel behaves as designed. Test: push well after construction, assert the
message survives its full timeout.

### B2 — Fix tracking and reconciliation (M-19, M-20)

- Move the `staleness_detector.add_message` call **after** a successful `try_push`, so only
  messages that entered a buffer are tracked.
- Add a monotonic per-push sequence number and reconcile by identity, not timestamp — stamps
  are not unique, and matching on them lets a ghost entry evict a valid message.
- Add `Buffer::remove_matching` so expiry works mid-buffer, not just at the front, and report
  an accurate removal count.

### B3 — Fix or delete the timer wheel (M-17)

- Drain every slot between the previous and new `current_slot`, not just one.
- Compute the insertion offset from `Instant::now()`, not `start_time`.
- Test by advancing a mock clock several slot durations in one step and asserting nothing is
  stranded.

Consider deleting the wheel even under Path B: it exists to absorb heap overflow, and with
buffers bounded at 100 the heap's 256-entry cap is never reached.

### B4 — Resolve `enable_immediate_expiration` (M-18)

Implement it — give the task a handle back into the detector so it can actually drain on the
timer, with a test asserting expiry without a poll — or delete the flag, the task, the channel
and the `panic!` at `staleness.rs:348`. Deleting is cheaper and matches actual usage. Do not
leave a third state where the flag exists and does nothing.

### B5 — Settle the clock (M-21)

Convert to message-time deadlines driven by the highest observed stamp. If wall-clock expiry
is genuinely wanted for live sensors with a stalled publisher, make it a separately named,
explicitly documented policy — not an implementation detail of the same knob.

### B6 — Wire it to the FFI

Repairing a subsystem no binding can reach is wasted work. Expose staleness configuration
through `ConfluxConfig`, the C++ wrapper and `conflux_py`, and add FFI-level tests. This
depends on Phase 7 Stage 2 having unified where ticking happens.

## Verification

- `just test-core` and `just test-rust` green **with `--features tokio`** — the flag that was
  missing while this code rotted.
- Under Path B: a soak test running longer than `heap_time_horizon` confirming messages expire
  on schedule rather than at construction + timeout.
- Under either path: `just demo mode=offline` on sample data produces the same group counts as
  before the change, since offline mode should be unaffected.

## Sequencing

Stage 0 first, and it may end the phase. Path A is a single focused change. Path B is
B1 → B2 → B3/B4 → B5 → B6, and should not start before Phase 7 Stage 2.
