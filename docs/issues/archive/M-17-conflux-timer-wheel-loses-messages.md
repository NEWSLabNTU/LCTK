# M-17 · Conflux staleness timer wheel skips slots and misplaces messages

- **Severity:** Medium
- **Area:** conflux-core (staleness subsystem)
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/staleness.rs:283-296` (`add_message`),
  `:298-322` (`advance_and_collect_expired`)

## Problem

Two independent defects in `TimerWheel`, which is where the constrained heap delegates every
message it cannot admit (size or horizon overflow).

**1. Only one slot is ever drained.** `advance_and_collect_expired` scans
`self.slots[self.current_slot]`, then recomputes `current_slot` from elapsed wall time:

```rust
let current_messages = &mut self.slots[self.current_slot];
current_messages.retain(|(key, message, exp_time)| { ... });
let time_passed = now.saturating_duration_since(self.start_time);
self.current_slot = (time_passed.as_nanos() / self.slot_duration.as_nanos()) as usize
                    % self.slots.len();
```

If more than one `slot_duration` elapsed between calls — trivially true with a 5–10 ms slot
and a ~100 ms ICP cycle — the intervening slots are jumped over and never drained. Their
messages sit in the wheel until it wraps all the way around, if it ever does.

**2. Insertion double-counts elapsed time.** `add_message` computes the slot offset relative
to `start_time`, then adds it to `current_slot`, which is *itself* derived from elapsed time:

```rust
let slots_from_now = expiration_time.saturating_duration_since(self.start_time).as_nanos()
                     / self.slot_duration.as_nanos();
let slot_index = (self.current_slot + slots_from_now as usize) % self.slots.len();
```

The offset should be measured from *now*, not from `start_time`. As written, a message lands
roughly `current_slot` slots later than intended and expires late or wraps into the past.

## Failure scenario

Messages delegated to the wheel expire at the wrong time or never expire, so staleness
silently stops bounding buffer occupancy. Combined with H-11 — which pushes *everything* into
the "already expired" regime — the subsystem's behaviour is not predictable from its config.

Not currently reachable from LCTK (the FFI disables staleness entirely), but it blocks turning
the feature on.

## Suggested fix

- Drain every slot from the previous `current_slot` up to the new one, not just one.
- Compute `slots_from_now` from `Instant::now()`, and drop `start_time` from the insertion
  path entirely.
- Add tests that advance a mock clock by several slot durations at once and assert nothing is
  stranded.

Given H-11, M-18, M-19 and M-20, consider whether the wheel is worth repairing at all — see
the phase doc for the repair-vs-remove decision.

Related: H-11, M-18.

## Resolution (2026-08-15) — removed

Closed by removing the staleness subsystem entirely (`jerry73204/conflux`@014a2c9; LCTK pins it),
per [Phase 8](../../roadmap/phase-8-conflux-staleness-subsystem.md) Stage 0. `ConstrainedHeap`,
`TimerWheel`, `StalenessDetector` and the placeholder background task are gone — about 700 lines
of source and 22 tests of the deleted machinery.

The decision rested on three facts, not on this defect alone:

- **Nothing reached it.** The FFI hardcoded `staleness_detector: None`, so no binding — and
  therefore no LCTK node — ever executed this code.
- **Every part of it was defective.** M-17 through M-21 were found in a single reading pass, and
  H-11 (expiry anchored to construction time) meant the subsystem had never worked correctly at all.
- **It was built on the wrong clock.** M-21: expiry ran on `Instant` while the rest of the pipeline
  runs on message time. For recorded playback — LCTK's default mode — wall-clock expiry is
  meaningless, so repair would have meant a rewrite rather than a patch.

**What remains** is `Buffer::drop_expired` / `State::drop_expired_messages`, driven by
`WithTimestamp::timeout`: message-time expiry, which is the semantics recorded data needs. Its
contract was pinned by `timeout_tests.rs` *before* the deletion and is unchanged after it.

The `tokio` feature on `conflux-core` went too, since it gated nothing else; `conflux-ros2` no
longer requests it. The `staleness:` block was dropped from `conflux_node`'s YAML schema — a
leftover key in an old config is ignored, and `config/example.yaml` records why.
