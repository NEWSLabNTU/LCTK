# M-19 · Staleness tracks messages before the ordering check, so rejected messages become ghost entries

- **Severity:** Medium
- **Area:** conflux-core (staleness subsystem)
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/state.rs:330-339`

## Problem

`State::push` registers the message with the staleness detector **before** the buffer decides
whether to accept it:

```rust
if let Some(ref mut staleness_detector) = self.staleness_detector {
    let staleness_timeout = item.timeout().unwrap_or_else(...);
    staleness_detector.add_message(key.clone(), item.clone(), staleness_timeout);
}

buffer.try_push(item).map_err(PushError::OutOfOrder)
```

When `try_push` rejects the message as out-of-order, the detector keeps a copy of a message
that never entered the buffer — a ghost entry.

The ghost is not merely a leak. `process_staleness_expiration` reconciles expired entries
against the buffers **by timestamp equality alone**:

```rust
if let Some(front_msg) = buffer.front()
    && front_msg.timestamp() == expired_message.timestamp()
{
    buffer.pop_front();
}
```

So when the ghost expires, it can evict a *different, legitimate* buffered message that
happens to carry the same timestamp — a real possibility for sensors sharing a trigger or for
messages whose stamps are quantized.

## Failure scenario

Out-of-order arrivals are normal under BEST_EFFORT QoS (this is what M-07 was about). Each
rejected message leaves a ghost; each ghost expiry has a chance to delete a valid message from
the front of a buffer. Symptoms are dropped groups with no corresponding rejection statistic.

## Suggested fix

Move the staleness registration **after** a successful `try_push`, so only messages that
actually entered a buffer are tracked. While there, make the reconciliation in
`process_staleness_expiration` identity-based rather than timestamp-based (see M-20).

Related: M-07 (same ordering-check class), M-20 (the reconciliation itself).

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
