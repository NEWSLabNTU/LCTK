# M-20 · Expired messages are only removed if they sit at the front of a buffer

- **Severity:** Medium
- **Area:** conflux-core (staleness subsystem)
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/state.rs:399-423`

## Problem

`State::process_staleness_expiration` drains the detector, then tries to remove each expired
message from its buffer — but only when it happens to be the front element, and only by
timestamp comparison:

```rust
// Since we can't remove specific messages from the middle of the buffer,
// we'll remove from the front if it matches the expired message
// This is a limitation of the current buffer implementation
if let Some(front_msg) = buffer.front()
    && front_msg.timestamp() == expired_message.timestamp()
{
    buffer.pop_front();
    removed_count += 1;
}
```

Anything expiring mid-buffer is dropped from the detector and silently retained in the buffer,
forever. The detector and the buffers drift permanently out of agreement, and `removed_count`
under-reports.

The comment acknowledges the defect. `Buffer` wraps a `VecDeque`, so removal from the middle
is entirely possible — the limitation is that `Buffer` exposes no method for it, not that the
structure forbids it.

## Failure scenario

Streams with mixed per-message timeouts (`WithTimestamp::timeout`) expire out of buffer order
by construction. Those messages stay buffered past their deadline and are matched into groups
as if fresh, which is exactly what staleness exists to prevent. Buffer occupancy also stops
being bounded by the staleness horizon.

## Suggested fix

- Add `Buffer::remove_matching` (or retain-by-predicate) and reconcile by message identity —
  a monotonic sequence number assigned at push — rather than by timestamp, which is not unique.
- Have `process_staleness_expiration` remove every expired entry wherever it sits, and report
  an accurate count.

Fixing the identity comparison also closes the ghost-entry eviction risk in M-19.

Related: M-19, H-11.

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
