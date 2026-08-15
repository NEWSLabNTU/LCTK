# M-20 · Expired messages are only removed if they sit at the front of a buffer

- **Severity:** Medium
- **Area:** conflux-core (staleness subsystem)
- **Status:** Open
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
