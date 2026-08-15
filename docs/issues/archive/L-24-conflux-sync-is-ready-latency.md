# L-24 · `sync()` holds a matched pair until every stream has two messages

- **Severity:** Low
- **Area:** conflux-core
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/sync.rs:128`,
  `ros/conflux/crates/conflux-core/src/state.rs:241-244`

## Problem

The `sync()` poll loop will not attempt a match until `State::is_ready()` holds — every buffer
carrying **at least two** messages:

```rust
pub fn is_ready(&self) -> bool {
    self.buffers.values().all(|buffer| buffer.len() >= 2)
}
```

`try_match` itself has no such requirement; it happily emits when `all_one()` is true. So a
perfectly aligned pair that is ready to emit is deliberately withheld until a *second* message
arrives on every stream — one additional inter-message period of latency per stream, bounded
by the slowest stream's rate.

The FFI path does not go through the poll loop and therefore does not pay this cost: it calls
`try_match` directly and emits immediately.

## Failure scenario

Two consumers of the same library see different end-to-end latency for identical input, with
nothing in the configuration to explain the difference. For a 10 Hz LiDAR paired with a 30 Hz
camera, the `sync()` path adds ~100 ms before the first group appears and keeps one extra
message of lag thereafter.

The gate also means `sync()` cannot emit at all from a stream that produces exactly one
message and then stops, until the input stream is depleted.

## Suggested fix

Decide the intended semantics and make both pipelines share it (H-12):

- If the two-message gate exists to establish ordering before matching, apply it once at
  startup rather than continuously.
- If it is vestigial, drop it and let `try_match`'s own `all_one()` / spread rules govern.

Either way, document the latency characteristic in the performance section.

Related: H-12, C-05.

## Resolution (2026-08-15) — closed by the H-12 rewrite

No separate fix was needed. The `is_ready()` emission gate was removed when `sync()`'s poll loop
was rewritten to route through `State::advance`
([H-12](./H-12-conflux-two-divergent-pipelines.md), `jerry73204/conflux`@bb490d9). `sync()` no
longer withholds a formable group waiting for a second message on every stream, so its latency now
matches the FFI path's.

Verified by inspecting every `is_ready` call site: it survives only as a public predicate on
`State`, the C ABI and `conflux_py`, with no caller in the matching path.

This was deliberately left open when H-12 closed, rather than assumed closed, until that check was
actually run.
