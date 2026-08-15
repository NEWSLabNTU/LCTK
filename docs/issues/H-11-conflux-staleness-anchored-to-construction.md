# H-11 · Conflux staleness expiry is anchored to construction time, not message arrival

- **Severity:** High
- **Area:** conflux-core (staleness subsystem)
- **Status:** Open
- **Verified:** Reproduced with a probe test against `ConstrainedHeap` (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/staleness.rs:115`, `:127`, `:139`

## Problem

`ConstrainedHeap::reference_time` is stamped once in `new()` and never advanced:

```rust
reference_time: Instant::now(),          // staleness.rs:127
...
let expiration_time = self.reference_time + staleness_timeout;   // staleness.rs:139
```

Expiry is therefore measured from **synchronizer construction**, not from when the message
arrived. Any message pushed more than `staleness_timeout` after construction is already
expired the moment it is tracked.

The temporal-constraint guard does not catch this. It reads:

```rust
if expiration_time.saturating_duration_since(now) > self.config.heap_time_horizon {
    return Err((key, message));   // delegate to timer wheel
}
```

For an `expiration_time` already in the past, `saturating_duration_since` returns zero, which
passes the check — so the entry is admitted to the heap and then drained as expired on the
very next `drain_expired()`.

## Failure scenario

Probe: build a `ConstrainedHeap` with a 10 s horizon, sleep 300 ms, add one fresh message with
a generous 200 ms staleness timeout, then drain.

```
expired immediately: 1 (expected 0 for a fresh message)
```

Every message is treated as stale once the synchronizer has been alive longer than the
configured timeout — i.e. within the first second of a real run. With staleness enabled the
buffers are continuously emptied of valid data and synchronization degrades to noise.

**Current blast radius is limited:** the FFI hardcodes `staleness_detector: None`
(`conflux_cpp/rust/src/lib.rs:179`), so no LCTK node reaches this path today. It is High
rather than Critical for that reason — but it means the feature cannot be switched on without
being fixed first, and the `--features tokio` test suite that covers it was itself not
compiling until 2026-08-15 (H-13).

## Suggested fix

Anchor expiry to the moment the message is tracked: `let expiration_time = Instant::now() +
staleness_timeout;`, and delete the `reference_time` field, which has no other purpose. Then
make the horizon check operate on a genuinely future instant so overflow delegation to the
timer wheel behaves as designed.

Add a test that pushes messages well after construction and asserts they survive their full
timeout.

Related: M-17, M-18, M-19, M-20, M-21 — the rest of the staleness subsystem.
