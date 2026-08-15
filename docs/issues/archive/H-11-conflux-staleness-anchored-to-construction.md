# H-11 · Conflux staleness expiry is anchored to construction time, not message arrival

- **Severity:** High
- **Area:** conflux-core (staleness subsystem)
- **Status:** Fixed (2026-08-15)
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

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@bb490d9; LCTK pins it). Deadlines are now measured from
`Instant::now()` at the moment the message is tracked, and the `reference_time` field — which had
no other purpose — is gone. The horizon check now compares `staleness_timeout` against
`heap_time_horizon` directly, instead of a `saturating_duration_since` that silently returned zero
for already-past instants and therefore always passed.

Regression coverage: `crates/conflux-core/tests/staleness_anchor_tests.rs` — a message pushed long
after construction keeps its full timeout, a message still expires once its own timeout elapses,
and expiry order follows arrival rather than construction.

**Two existing tests were rewritten, not relaxed**, because they asserted outcomes that only held
while the bug was present:

- `test_staleness_with_actual_delays` placed its long delay *between* two complete pairs, so no
  message ever waited alone past its timeout. It now strands one message, which is what the test
  always meant to exercise.
- `test_message_popped_before_matching` asserted which partner survived. `b1` is only microseconds
  old when it arrives, so it is not stale, and the 50 ms window legitimately admits
  `(a2=1250, b1=1200)`. The test now asserts what staleness actually owes: the stale `a1` is gone.

The subsystem remains unreachable from LCTK — the FFI still hardcodes `staleness_detector: None`.
M-17 through M-21 remain open; see [Phase 8](../../roadmap/phase-8-conflux-staleness-subsystem.md)
for the repair-vs-remove decision.
