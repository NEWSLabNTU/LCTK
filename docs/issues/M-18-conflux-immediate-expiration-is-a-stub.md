# M-18 · `enable_immediate_expiration` spawns a task that does nothing

- **Severity:** Medium
- **Area:** conflux-core (staleness subsystem)
- **Status:** Open
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/staleness.rs:380-416`

## Problem

`StalenessDetector::start_background_task` spawns a tokio task whose timer branch is empty:

```rust
tokio::select! {
    _ = tokio::time::sleep(sleep_duration) => {
        // Process expired messages
        // In a real implementation, we'd need a way to communicate back to the detector
        // For now, this is a placeholder that demonstrates the structure
    }
    ...
}
```

`ExpirationCommand::ProcessExpired` is likewise handled with a bare comment. The task consumes
a tokio task slot and an unbounded channel, reschedules a timer, and never expires anything.
Expiration only ever happens lazily, when `State::process_staleness_expiration` is called from
the poll loop.

The option is not marked experimental. `StalenessConfig::high_frequency()` and
`low_frequency()` both set `enable_immediate_expiration: true`, so the two presets a user is
most likely to pick are the two that promise behaviour the code does not implement. Worse,
constructing a detector with it set while the `tokio` feature is off triggers an outright
`panic!` (`staleness.rs:348`) — the flag either aborts the process or does nothing.

## Failure scenario

A user selects `high_frequency()` expecting sub-100 ms expiration, gets lazy expiration tied
to poll cadence, and sees stale messages matched into groups. Nothing logs or warns. Debugging
leads into a background task that is structurally convincing and functionally inert.

## Suggested fix

Pick one:

- **Implement it** — give the task a handle back into the detector (`Arc<Mutex<...>>` or a
  command/response pair) so it can actually drain on the timer, and cover it with a test that
  asserts expiry happens without a poll.
- **Remove it** — delete the flag, the task, the channel and the `panic!`, and document that
  expiration is lazy. Lazy expiration is sufficient for every current caller.

Removal is the cheaper option and matches how the subsystem is actually used.

Related: H-11, M-17.
