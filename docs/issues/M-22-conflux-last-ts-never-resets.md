# M-22 · A stream whose clock goes backwards is permanently dead (`last_ts` never resets)

- **Severity:** Medium
- **Area:** conflux-core
- **Status:** Open
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/buffer.rs:80-82`, `:140-153`

## Problem

`Buffer` keeps a monotonic high-water mark and rejects anything at or below it:

```rust
match self.last_ts {
    Some(last_ts) if last_ts >= timestamp => return Err(item),
    _ => {}
}
```

`last_ts` is set on every accepted push and **never cleared** — not when the buffer drains to
empty, not on match, not through any public API. There is no `reset()`, and neither the FFI
nor `conflux_py` exposes one.

Correct for ordering within a monotonic stream. Fatal when the stream's clock legitimately
goes backwards.

## Failure scenario

Every case where a timestamp source restarts:

- `ros2 bag play --loop`, or restarting `just sample-data` while the solver keeps running
- switching `use_sim_time`, or a sim-time reset
- a sensor reconnecting and restarting its stamp counter
- any node using `/clock` across a jump

The affected buffer rejects **every** subsequent message as `OutOfOrder`, forever. Because
`State::try_match` needs all buffers non-empty, one dead stream stalls the whole
synchronizer — silently, since out-of-order rejections are (correctly, per H-05) not counted
as overflows. The operator sees a solver that simply stops producing transforms after a
replay restart.

## Suggested fix

- Add `Buffer::reset()` clearing both the deque and `last_ts`, plumbed through `State`, the
  FFI (`conflux_synchronizer_reset`) and `conflux_py`.
- Optionally detect the case: a push whose stamp precedes `last_ts` by more than some large
  margin is a clock reset rather than a late message — log it distinctly and offer automatic
  reset as an opt-in policy.
- `ROS2Synchronizer` should reset on a detected time jump when `use_sim_time` is set.

Related: C-05 (the other unrecoverable-state issue), M-23 (invisible from outside).
