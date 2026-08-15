# L-23 · conflux-core carries a half-built feedback path, a dead assert, and large commented-out blocks

- **Severity:** Low
- **Area:** conflux-core
- **Status:** Open
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/state.rs:79-84`, `:99-120`, `:170-173`;
  `ros/conflux/crates/conflux-core/src/sync.rs:127-135`, `:179-187`, `:246-253`;
  `ros/conflux/crates/conflux-core/src/buffer.rs:59-76`, `:188-210`

## Problem

Three related pieces of decay in the core:

**1. The feedback path is half-built.** `Feedback::accepted_max_timestamp` is hardcoded to
`None` (`state.rs:111`) with its computation commented out immediately above. The FFI passes
`feedback_tx: None` and never reads feedback at all, so the entire mechanism is inert for
every shipped consumer.

**2. A dead assert sits in an FFI-reachable path.** `try_match` asserts
`item.timestamp() <= window_end` (`state.rs:172`). It cannot fire: `inf_ts` is the maximum of
all buffer fronts, so every popped front is ≤ `inf_ts` ≤ `inf_ts + window`. It is dead
weight — and a *live* assert on this path would be worse than useless, since a panic
unwinding through `extern "C"` aborts the whole ROS node.

**3. Commented-out code throughout.** `print_debug_info`, the feedback threshold computation,
`BackEntry`/`pop_back`/`front_ts` in `buffer.rs`, and several push/match blocks in `sync.rs`
that duplicate the live logic just above or below them. In `sync.rs` the commented blocks sit
next to near-identical live code, which makes the control flow hard to read while auditing.

## Failure scenario

No runtime impact. The cost is review friction: this audit spent time distinguishing live from
dead paths, and the duplicated commented blocks in `sync.rs:179-187` and `:246-253` initially
read as the active implementation.

## Suggested fix

- Either finish the feedback path (compute `accepted_max_timestamp`, wire `feedback_tx`
  through the FFI) or delete it along with `Feedback`'s unused fields.
- Delete the dead assert; if the invariant is worth stating, make it a `debug_assert!` with a
  comment explaining why it holds.
- Delete the commented-out blocks. Git history preserves them.

Related: H-12.
