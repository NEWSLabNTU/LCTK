# H-14 · A third, independent synchronization implementation lives in `conflux-ros2`

- **Severity:** High
- **Area:** conflux-ros2 / conflux_node
- **Status:** Open
- **Verified:** Found while closing L-24 (2026-08-15); confirmed by reading the source
- **Location:** `ros/conflux/crates/conflux-ros2/src/ros2_sync_state.rs:194-260`,
  used from `ros/conflux/crates/conflux-ros2/src/ros2_sync_node.rs:124`

## Problem

[H-12](./archive/H-12-conflux-two-divergent-pipelines.md) was filed as "conflux has **two**
divergent pipelines" and closed by unifying them behind `State::advance`. That count was wrong.

`Ros2SyncState` in `conflux-ros2` is a **third**, entirely independent implementation. It does not
use `conflux_core::State` at all — it has its own buffers, its own `try_match`, its own drop logic,
and its own `is_ready`/`is_well_filled` predicates:

```rust
// crates/conflux-ros2/src/ros2_sync_state.rs
pub fn is_ready(&self) -> bool {
    self.buffers.values().all(|b| !b.is_empty())
}

pub fn try_match(&mut self) -> Option<IndexMap<String, Ros2Message>> {
    if !self.is_ready() { return None; }
    // ...its own inf_ts computation, window check, and drop-oldest fallback
}
```

It is used by `Ros2SyncNode`, which is what the standalone `conflux_node` package runs. So the
node advertised as *the* conflux ROS 2 node does not execute the algorithm that conflux-core's
test suite covers, and did not receive any of the C-05, H-11 or H-12 fixes.

## Failure scenario

Every hazard H-12 described applies again, one layer over:

- Fixes land in `conflux-core` and silently do not reach `conflux_node`.
- The semantics differ by construction. This implementation checks "are all fronts within one
  window of `inf_ts`" and drops the oldest when not — closer to the *relaxed* match added for C-05
  than to the original `try_match`, so its behaviour on the same input is a third answer again.
- Its emission gate is "all buffers non-empty", not the two-message `is_ready` the core used, so
  latency differs as well.

Nothing tells a reader which of the three they are getting; the choice is made by which crate they
depend on.

## Suggested fix

1. Establish whether `conflux_node` / `Ros2SyncNode` has any user. The LCTK pipeline does not use
   it — every solver reaches conflux through `conflux_py` → FFI — so deletion may be the cheapest
   correct answer, exactly as it was for the staleness subsystem.
2. If it is kept, reduce `Ros2SyncState` to a thin adapter over `conflux_core::State`, the same
   way `sync()` and `conflux_poll` were reduced in H-12. It should not own matching rules.
3. Either way, add a test asserting that whatever `conflux_node` runs produces the same group
   sequence as the core for identical input.

Related: [H-12](./archive/H-12-conflux-two-divergent-pipelines.md) (the same defect, one layer
down), [C-05](./archive/C-05-conflux-ffi-sync-wedges.md) (what divergence cost last time).
