# H-14 · A third, independent synchronization implementation lives in `conflux-ros2`

- **Severity:** High
- **Area:** conflux-ros2 / conflux_node
- **Status:** Fixed (2026-08-15)
- **Verified:** Found while closing L-24 (2026-08-15); confirmed by reading the source
- **Location:** `ros/conflux/crates/conflux-ros2/src/ros2_sync_state.rs:194-260`,
  used from `ros/conflux/crates/conflux-ros2/src/ros2_sync_node.rs:124`

## Problem

[H-12](./H-12-conflux-two-divergent-pipelines.md) was filed as "conflux has **two**
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

Related: [H-12](./H-12-conflux-two-divergent-pipelines.md) (the same defect, one layer
down), [C-05](./C-05-conflux-ffi-sync-wedges.md) (what divergence cost last time).

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@a3dcb23; LCTK pins it). `Ros2SyncState` is now ~170 lines
of adapter with no matching logic of its own: it registers topics, forwards pushes, and delegates
`try_match` to `State::advance`. It gains `reset()` (M-22) and `match_status()` (M-23) for free,
neither of which it previously had.

### What made the duplicate possible to delete

The duplication had a real cause rather than being an oversight: `Ros2Message` wraps a
`DynamicMessage`, which is not `Clone`, and `conflux_core::State` required `T: Clone`. That bound
existed **only** for the staleness detector — no production code path ever cloned a message. So
removing staleness (M-17…M-21) also removed the reason this code had to exist. Dropping `T: Clone`
from `State` and `sync()` lets the core hold move-only messages, and the duplicate had nothing left
to justify it.

`Ros2Message` also gained the `WithTimestamp` impl it never had; it had carried a bare `timestamp`
field instead, which is part of how it drifted out of the core's type system in the first place.

### Correction to this issue's premise

**This pipeline was not exposed to C-05,** contrary to what is written above. The old
implementation had no wait-for-spread rule — it dropped the oldest message whenever the fronts did
not all fit the window — so it could not wedge the way the core did.

Replaying the C-05 scenario against a faithful reproduction of the old algorithm gives **40/40
pushes accepted and 20 groups**, the same as the fixed core. The divergence was real but ran in the
other direction: this pipeline discarded data more eagerly and produced a third answer on the same
input. What it genuinely lacked was `reset`, status reporting, and any real test coverage.

Behaviour for `conflux_node` is unchanged on that scenario — old and new both emit 20 groups.
`DropOldest` is kept as the default policy, preserving the old buffers' always-evict-oldest
behaviour.

### Why it was never caught

`conflux-ros2` is excluded from the cargo workspace — its ROS message dependencies are wildcards
patched by colcon at build time — so **nothing ever ran its tests**. The tests it shipped with
exercised a bare `VecDeque` rather than any of the real code. `just test-ros2` now runs them and is
wired into `test-rust`; it was verified to fail (exit 101) on a deliberately broken assertion,
per the standing lesson from H-13, M-25 and L-22.

Regression coverage: seven tests in `crates/conflux-ros2/src/ros2_sync_state.rs`, including the
C-05 divergence scenario, an M-22 clock-jump recovery, and an M-23 status check — the three
behaviours this pipeline previously could not exhibit at all.
