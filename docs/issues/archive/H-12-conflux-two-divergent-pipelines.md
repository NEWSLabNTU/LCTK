# H-12 · conflux has two divergent pipelines; the test suite covers the one production does not use

- **Severity:** High
- **Area:** conflux-core / conflux FFI
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review + test-path inspection (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/sync.rs:104-289` vs
  `ros/conflux/conflux_cpp/rust/src/lib.rs:124-190`, `:293-330`

## Problem

conflux ships two independent implementations of the synchronization pipeline over one shared
`State`:

| | `sync()` (pure Rust) | FFI (`conflux_cpp/rust`) |
|---|---|---|
| Driver | `poll()` stream state machine | `conflux_poll` → `try_match()` only |
| Forced progress when full | `is_full → drop_min` | **none** |
| Staleness | configurable | **hardcoded `None`** (`lib.rs:179`) |
| Feedback channel | `watch` sender wired | **`feedback_tx: None`** |
| Emission gate | `is_ready()` (≥2 per buffer) | `try_match` directly, so `all_one` emits |

The FFI does not *use* `sync()`; it constructs `State` by hand and calls into it. So the two
paths have genuinely different semantics, not just different wrappers.

Every LCTK solver node — `extrinsic_solver_node`, `advanced_extrinsic_solver`,
`lidar_to_lidar_solver` — reaches conflux through `conflux_py` → FFI. **None of them exercise
`sync()`.** Meanwhile conflux-core's 156 tests are overwhelmingly written against `sync()`.

## Failure scenario

The tested path and the shipped path disagree, so tests pass while production misbehaves.
C-05 is the concrete instance: `sync()` cannot wedge because its poll loop calls `drop_min()`;
the FFI has no such escape and wedges permanently. The core test suite is green throughout.

The same gap hides latency differences (`sync()` holds a matched pair until every stream has
2 messages; the FFI emits immediately via `all_one`) and means the staleness subsystem
(H-11, M-17…M-21) has never run in production at all.

## Suggested fix

Collapse the duplication rather than testing both:

1. Lift the pipeline rules — forced progress, emission gating, staleness ticking — out of
   `sync()`'s poll loop and into `State`, so both drivers share one implementation and one
   set of semantics.
2. Reduce `sync()` and `conflux_poll` to thin adapters over that shared core.
3. Add an FFI-level integration suite (drive `conflux_push_message` / `conflux_poll` directly)
   so the shipped ABI has coverage independent of the Rust stream API.

Related: C-05, H-11, L-24.

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@bb490d9; LCTK pins it). The matching policy now lives in
one place — `State::advance` — and both drivers call it. `sync()`'s poll loop was reduced from a
three-branch state machine carrying its own copy of the rules to a feeder that pushes input and
drains at end of stream (`sync.rs` shrank by ~90 lines).

The divergence was not theoretical. On identical input — 12 aligned pairs, 50 ms window,
buffer 8 — `sync()` emitted **11** groups where the FFI emitted **12**, silently dropping the pair
at t=1330 when its buffers filled. After unification both emit 12.

Regression coverage: `crates/conflux-core/tests/pipeline_parity_tests.rs` pins `sync()`'s side of
the contract (divergence recovery under both drop policies, and one group per aligned pair); the
FFI side is covered by `test_recovers_from_divergence_*` in the `conflux-ffi` crate.

Still outstanding from this issue's suggested fix: the FFI integration suite is still thin
(L-22), and the `is_ready()` emission-gate difference is tracked separately as L-24.
