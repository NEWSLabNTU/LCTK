# H-12 · conflux has two divergent pipelines; the test suite covers the one production does not use

- **Severity:** High
- **Area:** conflux-core / conflux FFI
- **Status:** Open
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
