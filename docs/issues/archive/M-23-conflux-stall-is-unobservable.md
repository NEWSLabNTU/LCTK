# M-23 · A stalled synchronizer is unobservable — statistics look perfect while nothing is emitted

- **Severity:** Medium
- **Area:** conflux_py / conflux FFI (diagnosability)
- **Status:** Fixed (2026-08-15)
- **Verified:** Reproduced alongside C-05 (2026-08-15)
- **Location:** `ros/conflux/conflux_py/conflux_py/synchronizer.py` (`SyncStatistics`),
  `ros/conflux/conflux_cpp/rust/src/lib.rs` (C ABI surface)

## Problem

The exported observability surface is: messages received, messages rejected, groups
synchronized, rejection rate, per-topic buffer length, `is_ready`, `is_empty`. All of it
describes *inputs*. Nothing describes why the matcher is not producing *outputs*.

When the synchronizer wedges under `DropOldest` (C-05), the reported state is:

- rejections: **0**
- overflow warnings: **none**
- buffer lengths: at capacity, which looks healthy
- groups synchronized: **0**, and no explanation

Every push is accepted and every metric is nominal. The operator sees a solver that produces
no transforms, with a statistics block that says everything is fine.

`is_empty()` is no help either — it reports whether *any* buffer is empty (L-17), not whether
the synchronizer is idle.

## Failure scenario

Diagnosing C-05 in the field means attaching to a running node and reasoning about `inf`/`sup`
timestamps that are not exposed anywhere. There is no way to answer the operator's actual
question — "why is it not matching?" — from logs or topics.

## Suggested fix

Export the matcher's own view across the FFI and surface it in `conflux_py`:

- `inf_ts`, `sup_ts`, and the current spread per synchronizer
- the shortfall against the window (`inf + window - sup`), i.e. how far from emitting
- a `last_match_reason` / `blocked_because` enum: `WaitingForData`, `SpreadTooNarrow`,
  `WindowExceeded`, `BufferFullNoMatch`
- a monotonic "time since last emitted group"

Then have `ROS2Synchronizer` warn (rate-limited) when it has accepted messages on every topic
but emitted nothing for N seconds — the signature of a wedge. That check alone would have
turned C-05 from an investigation into a log line.

Related: C-05, L-17, M-22.

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@014a2c9; LCTK pins it). `State::match_status()` returns the
matcher's own view — `inf_ts`, `sup_ts`, the current spread, the shortfall against the window, and
a `BlockedReason` of `WaitingForData`, `SpreadTooNarrow` or `BufferFullNoMatch`. It is read-only,
so it is safe to call on every poll.

The same view is exported as `conflux_get_status` / `ConfluxStatus` across the C ABI (nanoseconds,
with `-1` for "not applicable"), and reaches Python as `Synchronizer.status`, returning a
`MatchStatus` dataclass with an `is_stalled` convenience property.

`ROS2Synchronizer` now warns — once per stall, and only after every topic has delivered at least
one message — when nothing has been emitted for `stall_warn_after` seconds (default 10, 0 to
disable). The message names the blocked reason, the per-topic buffer occupancy, and how far short
of the window the spread is, then points at clock sync and `sync_tolerance_ms`.

That check is aimed squarely at the failure this issue was filed for: under C-05's DropOldest
wedge, every push was accepted and the statistics showed zero rejections while nothing came out.
`BufferFullNoMatch` is exactly that shape.

Regression coverage: `crates/conflux-core/tests/observability_tests.rs` (including a case asserting
`match_status` mutates nothing), `test_status_reports_blocked_reason` in the `conflux-ffi` crate,
and three `TestStatus` cases in `conflux_py`.
