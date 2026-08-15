# L-21 · `buf_size >= 2` is enforced without explanation

- **Severity:** Low
- **Area:** conflux-core / conflux_py
- **Status:** Open
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/sync.rs:51`,
  `ros/conflux/conflux_cpp/rust/src/lib.rs:137-139`

## Problem

Three layers reject a buffer size below 2, none of them says why:

- `sync()` — `ensure!(buf_size >= 2)`, producing a bare eyre error with no message
- the FFI — returns a null handle with no diagnostic at all
- `conflux_py` — raises `ValueError` (added 2026-08-15; before that the null handle surfaced
  as a generic `RuntimeError`, see M-24)

The real reason is `State::is_ready()`, which gates the `sync()` pipeline on every buffer
holding at least two messages; a capacity-1 buffer can never satisfy it and would deadlock.
That rationale appears nowhere in the API docs, the `Config` docstrings or the LCTK parameter
table.

## Failure scenario

A user tuning for minimum latency sets `sync_queue_size: 1`, which is a reasonable thing to
want, and gets a null handle or an unexplained error. Nothing indicates that 2 is the floor or
why, so the natural next step is to assume the library is broken.

## Suggested fix

- Give each check a message naming the reason and the floor, e.g. `buf_size must be >= 2
  (the matcher requires two messages per stream to establish ordering)`.
- Document the constraint on `Config`, `SyncConfig` and in the LCTK synchronizer-parameter
  table.
- Reconsider the constraint itself once H-12 unifies the pipelines: the FFI path matches via
  `all_one()` and does not actually need two messages per stream.

Related: H-12, M-24.
