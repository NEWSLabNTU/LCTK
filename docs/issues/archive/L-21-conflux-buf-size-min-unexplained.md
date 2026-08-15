# L-21 · `buf_size >= 2` is enforced without explanation

- **Severity:** Low
- **Area:** conflux-core / conflux_py
- **Status:** Fixed (2026-08-15)
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

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@0a9c901; LCTK pins it). Every layer that enforces the floor
now states it and the reason — the matcher needs room for a second message per stream to compare
candidate pairings:

- `sync()`'s `ensure!` carries a message instead of failing bare.
- `SyncConfig` validates eagerly in `__post_init__`, so the error lands at the point the bad value
  was written rather than later at `Synchronizer()`.
- `FFISynchronizer` keeps its own check for callers that bypass `SyncConfig`.

Regression coverage: `test_buffer_size_error_explains_the_floor`,
`test_create_synchronizer_invalid_buffer_size` and `test_ffi_layer_also_rejects_invalid_buffer_size`
in `conflux_py`.
