# H-05 · Conflux FFI collapses all push errors to BufferFull → corrupted rejection stats

- **Severity:** High
- **Area:** conflux synchronizer (FFI + stats)
- **Status:** Fixed (2026-07-11, conflux submodule)
- **Verified:** Static review
- **Location:**
  - `ros/conflux/conflux_cpp/rust/src/lib.rs:240-243`
  - `ros/conflux/conflux_py/conflux_py/_ffi.py:251-256`
  - `ros/conflux/conflux_py/conflux_py/synchronizer.py:153-155`

## Problem

The FFI maps `LateMessage`, `OutOfOrder`, and `BufferFull` all to a single `ConfluxResult::BufferFull`. The Python layer then counts every one as a buffer rejection and logs "Buffer overflow on '<topic>'".

## Failure scenario

Under BEST_EFFORT / realtime, late and out-of-order messages are normal. They inflate `rejection_rate()` and emit false overflow warnings — even under DropOldest, which is supposed to "always accept". This defeats the synchronization stats feature that the CLAUDE.md profiling tables rely on.

## Suggested fix

Preserve distinct error variants across the FFI (add `LateMessage` / `OutOfOrder` result codes) and account for them separately in the stats. Only true buffer-overflow rejections should trigger the overflow warning.

## Resolution (2026-07-11)

Fixed in the conflux submodule (`jerry73204/conflux`, branch `fix/h02-h05-stats`,
commit da9f101; LCTK pins it). The FFI now maps each `PushError` to a distinct
`ConfluxResult` (`LateMessage=6`, `OutOfOrder=7`, `Timeout=8`, `UnknownKey ->
KeyNotFound`) instead of collapsing all to `BufferFull`; cbindgen regenerated
`conflux_ffi.h` accordingly. The Python binding records the last push result, and
`ROS2Synchronizer` now counts only `BufferFull` as a rejection / overflow warning,
tracking late and out-of-order drops in a separate `messages_dropped` statistic so
the rejection rate is no longer inflated. Verified by conflux's Rust FFI tests and
a full LCTK `just build` + `just test` (268 Rust, 48 Python). Should be upstreamed
to conflux `main` via PR.
