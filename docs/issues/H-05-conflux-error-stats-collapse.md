# H-05 · Conflux FFI collapses all push errors to BufferFull → corrupted rejection stats

- **Severity:** High
- **Area:** conflux synchronizer (FFI + stats)
- **Status:** Open
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
