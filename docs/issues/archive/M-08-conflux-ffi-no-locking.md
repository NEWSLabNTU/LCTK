# M-08 · Mutable Rust conflux State crosses FFI with no locking

- **Severity:** Medium
- **Area:** conflux_cpp FFI
- **Status:** Fixed (2026-07-11)
- **Verified:** Static review
- **Location:** `ros/conflux/conflux_cpp/rust/src/lib.rs:219, 289`

## Problem

`conflux_push_message` and `conflux_poll` take `&mut State`. ctypes releases the GIL during the FFI call, and the C++ mutex guards only the pending map, not the Rust `State`. Under a `MultiThreadedExecutor`, concurrent push/poll produce Rust mutable-aliasing UB and can corrupt the internal `VecDeque`.

## Failure scenario

Any node running conflux under a multithreaded executor risks data corruption or crashes. It is safe only with a single-threaded executor, which is neither enforced nor documented.

## Suggested fix

Guard `State` with a `Mutex` on the Rust side (or document and enforce single-threaded-executor usage). The lock should cover the full push/poll operation, not just the C++ pending map.

## Resolution (2026-07-11)
Fixed in conflux (`jerry73204/conflux`@b9912a4; LCTK pins it). The FFI wraps the
core `State` in a `Mutex` and locks it in every entry point (releasing before
Python callbacks to avoid re-entrant deadlock), so concurrent push/poll under a
MultiThreadedExecutor is serialized instead of aliasing `&mut State`.
