# L-17 · `is_empty()` returns true when *any* buffer is empty

- **Severity:** Low
- **Area:** conflux-core / conflux_py
- **Status:** Open
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/state.rs:246-259`,
  `ros/conflux/conflux_py/conflux_py/_core.py:270-272`

## Problem

`State::is_empty` returns `true` if **any** buffer is empty, not if all are:

```rust
for item in buffers {
    let (_key, buffer) = item;
    if buffer.is_empty() {
        return true;
    } else {
        continue;
    }
}
false
```

The name states the opposite of the behaviour for the multi-stream case, which is the only
case conflux exists to handle. A one-line `.all(...)` version sits commented out directly
above it. The Python binding inherits the method and documents it as "Check if any buffer is
empty" — accurate, but reached through a method named `is_empty`.

## Failure scenario

Mostly a readability and API-misuse hazard: `sync.rs:154` and `sync.rs:277` both depend on the
any-semantics, so the current call sites are correct. A caller reading the name and writing
`if not sync.is_empty(): process()` gets the opposite of the intended guard.

## Suggested fix

Rename to `has_empty_buffer()` (or `any_empty()`) in the core, the FFI and `conflux_py`,
keeping `is_empty()` as a deprecated alias for one release. Add the genuine all-empty
predicate if a caller needs it. Delete the commented-out variant.

Related: M-23.
