# L-17 · `is_empty()` returns true when *any* buffer is empty

- **Severity:** Low
- **Area:** conflux-core / conflux_py
- **Status:** Fixed (2026-08-15)
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

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@0a9c901; LCTK pins it). The predicate is split in two so
each name states its own meaning:

- `has_empty_buffer()` — at least one buffer is empty, so no group can form. This is what the
  matcher actually uses, and what the old `is_empty` did.
- `all_buffers_empty()` — every buffer is empty, i.e. the synchronizer is idle. This is what the
  old name *implied*.

Both are exposed across core, the C ABI (`conflux_has_empty_buffer`, `conflux_all_buffers_empty`)
and `conflux_py`. `is_empty` is retained as a deprecated alias in all three — `#[deprecated]` in
Rust, a `DeprecationWarning` in Python, and the original `conflux_is_empty` C symbol kept so the
C++ wrapper and any external caller keep linking.

Regression coverage: `crates/conflux-core/tests/naming_tests.rs` (including a case asserting the
two predicates genuinely disagree when one stream has data and another does not — the distinction
the single name collapsed), plus `TestNamingAndValidation` in `conflux_py`.
