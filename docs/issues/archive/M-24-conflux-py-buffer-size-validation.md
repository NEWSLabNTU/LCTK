# M-24 · `conflux_py` reported an invalid `buffer_size` as a generic RuntimeError, and `__del__` ran on a partially built object

- **Severity:** Medium
- **Area:** conflux_py
- **Status:** Fixed (2026-08-15)
- **Verified:** Test `test_create_synchronizer_invalid_buffer_size` now passes
- **Location:** `ros/conflux/conflux_py/conflux_py/_ffi.py:168-220`

## Problem

Two defects in `FFISynchronizer.__init__`, both surfaced once `just test-python` was made to
actually run (M-25).

**1. Wrong exception for an invalid buffer size.** The Rust FFI rejects `buffer_size < 2` by
returning a null handle (`conflux_cpp/rust/src/lib.rs:137`). Python had no matching check, so
the null handle fell through to a generic:

```
RuntimeError: Failed to create synchronizer
```

`conflux_py`'s own test suite expected `ValueError` mentioning `buffer_size`, matching the
existing empty-topics validation. The message named no cause, so a caller could not tell an
invalid argument from a genuine allocation failure.

**2. `__del__` on a partially constructed object.** `self._handle` was assigned *after* the
validation block, so any `__init__` that raised left the object without the attribute its
destructor reads:

```
AttributeError: 'FFISynchronizer' object has no attribute '_handle'
  File "conflux_py/_ffi.py", line 237, in __del__
    if self._handle and _lib:
```

Raised during garbage collection as an unraisable exception, so it surfaced as a warning
detached from the failing construction. Pre-existing: the empty-topics path hit it too.

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@6695b66; LCTK pins it):

- `buffer_size < 2` now raises `ValueError(f"buffer_size must be at least 2, got {buffer_size}")`
  before the FFI call, mirroring the empty-topics check, with the docstring's `Raises:` list
  updated.
- `self._handle = None` moved above all validation, so `__del__` is safe on any partially
  built object.

Related: L-21 (the constraint itself is undocumented), M-25.
