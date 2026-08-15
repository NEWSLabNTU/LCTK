# L-20 · `window_size_ms = 0` is a magic sentinel for "infinite window"

- **Severity:** Low
- **Area:** conflux_py / conflux FFI (API ergonomics)
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/conflux_cpp/rust/src/lib.rs:164-169`,
  `ros/conflux/conflux_py/conflux_py/_ffi.py:199-204`

## Problem

The Rust core models the window honestly as `Option<Duration>`, where `None` means infinite.
The C ABI cannot carry an `Option`, so it encodes infinite as `0`:

```rust
let window_size = if config.window_size_ms == 0 { None } else { Some(...) };
```

Python then re-encodes `None` back to `0` on the way in
(`window_size_ms if window_size_ms is not None else 0`). The round trip works, but the value
`0` is exposed all the way out to YAML: `sync_tolerance_ms: 0` in the LCTK launch configs
means *infinite tolerance*.

Zero is the most natural way to write "no tolerance at all", so the config reads as the exact
opposite of what it does. The LCTK CLAUDE.md has to spell this out ("0 = infinite window") in
the parameter table, which is a sign the encoding is wrong rather than the docs.

## Failure scenario

An operator tightening synchronization sets `sync_tolerance_ms: 0` expecting strict matching
and silently disables windowing entirely, matching messages an arbitrary distance apart. There
is no validation or warning that distinguishes intent.

## Suggested fix

- Add an explicit `window_infinite: bool` (or a `-1` sentinel that cannot be confused with a
  tightening) to `ConfluxConfig`, keeping `0` as a hard error.
- In `SyncConfig`, accept `window_size_ms=None` as the only spelling of infinite and reject
  `0` with a message pointing at `None`.
- Log the resolved mode at construction: "sync window: infinite" vs "sync window: 50 ms".

Related: L-18, L-21.

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@0a9c901; LCTK pins it).

- `SyncConfig.__post_init__` rejects any non-positive `window_size_ms` with a message pointing at
  `None`, stating explicitly that 0 is not a synonym for infinite.
- `sync()` rejects a zero `Duration` with the same guidance instead of a bare `ensure!`.
- `SyncConfig.window_description` gives callers a resolved string (`"infinite (no time-based
  dropping)"` vs `"50 ms"`) for startup logging.

The C ABI keeps `0` as its internal encoding for infinite, since a C struct cannot carry an
`Option` — but that is now an implementation detail that no longer surfaces in any user-facing
API.

**LCTK is unaffected.** Its solver nodes already convert at the Python boundary
(`window_size_ms=int(sync_tolerance_ms) if sync_tolerance_ms > 0 else None`), so they pass `None`
and never the sentinel. The ROS parameter `sync_tolerance_ms: 0.0` keeps its documented meaning as
LCTK's own convention.

Regression coverage: `test_zero_window_rejected_with_a_pointer_to_none` and
`test_none_window_means_infinite` in `conflux_py`.
