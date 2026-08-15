# L-20 · `window_size_ms = 0` is a magic sentinel for "infinite window"

- **Severity:** Low
- **Area:** conflux_py / conflux FFI (API ergonomics)
- **Status:** Open
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
