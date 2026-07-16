# L-16 · `bindgen.lock` silently skips rosidl regeneration after partial cleanup

- **Severity:** Low (but cost a broken-build debugging session)
- **Area:** build tooling (colcon-cargo-ros2, upstream)
- **Status:** Open
- **Verified:** 2026-07-16, reproduced twice during the lctk_interfaces CalibrationMetrics cleanup

## Problem

colcon-cargo-ros2's workspace binding generation
(`colcon_cargo_ros2/workspace_bindgen.py`) writes Rust bindings to
`build/<pkg>/rosidl_cargo/` **once**, then marks itself done by creating
`build/.colcon/bindgen.lock`. `should_generate()` returns False whenever the lock file
exists — it never checks whether the outputs it guards still exist or are current.

Two real failures follow:

1. **Partial clean**: the CLAUDE.md-recommended recovery `rm -rf build/<pkg> install/<pkg>`
   deletes the bindings but not the lock. Every subsequent `just build` skips regeneration
   and all Rust packages fail with
   `failed to read .../build/lctk_interfaces/rosidl_cargo/lctk_interfaces/Cargo.toml`.
2. **Message set changes**: after L-13 deleted `CalibrationMetrics.msg`, stale generated C
   sources in `build/lctk_interfaces` kept referencing it and the package failed at link
   time (`undefined reference to lctk_interfaces__msg__CalibrationMetrics__create`).

Both are documented as Known Issue 7 in CLAUDE.md with the manual workaround
(`rm -f build/.colcon/bindgen.lock`), but the tool itself stays a trap for anyone who
hasn't read it.

## Suggested fix

Upstream in colcon-cargo-ros2 (we control the pin): make `should_generate()` validate,
not just check existence — regenerate if any expected `build/<pkg>/rosidl_cargo/<pkg>/Cargo.toml`
is missing, or key the lock on a hash of the interface packages' `msg/srv/action` files so
message changes invalidate it. Until then the CLAUDE.md workaround stands.
