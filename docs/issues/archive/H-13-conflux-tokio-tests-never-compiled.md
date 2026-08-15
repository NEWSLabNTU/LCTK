# H-13 · Conflux tokio integration tests had not compiled for an unknown period; `just test` hid it

- **Severity:** High
- **Area:** conflux-core (test suite)
- **Status:** Fixed (2026-08-15)
- **Verified:** Reproduced and fixed on `jerry73204/conflux`@6695b66
- **Location:** `ros/conflux/crates/conflux-core/tests/staleness_tokio_tests.rs`,
  `ros/conflux/justfile:78-86`

## Problem

`staleness_tokio_tests.rs` was never updated when `Config` gained an `Option<Duration>` window
and a `drop_policy` parameter. All 20 `Config::with_staleness` / `Config::basic` call sites
failed to compile:

```
error[E0061]: this function takes 5 arguments but 4 arguments were supplied   (×15)
error[E0308]: mismatched types                                                (×5)
error: could not compile `conflux-core` (test "staleness_tokio_tests") due to 20 previous errors
```

The breakage was invisible because `just test-rust` ran `cargo test --workspace` **without**
`--features tokio`. The test file is feature-gated, so without the flag it compiles to nothing
and the suite reports success. Only `just test-core` — which nobody in the aggregate path
calls — passed the flag and surfaced the errors.

## Failure scenario

The entire staleness test suite (20 tests) was dead while `just test` reported green. Every
staleness defect filed from the 2026-08-15 audit (H-11, M-17, M-18, M-19, M-20, M-21) sits in
code this suite was supposed to cover.

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@6695b66; LCTK pins it):

- All 20 call sites moved to the current `Config` API — `Some(Duration::…)` for the window,
  explicit `DropPolicy::RejectNew`, `DropPolicy` added to the imports.
- `just test-rust` and `just test-rust-nextest` now pass `--features tokio`, so the aggregate
  suite compiles and runs the gated tests.

`just test-rust` went from 139 to 159 reported tests. Full suite: 156 conflux-core +
3 conflux-ffi + 19 conflux_py, all passing.

Related: L-22 (the C++ half of the same coverage gap), M-25.
