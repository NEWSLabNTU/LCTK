# L-27 · `ament_lint` is red on `conflux_cpp`, including generated and build-artifact files

- **Severity:** Low
- **Area:** conflux build tooling
- **Status:** Open
- **Verified:** Observed while wiring the gtest target for L-22 (2026-08-15)
- **Location:** `ros/conflux/conflux_cpp/CMakeLists.txt` (the `BUILD_TESTING` block)

## Problem

`conflux_cpp` enables `ament_lint_auto_find_test_dependencies()`, which registers `copyright`,
`cpplint` and `uncrustify` as ctest cases. All three fail:

```
The following tests FAILED:
	  2 - copyright (Failed)
	  4 - cpplint (Failed)
	  6 - uncrustify (Failed)
```

Two separate problems are tangled together.

**1. Real lint debt on the sources.** `ament_copyright` reports no copyright notice on
`examples/sync_node.cpp`, `include/conflux/synchronizer.hpp`, `include/conflux/types.hpp` and
`include/conflux/visibility.h`.

**2. The linters are scanning files they should not see.** The same run flags:

- `include/conflux_ffi.h` — **generated** by cbindgen at build time. Reformatting it is pointless:
  the next build regenerates it, so `just format-cpp` and the linter fight each other forever.
- `rust/target/debug/build/conflux-ffi-*/out/conflux_ffi.h` — Rust **build artifacts**, several
  copies, one per build hash. These are not source in any sense.

This was invisible until 2026-08-15 because `just test-cpp` never ran anything
([L-22](./archive/L-22-conflux-cpp-has-no-tests.md)), and `just colcon-test` is not part of
`just test`.

## Failure scenario

Not a runtime defect. The cost is that the lint signal is unusable: three permanently-red cases
mean a genuinely new violation cannot be noticed. Same shape as the archived
[L-14](./archive/L-14-lint-red-on-main.md), where `just lint` was red on an untouched checkout.

Because of this, `just test-cpp` is deliberately scoped to the gtest target
(`--ctest-args -R test_conflux_cpp`) so that L-22's coverage is not held hostage to lint debt.

## Suggested fix

1. Exclude generated and build-artifact paths from the linters — `AMENT_LINT_AUTO_FILE_EXCLUDE`
   for `include/conflux_ffi.h`, and `rust/target/` in its entirety. Fixing the scope first is what
   makes the remaining failures meaningful.
2. Add copyright headers to the four flagged sources, or configure `ament_copyright` for the
   project's actual licence (the package is `MIT OR Apache-2.0`).
3. Work through the residual `cpplint` / `uncrustify` findings, then unscope `just test-cpp` or
   add the lint cases to `just check`.

Related: [L-22](./archive/L-22-conflux-cpp-has-no-tests.md),
[L-14](./archive/L-14-lint-red-on-main.md).
