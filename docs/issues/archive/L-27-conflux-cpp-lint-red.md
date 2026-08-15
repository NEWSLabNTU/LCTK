# L-27 · `ament_lint` is red on `conflux_cpp`, including generated and build-artifact files

- **Severity:** Low
- **Area:** conflux build tooling
- **Status:** Fixed (2026-08-15)
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
([L-22](./L-22-conflux-cpp-has-no-tests.md)), and `just colcon-test` is not part of
`just test`.

## Failure scenario

Not a runtime defect. The cost is that the lint signal is unusable: three permanently-red cases
mean a genuinely new violation cannot be noticed. Same shape as the archived
[L-14](./L-14-lint-red-on-main.md), where `just lint` was red on an untouched checkout.

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

Related: [L-22](./L-22-conflux-cpp-has-no-tests.md),
[L-14](./L-14-lint-red-on-main.md).

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@7d31293; LCTK pins it). `just test-cpp` now runs the gtest
target **and** the linters — 44 tests, 0 failures — and is no longer scoped with `-R`.

### Scope first

`AMENT_LINT_AUTO_FILE_EXCLUDE` now covers `rust/target/**/*.h` and the generated
`include/conflux/conflux_ffi.h`. Those accounted for the large majority of findings, which is
exactly what buried the real ones.

The generated header was also **tracked in git**, which made it oscillate: `just format-cpp`
reformatted it, the next build regenerated it unformatted, and every commit carried the churn. It
is now gitignored and moved under `include/conflux/`, alongside the other public headers — which
also satisfies cpplint's `build/include_subdir` rule properly instead of suppressing it.

### Two formatters cannot both own style

The substantive finding, and the reason this was not simply "add some headers".

This project formats C++ with clang-format (`.clang-format`: Google, 4-space indent);
`ament_uncrustify` enforces ROS style. They are irreconcilable, so one of them is permanently red
no matter what is fixed. Reformatting the package to ROS style would have contradicted an existing
project decision that was never in scope here, so instead `package.xml` now depends on the
individual linters — `ament_cmake_copyright`, `cppcheck`, `cpplint`, `lint_cmake`, `xmllint` —
rather than the `ament_lint_common` meta-package, omitting uncrustify. Formatting stays owned by
`just format` / `just format-check`.

The same conflict existed in miniature over include ordering, and there it **was** reconcilable:
`.clang-format` had `IncludeBlocks: Regroup` with categories placing local headers *first*, the
exact inverse of cpplint's `build/include_order`. Setting `Preserve`/`Never` lets clang-format own
whitespace and cpplint own ordering. That difference — reconcilable vs not — is what decided
uncrustify's fate rather than a general dislike of it.

### Residual debt

Apache-2.0 copyright headers on all eight sources, with an explicit note that the package is
dual-licensed `MIT OR Apache-2.0` so the single-license boilerplate the linter demands is not read
as narrowing the choice. Plus ament-style header guards, include ordering, `<utility>` for
`std::move`, and an `explicit` single-argument constructor.

### Verified

- `just test-cpp` exits **1** on a lint violation (un-`explicit`-ing a constructor), not only on a
  test failure — the property that matters, per the standing lesson from H-13, M-25 and L-22.
- `just format-check` passes **immediately after a build**, which is the property the tracked
  generated header used to break.
