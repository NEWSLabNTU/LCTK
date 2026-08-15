# Phase 9: Conflux API Ergonomics & Test Tooling

## Overview

The low-severity half of the 2026-08-15 conflux audit: API surfaces that mislead, configuration
encodings that read as their own opposite, and a test suite that reported success while running
nothing. None of it corrupts a calibration. All of it costs time — and two of the items
(H-13, M-25) are the direct reason the Phase 7 and Phase 8 defects went unnoticed for so long.

Three findings here are already **fixed** (2026-08-15) and are recorded for continuity. The
rest are open.

## Status — complete

All stages landed in `jerry73204/conflux`@0a9c901 (building on the earlier fixes recorded below).

| Stage | State |
|-------|-------|
| 1 — C++ coverage (L-22) | **Done** — five gtest cases driving the wrapper through a live rclcpp node; `just test-cpp` verified to exit 1 on a deliberate break |
| 2 — Python API (L-18, L-19) | **Done** — `ConfluxResult` exported as an IntEnum; the import guard now probes rclpy specifically |
| 3 — Config encodings (L-20, L-21) | **Done** — `window_size_ms = 0` rejected with a pointer to `None`; the `buf_size >= 2` floor explained wherever enforced |
| 4 — Docs (L-25) | **Done** — hardcoded test count removed, recipes refreshed, Known Issues section added |

**Follow-up filed:** wiring the gtest target exposed that `ament_lint` is red on `conflux_cpp`,
partly because it scans generated headers and Rust build artifacts. Tracked as
[L-27](../issues/L-27-conflux-cpp-lint-red.md); `just test-cpp` is scoped to the gtest target so
L-22's coverage is not held hostage to it.

## Problem Statement

A suite that reports green while running nothing is worse than no suite, because it converts
"untested" into "believed tested". Both conflux test paths were in that state:

- `just test-rust` ran `cargo test --workspace` without `--features tokio`. The 20-test
  staleness file is feature-gated, so it compiled to nothing — while it had in fact been
  failing to compile for an unknown period after the `Config` API changed under it (H-13).
- `just test-python` ran `colcon test`, which invokes `setup.py test` (unittest) for
  `ament_python` packages. The tests are pytest-style functions, so unittest collected 0 of 19
  and exited 0 (M-25). Repairing this immediately exposed a real bug (M-24).
- `conflux_cpp` — which builds the `libconflux_ffi.so` every solver loads — still has **no
  tests at all**, behind a `just test-cpp` recipe that echoes two lines and exits 0 (L-22).

## Scope

### Already fixed (2026-08-15, `jerry73204/conflux`@6695b66)

| Issue | Sev | Summary |
|-------|-----|---------|
| [H-13](../issues/archive/H-13-conflux-tokio-tests-never-compiled.md) | High | Tokio tests had not compiled; `just test-rust` now passes `--features tokio` (the feature itself was later removed with the staleness subsystem — see Phase 8) |
| [L-18](../issues/archive/L-18-conflux-result-not-exported.md) | Low | `ConfluxResult` is an IntEnum, exported; `last_push_result` returns it (closed alongside M-23) |
| [M-24](../issues/archive/M-24-conflux-py-buffer-size-validation.md) | Medium | `buffer_size < 2` now raises `ValueError`; `_handle` set before validation |
| [M-25](../issues/archive/M-25-conflux-py-tests-never-ran.md) | Medium | `test-python` now invokes pytest directly; exit codes propagate |
| [L-26](../issues/archive/L-26-anyio-breaks-pytest.md) | Low | pip `--user` `anyio` uninstalled; hazard documented in CLAUDE.md |

### Open

| Issue | Sev | Summary |
|-------|-----|---------|
| [L-22](../issues/archive/L-22-conflux-cpp-has-no-tests.md) | Low | `just test-cpp` reports success with zero tests |

| [L-19](../issues/archive/L-19-conflux-py-swallows-import-error.md) | Low | `conflux_py/__init__.py` swallows real ImportErrors |
| [L-20](../issues/archive/L-20-conflux-window-zero-sentinel.md) | Low | `window_size_ms = 0` is a magic sentinel for infinite window |
| [L-21](../issues/archive/L-21-conflux-buf-size-min-unexplained.md) | Low | `buf_size >= 2` enforced without explanation |
| [L-25](../issues/archive/L-25-conflux-docs-stale-test-count.md) | Low | conflux CLAUDE.md documents a test count the suite does not have |

## Stages

### Stage 1 — Close the coverage hole in `conflux_cpp` (L-22)

The largest remaining gap, and the one that would have caught C-05.

1. Add a GTest target to `conflux_cpp` covering the C++ `Synchronizer` wrapper: construction,
   `add_subscription` bookkeeping, callback dispatch, destruction ordering.
2. Extend the FFI crate's Rust tests past the three existing smoke tests — window behaviour,
   both drop policies, the C-05 wedge scenario, `conflux_for_each_live` reconciliation.
   (Phase 7 Stage 1 creates this suite; this stage grows it.)
3. Make `just test-cpp` **fail** while no test target exists, rather than echoing success.
   A recipe that cannot fail is not a test step.

**Exit:** `just test` fails if the C++ wrapper or the FFI regresses.

### Stage 2 — Make the Python API honest (L-19)

Steps 1 and 2 are **done** (L-18, closed alongside M-23): `ConfluxResult` is an `IntEnum` exported
from `conflux_py` next to `BlockedReason` and `MatchStatus`, `FFISynchronizer` has a public
`last_result`, and `last_push_result` returns the enum member. Remaining:

1. Narrow the `__init__.py` import guard to the condition actually being probed:

   ```python
   try:
       import rclpy  # noqa: F401
   except ImportError:
       pass
   else:
       from .synchronizer import ROS2Synchronizer, SyncStatistics  # noqa: F401
       __all__.extend(["ROS2Synchronizer", "SyncStatistics"])
   ```

   Any genuine failure inside `synchronizer.py` then propagates with its real traceback,
   instead of making `ROS2Synchronizer` silently vanish. Same pattern as the archived L-06.

**Exit:** a caller can act on push results using only public imports; a broken
`synchronizer.py` produces a traceback naming `synchronizer.py`.

### Stage 3 — Fix the configuration encodings (L-20, L-21)

1. Replace the `window_size_ms = 0` sentinel. `0` is the most natural spelling of "no
   tolerance at all", and it currently means *infinite* tolerance — which propagates out to
   `sync_tolerance_ms: 0` in LCTK launch configs. Add an explicit `window_infinite` flag (or
   a `-1` sentinel that cannot be confused with tightening), make `0` a hard error, and accept
   `None` as the only Python spelling of infinite.
2. Log the resolved mode at construction: `sync window: infinite` vs `sync window: 50 ms`.
3. Give every `buf_size >= 2` check a message naming the reason and the floor. Document the
   constraint on `Config`, `SyncConfig`, and in the LCTK synchronizer-parameter table.
4. Revisit the constraint itself once Phase 7 Stage 2 lands — the FFI path matches via
   `all_one()` and may not need two messages per stream at all.

**Exit:** no magic numbers in the public config; every rejection explains itself.

### Stage 4 — Refresh the docs (L-25)

1. Drop the hardcoded `# 166 tests` from conflux CLAUDE.md — the real count is 156, and any
   fixed number is a check nobody runs. Describe coverage instead.
2. Update the Testing section for the current recipes (`test-python` no longer uses colcon;
   `test-rust` passes `--features tokio`).
3. Re-measure or date-stamp the profiling and mode tables, unchanged since 2026-01-18.
4. Propagate the LCTK-side parameter-table changes from Stage 3.

**Exit:** conflux CLAUDE.md describes the code as it is on the day it is read.

## Verification

- `just test` in conflux exercises Rust core, FFI, C++ and Python, and **fails** if any of the
  four regresses. Verify by deliberately breaking one assertion in each and confirming a
  non-zero exit.
- `python3 -c "from conflux_py import ConfluxResult; print(ConfluxResult(2).name)"` works.
- A deliberate syntax error in `synchronizer.py` produces a traceback naming that file.
- `sync_tolerance_ms: 0` is rejected with a message pointing at the infinite-window spelling.

## Sequencing

Independent of Phases 7 and 8 except Stage 1, which shares the FFI test suite created in
Phase 7 Stage 1 — do that one after, or jointly. Stages 2–4 can land at any time and are
good candidates for parallel work, since they touch disjoint files.

## Standing lesson

Both masking defects had the same shape: a test step that could not fail. When adding a test
recipe, verify it fails on a deliberately broken assertion **before** trusting it — the
verification section above exists for that reason, not as a formality.
