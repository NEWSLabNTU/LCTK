# L-22 · `just test-cpp` reports success while `conflux_cpp` has zero tests

- **Severity:** Low
- **Area:** conflux build tooling
- **Status:** Open
- **Verified:** Observed during the 2026-08-15 test run
- **Location:** `ros/conflux/justfile:88-91`

## Problem

```make
# Run C++ tests (currently no unit tests, only lint checks available)
test-cpp:
    @echo "C++ unit tests: none defined yet"
    @echo "Run 'just colcon-test' for ament_lint style checks"
```

`just test` depends on `test-cpp`, which prints two lines and exits 0. `conflux_cpp` is a
shipped package — it builds `libconflux_ffi.so`, the library every LCTK solver node loads —
and it has no test of any kind. The aggregate suite reports success regardless.

The C++ wrapper (`synchronizer.hpp`, `types.hpp`) is entirely uncovered, and the FFI crate's
own coverage is three smoke tests (`create_and_free`, `push_and_poll`, `invalid_key`).

## Failure scenario

Same class as the two masking defects found on 2026-08-15 (H-13, M-25): a green suite that
proves nothing about the component it names. C-05 — a permanent wedge in the FFI matching
path — would have been caught by any test that drove push/poll across a stream divergence.

## Suggested fix

- Add a GTest (or Catch2) target to `conflux_cpp` covering the C++ `Synchronizer` wrapper:
  construction, `add_subscription` bookkeeping, callback dispatch, destruction.
- Extend the FFI crate's Rust tests beyond smoke coverage — window behaviour, both drop
  policies, the wedge scenario from C-05, and `conflux_for_each_live` reconciliation.
- Make `test-cpp` fail, not pass, while no test target exists, or drop the recipe from
  `just test` so the suite stops implying coverage it does not have.

Related: C-05, H-12, H-13, M-25.
