# M-25 · `just test-python` collected zero tests and reported success

- **Severity:** Medium
- **Area:** conflux build tooling
- **Status:** Fixed (2026-08-15)
- **Verified:** `just test-python` now runs 19 tests; exit code propagates
- **Location:** `ros/conflux/justfile:93-100`

## Problem

`just test-python` drove the suite through `colcon test --packages-select conflux_py`. For an
`ament_python` package colcon invokes `setup.py test`, which is unittest-based. The conflux
tests are pytest-style functions in a plain class, not `unittest.TestCase` subclasses, so
discovery matched nothing:

```
----------------------------------------------------------------------
Ran 0 tests in 0.000s

OK
---
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
```

Exit code 0. `just test` therefore reported a passing Python suite while running none of its
19 tests, including the one that would have caught M-24.

A second, independent failure compounded it: a pip `--user` `anyio` broke pytest at plugin
load time workspace-wide (L-26), so even a direct `pytest` invocation aborted before
collection.

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@6695b66; LCTK pins it):

- `test-python` now invokes `python3 -m pytest conflux_py/test/ -v` directly, with a comment
  recording why colcon was dropped. The guard changed from `-d conflux_py` to
  `-d conflux_py/test`.
- Exit codes verified to propagate: passing run → 0, failing run → 1.
- `anyio` uninstalled to unblock pytest (L-26).

Running the suite immediately exposed M-24.

Related: H-13 (the Rust half of the same masking problem), L-22, L-26, M-24.
