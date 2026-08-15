# L-28 · `just test` invoked a bare `pytest`, so the Python suites never ran

- **Severity:** Low
- **Area:** build tooling
- **Status:** Fixed (2026-08-15)
- **Verified:** Observed while verifying M-12
- **Location:** `justfile` (the `test` recipe)

## Problem

The `test` recipe ended with:

```bash
pytest ros/lctk_launch/test/ ros/advanced_extrinsic_solver/test/ ros/lctk_quality/test/ ros/lctk_autoware_export/test/ -v --no-header
```

apt's `python3-pytest` installs the package but **no `pytest` executable**, so on this machine the
line exits 127:

```
/run/user/.../just-.../test: line 150: pytest: command not found
error: recipe `test` failed with exit code 127
```

The Rust half (273 tests) ran first and passed, then the recipe died. So `just test` had never run
the four Python suites — 92 tests covering the config parser, the calibration planner, pose
weighting, the quality metric, and the whole Autoware export path.

## Failure scenario

Same class as conflux's [H-13](./H-13-conflux-tokio-tests-never-compiled.md) and
[M-25](./M-25-conflux-py-tests-never-ran.md), with one mercy: this one fails **loudly**. It exits
non-zero rather than reporting success, so it could not have masked a regression indefinitely —
but it does mean nobody was running those 92 tests through the documented entry point, and the
failure is easy to read as "pytest isn't installed" and skip past.

## Resolution (2026-08-15)

Invoke it as a module: `python3 -m pytest ...`, with a comment recording why. All 92 tests run and
pass.

The reusable lesson is the one already recorded in conflux's CLAUDE.md: a test recipe that cannot
run, or cannot fail, is worse than no recipe. Verify a new one by breaking an assertion
deliberately before trusting it.
