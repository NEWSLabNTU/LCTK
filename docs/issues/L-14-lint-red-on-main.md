# L-14 · `just lint` is red on main

- **Severity:** Low
- **Area:** build system / CI hygiene
- **Status:** Open
- **Verified:** 2026-07-16, `ruff format --check ros/` on a clean main checkout

## Problem

`just lint` fails on an untouched main checkout:

```
Would reformat: ros/lctk_launch/launch/calibrate.launch.py
1 file would be reformatted, 57 files already formatted
```

A permanently red lint gate is worse than no gate: every contributor learns to ignore the
failure, so real lint regressions land unseen. It also breaks any future CI wiring of
`just lint`.

Secondary annoyance: the `cargo clippy --all-targets` step takes minutes on a cold target
dir, so quick `just lint` runs time out casual use; consider a `lint-py` shortcut.

## Suggested fix

`ruff format ros/lctk_launch/launch/calibrate.launch.py`, commit, and keep main green from
then on. (One line of reformatting; the file was touched by the M-13 work without running
the formatter.)
