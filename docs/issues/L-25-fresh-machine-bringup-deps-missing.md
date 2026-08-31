# L-25 · `setup.sh` installs none of the tools `just test` and `just lint` need

- **Severity:** Low
- **Area:** setup / developer onboarding
- **Status:** Open
- **Verified:** Reproduced 2026-08-27 bringing up a freshly cloned tree on a new machine (Jetson/Tegra, Ubuntu 22.04)

## Problem

`./setup.sh` is documented in CLAUDE.md's Quick Start as the way to set up a development
environment, but a machine set up that way still cannot run `just test` or `just lint` to
completion. Four separate gaps, each failing well after setup reports success:

**1. `python3-json5` is not installed.** `ros/lctk_target/package.xml` declares
`<exec_depend>python3-json5</exec_depend>` and `rosdep check` resolves it to the apt package, but
nothing in `setup/` installs it and `setup.sh` does not run `rosdep install`. Every Python suite
that reaches `lctk_target.load_target` — which is most of them — dies at *collection* time:

```
build/lctk_target/lctk_target/target.py:18: in <module>
    import json5
E   ModuleNotFoundError: No module named 'json5'
```

**2. `ruff` is not installed and is not in apt.** `just lint-py` and `just lint` both run
`ruff check` / `ruff format --check`. `grep -rn ruff setup/` returns nothing, and there is no
`ruff` or `python3-ruff` apt package on Ubuntu 22.04, so the recipe exits 127 with no explanation
of where the tool is meant to come from.

**3. `uv` is not installed and is not in apt.** Two `board-cluster-detector` tests
(`harness_smoke::fixtures_load_and_are_nonempty` and
`detect_parity::real_one_metre_fixture_has_equivalent_neutral_and_compatibility_evidence`) fail on a
fresh clone with `no fixtures found — run export_golden.py`. The fixtures are generated data:
`rust/board-cluster-detector/.gitignore:7` excludes `/tests/fixtures/*.f32`, and regenerating them
needs `uv run python tools/export_golden.py` from `experiments/board-detection-2d/`.

**4. Nothing tells you (3) is expected.** The two failures look like real regressions. The
regeneration command is documented only in `rust/board-cluster-detector/tests/fixtures/README.md`,
which you find only after tracing the panic message. Neither CLAUDE.md nor the phase docs mention
that a fresh clone starts with an incomplete Rust suite.

Also worth noting: the submodules (`ros/conflux`; `rust/multi-stream-synchronizer` too,
until it was removed unused on 2026-08-31) come up
uninitialised on a fresh clone, and `just build` depends on `build-conflux`, so
`git submodule update --init --recursive` is a required step that Quick Start does not mention.

## Why it is Low

Each gap is a one-command fix once identified, and none of them affect a machine that is already
working. The cost is entirely in onboarding time and in the false signal that failing tests give a
newcomer.

## Suggested fix

- Have `setup.sh` run `rosdep install --from-paths ros --ignore-src -r` (the `rclrs` and
  `ament_python` keys are already unresolvable and must stay ignored), or install
  `python3-json5` explicitly alongside the other apt dependencies.
- Install `ruff` and `uv` in `setup/`, version-pinned with env overrides like the other installers
  (L-09), noting that both ship as self-contained binaries with no Python dependencies, so they
  cannot drag in the setuptools/numpy/scipy that CLAUDE.md Known Issue 3 warns about.
- Initialise submodules in `setup.sh`, or add the command to CLAUDE.md's Quick Start.
- Either generate the board-cluster-detector fixtures as part of setup, or make the two tests skip
  with an explicit "fixtures not generated; run export_golden.py" message instead of failing, so a
  fresh clone's Rust suite is honestly green.

## Related

- [L-09](./archive/L-09-setup-fragility-export-labeling.md) — setup installers are version-pinned with env overrides
- [M-22](./archive/M-22-root-cargo-patch-block-single-source.md) — the other fresh-clone blocker found in the same bring-up
