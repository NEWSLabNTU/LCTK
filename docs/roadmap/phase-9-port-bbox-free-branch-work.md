# Phase 9: Port the bbox-free branch's work onto the live branch

- **Status:** Complete (10 of 11 ported; item 10 dropped as superseded)
- **Date:** 2026-09-03
- **Source branch:** `feat/bbox-free-parity-validation` (pushed, tip `375ab1c`)
- **Target branch:** `feat/selectable-calibration-targets`

## Why this phase exists

Two branches rebased onto `main` independently and both grew work. The merge-base of
`feat/bbox-free-parity-validation` and `feat/selectable-calibration-targets` is now
`main`'s tip (`8001437`), so a merge or rebase between them would replay ~250 commits of
the same logical history under different SHAs. Cherry-picking the genuinely-unique work
is the only sane route.

This doc tracks that port, item by item, so nothing is silently dropped and nothing is
ported twice.

## Already converged — do NOT port

The live branch reached these independently. Porting ours would only conflict.

| finding | live-branch state |
|---|---|
| M-12 pose-granularity outlier rejection | present, a different implementation (218 differing lines) |
| M-01 publish with ROS TF semantics | present |
| solid board's real ArUco id `24` | present, with its own comment noting `aruco_1` is historical |
| L-29 dangling-symlink prune | present (came from `main`) |

The id-24 result reaching both branches by independent routes is useful corroboration:
the physical board carries marker 24, and `solid_600_aruco_1`'s `1` was a schema-era
placeholder.

## Port list

Each item is independent; order below is roughly lowest-risk first.

| # | Item | Files | State |
|---|---|---|---|
| 1 | `package.xml` runtime dependencies | `ros/{calibration_judge,extrinsic_solver_node,lctk_launch,lidar_to_camera_solver}/package.xml` | ☑ |
| 2 | `colcon-cargo-ros2` version floor `>=0.5.3` | `setup/scripts/install-colcon-rust.sh` | ☑ |
| 3 | `sync-root-cargo-config.sh` stands down under colcon >= 0.5.3 | `setup/scripts/sync-root-cargo-config.sh` | ☑ |
| 4 | Unify the two colcon invocations; refresh the stale M-18 comment | `justfile`, `CLAUDE.md` | ☑ |
| 5 | Shared pip-shadow guard | `setup/scripts/check-python-env.sh`, `justfile` | ☑ |
| 6 | `install-dev-tools` split into debug + docs | `setup/scripts/install-dev-tools{,-debug,-docs}.sh` | ☑ |
| 7 | New install steps: `just`, `ruff`/`uv` | `setup/scripts/install-{just,lint-tools}.sh` | ☑ |
| 8 | Setup engine + curses TUI + tests | `setup/{steps.py,tui.py,setup.sh,justfile,test/}` | ☑ |
| 9 | Phase-8 rename and framing note | `docs/roadmap/`, `docs/superpowers/specs/`, `docs/adr/`, refs | ☑ |
| 10 | Assisted capture and review design | — | **not ported, superseded** |
| 11 | H-17 and its real-data investigation | `docs/issues/`, tracker row | ☑ |

### Notes per item

**1.** Four packages import modules their `package.xml` never declared, so `rosdep` cannot
know about them and a fresh machine fails at node startup rather than at setup.

**3–4.** These two are coupled. `colcon-cargo-ros2 >= 0.5.3` writes the workspace-root
`.cargo/config.toml` itself, which is exactly what M-18's `sync-root-cargo-config.sh`
synthesises by hand. Without the stand-down the script fails the build looking for a
per-package layout that no longer exists. **The live branch's build recipe still carries
the stale M-18 comment**, so this will bite it on the next colcon upgrade.

**8.** The largest item and a wholesale addition to a directory the live branch has not
touched. Its two load-bearing properties: markers are content-addressed (a script edit
invalidates them) and every step carries a cheap verifier, so a script that exits 0
without installing anything is an error rather than a completed step. Both were reactions
to real marker/reality divergence.

**10. Dropped after reading the live branch.** It already carries
`docs/superpowers/plans/2026-08-31-assisted-extrinsic-solver.md` and its design spec, both
further along than the source branch's sketch: a third `solver_mode=assisted` with
`stability.py`, `preview.py` and a Flask `review_server.py`, and `python3-flask` already
declared in `package.xml`.

They also already cover the caveat the source design was built around. That design argued
stability measures repeatability rather than correctness, because this pipeline's
characteristic failure is a *stable wrong answer* -- the M-14 quarter-turn agrees with
itself on every frame. The live spec states the same risk in its own terms: in `assisted`
mode "the node captures unattended, so a wrong corner convention would be baked into an
entire session before anyone looked", and cites M-14. A second, weaker design doc would
only compete with it.

**11.** H-16 was closed on the source branch and its content is already reflected here by
convergence, so it is not ported. H-17 is unique and open. Highest ID on this branch is
`M-29`/`H-15`, so `H-17` is free and keeps continuity with the source branch's numbering.

## Acceptance

- `just build` green
- `just test` green, with `setup/test/` reached by the recipe
- `just smoke` still green (the live branch's sessions gate)
- every relative Markdown link under `docs/` resolves
- `ruff check` clean over `ros/` and `setup/`

## Deliberately not ported

- The source branch's rebase-reconciliation commits (`fix(rebase): ...`). They repaired
  that branch's own replay onto `main` and mean nothing here.
- H-16, closed on the source branch; its two fixes are present here by convergence.
