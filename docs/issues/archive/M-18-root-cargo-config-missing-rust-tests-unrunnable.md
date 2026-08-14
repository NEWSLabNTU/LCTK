# M-18 · No root `.cargo/config.toml` → Rust test suite unrunnable and the L-16 guard is inert

- **Severity:** Medium
- **Area:** build / test tooling
- **Status:** Fixed
- **Verified:** Reproduced 2026-08-11 — `cargo nextest` at the workspace root fails; running from a package directory succeeds
- **Fixed:** 2026-08-14 — root config is now synthesised per build; 240/240 tests run from the repo root
- **Related:** [L-16](./L-16-bindgen-lock-stale-skip.md), [L-14](./L-14-lint-red-on-main.md), [L-15](./L-15-build-dirties-worktree.md)

## Problem

`colcon-cargo-ros2` now writes its `[patch.crates-io]` block into a **per-package**
`.cargo/config.toml` (e.g. `ros/lidar_board_detector/.cargo/`, with relative paths like
`../../build/sensor_msgs/rosidl_cargo/sensor_msgs`). The workspace-root `.cargo/` is empty. Root is
gitignored (`/.cargo/`), so the file is expected to be generated — but nothing generates it there
any more.

Two consequences, both silent:

**1. The Rust half of `just test` cannot run.** `just test` invokes `cargo nextest run` from the
workspace root. Cargo discovers config from the CWD upward, so at the root there are no patches; it
re-resolves the wildcard ROS message dependencies against crates.io and dies:

```
error: failed to select a version for the requirement `sensor_msgs = "*"`
  version 4.2.3 is yanked
required by package `aruco-detector v0.1.1`
```

This is the same yanked-crate failure CLAUDE.md documents for dependency updates, but here it blocks
*testing*, and `--offline` does not help (the patch is missing, not the index).

**Demonstrated impact:** the `lidar_board_detector` unit tests were left referencing the pre-flatten
config API by commit `2a4fd49` and did not compile. `colcon build` does not build `#[cfg(test)]`
code, and the test suite could not run, so the breakage went unnoticed until 2026-08-11.

**2. The L-16 guard is inert.** The `just build` recipe drops a stale `build/.colcon/bindgen.lock`
"whenever any binding path pinned in `.cargo/config.toml` is missing", via
`grep -oP 'path = "\K[^"]+' .cargo/config.toml`. With no such file the `grep` fails, the loop reads
nothing, and the guard never fires — so L-16's auto-recovery after a partial `rm -rf build/<pkg>`
silently no longer works.

## Workaround

Run cargo from inside a package directory that has its own generated config; the workspace still
resolves, and the patches are picked up:

```bash
cd ros/lidar_board_detector
cargo nextest run --cargo-profile test-release -p board-cluster-detector -p lidar_board_detector
```

## Suggested fix

Either:

1. **Restore a root config.** Have `just build` synthesise `.cargo/config.toml` at the root by
   copying one of the generated per-package blocks with the paths rewritten relative to the root.
   That fixes both consequences at once. (Note CLAUDE.md Known Issue 1: a *stale* root config causes
   `Unable to update .../install/.../rust`, so it must be regenerated per build, never hand-kept.)
2. **Or point both consumers at a per-package config.** Change `just test` to run cargo from a
   package directory, and change the L-16 guard to read whichever per-package config exists.

Whichever is chosen, add a check that fails loudly when the Rust tests cannot be collected, so an
unrunnable suite is never mistaken for a passing one.

## Resolution (2026-08-14)

Fix 1 from above. `setup/scripts/sync-root-cargo-config.sh` synthesises
`.cargo/config.toml` at the workspace root by copying the first (sorted, so the choice is
deterministic and the file is byte-stable across runs) generated `ros/*/.cargo/config.toml` and
rewriting the patch paths root-relative — `path = "../../build/x"` → `path = "build/x"`. The
`[build] rustflags` are absolute already and carry over untouched.

Fix 2 was rejected: it fixes the two known consumers but leaves the root a place where cargo
still does not work, so the next person to type `cargo test`, `cargo clippy`, or `cargo audit` at
the root hits the same yanked-`sensor_msgs` wall. Fix 1 makes the root work for every cargo
command, and it repairs the L-16 guard without that guard having to know about package layout.

**Staleness.** The script runs from `just build` twice — once before the L-16 guard, which reads
the file it produces, and once after `colcon build`, so what lands is what colcon just wrote — and
again from `just test`. It always overwrites; the file is never hand-maintained, which is what
CLAUDE.md Known Issue 1 (`Unable to update .../install/.../rust`) warns about. `/.cargo/` stays
gitignored. The pre-guard call is allowed to fail, because on a never-yet-built tree there are no
per-package configs to copy from yet; the post-build call is not.

The script refuses rather than guesses if colcon-cargo-ros2 ever emits a patch path that is not
`../../…`, since the rewrite would otherwise silently produce a wrong root config.

**Collection guard.** New `just test` prerequisite `_check-rust-tests-collectable` runs
`cargo nextest list --workspace`, which compiles every test target without running one, and fails
with a banner if the config cannot be synthesised, if the listing command fails, or if the listing
is empty. It counts only stdout — cargo's `Compiling …`/`Finished` progress goes to stderr, and
folding that in would let an empty suite report a nonzero test count.

Also fixed in passing, same failure class: `just test` invoked bare `pytest`, but apt's
`python3-pytest` installs the module and a `pytest-3` script and no `pytest` on `PATH`, so the
Python half exited 127 without running. Now `python3 -m pytest`.
