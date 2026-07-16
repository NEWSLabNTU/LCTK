# L-15 · Every build dirties the worktree (Cargo.lock churn, always-dirty submodule)

- **Severity:** Low
- **Area:** build system / git hygiene
- **Status:** Fixed (2026-07-16)
- **Verified:** 2026-07-16, `git status` after any `just build`

## Problem

A plain `just build` mutates tracked files:

- `Cargo.lock` in the workspace root (colcon-cargo-ros2 re-resolves against the
  `build/<pkg>/rosidl_cargo` path deps, whose versions differ from the committed lock), and
- inside the `ros/conflux` submodule: `Cargo.lock`, `conflux_cpp/rust/Cargo.lock`,
  `conflux_node/Cargo.lock` — so the submodule pointer shows permanently `-dirty`.

Consequences observed in practice:

- `git pull --rebase` refuses to run ("You have unstaged changes") after every build.
- `git stash` / `stash pop` conflicts on `Cargo.lock` — one such conflict aborted a stash
  pop mid-operation on 2026-07-16 and needed manual recovery.
- The `-dirty` submodule makes it impossible to tell at a glance whether conflux has real
  uncommitted work.
- Contributors either commit lockfile noise or `checkout --` it before every git operation.

## Suggested fix

Pick one deliberately:

1. **Commit the post-build lockfiles** once, if they are stable across machines — churn
   ends when the committed state matches what the build produces.
2. If they are *not* stable (path deps regenerate with different metadata), stop tracking
   the volatile locks (`.gitignore` + `git rm --cached`) and document that reproducibility
   comes from the workspace `Cargo.toml` pins instead.
3. For the conflux submodule, same decision upstream (jerry73204/conflux — we maintain it),
   plus `ignore = dirty` in `.gitmodules` if only generated files ever change.

Option 2/3 trades lockfile reproducibility for a clean worktree; whichever way, the current
"always dirty" state is the worst of both.

## Resolution (2026-07-16)

Two-part fix, matching options 1 and 3:

- **Root `Cargo.lock`**: committed the post-build state. The churn was the committed lock
  predating the current apt interface-package versions (e.g. `geometry_msgs` 4.2.4 → 4.9.1);
  once committed, repeat builds leave it untouched. It will legitimately change again when
  apt upgrades the msg packages — commit that too.
- **conflux submodule**: `ignore = dirty` in `.gitmodules`. The lockfile churn there is
  LCTK-workspace contamination (entries like `lctk_interfaces` injected by building inside
  this workspace) and must never be committed upstream, so it is now hidden from
  status/diff. Real conflux work is done by committing inside `ros/conflux` explicitly.

`git status` after a full `just build` is now clean.
