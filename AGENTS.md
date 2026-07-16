# AGENTS.md

Instructions for AI coding agents working in this repository.

**Read [CLAUDE.md](./CLAUDE.md) first** — it is the canonical project guide (build system,
known issues, calibration workflow, coding guidelines). This file only adds the
agent-workflow rules that go beyond it.

## Quick facts

- Build: `just build` (never raw `colcon build` or bare `cargo build`)
- Test: `just test` · Lint: `just lint-py` (fast) / `just lint` (full, clippy is slow)
- Dependency audit: `just audit`
- Environment: Ubuntu 22.04, ROS 2 Humble, system Python — **never `pip3 install --user`
  setuptools, numpy, or scipy** (see CLAUDE.md Known Issue 3)
- Temporary files go to `$project/tmp/`, not `/tmp/`

## Workflow rules

1. **Branch, then fast-forward.** Work on a `fix/...` / `feat/...` / `docs/...` branch,
   verify, then `git checkout main && git merge --ff-only <branch>` before pushing.
2. **Verify before claiming done.** A change is done when `just build` and `just test`
   pass (plus `just lint-py` for Python edits). Paste-worthy evidence beats assertion.
3. **Multiple agents may work this repo concurrently.**
   - Before starting an issue, check `docs/issues/README.md` for 🟡 (in-progress) markers —
     don't take an issue another agent holds.
   - Always `git fetch` and rebase before pushing; expect origin/main to have moved.
4. **Issue tracking.** Findings get one file each under `docs/issues/` plus a row in the
   README status table. Closing an issue means: resolution note in the file, move it to
   `docs/issues/archive/`, repair every relative link that crosses the move, and verify
   zero dangling `](...*.md)` targets under `docs/`.
5. **Bigger work gets docs.** Multi-step remediations get a phase doc in `docs/roadmap/`;
   designs worth reviewing land in `docs/superpowers/specs/` before implementation.
6. **Don't hand-fix what a tool owns.** Transform-direction and frame-convention handling
   for Autoware lives in `lctk_autoware_export` (see CLAUDE.md → Exporting to Autoware);
   ad-hoc inversions elsewhere reintroduce M-01-class bugs.
7. **sudo**: show the command to the user; don't run it.

## Current state pointers

- Open issues + statuses: `docs/issues/README.md`
- Remaining open issues need hardware/operator work (field data, RViz verification) —
  check the tracker before assuming something is fixable headlessly.
- The conflux submodule (`ros/conflux`) is maintained by the repo owner; its lockfile
  churn from workspace builds is intentionally hidden (`ignore = dirty`) and must never
  be committed upstream.
