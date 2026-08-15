# L-25 · conflux CLAUDE.md documents a test count the suite does not have

- **Severity:** Low
- **Area:** conflux docs
- **Status:** Fixed (2026-08-15)
- **Verified:** Measured 2026-08-15
- **Location:** `ros/conflux/CLAUDE.md` (Testing section)

## Problem

`ros/conflux/CLAUDE.md` states:

```bash
just test-core          # 166 tests
```

The actual count is **156** (measured after H-13 repaired the tokio suite, so this is the
count with every gated test compiling and running). The documented figure was already wrong
while 20 tests were failing to compile, which means it has been stale for at least as long as
H-13 went unnoticed.

Same section also predates the 2026-08-15 tooling changes: `just test-python` no longer goes
through colcon (M-25), and `just test-rust` now passes `--features tokio` (H-13).

## Failure scenario

A hardcoded count in prose is a check nobody runs. It gave false assurance about suite size
during exactly the period when a fifth of the core tests were not compiling.

## Suggested fix

- Correct the figure, or better, drop the hardcoded count and describe what the suite covers
  instead — the number changes on every test added.
- Refresh the Testing section for the current recipes.
- Same audit applies to the profiling and mode tables further down, which have not been
  re-measured since the 2026-01-18 run.

Related: H-13, M-25.

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@0a9c901; LCTK pins it). The hardcoded `# 166 tests` is gone
rather than corrected — a fixed number in prose is a check nobody runs, and it was wrong during
exactly the period when a fifth of the core tests were not compiling.

Also refreshed: the source-tree diagram (staleness removed, `config.rs` added, `state.rs` noted as
carrying `advance`/`match_status`/`reset`), and the recipe descriptions (`test-python` no longer
goes through colcon, `test-cpp` now exists).

Added a **Known Issues** section recording that three test recipes once could not fail — H-13,
M-25 and L-22 — with the standing instruction to break an assertion deliberately and confirm a
non-zero exit before trusting a new recipe. That is the reusable lesson; the individual counts were
not.
