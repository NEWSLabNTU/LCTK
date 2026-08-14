# M-21 · ICP's "stable pose" exit is unreachable, so `icp_pose_weight_threshold` is an inert knob

- **Severity:** Medium — no wrong number reaches a consumer, but a shipped tuning parameter has no
  observable effect at any value, and every detection burns the full iteration budget in a stage
  `CLAUDE.md` already names the pipeline's ~100 ms bottleneck
- **Area:** hollow-board-detector (`src/algo.rs`), board detector configs
- **Status:** Open
- **Verified:** 2026-08-14 — measured by instrumenting `BoardIcpIterator::step` from
  `rust/hollow-board-detector/tests/test_icp_correctness.rs` on a noiseless synthetic board;
  code read at `src/algo.rs:381-386` (the counter) and `src/algo.rs:404-434` (`should_terminate`)
- **Related:** [C-04](./archive/C-04-board-detector-gate-unreachable.md),
  [L-01](./archive/L-01-fit-board-icp-false-success.md),
  [L-21](./archive/L-21-find-correspondences-duplicated-tests-wrong-body.md),
  [M-18](./archive/M-18-root-cargo-config-missing-rust-tests-unrunnable.md)

## Problem

`BoardIcpIterator` has two success exits (`src/algo.rs:404-414`):

```rust
if state.avg_loss < config.icp_rejection_threshold { return true; }   // "Converged (good fit)"
if state.termination_count > 100 { return true; }                     // "Converged (stable pose)"
```

`termination_count` is incremented by `step` on any iteration whose pose weight
(`|Δtranslation| + |Δangle|`) is at or below `icp_pose_weight_threshold`, and reset to zero
otherwise (`src/algo.rs:381-386`). The second exit therefore requires **101 consecutive** quiet
iterations.

The iteration converges geometrically but *slowly* — the per-step pose weight shrinks by a factor of
only about **0.987** at the shipped `icp_damping_factor: 0.5`. Measured on a 41×41 noiseless grid
over the 0.5 m board, seeded 2 cm and 3° from truth:

| pose weight | step |
|---|---|
| 1.56e-3 | 0 |
| 8.64e-4 | 20 |
| 2.29e-4 | 100 |
| 1.40e-4 | 140 |
| 7.10e-6 | 380 |

Crossing points, and the step at which the stable-pose exit finally fires:

| `icp_damping_factor` | reaches 1e-4 | reaches 1e-6 | first `termination_count > 0` | `> 100`, i.e. exit |
|---|---|---|---|---|
| 0.5 (`board_detector.json5`, `_seyond`) | step 168 | step 539 | step 539 | **step 639** |
| 1.0 (`board_detector_velodyne.json5`) | step 112 | step 296 | step 296 | **step 396** |

Against the shipped caps:

| config | `max_icp_iterations` | `icp_pose_weight_threshold` | steps actually needed |
|---|---|---|---|
| `board/board_detector.json5` | 50 | 1e-4 | ~268 |
| `board/board_detector_seyond.json5` | 50 | 1e-4 | ~268 |
| `board/board_detector_velodyne.json5` | 100 | 1e-4 | ~212 |

Every preset is short by a factor of 2–5, and that is on *noiseless* data; real returns carry the
VLP-32C's ±3 cm range noise, which floors the residual and can only make the pose jitter more, not
less.

## Why it matters

1. **`icp_pose_weight_threshold` is inert.** It is the only consumer of the pose weight, and the
   only consumer of `termination_count` is a branch that never runs. An operator can set it to 1e-4,
   1e-13 (as `config/multi_wayside/detector.json5` does) or anything else and observe *no difference
   whatsoever* in the detector's output. This is the tracker's recurring shape yet again — a control
   that appears to work and does nothing — and it is exactly the trap
   [C-04](./archive/C-04-board-detector-gate-unreachable.md) sprang with `icp_good_fit_threshold`,
   with the polarity reversed: there a gate could never *pass*, here a gate can never *fire*.
2. **Runs always exhaust the budget.** The remaining exits are the loss gate and
   `state.iteration >= max_icp_iterations`. The shipped `icp_rejection_threshold` is 0.005–0.008,
   while `CLAUDE.md`'s own profiling section records the real per-point residual floor at
   0.026–0.029 m for a VLP-32C — so on live data the loss gate cannot fire either, and **every**
   detection runs its full 50 or 100 iterations. `max_icp_iterations` is the de-facto termination
   criterion of this detector. That is the same observation
   [C-04](./archive/C-04-board-detector-gate-unreachable.md) recorded in passing
   ("`max_icp_iterations: 50` truncates convergence — the loss is still decreasing when ICP stops"),
   now quantified: at step 50 the pose weight is still ~4.6e-4, roughly 5× the configured
   threshold and two orders of magnitude above the settled value.
3. **Stopping at the cap is reported as an ordinary result.** `termination_reason` says
   `"Max iterations reached: N"`, which the library-side success test treats as a *successful* fit —
   that is [L-01](./archive/L-01-fit-board-icp-false-success.md), and this finding is why the
   "non-converged" branch is not an edge case but the norm.

## Analysis — why convergence is this slow (not proven)

The rate looks **inherent to the correspondence model rather than to the damping**, but this is
reasoning from the geometry, not a measured decomposition, and should be treated as a hypothesis:

`find_correspondences` maps each input point to the nearest point *on the board model*. For any
point in the plate's interior that nearest point is the point's own projection onto the plate plane —
identical whichever way the board is rotated in-plane or slid across its own plane. Such a
correspondence contributes an exactly zero residual to the in-plane degrees of freedom, so it adds
weight to the Kabsch fit without adding information. Only samples near the plate's outer edges and
the three hole rims — a small minority of a dense grid, and a smaller one still on a real cloud —
actually constrain in-plane pose. The fit therefore moves a small fraction of the way per step, and
halving the damping factor changes that fraction but not the mechanism (the 0.5 → 1.0 row above
moves the exit from 639 to 396 steps: a real improvement, nowhere near the 2× a damping-limited
process would give, and still far past the caps).

If that is right, raising `max_icp_iterations` is a poor fix — it buys convergence with latency in
the pipeline's slowest stage. A point-to-plane residual, or weighting correspondences by whether
they are edge/rim constrained, would attack the rate itself.

## A second, related wart: `should_terminate` says "stop" before ICP has started

`initial_state` builds a state with `good_correspondences: 0` (`src/algo.rs:264`), and
`should_terminate` reads anything below 3 as a reason to stop (`src/algo.rs:425`). **A freshly built
initial state therefore always reports "terminate".**

Production is unaffected: `ros/lidar_board_detector` and the library helpers call `step` first and
consult the predicate on the *result*. But the natural-looking

```rust
let mut state = iterator.initial_state(pose, points);
while !iterator.should_terminate(&state) { state = iterator.step(&state); }
```

executes its body **zero times**, and any assertion after such a loop describes the seed rather than
ICP. Four tests in `tests/test_icp_correctness.rs` — `test_identity_transformation_convergence`,
`test_small_translation_recovery`, `test_small_rotation_handling`,
`test_convergence_counter_increases` — were written that way and asserted nothing about ICP for as
long as they existed; each ended on `assert!(iterator.should_terminate(&state))`, which was
trivially true because the loop had never run. They are fixed as of 2026-08-14 (every ICP test now
goes through a `run_icp` helper that steps *before* consulting the predicate, and asserts on pose
error rather than on control flow), and the trap is documented in that helper's doc comment — but
the API shape that invites it is still there.

This is the same family as [L-21](./archive/L-21-find-correspondences-duplicated-tests-wrong-body.md)
(tests that ran, passed, and exercised nothing that mattered), and it went unnoticed the longer for
[M-18](./archive/M-18-root-cargo-config-missing-rust-tests-unrunnable.md), which left the Rust suite
unrunnable from the workspace root.

## Suggested fix

Pick one of:

1. **Make the knob real.** Lower the `termination_count > 100` bar to something a converging run
   actually reaches (a handful of quiet steps is what "the pose stopped moving" means), so
   `icp_pose_weight_threshold` starts to control something. Note this changes detector behaviour and
   should land with before/after detection counts on the sample data, as C-04's fix did.
2. **Or delete it.** Drop `termination_count`, the pose-weight computation and
   `icp_pose_weight_threshold` from `Config` and from all four board configs, and document
   `max_icp_iterations` as what actually ends the loop. An honestly absent knob beats an inert one.

Either way:

3. Address the rate, or accept it explicitly. If the correspondence-model analysis above holds,
   record the measured convergence budget next to `max_icp_iterations` in the configs so the value
   reads as the deliberate latency/accuracy trade-off it is, rather than as a safety backstop that
   is never hit.
4. Consider making `should_terminate` answer honestly for `state.iteration == 0` (for instance by
   not treating "no correspondences computed yet" as a stop reason), or rename it so the
   step-then-check order is part of the contract.

## Notes

Found on 2026-08-14 while giving the four vacuous ICP tests real assertions; the slow-convergence
numbers are a by-product of making them run for the first time. No production code was changed —
this issue is the deliverable. The measurements are reproducible from
`rust/hollow-board-detector/tests/test_icp_correctness.rs`, whose
`test_convergence_counter_increases` carries the same figures in a comment and has to raise
`max_icp_iterations` to 3000 before the stable-pose exit can be observed at all.
