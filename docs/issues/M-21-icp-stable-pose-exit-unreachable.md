# M-21 · ICP's "stable pose" exit is unreachable, so `icp_pose_weight_threshold` is an inert knob

- **Severity:** Medium — no wrong number reaches a consumer, but a shipped tuning parameter has no
  observable effect at any value, and every detection burns the full iteration budget in a stage
  `CLAUDE.md` already names the pipeline's ~100 ms bottleneck
- **Area:** calibration-target-detector (`src/perforated.rs`), board detector configs
- **Status:** Open
- **Verified:** 2026-08-14 — measured by instrumenting `BoardIcpIterator::step` from the
  since-deleted `rust/hollow-board-detector/tests/test_icp_correctness.rs` (W5-E1/E2, Phase 8) on a
  noiseless synthetic 0.5 m board; code read at the deleted crate's `src/algo.rs:381-386` (the
  counter) and `src/algo.rs:404-434` (`should_terminate`). **2026-08-28:** re-pointed at the
  successor `PerforatedBoardIcpIterator` in `rust/calibration-target-detector/src/perforated.rs`
  and corroborated on the new 1 m target geometry — see the dated update at the bottom of this
  file.
- **Related:** [C-04](./archive/C-04-board-detector-gate-unreachable.md),
  [L-01](./archive/L-01-fit-board-icp-false-success.md),
  [L-21](./archive/L-21-find-correspondences-duplicated-tests-wrong-body.md),
  [M-18](./archive/M-18-root-cargo-config-missing-rust-tests-unrunnable.md)

## Problem

`BoardIcpIterator` had two success exits, written as sequential early returns
(deleted `src/algo.rs:404-414`):

```rust
if state.avg_loss < config.icp_rejection_threshold { return true; }   // "Converged (good fit)"
if state.termination_count > 100 { return true; }                     // "Converged (stable pose)"
```

The successor, `PerforatedBoardIcpIterator::should_terminate`
(`rust/calibration-target-detector/src/perforated.rs:234-241`), keeps exactly the same two
conditions but combines them into one boolean expression instead of sequential returns:

```rust
state.avg_loss < self.config.rejection_threshold_m   // "good fit"
    || state.termination_count > 100                  // "stable pose"
    || state.iteration >= self.config.max_iterations
    || state.inlier_points.len() < self.config.min_inlier_points
    || state.good_correspondences < 3
    || state.correspondences.is_empty()
```

`termination_count` is incremented by `step` on any iteration whose pose weight
(`|Δtranslation| + |Δangle|`) is at or below `icp_pose_weight_threshold` (now
`pose_weight_threshold` on `PerforatedIcpConfig`), and reset to zero otherwise — deleted
`src/algo.rs:381-386`, successor `rust/calibration-target-detector/src/perforated.rs:210-219`. The
second exit therefore still requires **101 consecutive** quiet iterations.

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

Crossing points, and the step at which the stable-pose exit finally fires — measured on the
deleted crate's 0.5 m board; the named configs are pointers to *config values*, not to a re-run
of these numbers on today's geometry (see the 2026-08-28 update for that):

| `icp_damping_factor` | reaches 1e-4 | reaches 1e-6 | first `termination_count > 0` | `> 100`, i.e. exit |
|---|---|---|---|---|
| 0.5 (`hollow_1000/seyond.json5`) | step 168 | step 539 | step 539 | **step 639** |
| 1.0 (`hollow_1000/velodyne.json5`) | step 112 | step 296 | step 296 | **step 396** |

Against the shipped caps (config values unchanged since this table was written; paths updated —
the originals, `board/board_detector.json5`, `board/board_detector_seyond.json5` and
`board/board_detector_velodyne.json5`, were deleted in W5-E1):

| config | `max_icp_iterations` | `icp_pose_weight_threshold` | steps actually needed |
|---|---|---|---|
| `board/hollow_1000/seyond.json5` | 50 | 1e-4 | ~268 |
| `board/hollow_1000/velodyne.json5` | 100 | 1e-4 | ~212 |

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

`initial_state` builds a state with `good_correspondences: 0` (deleted `src/algo.rs:264`;
successor `rust/calibration-target-detector/src/perforated.rs:140`), and `should_terminate` reads
anything below 3 as a reason to stop (deleted `src/algo.rs:425`; successor
`rust/calibration-target-detector/src/perforated.rs:239`). **A freshly built initial state
therefore still always reports "terminate".**

Production is unaffected: `ros/lidar_board_detector` and the library helpers call `step` first and
consult the predicate on the *result*. But the natural-looking

```rust
let mut state = iterator.initial_state(pose, points);
while !iterator.should_terminate(&state) { state = iterator.step(&state); }
```

executes its body **zero times**, and any assertion after such a loop describes the seed rather than
ICP. Four tests in the deleted `rust/hollow-board-detector/tests/test_icp_correctness.rs` —
`test_identity_transformation_convergence`,
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
this issue is the deliverable. The measurements were reproducible from
`rust/hollow-board-detector/tests/test_icp_correctness.rs` (deleted W5-E2), whose
`test_convergence_counter_increases` carried the same figures in a comment and had to raise
`max_icp_iterations` to 3000 before the stable-pose exit could be observed at all. See the
2026-08-28 update below for the migrated equivalent and its own iteration count.

## Update (2026-08-28) — evidence pointers repaired; finding corroborated on the new geometry

W5-E2 (`21142ac`) deleted `rust/hollow-board-detector` and `rust/hollow-board-config`; the ICP
implementation this issue is about now lives in `PerforatedBoardIcpIterator`
(`rust/calibration-target-detector/src/perforated.rs`), and the six-test ICP convergence suite
this issue's numbers came from was migrated to
`rust/calibration-target-detector/tests/perforated_convergence.rs`. Every `src/algo.rs` and
`tests/test_icp_correctness.rs` reference above has been re-pointed at its successor in place.

`termination_count > 100` is still the stable-pose bar
(`rust/calibration-target-detector/src/perforated.rs:236`), so the core finding — the knob is
inert unless a run survives past step ~100 — is unchanged in the successor code. The shipped
iteration caps checked against `ros/lctk_launch/config/board/`
(`hollow_1000/seyond.json5`, `hollow_1000/velodyne.json5`, `hollow_1000/velodyne_bbox.json5`, and
the newer `solid_600/seyond.json5`, `solid_600/velodyne.json5`) are still 50 or 100
`max_icp_iterations`, `icp_pose_weight_threshold: 1e-4`, `icp_damping_factor` 0.5 or 1.0 depending
on sensor — the same values this issue's table already covered.

The migrated convergence suite's `test_convergence_counter_increases`
(`rust/calibration-target-detector/tests/perforated_convergence.rs:667-730`) re-measures the
stable-pose exit — this time on the shipped **1 m** hollow target manifest
(`fixtures/targets/hollow_1000_aruco_4_v1.json5`, the fixture mirror of
`ros/lctk_launch/config/targets/hollow_1000_aruco_4_v1.json5`) rather than the deleted crate's
0.5 m board, and going through the crate's public `TargetPoseEstimator` facade rather than a raw
iterator (`PerforatedBoardIcpIterator` is private to the crate). Its doc comment states the
stable-pose exit "needs ~1809 iterations" on this manifest and seed, and the test asserts exactly
that shape: `evidence.termination == IcpTermination::StablePose` only after raising the test's own
cap to 5000 iterations (`MAX_ITERATIONS: usize = 5000`, run against `rejection_threshold_m = 0.0`
so the good-fit exit cannot pre-empt it), plus a converged-pose check (`corner_error < 1e-3` m)
so a frozen-but-wrong pose would not pass for a converged one. I read this test and its
surrounding comments directly to confirm the ~1809 figure and the assertion shape described here.

~1809 iterations is the same order of magnitude as the deleted crate's ~639 (measured at
`icp_damping_factor: 0.5`) — corroborating M-21's finding on new geometry and new code: on both
the old 0.5 m board and the new 1 m target, the stable-pose exit needs on the order of one to two
thousand iterations, roughly one to two orders of magnitude past every shipped preset's
`max_icp_iterations` (50–100). `icp_pose_weight_threshold` remains inert on the current code for
the same reason it was inert on the deleted code.

Two things about the "Notes" section's old figure are now stale, flagged rather than silently
edited: the deleted suite needed to raise its cap to **3000** to observe the stable-pose exit on
the 0.5 m board; the migrated suite needs **5000** on the 1 m manifest (both are test-only
iteration caps, well above any shipped preset). This is a different geometry and a fixed sign bug
(H-15) between the two measurements, so the two cap values are not expected to match and neither
should be read as more precise than "several hundred to a couple thousand."
