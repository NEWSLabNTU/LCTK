# Perforated ICP termination and dormant ArUco cleanup implementation plan

> Planning document only. This pass records the agreed behavior and does not modify source code.

**Goal:** Make perforated-target ICP use one loss-based termination threshold, treat iteration-limit
termination as failure, preserve the existing structural-evidence gates, and remove the unused
ArUco pose/ICP implementation while keeping camera-frame pose solving in
`ros/lidar_to_camera_solver`.

**Primary code:** `rust/calibration-target-detector/src/perforated.rs` and
`ros/lidar_board_detector/src/main.rs`.

**Related decisions:** [ADR 0004](../../adr/0004-lidar-camera-solver-owns-camera-board-pose.md).

## Agreed behavior

The detector will have three meaningful ICP outcomes:

| Outcome | Condition | Selection result |
|---|---|---|
| `GoodFit` | `state.avg_loss < good_fit_threshold_m` | successful hypothesis |
| `StablePose` | `termination_count >= stable_pose_iterations` | successful hypothesis, even when residual is above the good-fit threshold |
| `MaxIterations` | iteration limit reached without either successful condition | failed hypothesis |

The hard-invalid states (`TooFewInliers`, `TooFewKabschPoints`, and
`NoCorrespondences`) remain failures and take precedence over the other outcomes. The precedence
inside one terminal state is therefore:

```text
hard invalid → GoodFit → StablePose → MaxIterations
```

`GoodFit` uses the existing `state.avg_loss` metric and retains its strict `<` comparison. Equality
with `good_fit_threshold_m` does not terminate via `GoodFit`. If the final permitted iteration also
meets `GoodFit`, `GoodFit` wins over `MaxIterations`.

There is no post-ICP residual acceptance gate. After termination, the existing structural gates
remain usable and retain their current placement and meaning:

- minimum final inlier points;
- minimum loss separation when at least two successful hypotheses exist; and
- minimum cutout-rim correspondences.

The residual threshold decides when ICP stops; structural evidence decides whether the selected
result is trustworthy enough to publish.

## Hypothesis policy

Each observation still runs four independent ICP attempts, one for each quarter-turn initial pose.
The maximum work remains four times `max_iterations`.

1. Run all four attempts to a terminal state.
2. Classify each attempt with the single termination-kind function.
3. Discard `MaxIterations` and hard-invalid attempts from candidate selection.
4. Rank only `GoodFit` and `StablePose` attempts by `avg_loss`, with the existing deterministic tie
   break.
5. Reject when no successful hypothesis exists. For diagnostics, retain the lowest-loss failed
   attempt, but never publish it.
6. Accept a single successful hypothesis without a separation comparison.
7. For two or more successful hypotheses, compare the best two with the existing separation gate.

`second_best_loss_m` and `loss_separation_m` become `Option<f64>` in the evidence types. They are
`None` when there is no successful runner-up, rather than synthetic `NaN` or infinity values.

## Configuration contract

- Remove `rejection_threshold_m` from `PerforatedIcpConfig` and all construction sites.
- Use the existing `good_fit_threshold_m` as the loss-based ICP termination threshold.
- Add `stable_pose_iterations: usize` to `PerforatedIcpConfig`.
- Default `stable_pose_iterations` to `3` consecutive stable updates and reject zero during
  configuration validation.
- Expose the field in board-detector configuration as `icp_stable_pose_iterations`.
- Add the new key only to perforated/hollow presets. Solid presets do not receive a perforated ICP
  setting.
- Remove `icp_rejection_threshold` from all in-repository presets, examples, tests, and active
  documentation.
- At the board-config load boundary, report a targeted error when the removed
  `icp_rejection_threshold` key is still present. Do not turn on blanket unknown-field rejection
  without auditing the flattened tuning configuration and unrelated legacy keys.

The solid detector is outside the ICP behavior change. Its outer-edge evidence and the upstream
`square_icp_residual_max` gate remain unchanged. Only the obsolete shared preset key is removed
from solid configurations.

## Implementation tasks

### Task 1 — Refactor perforated ICP configuration and terminal-state classification

**Files:**

- Modify `rust/calibration-target-detector/src/perforated.rs`.
- Modify `rust/calibration-target-detector/src/lib.rs` if public evidence fields change there.
- Update all Rust constructors and fixtures under `rust/calibration-target-detector/`.

**Steps:**

- Remove `rejection_threshold_m` and add validated `stable_pose_iterations`.
- Make `should_terminate`, `termination_kind`, and `successful_termination` share one explicit
  classification path so no caller can apply a different success meaning.
- Evaluate hard-invalid conditions before `GoodFit`, then `StablePose`, then `MaxIterations`.
- Use `good_fit_threshold_m` with a strict `<` comparison.
- Replace the hard-coded stable-pose count with `stable_pose_iterations`, using
  `termination_count >= stable_pose_iterations`.
- Keep `termination_count` as the number of consecutive completed updates whose pose weight is at
  or below `pose_weight_threshold`; reset it on a non-stable update.
- Keep `iteration` as the number of completed ICP updates and preserve the existing max-iteration
  boundary.
- Make `MaxIterations` unsuccessful and remove the separate final residual check.

### Task 2 — Filter and rank successful hypotheses

**Files:**

- Modify `rust/calibration-target-detector/src/perforated.rs`.
- Modify evidence/result definitions in `rust/calibration-target-detector/src/lib.rs` as needed.

**Steps:**

- Classify all four terminal states before sorting.
- Sort only successful (`GoodFit` or `StablePose`) hypotheses for publication.
- Preserve current loss ordering and deterministic tie behavior.
- Implement the zero-, one-, and two-or-more-successful cases explicitly.
- Carry the lowest-loss failed state only as diagnostic evidence in the zero-success case.
- Change runner-up and separation evidence to optional values and update all logging/formatting
  callsites.
- Apply minimum-inlier, separation, and rim-correspondence gates after successful selection, as
  they are today, with separation skipped when there is no successful runner-up.

### Task 3 — Migrate board-detector configuration

**Files:**

- Modify `ros/lidar_board_detector/src/main.rs`.
- Modify perforated/hollow and solid JSON5 presets and any checked-in config fixtures.
- Update configuration documentation and examples found by searching for
  `icp_rejection_threshold`.

**Steps:**

- Remove the shared `icp_rejection_threshold` field, default, and mapping into
  `PerforatedIcpConfig`.
- Add `icp_stable_pose_iterations` with default `3`, pass it only to the perforated configuration,
  and validate it before constructing the detector.
- Keep `icp_good_fit_threshold` as the single loss threshold for perforated ICP.
- Delete the removed key from every shipped profile, including solid profiles.
- Add the targeted stale-key diagnostic at config loading, with an actionable message naming the
  replacement (`icp_good_fit_threshold`) and the new stable-pose option where relevant.

### Task 4 — Delete dormant ArUco pose/ICP code

**Files:**

- Modify `rust/aruco-detector/src/multi_aruco.rs` and its public exports/tests/examples.
- Modify `rust/aruco-detector/Cargo.toml` and the workspace lockfile only if dependency resolution
  requires it.
- Update active documentation that presents the dormant ArUco solver as an owner.

**Steps:**

- Remove `ImageDetection::estimate_pose`, `PoseEstimation`, `ImagePoseMarker`, `fit_icp`,
  `IcpRegression`, the dead `Params` ICP threshold, and supporting dead helpers.
- Preserve marker detection, corner undistortion, `ImageMarker`, and the live detector interface.
- Remove ArUco-only dependencies that become unused, especially `newslab-geom-algo` and
  `cv-convert` if the post-deletion compile confirms no remaining use in that crate.
- Do not add a replacement PnP implementation. The existing
  `ros/lidar_to_camera_solver` remains the owner of camera-frame board pose, PnP initialization,
  refinement, and extrinsic solving.

### Task 5 — Update tests and regression coverage

**Files:**

- Modify unit tests in `rust/calibration-target-detector/src/perforated.rs`.
- Modify `rust/calibration-target-detector/tests/perforated_convergence.rs` and
  `tests/perforated_facade.rs`.
- Add/update board-detector config tests and ArUco crate tests as required.

**Required cases:**

- residual below the good-fit threshold terminates as `GoodFit`;
- equality with the threshold does not terminate as `GoodFit`;
- three consecutive stable updates terminate as `StablePose`;
- two stable updates do not terminate when the configured window is three;
- a stable pose may succeed above the residual threshold;
- max iteration without a successful condition is `MaxIterations` and is not publishable;
- hard-invalid conditions beat low residual, stable count, and iteration limit;
- a good-fit condition on the final permitted iteration beats `MaxIterations`;
- failed hypotheses are excluded from ranking;
- zero successful hypotheses reject while preserving the lowest-loss failure diagnostically;
- one successful hypothesis does not require a runner-up or separation value;
- two successful hypotheses still enforce the existing separation gate;
- minimum inlier and cutout-rim structural gates remain active;
- old `icp_rejection_threshold` configuration is rejected clearly;
- solid detection behavior and `square_icp_residual_max` remain unchanged;
- live ArUco marker detection still compiles and passes its existing contract tests after dead pose
  code deletion.

### Task 6 — Close the documentation loop

**Files:**

- Update `docs/issues/archive/M-21-icp-stable-pose-exit-unreachable.md` with the resolution and move it to
  `docs/issues/archive/` after implementation is verified.
- Update `docs/issues/README.md` and any active phase/spec documentation that describes the old
  threshold roles or ArUco ownership.
- Preserve archived findings as historical records; repair every relative link affected by the
  move.

**Steps:**

- Explain that the old unreachable hard-coded stability bar is replaced by the configurable,
  positive `icp_stable_pose_iterations` window.
- Record that max-iteration termination is intentionally a failed hypothesis.
- Record that residual termination and structural acceptance are separate concerns, without calling
  the residual check a final acceptance gate.
- State that camera-frame board pose belongs to `lidar_to_camera_solver`, and that the dormant
  ArUco solver API was deleted rather than rebuilt.
- Run a repository-wide Markdown-link check and confirm zero dangling `](...*.md)` targets.

## Verification

After implementation, run from the repository root:

```text
just build
just test
just lint
```

Run the focused Rust suites during development as well, including the perforated detector and
ArUco detector packages. Run the ROS-backed solid detector tests from a sourced ROS environment.
The final evidence must include the commands and pass/fail output, plus a sample-data smoke check
showing that a noise-floor residual below `icp_good_fit_threshold` can publish without requiring
the removed rejection threshold.

## Commit sequence

Keep the implementation reviewable and bisectable:

1. Refactor perforated termination and evidence types with unit/integration tests.
2. Migrate board-detector configuration and presets.
3. Delete dormant ArUco pose/ICP code and prune dependencies.
4. Update issue resolution, active documentation, and link verification.

Do not include the unrelated `sessions/solid600-handheld-zed/rviz.rviz` worktree change in these
commits.
