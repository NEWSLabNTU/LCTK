# M-31 · Parked findings from the perforated-ICP termination cleanup

- **Severity:** Medium
- **Area:** calibration-target-detector (perforated ICP), lidar_board_detector, aruco-detector
- **Status:** Open
- **Filed:** 2026-09-03 — final whole-branch review of `feat/selectable-calibration-targets`
  (`44098ef..318f009`), which implemented
  [the perforated ICP termination cleanup plan](../superpowers/plans/2026-09-03-perforated-icp-termination-cleanup.md)

## Why this file exists

The termination cleanup collapsed perforated-target ICP onto one loss threshold
(`good_fit_threshold_m`, strict `<`), made `MaxIterations` an unconditional failure, replaced the
unreachable hard-coded `termination_count > 100` stability bar with a configurable
`stable_pose_iterations` window, and filtered hypothesis ranking to successes only.

The final review of that branch found four things that are real but that each would **reverse or
exceed an explicit decision the plan made**, so none of them was changed during the fix wave. They
are collected here so the decisions are made deliberately rather than by omission.

None of these is a regression introduced by the branch. Items 3 and 4 pre-date it; items 1 and 2
are consequences of decisions the plan took on purpose.

---

## 1. `StablePose` publishes with the residual unbounded, and it is reachable today

**Measured 2026-09-03** on shipped sample dataset 3 (`sessions/sample3-hollow-velodyne`, VLP-32C
pcap, bbox detection path) with temporary per-hypothesis instrumentation in
`estimate_perforated_pose`. Full method and raw numbers:
[M-21, "Reachability re-measured"](./archive/M-21-icp-stable-pose-exit-unreachable.md).

`StablePose` is a **successful** termination that never examines `avg_loss` — it counts consecutive
iterations whose `pose_weight` is at or below `icp_pose_weight_threshold`, nothing more. It can only
fire on a frame where the residual stayed at or above `icp_good_fit_threshold` for the whole
iteration budget, i.e. precisely a frame `GoodFit` has already judged to be a bad fit. The result is
an accept path with **no upper bound on the published residual**.

M-21 assumed this exit was unreachable at shipped budgets, which would have made the exposure
theoretical. It is not:

| preset | `max_icp_iterations` | `StablePose` reachable? |
|---|---|---|
| `hollow_1000/velodyne.json5` | 100 | **yes — 30.7 % of hypotheses** (678 / 2208) |
| `hollow_1000/velodyne_bbox.json5` | 50 | no |
| `hollow_1000/seyond.json5` | 50 | no (inferred from Velodyne data; Falcon unmeasured) |

The first iteration at which `pose_weight` drops to or below the shipped `1e-4` is 49–291 (median
112, n = 55 uncensored). With `icp_stable_pose_iterations: 3`, `StablePose` fires at about
`first_quiet + 2`, which clears a 100-iteration cap for roughly a third of hypotheses and clears a
50-iteration cap for none.

Run with an unreachable `icp_good_fit_threshold` at the 100-iteration cap to isolate the path,
**491 detections published on `StablePose` alone**, with `avg_loss` 0.0215–0.0272 — every one above
the configured good-fit threshold, against only 61 rejections.

**Parked because** closing this means adding a `stable_pose_max_residual_m` (or equivalent) config
key, or removing `StablePose` from the successful set. New configuration surface the plan never
authorised is a product decision, not a reviewer's, and the plan explicitly lists `StablePose` as a
"successful hypothesis, even when residual is above the good-fit threshold".

**Options, in rough order of cost:**

1. Bound the residual on the `StablePose` path with a new config key.
2. Drop `StablePose` from the successful set, making `GoodFit` the only accept condition. Cheapest,
   closes the hole completely, but discards the knob the cleanup just made configurable — and
   re-opens M-21's original complaint that `icp_pose_weight_threshold` is inert.
3. Leave it and document the exposure. Partly done already: the three `hollow_1000` presets now say
   so in the `icp_good_fit_threshold` comment. A documented unbounded accept path is still an
   unbounded accept path.

Note this compounds with item 2 below: a *lone* `StablePose` success also skips the separation gate,
so a frame can publish on pose stability alone, above the good-fit residual, with no
cutout-ambiguity check at all. In the same probe run, **54.7 % of frames (401 / 733) had exactly one
successful hypothesis**, so that combination is the common case rather than a corner.

---

## 2. The loss-separation gate is skipped whenever exactly one hypothesis succeeds

`estimate_perforated_pose` (`rust/calibration-target-detector/src/perforated.rs`) runs four ICP
attempts, one per quarter-turn initial pose, then ranks **only the successful ones**. The runner-up
— and therefore `min_hypothesis_loss_separation_m` — is drawn from that filtered list:

```rust
let runner_up = successful_indices.get(1).map(|&index| &hypotheses[index]);
let separation = runner_up.map(|second| second.state.avg_loss - best.state.avg_loss);
```

With exactly one success, `runner_up` is `None`, `second_best_loss_m` and `loss_separation_m` are
`None`, and the separation gate is skipped entirely.

**The problem case.** Consider `good_fit_threshold_m: 0.035` on a VLP-32C whose range-noise floor
sits at 0.022–0.029:

- the correct quadrant reaches `GoodFit` at `avg_loss = 0.0349`;
- a wrong quarter-turn plateaus at `avg_loss = 0.0351` and exits `MaxIterations`.

Head publishes the 0.0349 pose with `second_best_loss_m: None`, no separation check, and **the
near-tie is not logged at all** — the rejection log that would have named it
(`reason=ambiguous_cutout_evidence`) is never reached. The base implementation rejected this
outright, because the runner-up was drawn from all four hypotheses regardless of outcome.

Because the good-fit threshold necessarily sits at the sensor noise floor (that is what C-04 and
M-29 established), a residual straddling it is the *expected* case on a marginal frame, not an
exotic one. The module's own docstring says "No candidate is accepted unless the cutouts, rather
than common square evidence, select it" — a near-tie between two quarter-turns is exactly the
situation where the cutouts have *not* selected it.

**Parked because** the plan says verbatim: "Accept a single successful hypothesis without a
separation comparison" and "`None` when there is no successful runner-up". Changing the publish
policy reverses two explicit decisions.

**Suggested direction (does not change the publish policy):** keep accepting the lone success, but
compute and log separation evidence against the lowest-loss *other* hypothesis regardless of its
termination kind, so the ambiguity is observable in the logs even when it does not block a publish.
That needs a second, diagnostic-only runner-up field rather than a change to
`second_best_loss_m`'s meaning.

---

## 3. `IcpTermination::NoCorrespondences` is unreachable

In `PerforatedBoardIcpIterator::step`, `correspondences: Vec::new()` is only ever produced together
with `good_correspondences: 0`. Both `should_terminate` and `termination_kind` check
`good_correspondences < 3` **before** `correspondences.is_empty()`, so the empty-correspondence
branch is always masked by `TooFewKabschPoints` and `NoCorrespondences` can never be reported by
the live iterator.

The precedence test in `perforated.rs` pins the ordering by hand-constructing a state the iterator
itself cannot produce, so the suite passes without the variant ever being reachable in production.

**Pre-existing** — the same masking is present at the branch base (`44098ef`); the cleanup
preserved the ordering rather than introducing it.

**Suggested direction:** swap the two checks so `correspondences.is_empty()` is tested first. That
makes the variant reachable and the diagnostic strictly more informative ("no correspondences at
all" is a different operator story from "only two usable ones"), at the cost of changing which
reason a small number of frames report. Either fix it or delete the variant — a terminal state that
cannot occur is a false affordance in the rejection log.

---

## 4. `Detection` in `rust/aruco-detector/src/multi_aruco.rs` is dead

```rust
pub struct Detection {
    pub id: i32,
    pub corners: [Point2<f32>; 4],
    pub pose: Isometry3<f64>,
}

impl Detection {
    pub fn center(&self) -> Point3<f64> { self.pose.translation.vector.into() }
}
```

Neither `Detection` nor `center()` is constructed or called anywhere under `rust/` or `ros/`. The
live detector path uses `ImageMarker` and `ImageDetection`; `Detection` is a separate, orphaned
type. Removing it also drops the now-sole use of the `Isometry3` import in that module. About 12
lines.

**Pre-existing** at both `d7a4d34` and head, and on no task's deletion list — but it is literally an
ArUco *pose* type, which is what the branch's Task 4 ("delete dormant ArUco pose/ICP code") set out
to remove, so it is very likely an oversight rather than a deliberate retention. Camera-frame board
pose belongs to `ros/lidar_to_camera_solver` ([ADR
0004](../adr/0004-lidar-camera-solver-owns-camera-board-pose.md)); nothing should be reintroducing a
pose type here.

**Parked because** it is outside the branch's stated scope and deleting public API is a decision
worth making explicitly.

---

## Related

- [M-21 · ICP stable-pose exit unreachable](./archive/M-21-icp-stable-pose-exit-unreachable.md) —
  the finding this branch set out to close; its resolution section carries the 2026-09-03
  re-measurement.
- [C-04 · ICP accept gate set below the sensor noise floor](./archive/C-04-board-detector-gate-unreachable.md)
- [M-29 · sample-data path dead: shared bbox and ICP gate](./M-29-sample-data-path-dead-shared-bbox-and-icp-gate.md)
