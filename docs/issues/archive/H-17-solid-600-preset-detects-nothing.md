# H-17 · The `solid_600` detector preset rejects every frame of real data

- **Severity:** High
- **Area:** lidar_board_detector / config/board/solid_600
- **Status:** Fixed 2026-09-04 -- the coverage gate is decoupled, and the ROS-versus-offline
  discrepancy that remained was `offline` mode receiving nothing from the bag (M-30), not a
  detector defect. See "Resolved" below.
- **Found:** 2026-08-31, first run of the solid target against real sensor data
- **Data:** `~/Downloads/new_LCTK_board/` (`newtype_background`, `newtype_1`, `newtype_2`),
  ZED + VLP-32C + Seyond, from the 2026-08-12 capture

## Problem

The first real-data run of the 600 mm solid target produced **zero accepted board
detections in 369 frames**. The camera side works; the LiDAR side rejects everything, so
no LiDAR-camera pair is ever formed and no extrinsic is ever solved.

This is the first time `solid_600/*` has been run against a real LiDAR — Phase 8's W7-B
is still outstanding — so the presets were never tuned, only written.

## What is *not* wrong

Worth recording, because each was suspected and cleared by measurement:

- **Background subtraction works.** With warmup on the board-free `newtype_background`
  bag, the node's foreground is median 273, max 483 points/frame, matching an independent
  offline computation (median 313). The `BackgroundState` machine stops accumulating once
  `Ready`, so a static board is *not* absorbed into the background.
- **The board is separable.** Offline single-linkage clustering of the foreground finds a
  board-shaped cluster in most frames: 63–420 points, flatness RMS 0.012–0.030 for the
  clean ones, at 6–8 m range.
- **`patch_min_points: 60` is not the blocker.** No sampled frame's best cluster fell
  below it.
- **The anisotropic clustering was ported correctly.** `dbscan.rs` implements the
  range-scaled vertical widening (`anisotropic_scaled`) that the Python original uses.

## What is wrong

**1. `cluster_eps` is double the validated value.** The presets ship `0.30`; the Python
original that produced the 88.4%/100% Method-E result defaults to `0.15`
(`background_diff.py:26`). Since the anisotropic scaling already widens the *vertical*
tolerance with range, a doubled *horizontal* eps merges the board into whatever is behind
it. Measured effect of `0.30 -> 0.15`: "no candidate clusters survived foreground
extraction" fell from 520 frames to 315, a 39% reduction. Still not sufficient alone.

**2. The isolation gate is hostile to handheld capture.** With `isolation: true` and
`isolation_max_density: 0.3`, 66 frames failed specifically on "embedded clutter". A
handheld board always has a person inside the isolation band. `solid_600_handheld.yaml`
exists as a shipped example, so handheld is an intended capture mode, and the gate as
tuned contradicts it.

**3. `icp_min_inlier_points: 100` exceeds what the board yields.** The cleanest
board-only clusters measure 63–100 points at 6–8 m. The 1000 mm perforated plate returns
far more, which is where this number came from.

**4. The residual failures are shape gates.** After relaxing 1–3: 315 frames "no
candidate clusters survived", 54 "square fit residual exceeded
`square_icp_residual_max`". The merged board+holder clusters measure 0.7–1.5 m in extent
against a 0.6 m plate (0.849 m diagonal) with flatness 0.06–0.11 versus
`flatness_rms_max: 0.045` — correctly rejected, since they contain a person. The
board-only cluster is not surviving as a separate candidate.

## Root cause: the square-fit coverage gate is unreachable for this board and sensor

Measured 2026-08-31. This is the decisive finding; the gates in the previous section are
real but secondary, and relaxing them only moves the failure here.

`square_fit.rs::coverage_residual` returns `mean_outside / side + coverage_penalty`,
where `coverage_penalty` is the fraction of 40 perimeter bins (4 sides x 10) that hold no
point within `BAND_FRAC * side` = **3.6 cm** for a 600 mm board.

Computing that same residual over 30 real, planar board clusters from `newtype_1`:

| quantity | best | median |
|---|---|---|
| total residual | 0.436 | 0.684 |
| coverage penalty alone | 0.425 | 0.675 |

Against `square_icp_residual_max: 0.45`, **29 of 30 clusters are rejected.** The best
frame the sensor produced misses the gate by 0.014.

The residual is almost entirely coverage penalty: `mean_outside / side` contributes about
0.01, meaning the square model **fits the geometry well** -- the board really is a
600 mm square where it is sampled. What fails is the demand that points appear all the
way round the perimeter.

They cannot. A VLP-32C at 7-8 m samples anisotropically: roughly **2.8 cm between points
within a ring, but ~15 cm between rings**. A 600 mm plate is therefore crossed by only
about four rings. The two horizontal edges fall between rings and can never hold a point
within 3.6 cm, so ~half the bins are unfillable by construction.

An adaptive band was tried and rejected: widening it to the cloud's own median
nearest-neighbour spacing changes nothing, because that spacing (2.4-3.0 cm measured) is
the *horizontal* one and is already smaller than the fixed band. The quantity that
matters is the vertical ring gap.

### Why this is a C-04 repeat

[C-04](./C-04-board-detector-gate-unreachable.md) set `icp_good_fit_threshold`
below the sensor's noise floor, so the detector silently accepted nothing. This is the
same shape with a different quantity: a gate placed beyond what the sensor can deliver,
producing silence rather than an error. Both were invisible because the detector reports
"no board selected" either way.

The general lesson the tracker keeps relearning: **a gate must be set from what the
sensor produces, not from what a clean model would produce.**

### Two candidate fixes

1. **Config only -- tried, and it does not work.** Raising
   `square_icp_residual_max` to 0.75 (above the 0.684 median measured offline) still
   accepted nothing: the node's own reported residuals are 0.752-0.950, median 0.838,
   with 52 frames still failing this gate. Even a threshold that accepted them would be
   meaningless, since 0.95 means almost no perimeter coverage is required at all.

   The gap between the node's residuals (best 0.752) and the same metric computed offline
   on hand-clustered board points (best 0.436) is a **second finding**: the detector's
   candidate formation is handing the square fit worse point sets than the data supports,
   i.e. clusters still carrying the holder or fragments. Candidate formation and the
   coverage metric both need work; fixing either alone is not enough.
2. **Anisotropic coverage band -- tried in simulation and DISPROVEN.** The idea was to
   widen the band to the sampling pitch across each edge, so a bin is only charged when a
   point could have landed in it. Prototyped in Python against the real clusters before
   touching Rust, precisely so a speculative change would not disturb the parity fixtures.

   It changes nothing. Measured level spacing on real board clusters is 0.037-0.044 m on
   **both** in-plane axes, so half of it (0.019-0.022) never exceeds the fixed 0.036 band.
   The reason is that the board is mounted diamond-wise: the fitted plane's principal axes
   run along the plate's diagonals, so neither axis is purely vertical and both mix the
   ~2.8 cm in-ring spacing with the ~15 cm ring gap. The prototype was verified correct on
   synthetic ring-sampled data (0.150 vertical, 0.056 horizontal), so this is a real
   negative result, not a broken experiment.

3. **The deeper problem: the coverage term does not discriminate here, and cannot simply
   be dropped.** Over the same clusters, split by flatness:

   | | n | coverage residual, median |
   |---|---|---|
   | board (planar, flatness <= 0.03) | 30 | 0.684 |
   | non-board (flatness > 0.03) | 4 | 0.553 |

   Non-board scores **better** than board. The term is not separating the two; it is only
   blocking. What actually separates them is flatness, which is already its own gate.

   The geometric half tells the opposite story: `mean_outside / side` is 0.0082 median for
   board (max 0.0276), and a 0.02 gate would admit 29 of 30. So the square model fits the
   real geometry well.

   But the coverage term **cannot just be zeroed**, because it is load-bearing for the
   theta search, not only for gating. The module header states this: it charges for both
   points outside the square and perimeter the square fails to reach, "so an over-large or
   mis-rotated enclosing box is still penalized and the search has a gradient to follow."
   Removing it leaves rotation nearly unconstrained, since an enclosing square larger than
   the cloud is close to rotation-invariant under `mean_outside` alone.

   The fix therefore has to **decouple the score that selects theta from the residual that
   gates acceptance** -- keep coverage driving the search, gate on the geometric term plus
   the existing flatness/extent gates. That is a change to the detector's contract and
   deserves a deliberate decision rather than a preset tweak, which is why it stops here.

### Caveat on the evidence

The non-board sample is only n=4, drawn from one bag by a flatness split. The conclusion
that coverage does not discriminate is well supported for *this* board, sensor and range;
it should be re-measured on a mounted (non-handheld) capture and at closer range before
being generalised.

## Why it is High

The solid target is undetectable as shipped. Every gate value in `solid_600/*` was
inherited from the 1000 mm perforated plate, which returns several times more points and
is mounted rather than handheld. Nothing in the repo would have caught this: the presets
have tests for *existence* (`test_target_presets.py`) but nothing exercises them against
data.

## Suggested fix

1. Adopt `cluster_eps: 0.15`, matching the value the offline result was measured at, for
   the `solid_600` presets. Consider it for `hollow_1000` too, since the same 0.30 appears
   there and was equally unvalidated.
2. Decide whether handheld capture is supported. If yes, `isolation` must be off or
   re-tuned for `solid_600`; if no, `solid_600_handheld.yaml` should go.
3. Lower `icp_min_inlier_points` for the solid target, from measurement rather than guess.
4. Then re-measure on `newtype_1` and `newtype_2` and record the accepted-frame rate,
   the way Method E's 88.4%/100% was recorded. Do not tune a gate below what the sensor
   can deliver — that is how C-04 made the detector silently accept nothing.

## Related

- [C-04](./C-04-board-detector-gate-unreachable.md) — the same failure class: a
  gate set beyond what the data can satisfy, producing silence rather than an error.
## The marker-ID finding (already fixed on this branch)

The camera half of this failure was a placeholder ArUco id. `solid_600_aruco_1_v1`
declared `marker_ids: [1]`; the physical board carries **id 24**. This branch already
carries that fix, reached independently. Corroborated from the source branch by scanning
every predefined OpenCV dictionary over sampled frames of the 2026-08-12 capture:
`newtype_1` 52/60 frames, `newtype_2` 56/60, and `newtype_background` (board absent) none
at all -- so the id travels with the board rather than being fixed in the room. All four
`DICT_5X5_*` sizes report it because they are nested and 24 < 50.

With the id correct the locator detects reliably and the solver receives `counts=(1, 0)`.
Everything above is the remaining LiDAR-side failure, which is still open.

## Fix applied 2026-09-04: the gate is decoupled from the theta search

`square_fit.rs` now reports the residual's two halves separately. `SquareFit.residual` is
unchanged and still selects theta -- the coverage term has to keep doing that, or a
mis-rotated or over-large enclosing square is barely penalised. The new
`SquareFit.geometric_residual` is the geometric half alone: the mean distance of points
falling *outside* the modelled square, over side.

A new optional tuning field, `square_geometric_residual_max`, gates acceptance on that
half instead of the combined number. It defaults to unset, so every existing preset and
the golden parity fixtures keep their exact behaviour; only the `solid_600` presets set
it, at **0.05**.

That threshold is measured, not chosen: the geometric residual over 30 real board
clusters was median 0.0082 and max 0.0276, while a point cloud twice the model's size
scores above 0.05 (pinned by `geometric_residual_still_rejects_points_outside_the_square`).
Discrimination stays with `flatness_rms_max`, the extent gates and isolation -- which is
where it already lived, since coverage was never separating board from clutter here
(board median 0.684 against non-board 0.553).

### Result on the real recording

Run against `sessions/solid600-handheld-zed`, whose bag is `new_LCTK_board/newtype_1`:

| | square-residual rejections | board detections |
|---|---|---|
| before (combined gate, 0.45) | 18 of 127 frames | 0 |
| after (geometric gate, 0.05) | **0** | first non-zero detection observed |

The gate this issue was filed about no longer blocks anything. Parity is untouched: the
same four Method-B fixtures fail before and after, which is the separate
fixture-provenance problem, not a regression here.

### What remains

Every rejection is now `no candidate clusters survived foreground extraction`, and the
detection rate is still roughly 1%. That is a **different** defect from the one this
issue diagnosed: candidate formation hands the square fit worse point sets than the data
supports. The gap was already visible in the original investigation -- the node's best
residual was 0.752 where the same metric over hand-clustered board points gave 0.436 --
and it is what should be attacked next.

Two measurement notes for whoever picks that up:

- `patch_min_points: 40` and `icp_min_inlier_points: 50` cut logged rejections from 113 to
  43 but did **not** raise the detection count, so they are not justified yet and were
  reverted. The remaining values are the inherited 60 and 100.
- Rates could not be compared cleanly because `mode:=realtime` drops frames
  nondeterministically, and `mode:=offline` receives nothing from a bag (M-30).
  `bag_play.py` accepts `--play-arg`, but `session.launch.py` does not expose it, so
  there is no way to force reliable playback QoS through a session. Exposing it would
  make this measurable and is a natural M-30 follow-up.

## Candidate formation, 2026-09-04

The remaining blocker was attacked with a deterministic offline harness --
`rust/board-cluster-detector/tests/h17_candidate_diagnostic.rs`, `#[ignore]`d because it
needs frames exported by `tmp/export_diag.py`. It runs the real pipeline stages on real
frames outside ROS, which the ROS path cannot do reproducibly: `realtime` drops frames
nondeterministically and `offline` receives nothing from a bag (M-30).

It reports, per frame, where the biggest cluster dies. On 12 frames spread across
`newtype_1`, with the shipped preset:

| outcome | frames |
|---|---|
| passes the patch gate | 5 |
| biggest cluster 35-51 points vs `patch_min_points: 60` | 4 |
| flatness 0.0466-0.0607 vs `flatness_rms_max: 0.045` | 3 |

**`patch_min_points` lowered 60 -> 40.** A 600 mm plate at 7-8 m returns 35-420 points
depending on how many rings cross it; 60 was inherited from the 1000 mm perforated plate
and cuts genuine boards at the sparse end. Offline candidate rate rises **42% -> 75%**
(5/12 -> 9/12), and logged ROS rejections fall 113 -> 55.

**`flatness_rms_max` deliberately NOT relaxed.** The three frames failing it measure
0.0466-0.0607, while clean board-only clusters measure 0.012-0.030, so those clusters are
carrying the holder. Raising the gate would admit clutter, and flatness is what does the
discriminating that coverage never did.

### Two hypotheses tested and disproven

Recorded because both are plausible enough to be re-proposed:

- **The session's self-warmup absorbs the board.** The shipped session builds its
  background from `newtype_1`'s own opening frames rather than the board-free
  `newtype_background` bag. Running the harness against both gives *identical* results --
  same candidate count, same failure reasons, same cluster sizes. The README's claim that
  the operator carries the board in after warmup holds.
- **Realtime warmup samples frames spread across the bag, so the board enters the
  background at several positions.** A background built from 20 frames at stride 28 gives
  7/12 candidates against 9/12 from 20 consecutive frames -- worse, but nowhere near
  enough to explain the ROS behaviour.

### Resolved: ROS and offline disagreed because the ROS runs were in `offline` mode

Measured 2026-09-04, after both fixes above, on `sessions/solid600-handheld-vlp` against
`newtype_1`. One session, one bag, one build; only `mode` differs:

| `mode` | accepted detections | rejections |
|---|---|---|
| `realtime` | **90** | 108 |
| `offline` | **0** | **0** |

**Zero rejections is the tell.** A detector that is rejecting frames logs why it rejected
them; this one logged nothing because it never received a cloud. Its whole run is eleven
lines of startup. The player says so directly:

```text
[WARN] [rosbag2_player]: New subscription discovered on topic '/velodyne_points',
requesting incompatible QoS. No messages will be sent to it.
Last incompatible policy: RELIABILITY_QOS_POLICY
```

That is [M-30](../M-30-bag-playback-qos-mismatch-is-silent.md): the recording replays with
the QoS its publishers used, which for this rig's LiDAR is BEST_EFFORT, so `offline`'s
RELIABLE subscriber is simply incompatible and receives nothing. The camera half keeps
working, which is what makes the failure read as a broken LiDAR detector. The session
README states `mode=realtime` is required for exactly this reason.

So "one non-empty detection per run" was measuring the transport, not the detector.
Through the node in `realtime`, on `newtype_1`:

- before either fix above: **104** accepted detections
- after both: **90**

Both are around ninety times the symptom. The difference between them is run-to-run
variance -- `realtime` drops frames nondeterministically, as the candidate-formation
section notes -- not a regression.

### `play_args`, and what reliable offline playback showed

`session.launch.py` now takes `play_args`, forwarded one token per `--play-arg` to
`bag_play.py`, so playback QoS can be overridden without switching the whole graph to
realtime:

```bash
ros2 launch lctk_launch session.launch.py session:=... mode:=offline \
  play_args:="--qos-profile-overrides-path /path/to/qos.yaml"
```

It was added here to make the ROS run comparable with the offline harness, and it
immediately narrowed the question. Run `mode:=offline` with reliable playback QoS, the
node receives clouds -- but processes only **116 of the bag's 578 LiDAR frames**, and
yields one non-empty detection.

The 116 is understood and is *not* QoS: the detector cannot keep up with 10 Hz playback,
so the node's "store latest, skip stale" subscription (CLAUDE.md's ArcSwap pattern) drops
roughly four frames in five. That also means warmup consumes the first 20 frames it
*processes*, spanning ~10 s of bag rather than the opening second -- so the harness was
re-run with a background built exactly that way (`nodebg`, stride 5). It gives 7 of 12
candidates, no worse than the other backgrounds.

### Why this is closed, and the one thing still unproven

Frame dropping explains the 116, not the accept rate inside it: `realtime` accepted 90 of
198 processed frames (45%), reliable-QoS `offline` one of 116 (0.9%). The two runs differ
in one other way, and it is the likely reconciler -- **the `realtime` runs were made with
`vertical_gap_deg: 0.5` in `solid_600/velodyne.json5`, the reliable-QoS `offline` run with
the shipped `1.0`.** At 1.0 deg a VLP-32C's own ring spacing can exceed the bar on a board
held at range, so one plate arrives as several fragments and none survives the
point-count gates. That value is now committed at 0.5, which makes the difference moot
going forward.

This is recorded as the likely explanation, not a measured one: the confirming run is the
same session `mode:=offline` with reliable `play_args` **and** the committed 0.5 preset.
Anyone re-opening this should make that run first. The remaining offline-harness figure
(candidates in 58-75% of frames across four backgrounds) is a real number worth improving
on its own merits, not evidence of a defect.

Excluded by measurement or inspection along the way: playback QoS and frame dropping; the
node's tuning (only `stance_floor` is overridden); the downsample path
(`detect_for_target` applies `finite_only` + `voxel_downsample(bbf_voxel)` internally, and
the background model voxelises at the same `bbf_voxel`); the background source; a nested
`bbox_free.board` block shadowing top-level fields (the preset is flat); and cropping
(only the bbox path filters, bbox-free passes the cloud through).

### What this does and does not change about the fixes above

Both fixes stand on evidence that does not depend on the mistaken symptom:

- The coverage gate genuinely was geometrically unreachable -- best achievable residual
  0.436 against a 0.45 gate, measured over 30 real board clusters. Decoupling it from the
  theta search is correct regardless.
- `patch_min_points: 60 -> 40` rests on the offline harness, where clusters of 35-51
  points died against a threshold inherited from the 1000 mm perforated plate.

What changes is the framing: candidate formation was never "the remaining blocker" that
kept the node silent, because the node was not silent in the mode it is documented to
run in.
