# Side Track: Method E — Motion / Background Subtraction

Implements strategy **E** of [`side-track_auto-bounding-box.md`](side-track_auto-bounding-box.md)
as generator `"e"` in the `boarddet` experiment
(`experiments/board-detection-2d/`), and validates it on the real sample
recordings by **leave-one-out cross-dataset background construction**.

Status: 🟢 validated on real data. Recommended operating point reaches
**88.4% recall at 100% precision**, against
[phase-7](phase-7-projection-board-detection.md)'s best prior single-frame
results of 49.3%/93.0% (stage 6) and 44.1%/100% (stage 8).

---

## Motivation

Phase 7 closed after eight stages with recall stuck at ~44–50%, and diagnosed
the cause as fundamental rather than a tuning gap. From its Decision section:

> closing it needs a session-level multi-pose cue (the board moves between
> calibration poses; static clutter does not), which is a capture-protocol
> change for a future phase, **not implementable on today's
> single-static-capture sample datasets**.

The premise of this side track is that the last clause is wrong. The five
sample datasets **already are** five board poses in one shared static room:

- Stage 3's pose-sanity check established `bbox.json5`'s reference box as
  "one physical rig setup shared by all five sample datasets, not
  ds3-specific."
- Static clutter attractors recur at identical scene-fixed coordinates across
  datasets — `~(-1.83,-2.89)` in ds1/3/5, `~(4.7,2.6)`, `~(-3.3,3.4)`.
- But each dataset places the board somewhere different (stage 4 pose sanity):

  | Dataset | 1 | 2 | 3 | 4 | 5 |
  |---|---|---|---|---|---|
  | board centre (x, y, z) | (2.256, −0.059, 0.074) | (2.147, **+0.420**, 0.076) | (2.101, −0.314, 0.074) | (2.077, −0.605, 0.066) | (2.090, −0.829, 0.039) |

Hold out dataset K, accumulate a background from the other four, and K's board
is the one thing that background has never seen.

**This is not a literal single-session multi-pose buffer** (what
`advanced_extrinsic_solver` maintains). These are five independent captures of
one room — a related but distinct instance of the same persistent-vs-transient
occupancy cue. The distinction matters when reading the numbers below: nothing
here demonstrates that a within-session buffer performs the same way.

## Method

`BackgroundModel` (`src/boarddet/background.py`) accumulates voxel occupancy
**per source**, then collapses sources into a consensus background: a voxel
counts as background only once `>= min_sources` distinct sources have seen it.
Query-time dilation (a 3×3×3 neighbour stencil at `dilation_radius=1`) absorbs
voxel-boundary aliasing, so range noise nudging a static point across a cell
edge does not make it read as newly-occupied.

Why per-source consensus rather than a plain union: each board is seen by
exactly one dataset while the room is seen by all of them, so a threshold above
1 drops every contributor's own board out of the background while keeping the
room. Without it the contributors' boards land in the background and suppress
the held-out board wherever the positions overlap — and they do overlap, since
the five board centres span only ~1.25 m in y while a 1 m diamond is 1.414 m
across its diagonal.

`generate_background_diff` (`src/boarddet/candidates/background_diff.py`) diffs
the frame, then reuses generator B's range-scaled anisotropic DBSCAN and the
shared `plausible_board_patch` gate. It runs **no** `_remove_big_planes` stage:
ground and walls are background by construction and are already gone, which
also drops B's most expensive stage. The anisotropic scaling is still required
— a revealed board is sampled through the same VLP-32C rings, so ring-gap
fragmentation survives the diff untouched.

All runs below use the stage-6 operating point (`--stance-gate
--flatness-rms-max 0.045`) so the comparison is like-for-like:

```bash
uv run python -m boarddet.benchmark_e_loo --datasets 1 2 3 4 5 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 --min-sources 3 \
  --isolation --isolation-max-density 0.3 --out results/methodE-ms3-iso
```

## Sensor-pose sanity check

LOO assumes all five captures share one sensor pose, so that a world coordinate
means the same physical spot in every dataset. The harness tests this itself:
`n_known_clutter_survived` counts accepted detections landing on the documented
static attractors, which a background built from four other datasets must
suppress.

**It passed (0 survivors) at every threshold except `min_sources=4`.** Every
number below is conditioned on that; the `min_sources=4` row is reported but
must not be trusted (see the ablation).

## Consensus-threshold ablation

All 535 frames, five folds, stage-6 operating point, isolation off:

| `min_sources` | recall | precision | clutter | known-clutter survived | median ms/frame |
|---|---|---|---|---|---|
| 1 (plain union) | 0/535 = **0.0%** | — | 1 | 0 | 85–103 |
| 2 | 204/535 = 38.1% | 99.51% | 1 | 0 | 80–110 |
| **3** | 473/535 = **88.4%** | 99.79% | 1 | 0 | 90–117 |
| 4 | 493/535 = 92.1% | 99.40% | 3 | **2** ⚠ | 128–136 |

Per fold (held-out dataset), same runs:

| `min_sources` | ds1 | ds2 | ds3 | ds4 | ds5 |
|---|---|---|---|---|---|
| 1 | 0% | 0% | 0% | 0% | 0% |
| 2 | 99.0% | 99.0% | **0%** | **0%** | **0%** |
| 3 | 99.0% | 100% | 99.1% | 99.1% | 42.7% |
| 4 | 99.0% | 100% | 99.1% | 99.1% | 62.1% |

Three findings, none of them tuning noise:

1. **The plain union is not merely worse, it is total failure (0/535).** With
   `min_sources=1` all four contributors' boards enter the background, and
   between them they cover every held-out board position. This is the single
   clearest result here: consensus is not an optimisation of Method E on this
   data, it is what makes Method E work at all.

2. **`min_sources=2` fails exactly where the geometry predicts.** It recovers
   only ds1 and ds2 — the two *outermost* board positions (y = −0.059 and
   +0.420). ds3, ds4 and ds5 are each sandwiched between two neighbours at
   Δy = 0.224–0.291 m, far inside the board's 1.414 m diagonal, so two
   contributors cover the held-out board, it reaches the 2-vote threshold, and
   it is absorbed into the background. Raising the threshold to 3 requires
   three overlapping boards, which no position has, and all three folds
   recover.

3. **`min_sources=4`'s higher recall is not usable.** It buys +3.7 pts of
   recall but its background shrinks to ~6.9k voxels (vs ~23k at threshold 3),
   too thin to suppress the documented static panels — `n_known_clutter_
   survived` goes to 2, meaning the sanity assumption the whole method rests on
   no longer holds. It also costs ~40 ms/frame more, pushing well past budget.
   Rejected.

## Recommended operating point

`--min-sources 3 --stance-gate --flatness-rms-max 0.045 --isolation
--isolation-max-density 0.3`:

| fold | true board | frames | recall | clutter | median ms |
|---|---|---|---|---|---|
| ds1 | 102 | 103 | 99.0% | 0 | 102 |
| ds2 | 103 | 103 | 100.0% | 0 | 108 |
| ds3 | 112 | 113 | 99.1% | 0 | 100 |
| ds4 | 112 | 113 | 99.1% | 0 | 101 |
| ds5 | 44 | 103 | 42.7% | 0 | 119 |
| **total** | **473** | **535** | **88.4%** | **0** | — |

Stage 8's isolation gate removes the single residual false positive at **zero**
recall cost (473 true both with and without it), taking precision 99.79% →
**100%** for ~10 ms/frame. It is worth keeping here, unlike in stage 8 where it
cost 5 points of recall.

Comparison against the phase's prior best, same stance/flatness settings:

| configuration | recall | precision |
|---|---|---|
| stage 6 (`--stance-gate --flatness-rms-max 0.045`) | 49.3% | 93.0% |
| stage 8 (+ isolation 0.3) | 44.1% | 100% |
| **Method E, ms=3 + isolation** | **88.4%** | **100%** |

This is a **strict Pareto improvement over both** — nearly double stage 6's
recall at stage 8's perfect precision. It is the first configuration in this
phase to break the ~44–50% recall ceiling, and it does so via precisely the
mechanism the phase-7 Decision named as the only path to it.

## Caveats

1. **Not a within-session multi-pose buffer.** See Motivation. Five independent
   captures of one room is a related but distinct setting; nothing here shows a
   single-session buffer behaves the same.

2. **ds5 is a genuine partial failure at 42.7%, not noise.** ds5's board sits at
   the extreme edge of the spread (y = −0.829) with ds4 only 0.224 m away, so
   even at threshold 3 much of its footprint is covered. Its fold is also the
   slowest (117–119 ms) and carried the only false positive in the ms=3 run.
   Board-position overlap is mitigated by consensus, not eliminated, and ds5 is
   where that shows.

3. **The result depends on board positions differing between captures.** That is
   true of this data by luck, not by protocol. A real deployment wanting these
   numbers must actually move the board between captures — which is the
   capture-protocol change phase 7 already identified, now with a measured
   payoff attached.

4. **Timing sits at the edge of the 100 ms budget.** Medians are 100–119 ms at
   the recommended point, versus ~60 ms for stage 6. Method E drops B's
   plane-strip stage but pays it back in the diff and in clustering a larger
   surviving foreground. Not disqualifying for offline calibration, but it is
   no longer comfortably inside budget.

5. **The deployment workflow (empty room, then board walked in) remains
   unvalidated.** No sample capture has a board-absent period. `BackgroundModel`
   supports it directly — `observe()` warmup frames, then `finalize()` — but
   nothing here exercises that path, so intra-dataset warmup mode was
   deliberately not built.

6. **A person or mount near a contributor's board** adds background occupancy
   there. No mitigation was needed: the flatness gate crops non-planar
   candidates, verified by a dedicated test.

## Verdict

Adopt for the static-mount scenario. Method E is the first mechanism in phase 7
to move recall past the ceiling stages 5–8 all converged on, and it does it
without trading precision away — 88.4%/100% against stage 6's 49.3%/93.0% and
stage 8's 44.1%/100%.

The load-bearing insight is the **consensus threshold**, not background
subtraction as such: a plain-union background scores 0/535 on this data, and
threshold 2 recovers only the two outermost board positions. Any future use of
this generator must set `min_sources` from the geometry of the capture set
(enough votes that no held-out object is covered by that many contributors),
and must check the background is still dense enough to suppress known static
clutter — the harness's `n_known_clutter_survived` is there for exactly that,
and it is what disqualifies the otherwise-tempting `min_sources=4`.

---

## Second rig: recorded TWO_LIDAR bags (VLP-32C)

The first real-data test of Method E on a **different rig**: four recorded
ROS 2 bags (`ros/lctk_sample_data/bags/TWO_LIDAR_*`, gitignored), each a
~20 s static-board capture from a two-LiDAR rig (VLP-32C + a solid-state
Falcon; the Falcon is a separate write-up). Exported to the npz frame cache
by `tools/export_bag_npz.py`, evaluated through the same
`benchmark_e_loo` harness, classified against the rig's own reference box
(`ros/lctk_launch/config/board/bbox-vlp.json5`), which loads through the
same rotation-aware `bbox_ref` path as the pcap box.

### The capture set is two board positions, not four

Measured, and decisive for how the LOO is run: the four bags hold only **two
distinct board positions** — `{TWO_LIDAR_1, TWO_LIDAR_2}` at one location
(call it **A**), `{TWO_LIDAR_3, TWO_LIDAR_4}` at another (**B**). A naive
4-fold LOO is therefore **confounded**: holding out bag 1 leaves its
same-position twin (bag 2) in the background, so the held-out board is
partly self-suppressed. That confound is visible in the numbers — a naive
4-bag sweep gave 0 % at `min_sources=1`, and a weak, asymmetric 5–16 % at
`min_sources=2/3`.

The correct experiment merges each pair into one source and runs a clean
**2-fold LOO at `min_sources=1`**: hold out A, build the background from B
(which has no board at A), and vice versa. No twin contamination, one clean
contributor per fold. `tools/bag_motion_probe.py` is the gate check — it
confirms foreground survives (the board is not in the same place across all
four), so the premise holds.

### The board is at ~9 m — 4× the pcap range — and that changes the tuning

The bag board sits at ~9–10 m (bbox centre `[9.2, 1.1, −0.5]` in the
velodyne frame) versus ~2 m for the pcap board. At the stage-6 operating
point *with the pcap's default `vertical_gap_deg=3.0`*, recall was only
**3.3 %** — even though the board is plainly present. The 6-panel overlay
(`--save-overlays`, added for exactly this diagnosis) showed why: the board
*is* a clean hollow diamond (all three holes visible in the plane raster),
but its candidate cluster absorbed a tail of ground points 2.5 m below,
inflating the fitted quad to 1.2 × 1.9 m and failing the size gate.

The cause is range-dependent and specific: the anisotropic clustering
z-compresses by `2·r·tan(gap_deg)`, which at r≈9 m and 3° is ~0.9 m — enough
to merge the ground into the board. Retuning `--vertical-gap-deg 1.0`
(~0.3 m tolerance at 9 m — still bridges the board's own ring gaps, no longer
reaches the ground) is the fix:

| `vertical_gap_deg` | recall (630/795 frames) |
|---|---|
| 3.0 (pcap default) | 3.3 % |
| 0 (isotropic) | 0 % — ring gaps at 9 m fragment the board |
| **1.0** | **79.2 %** |

### Operating point and results

`--stance-gate --flatness-rms-max 0.045 --vertical-gap-deg 1.0
--min-sources 1`, merged 2-fold LOO, all frames:

| fold (position) | true board | frames | recall | clutter |
|---|---|---|---|---|
| A = bags 1+2 | 397 | 397 | **100 %** | 0 |
| B = bags 3+4 | 233 | 398 | 58.5 % | 0 |
| **total** | **630** | **795** | **79.2 %** | **0 → 100 % precision** |

Verified visually: the accepted quad traces the hollow-diamond board (holes
and all) at score ~0.88, clear of the ground.

#### Follow-up (2026-07-25): `cluster_min_points` for the sparse far board

Overlay review of the clean `TWO_LIDAR_1`/`TWO_LIDAR_3` pair showed the fitted
quad *truncating* the board — spanning only its dense middle band, not the
full diamond. Root cause: at 9 m the board's corner/edge points fall below
generator E's hardcoded `cluster_min_points=30` DBSCAN density and are dropped
as **noise** (measured: 40 % of the foreground labelled noise, cluster split
in two), so the surviving cluster's `minAreaRect` is short. This is now a
config/CLI knob (`--cluster-min-points`, `BoardConfig.cluster_min_points`).
Lowering it to 20 keeps the sparse corners:

| `cluster_min_points` | recall (`TWO_LIDAR_1`/`3`, 398 frames) | clutter |
|---|---|---|
| 30 (old default) | 79.9 % | 0 |
| **20** | **91.2 %** (100 % / 82.4 %) | 0 → 100 % precision |
| 15 | 88.9 % | 0 |

`20` is the operating point; precision stays 100 %. Recommended VLP bag command
gains `--cluster-min-points 20`.

### What does not transfer from the pcap rig

- **`n_known_clutter_survived` is meaningless here.** Its coordinates are the
  *pcap* rig's static attractors; on this rig it is not a valid sanity
  check, and is reported as 0 only because those specific points are empty
  here. The genuine precision evidence is the bbox classification: 0 clutter
  across 795 frames.
- **Isolation *hurts* at this range.** Adding `--isolation
  --isolation-max-density 0.3` collapses recall to 50.9 % (position B falls
  233 → 8): at 9 m the board's backing structure trips the exterior-band
  coplanar test that, on the pcap rig, cleanly separated a free-standing
  board from a wall panel. It is correctly **off** for this rig.
- **Timing is ~200 ms/frame, 2× the 100 ms budget** — the noisier
  long-range foreground leaves more points to cluster than the pcap clouds.
  Not disqualifying for offline calibration, but no longer inside budget.

### Honest read

- **Method E transfers to a second rig and a 4×-farther board — 79.2 % /
  100 % — but only after a range-appropriate `vertical_gap_deg`.** The
  anisotropic tolerance is not rig-independent; a board-distance-aware
  `vertical_gap_deg` (or an explicit per-rig setting) is the clean
  follow-on, and the fixed default carried over from the near-board pcap
  case is the wrong value here.
- **Position B (58.5 %) is a real partial failure, not noise** — B is the
  harder of the two placements (farther / more grazing), and its fold is the
  one that loses frames. A is a clean 100 %.
- **Only two board positions** means this is a weaker cross-position test
  than the pcap's five; it is one honest clean-LOO pair, not five.

## Solid-state: the Falcon topic

The same bags carry a second sensor — an Innovusion/Seyond Falcon
(`/lidar/falcon/iv_points`, solid-state, ~92 k points/frame, no ring
structure) — classified against its own reference box
(`bbox-seyond.json5`, board at ~7.4 m in the seyond frame). This is the
**first test of the projection pipeline on a real solid-state LiDAR**;
phase 7 had only synthetic uniform-sampling evidence for that case, with its
own caveat that "real spinning-LiDAR data is the hard case here, not the easy
one." The same physical board, scene, and capture as the VLP-32C run above —
only the sensor differs — so this is the cleanest sensor-to-sensor
comparison in the phase.

### Anisotropic clustering must be OFF for a ring-less sensor

The mirror image of the VLP-32C finding. `vertical_gap_deg` exists to bridge
a spinning LiDAR's ring gaps; the Falcon has none, and applying the
z-compression to its dense uniform cloud **destroys** detection:

| `vertical_gap_deg` | recall |
|---|---|
| **0 (isotropic)** | **92.9 %** |
| 1.0 | 0 % — z-compression corrupts the dense cloud |

### Results

`--stance-gate --flatness-rms-max 0.045 --vertical-gap-deg 0
--min-sources 1`, merged 2-fold LOO, all frames:

| fold (position) | true board | frames | recall | clutter |
|---|---|---|---|---|
| A = bags 1+2 | 391 | 397 | 98.5 % | 0 |
| B = bags 3+4 | 347 | 397 | 87.4 % | 0 |
| **total** | **738** | **794** | **92.9 %** | **0 → 100 % precision** |

The plane raster of an accepted Falcon detection is a dense, crisp hollow
diamond — all three holes cleanly resolved, far sharper than the sparse
VLP-32C board at the same range.

### Read

- **The projection + 2D-scorer pipeline works on a real solid-state LiDAR —
  92.9 % / 100 %.** Phase 7's synthetic solid-state claim holds on real
  data, and its worry that spinning data was "the easy case" is inverted
  here: the Falcon's dense uniform sampling has no ring-gap fragmentation,
  so it is the *easier* sensor. On the identical board and scene it beats the
  VLP-32C (92.9 % vs 79.2 %), and both positions clear 87 % where the VLP's
  position B stalled at 58.5 %.
- **The one knob that flips between sensors is `vertical_gap_deg`** — tuned
  down for the VLP's far board, off entirely for the Falcon. Everything else
  (stance, flatness, `min_sources`, isolation-off) is shared. A
  sensor-aware default for that one parameter is the obvious follow-on.
- **Timing is ~270 ms/frame** — the highest here, from the 67 k-point
  downsampled Falcon cloud. Comfortably usable offline, well over the 100 ms
  real-time budget.
- Same two-position caveat as the VLP-32C run: a clean-LOO pair, not five.

#### Follow-up (2026-07-25): fixed-square fitter + per-rig gravity axis

Overlay review found frames where the orange near-miss quad traced the board
perfectly yet the verdict was negative. Two coupled root causes, now fixed:

1. **The score, not gravity, was rejecting it.** With `square_icp` off, the
   side length comes from `cv2.minAreaRect` — the minimum *enclosing*
   rectangle, which over-sizes on a dense, hole-punched, foreshortened Falcon
   board (measured mean side 1.18 m vs 1.0 m). The score's size penalty
   `exp(−2·|mean−1|)` then pushed it to **0.499**, just under `min_score=0.5`.
   Enabling the fixed-side square fitter (`--square-icp`) pins the side to
   `side_m` and spends its DOF on pose, taking the score to ~1.0.
2. **The gravity axis was hardcoded to world +z.** `_stance`/`_up_2d` assumed
   a REP-103 z-up frame, but the Falcon frame is **z-forward** (board at
   z ≈ 7.4 m; gravity ≈ ∓y). Turning on `square_icp` re-activates the stance
   gate, which — still reading up = +z — then rejected *every* upright board
   (recall 0/397). The up axis is now a per-rig config/CLI value
   (`--up-axis`, `BoardConfig.up_axis`); `0 1 0` is correct here.

| Falcon config (`TWO_LIDAR_1`/`3`) | recall | precision |
|---|---|---|
| baseline (`square_icp` off, stance off) | 92.7 % | 100 % |
| `--square-icp` (stance off) | 100 % | 100 % |
| `--square-icp`, stance on, `--up-axis 0 0 1` (wrong) | **0 %** | — |
| **`--square-icp`, stance on, `--up-axis 0 1 0`** | **100 %** | 100 % |

This retires the "sensor-aware default" follow-on above for gravity: the one
knob that flips between the z-up rigs and the z-forward Falcon is `up_axis`,
now explicit. Recommended Falcon bag command gains `--square-icp --up-axis 0 1 0`.

---

## Appendix: `benchmark_e_loo` CLI reference

Every result above was produced by `boarddet.benchmark_e_loo`, but the
argument set is only shown by example. Full reference, grouped by role and
wired to the code that consumes each flag (`src/boarddet/benchmark_e_loo.py`
unless noted).

### Data selection

| flag | default | effect |
|---|---|---|
| `--source {pcap,bag}` | `pcap` | Selects the frame reader. `pcap` = sample datasets 1–5 (`ingest.load_frames`); `bag` = exported TWO_LIDAR bags (`ingest.load_bag_frames`, which requires `tools/export_bag_npz.py` to have been run first). |
| `--names N …` | `1 2 3 4 5` (pcap) / `TWO_LIDAR_1 … 4` (bag) | The captures to use. Each name is simultaneously one held-out LOO fold *and* one background-contributing source in the other folds. |
| `--sensor {vlp32,falcon}` | `vlp32` | **Bag sources only** (ignored for pcap). Chooses which sensor's exported cache to read. `falcon` is solid-state (no rings) — pair it with `--vertical-gap-deg 0`. |
| `--max-frames N` | all | Truncates each capture to its first `N` frames. Smoke-test knob only. |
| `--out DIR` | **required** | Destination for `loo_summary.json` and any overlays. |

### Background model (`background.BackgroundModel`)

| flag | default | effect |
|---|---|---|
| `--background-voxel M` | `0.06` | Occupancy cell size in metres. Smaller = stricter diff but more voxel-boundary aliasing; larger = coarser and risks absorbing the board. |
| `--dilation-radius R` | `1` | Query-time neighbour stencil, `(2R+1)³` cells, that absorbs voxel-edge aliasing from range noise. `0` disables it (reproduces the aliasing bug). |
| `--min-sources K` | `2` | **The load-bearing knob.** A voxel is background only once `≥ K` distinct sources have seen it; `1` is a plain union (scores 0/535 on the pcap set). Set it from the capture geometry — enough votes that no held-out board is covered by that many contributors. A fold with fewer than `K` contributors is a hard error (would finalize an empty background). |

### Clustering / detector tuning

| flag | default | effect |
|---|---|---|
| `--vertical-gap-deg D` | `3.0` | Anisotropic DBSCAN z-compression, `2·r·tan(D)`, that bridges spinning-LiDAR ring gaps. `3.0` suits the near VLP board; `1.0` for the far (9 m) VLP board (else it merges the ground); `0` for the Falcon (ringless — z-compression corrupts a dense cloud). Also gates whether `up_2d`/`close_height_m` are computed at all (`detector.py`). |
| `--cluster-min-points N` | `30` | Generator E's foreground-DBSCAN core-point density (min neighbours within eps). `30` suits the near pcap board; the far (9 m) VLP board is sampled so sparsely that its corner points fall below it and drop out as noise, truncating the fitted quad — `20` keeps them (VLP bag recall 79.9% → 91.2%, precision unchanged). |
| `--side M` | `1.0` | Physical board side length in metres; feeds every size/extent gate (and the pinned side of `--square-icp`). |

### Acceptance gates

| flag | default | effect |
|---|---|---|
| `--stance-gate` | off | Sets `stance_floor = 0.9` — reject a quad whose best diamond diagonal is more than ~25° off vertical. **Only actually fires when `--square-icp` is on or `vertical_gap_deg > 0`** (the scorer's stance gate needs an in-plane up direction); it is inert on a bare `--vertical-gap-deg 0` run. Reads world-up from `--up-axis`. |
| `--square-icp` | off | Refine each candidate with the fixed-side square fitter (`square_fit.fit_fixed_square`): pins the side to `--side` and spends its DOF on pose. Fixes the `minAreaRect` oversize that sinks a dense board's score below `min_score` (Falcon bag recall 92.7% → 100%). It **re-activates the stance gate**, so on a non-z-up rig it must be paired with the correct `--up-axis`. |
| `--up-axis X Y Z` | `0 0 1` | World-up direction in the sensor frame for the stance gate. `0 0 1` for a REP-103 z-up rig (pcap, VLP bag); `0 1 0` for the z-forward Falcon frame (board at z ≈ 7.4 m). With the wrong axis the stance gate reads every upright board as lying flat and rejects it (Falcon recall → 0/397). |
| `--flatness-rms-max M` | `0.035` | Plane-fit RMS ceiling in `plausible_board_patch` (`candidates/__init__.py`). Above the VLP-32C noise floor. Stage 6 adopted `0.045`. |
| `--isolation` | off | Free-standing gate: rejects a candidate whose plane has coplanar points continuing past the fitted quad (an embedded panel/wall rather than a free board). Helps the pcap rig; *hurts* the far bags (the board's own backing structure trips it), so it is off for both bag rigs. |
| `--isolation-max-density X` | `0.3` | Threshold for the above (points per metre of quad perimeter). Only consulted when `--isolation` is set. |

### Output

| flag | default | effect |
|---|---|---|
| `--save-overlays N` | `0` | Render up to `N` six-panel Method-E overlays per fold. The frames chosen are the first detection, the **highest-scoring rejection**, and an even spread (`_pick_overlay_indices`) — which is why a clean-looking orange near-miss (a `best_rejected` quad) is *always* among them, by design. |
| `--bbox PATH` | pcap `bbox.json5` | Reference box used for precision scoring (`box.contains(center)`). **Per-rig** — the VLP and Falcon runs each pass their own (`bbox-vlp.json5`, `bbox-seyond.json5`). |

### Not exposed as flags

These are fixed in `BoardConfig`: `min_score = 0.5` (soft acceptance
threshold), `stance_weight = 0.0`, `strict_squareness = False`,
`edge_support_min = 0.0`, `cell_m = 0.02`, `side_tol = 0.20`. Changing them
currently means editing the `BoardConfig` construction in
`benchmark_e_loo.main()`.
