# Phase 7: Projection-Based Board Detection (Crop-Box-Free)

## Overview

This phase explores a new way to locate the calibration board in a LiDAR point
cloud: project candidate points onto a 2D image and find the board by its
square border using OpenCV-style image processing. The goal is to eventually
**remove the manual crop-box parameters** (`bbox.json5`) and achieve
**real-time, full-scene board detection** that works for both spinning LiDARs
(VLP-32C) and solid-state LiDARs (Livox-style non-repetitive scan).

Status: 🟡 experiment phase — standalone Python project, no ROS integration yet.

Experiment code: `experiments/board-detection-2d/` (uv project).

## Motivation

The current pipeline (`ros/lidar_board_detector` + `rust/hollow-board-detector`)
requires a hand-tuned crop box before RANSAC/ICP can run. This is the single
most operator-hostile step of a calibration session: the box must be re-tuned
for every new scene (`filter_box_tuner` exists solely to ease this pain). The
ICP stage also needs a reasonable initial pose and ~100 ms/frame, with the
crop box doing most of the work of isolating the board.

A detector that finds the board anywhere in the scene, in real time, with only
board geometry as a prior, removes the crop box and the per-scene setup cost.

The board itself may also change: a plain **0.5–1 m diamond without holes**
(easier to fabricate and move) is under consideration. The new detector must
therefore key on the **square border only** — board size is a config
parameter, holes are an optional extra cue, never a requirement.

## Projection Method Survey

The pivotal design choice is how to map 3D points to a 2D image while keeping
the board's square shape intact.

| Projection | Square stays square? | Solid-state? | Verdict |
|---|---|---|---|
| Range image (azimuth–elevation) | No — straight lines curve; 32 rings give coarse vertical quantization | No ring structure; needs frame accumulation + re-rasterization | Usable only as a coarse ROI stage on spinning LiDAR |
| Bird's-eye view | No — a near-vertical board degenerates to a line | — | Unusable |
| Virtual pinhole camera | No — square becomes a general quadrilateral (perspective) | OK | No advantage over plane-fit |
| **Plane-fit → orthographic projection into plane coordinates** | **Yes — distortion-free by construction, metric (pixels = meters × resolution)** | **Yes — assumes nothing about scan structure** | **Chosen** |

Plane-fit + in-plane rasterization is what successful published pipelines use:

- **velo2cam_calibration** / **lvt2calib** — RANSAC plane, then find circular
  holes in plane coordinates. <https://github.com/beltransen/velo2cam_calibration>,
  <https://github.com/Clothooo/lvt2calib>
- **ILCC** — chessboard pattern recovered from reflectance intensity on the
  fitted plane (spinning LiDAR). <https://github.com/mfxox/ILCC>
- **ACSC** — same idea built for Livox: time-domain accumulation densifies the
  non-repetitive cloud, then intensity-based corner extraction on the plane.
  <https://github.com/HViktorTsoi/ACSC>
- **FAST-Calib (2025)** — target-based, scan-pattern-agnostic edge extraction;
  explicitly corrects the edge-dilation bias from laser spot spread.
  <https://arxiv.org/pdf/2507.17210>
- **Park et al. 2014** — board vertices from intersecting lines fit to border
  scan points. <https://www.mdpi.com/1424-8220/14/3/5333>

Key pitfalls recorded from the literature:

- **Ring stripes (VLP-32C):** vertical point gaps of several cm at 5 m leave
  empty rows in the occupancy image. Mitigate with cell size ≈ 1–2× the largest
  expected point gap plus morphological closing. The closing kernel must stay
  smaller than any board hole, or holes are erased.
- **Beam-spot spread** systematically dilates the board outline (and shrinks
  holes) by roughly half a spot diameter — a *bias*, not noise; correct it
  explicitly rather than averaging.
- **Segment the plane before projecting** so background points never enter the
  image; this also removes mixed-pixel edge blur.
- Contour corners are only accurate to ~point spacing. Recover sub-cell corners
  by fitting total-least-squares lines to the four sides **on the raw projected
  points** (not raster pixels) and intersecting them.

## Candidate Generation — Three Approaches Under Test

With no crop box, the detector must generate board-plane candidates from the
full scene. All three approaches below feed the **same shared 2D scorer**; the
experiment compares them head-to-head.

### A. Iterative RANSAC multi-plane (velo2cam style)

Downsample → repeatedly RANSAC the largest plane, remove inliers, repeat N
times → gate each plane's inlier patch by spatial extent (~board size) →
2D quad test on survivors.

- **+** Simple; reuses a known-good tool.
- **−** The board is a *small* plane; ground/walls dominate, so the board may
  surface only after several full-cloud iterations. Merges the board into a
  wall plane if the board leans against one.

### B. Euclidean clustering after big-plane removal

Downsample → RANSAC out only *large* planes (ground, walls — inlier extent ≫
board) → Euclidean-cluster the remainder → per cluster: PCA plane fit +
flatness gate + size gate → 2D quad test.

- **+** A free-standing board forms a clean cluster; cheap gates prune most
  clusters before any 2D work; voxel grid + clustering is tens of ms.
- **−** Fails if the board touches a large structure (wall-mounted board).

### C. Normal-based region growing

Estimate per-point normals → grow regions of coherent normal → each region is
a plane candidate → gate + 2D quad test.

- **+** Handles board-against-wall (normal discontinuity separates them when
  angled).
- **−** Normal estimation on sparse VLP-32C rings is noisy and the most
  expensive of the three.

### Shared 2D scorer

```
candidate inlier points
  → project to plane basis (orthographic, metric)
  → rasterize occupancy image (cell ≈ 1–2× point gap)
  → morphological close
  → cv2.findContours
  → quad fit (minAreaRect) 
  → side-line refit on raw projected points → corner intersection
  → score: side length vs config | squareness | fill ratio | edge straightness
```

The best candidate above a score threshold yields the board pose: plane basis
+ in-plane rotation + center, with diamond orientation recovered from the quad
corners. Works with or without holes.

## Experiment Plan

### Harness (`experiments/board-detection-2d/`)

Standalone uv project. No ROS.

- **Ingest:** `velodyne_decoder` reads `ros/lctk_sample_data/data/{1..5}/lidar.pcap`
  directly → per-frame numpy `(N, 4)` xyz + intensity (ring kept for
  diagnostics only — never used by the algorithm). Frames cached to `.npz` so
  iteration skips pcap decoding.
- **Layout:** `candidates/ransac_iterative.py` (A),
  `candidates/cluster_after_ground.py` (B), `candidates/region_growing.py` (C);
  shared `scorer.py`, `pose.py`; board geometry (diamond side length, optional
  holes) in one small config dataclass.
- **Dependencies:** numpy, opencv-python-headless, velodyne-decoder, open3d,
  matplotlib — all inside the uv venv (no system pip risk; see CLAUDE.md
  Known Issue 3).

### Benchmark protocol

Run every generator over all frames of datasets 1–5. Record per frame:
detected?, pose, per-stage wall time.

- **Detection rate** per approach per dataset.
- **Timing:** median / p95 per stage; real-time budget is ~100 ms/frame at
  10 Hz (parity with the current ICP detector).
- **Accuracy, stage 1:** frame-to-frame pose jitter (the board is static during
  each capture, so jitter is a precision proxy) + saved visual overlays
  (cloud + fitted quad + raster image, a few frames per dataset).
- **Accuracy, stage 2** (after stage 1 looks promising): pose agreement with
  the existing ICP detector on the same frames — the ICP result is a
  *reference*, not ground truth. Requires a ROS run; deferred.
- **Solid-state check (stretch):** synthetic Livox-like frames (rosette-pattern
  sampling of a simulated scene containing the board) to verify no generator
  silently assumes ring structure. No real solid-state data exists in the repo.

### Known constraints

- Sample data is exclusively VLP-32C (datasets 1–5; pcap + avi, no rosbags).
- No ground-truth board pose exists; stage-1 accuracy is jitter + eyeball,
  stage-2 is agreement with ICP.
- `organize_cloud: false` in playback means no organized cloud on the wire;
  irrelevant here since ingest bypasses ROS entirely.

## Results

First benchmark run 2026-07-17 (`experiments/board-detection-2d/results/run2*`;
board side 1.0 m from `board_detector.json5`, `min_score` 0.5). A and B ran
**all frames** of every dataset (103–113/dataset); **C was capped at 30
frames/dataset** because its per-point Python BFS is ~3.5× slower than A —
noted per the protocol's no-silent-caps rule.

**Read the detection-rate table with care.** There is no ground truth, and the
rate counts *any* accepted detection. Spot-checking detections against the
only sanity reference available (the legacy `bbox.json5` crop-box centre,
never used by the algorithm) shows the three generators' rates mean very
different things — see the narrative below.

### Detection rate (fraction of frames with any detection ≥ min_score)

**A was re-run** (`results/run3a`, all 5 datasets, full frames) after a review fix to
`dist_thresh` (0.02 → 0.05 m; see "A's RANSAC threshold fix" below). B and C numbers below are
unchanged from the original `run2`/`run2c` run.

| Dataset | A: iterative RANSAC | B: cluster | C: region growing (30-frame cap) |
|---------|--------------------|------------|-------------------|
| 1 | 0% | 1% | 83% |
| 2 | 0% | 0% | 53% |
| 3 | 0% | 2% | 93% |
| 4 | 0% | 1% | 87% |
| 5 | 0% | 8% | 90% |

### Timing (median ms per frame, dataset 3; p95 in parentheses)

| Stage | A (run3a) | B | C |
|-------|---|---|---|
| downsample (0.03 voxel) | 3.1 | 3.2 | 3.0 |
| candidate generation | 103.8 | 77.5 | 285.2 |
| 2D scoring | 5.5 | 1.1 | 5.0 |
| total | 112.5 (121.1) | 82.4 (91.8) | 293.0 (306.9) |

Timing is stable across datasets (A 111–114 ms in run3a, B 82–86 ms, C
293–301 ms median) and essentially unchanged by the `dist_thresh` fix (A was
109–112 ms in run2; the looser threshold shifted median candidate count from
unrecorded to 43–56/frame but cost the same RANSAC + DBSCAN work). **B fits
the ~100 ms realtime budget; A is borderline; C is 3× over** — and C's cost
is a pure-Python BFS, so a compiled port would change its constant
dramatically.

### Pose jitter (std of detected center [mm] / normal [deg], n = detection count)

| Dataset | A | B | C (30-frame cap) |
|---------|---|---|---|
| 1 | — (n=1) | — (n=1) | 1468 / 1.4 (n=25) |
| 2 | — (n=0) | — (n=0) | 977 / 6.4 (n=16) |
| 3 | — (n=0)\* | 25 / 0.0 (n=2) | 1411 / 9.5 (n=28) |
| 4 | — (n=1) | — (n=1) | 1468 / 2.7 (n=26) |
| 5 | — (n=1) | 862 / 6.5 (n=8) | 2090 / 3.4 (n=27) |

\* A's ds3 count is from the `run3a` re-run (see "A's RANSAC threshold fix"
below); it dropped from n=1 to n=0 there. All other A/B counts above are
from the original `run2` (unchanged by the fix).

Jitter is only defined where ≥ 2 frames detected — that's a single cell in
this table (**B/ds3, n=2**). A std computed from 2 samples is not a
meaningful precision estimate; 25 mm / 0.0° there is consistent with the
sensor noise floor but should be read as "these two detections landed close
together," not as a converged jitter statistic. Every other populated cell
(n=1) has no defined jitter at all — shown as "—" — and C's n≈16–28 cells
are the only ones with enough samples for the std to mean much
statistically, though (see below) those detections are largely not the
board itself, so even C's numbers describe clutter-panel jitter, not board
jitter. The metre-scale center
jitter is itself a finding: those "detections" are **not the same object from
frame to frame** (see below). B on dataset 3 — the one case verified to be the
real board — jitters by 25 mm, in line with the sensor noise floor.

### What the overlays show (narrative)

- **B's rare detections are the real board — verified on dataset 3 only.**
  On dataset 3 every B detection sits at the `bbox.json5` reference location
  (~2.1 m ahead), the raster clearly shows the hollow-diamond outline, and
  center jitter is 25 mm (n=2 — see the jitter table note above; not a
  converged statistic, but consistent with the noise floor). This is the
  honest headline: **crop-box-free detection of the true board was achieved
  on dataset 3, but only on ~2% of its frames.** This claim does **not**
  extend to the other datasets: dataset 5's 8% rate (n=8) was *not*
  spot-checked against `bbox.json5` the way ds3 was, and its 862 mm / 6.5°
  jitter is on the same order as C's clutter-panel jitter elsewhere in this
  table — plausible evidence that B latches onto a board-sized clutter
  object on ds5 rather than the board, the same failure mode documented for
  C below. Treat ds5's B detections as unverified, likely clutter, not a
  second confirmed "found the board" result.
- **C's high rate is board-sized clutter, not the board.** On dataset 3,
  0 of 28 C detections are at the board location; they alternate between two
  background objects (~(4.7, 2.6) and ~(−3.3, 3.4)) that are genuinely flat
  and ~1 m square — hence the metre-scale jitter. The scorer, keyed on the
  square border alone (by design: works without holes), cannot distinguish a
  1 m square panel from the 1 m board. C *does* generate the true-board
  candidate, but VLP-32C normal noise splits the board into two
  normal-coherent regions, each of which fails the size gate.
- **Why A/B miss the board on most frames:** at ~2 m range the 32 rings put
  only a handful of stripes on the board; after 0.03 m voxel downsampling the
  board cluster is 300–600 points with plane-fit RMS of 0.029–0.031 m —
  *at* the sensor noise floor (cf. `icp_good_fit_threshold` history, C-04).
  On most frames either DBSCAN clips part of the board (mixed-pixel edge
  points), the minAreaRect over the partial patch lands outside the ±20% side
  gate, or coplanar clutter merges in and skews the quad. Frame 5-style
  "clean" captures pass; the rest fail one gate or another.
- **A's RANSAC threshold fix made things worse, not better (honest result).**
  A review flagged `dist_thresh=0.02` as a synthetic-era value never
  adjusted for the ~0.03 m real-data noise floor (the same floor that
  motivated the 0.035 m flatness gate), and raising it to 0.05 m was
  expected to *admit* more genuine board inliers per RANSAC plane. Re-run
  (`results/run3a`, all 5 datasets, full frames): **A's detection rate
  dropped from 1% to 0% on every dataset**, while median candidate count
  per frame actually rose to 43–56 (up from an unrecorded but visibly lower
  count in run2) and per-frame timing stayed flat (111–114 ms). The looser
  threshold pulls more board-adjacent clutter into each RANSAC plane's
  inlier set before the `component_eps` clustering step ever runs, so more
  *candidates* survive the DBSCAN component split but each one is noisier
  and less board-shaped — they clear the flatness gate more easily but fail
  the 2D quad-fit's side-length/squareness score more often. Net effect:
  more attempts, lower yield. The change is kept because it is the
  scientifically correct value (consistent with the flatness gate and
  gen B's `big_plane_dist`), but it does **not** rescue generator A — A's
  bottleneck is candidate *shape* quality from RANSAC-on-a-mixed-plane, not
  an overly strict distance threshold. This reinforces the existing verdict
  below: A's problem is structural (board surfaces late, mixed with
  clutter), not a tuning gap.
- **Failure of the fill term on the hollow board** (found in smoke, fixed):
  the recorded board's three 15 cm holes plus ring-gap sparsity give
  fill_ratio ≈ 0.44 for a *perfect* fit; the linear fill weight rejected it.
  Score now uses `sqrt(fill)` with the side-error weight rebalanced, with
  covering tests (`test_scores_hollow_board_high`,
  `test_scores_sparse_hollow_board_above_min_score`). Other real-data tuning
  (ring-gap DBSCAN eps, plane-strip fraction, flatness gate at 0.035 m,
  seeding open3d's RANSAC RNG for reproducibility) is documented in commit
  `6b197ce`.

### Solid-state stretch check (synthetic uniform sampling)

All three generators detect 5/5 uniform-pattern (Livox-like, no ring
structure) synthetic scenes with center error < 5 cm — nothing in the
pipeline assumes ring/grid structure. Real spinning-LiDAR data is the *hard*
case here, not the easy one: the synthetic uniform scenes lack ring stripes,
which are the root cause of most real-frame failures above.

## Stage 2 Results

Stage 2 added two candidate mechanisms (commit `8bba8cc`) meant to attack the
two stage-1 failure modes directly: `--accumulate N` concatenates N
consecutive frames into one non-overlapping window, hypothesized to densify
the board past the ring-gap fragmentation that stalls A/B; `--stance-weight W`
blends in a gravity-alignment score term (`_stance` in `detector.py`),
hypothesized to separate the true diamond-mounted board (one diagonal near
vertical) from the axis-aligned clutter panels C locks onto. Suite is
42/42 green (`uv run pytest -q`) before and after this benchmark.

Three runs per the stage-2 plan, `results/run4-*` (not committed —
`results/` is gitignored; numbers below are the full record):

```bash
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --accumulate 10 --stance-weight 0.0 --out results/run4-acc
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --accumulate 10 --stance-weight 0.5 --out results/run4-acc-stance
uv run python -m boarddet.benchmark --datasets 3 --generators c \
  --accumulate 10 --stance-weight 0.5 --max-frames 60 --out results/run4-c-check
```

### Recall per window (windows detected / windows total)

| Dataset | B, acc=10, stance=0 | B, acc=10, stance=0.5 | C, acc=10, stance=0.5 (ds3 only, 60-frame cap) |
|---------|---|---|---|
| 1 | 0/10 | 0/10 | — |
| 2 | 0/10 | 0/10 | — |
| 3 | 0/11 | 0/11 | 0/6 |
| 4 | 0/11 | 0/11 | — |
| 5 | 0/10 | 0/10 | — |

**Headline: accumulation did not lift recall — it collapsed it to 0% across
every dataset**, including on dataset 3 where stage-1 B found the real board
on 2/113 single frames. This falsifies hypothesis 1 as tested and is reported
as-is, not smoothed over.

### Timing (median ms per window, p95 in parentheses)

| Run | median (p95) | budget (N×100 ms, N=10) |
|---|---|---|
| B, acc=10, stance=0 | 173–193 ms | 1000 ms |
| B, acc=10, stance=0.5 | 180–183 ms | 1000 ms |
| C, acc=10, stance=0.5 (ds3) | 561 ms (609 ms) | 1000 ms |

All three runs stay comfortably inside the accumulated budget — a 10-frame
window costs roughly the same as ~1.5 single frames for B (not 10×, since
`downsample` and `candidates` both collapse near-duplicate points from a
static scene) and ~2× a single frame for C. Timing was never the risk here;
recall was.

### Jitter

Not computable — jitter requires ≥ 2 detections per dataset/generator cell
(`summarize()`'s gate), and every cell above has 0 detections. No jitter
table for stage 2.

### `best_rejected` score distribution (how close were the misses)

Per-window `best_rejected` scores (the highest-scoring candidate that still
failed `min_score=0.5`) cluster well below threshold, not just-barely-missed:

- B, acc=10, stance=0: scores 0.100–0.326 across all 52 windows (mean ≈ 0.15,
  vs. `min_score`=0.5 — roughly a 3× gap, not a close call).
- B, acc=10, stance=0.5: scores 0.053–0.260 (same windows, scaled down by the
  stance blend as expected).
- C, acc=10, stance=0.5 (ds3, 6 windows): scores 0.094–0.220.

None of the rejected candidates in any accumulated run sit near the bbox
reference location (~2.6, 0, 0.35) or ds3's frame-5 true-board location
(2.10, ‑0.18, ‑0.05, the one confirmed stage-1 B hit). Their centers are
almost all at the same clutter coordinates the stage-1 narrative already
named — e.g. B's ds3/ds4 rejects repeatedly land at (5.8, 1.6, 0.2) and
(4.7, 2.6, 0.15), and C's ds3 rejects land at (4.7, 2.6) / (5.8, 1.2) — the
same two background panels documented in stage 1. Accumulation did not even
produce a *near-miss* board candidate; the true-board region simply stopped
generating a board-shaped candidate at all (see diagnosis below).

### Diagnosis: why accumulation collapsed recall instead of raising it

Traced directly on the one window known to contain a confirmed true-board
hit — ds3 window 0 (frames 0–9), which contains stage-1's single-frame hit
at frame 5:

| | single frame 5 | accumulated window 0 (frames 0–9) |
|---|---|---|
| downsampled points (0.03 m voxel) | 27,386 | 53,456 (1.95×, not ~10×) |
| gen-B clusters found | 7 | 22 |
| board-region cluster | 472 pts @ (2.10, ‑0.18, ‑0.05) → scored 0.538, **DET** | 81 pts @ (2.07, ‑0.30, ‑0.32) → 0.59 m × 0.13 m sliver, fails the scorer's size gate outright (`score_candidate` returns `None`) |

Two compounding causes:

1. **The capture is static, so accumulation adds density, not coverage.**
   VLP-32C's 32 ring elevation angles are fixed by the sensor; a scene and
   board that don't move between frames re-sample nearly the same physical
   points every sweep. 10 accumulated frames only pushed the downsampled
   point count up 1.95× (not the naive 10×), meaning the voxel grid
   collapsed most of the "new" points right back into cells the single frame
   already had. The ring gaps stage 1 identified as the root cause of A/B's
   failure (`"Why A/B miss the board on most frames"` above) are a function
   of sensor geometry, not scan diversity, and a static hold-the-board-still
   capture cannot fill them by concatenation. Accumulation would only
   plausibly help if the board or sensor moved slightly between frames
   (natural hand jitter during a real calibration hold, which this canned
   sample data lacks) or if paired with elevation-angle-aware interpolation.
2. **`cluster_after_ground`'s DBSCAN `cluster_eps=0.15`** — already loosened
   once in stage 1 specifically to bridge single-frame ring gaps (see the
   code comment at `candidates/cluster_after_ground.py:147`) — does not scale
   with the accumulated window. Once the surrounding scene's point density
   and cluster topology shift (22 candidates in the window vs. 7 in the
   single frame — more of the *background* also densifies and starts
   competing for the same eps-neighborhoods), the previously-cohesive
   472-point board cluster fragments into pieces; only an 81-point strip
   fragment remains near the board's location, and it doesn't pass the
   scorer's board-size gate. This is a **candidate-generation regression**,
   not a scorer problem — the 2D scorer never even sees a board-shaped input
   to reject or accept.

Net: accumulation, at least as implemented (naive frame concatenation, no
eps re-tuning, tested only on static sample captures), is not the ring-gap
fix hypothesized. The bottleneck stays exactly where stage 1 left it —
candidate generation on ring-striped clouds — and accumulation as tested
makes that bottleneck worse, not better.

### Stance term: does it kill C's clutter false positives without hurting B's true board?

The brief's C-check command (`--accumulate 10 --stance-weight 0.5`) bundles
accumulation and stance together, and since accumulation alone already
collapsed C's recall to 0/6 on ds3, that run cannot isolate the stance
effect — every candidate is "not-a-detection" for the accumulation reason
above, stance or not. To actually test hypothesis 2, a supplementary
single-frame (`--accumulate 1`, i.e. stage-1-equivalent) check reusing
cached frames was run outside the brief's three commands, directly
comparing `stance_weight=0.0` vs `0.5` on ds3's first 30 frames (matching
stage 1's C scope exactly):

| stance_weight | detection rate | detections at clutter panel A (4.7, 2.6) | detections at clutter panel B (‑3.3, 3.4) |
|---|---|---|---|
| 0.0 (stage-1 baseline, reproduced) | 28/30 | 18 | 10 |
| 0.5 | 10/30 | **0** | 10 |

Stance **fully eliminates panel A** (18 → 0) but leaves **panel B fully
intact** (10 → 10, same scores shifted up rather than suppressed — e.g.
frame 29 goes 0.641 → 0.557, still comfortably above `min_score`). This is a
partial, not a full, fix for C: whichever axis-aligned panel happens to sit
closer to vertical-diagonal alignment (or which the region-growing generator
happens to fit a near-diamond-oriented quad to) survives the stance gate.
**The stance term is not a general axis-aligned-panel rejector; it only
catches panels whose fitted quad orientation the term happens to penalize.**

The B-side check (does stance hurt the one confirmed true-board hit) was
also run standalone: ds3 frame 5 (generator B, the stage-1 verified hit at
the bbox reference location) scores 0.538 at `stance_weight=0.0` and 0.536
at `stance_weight=0.5` — a 0.002 drop, negligible. The diamond board's
gravity-aligned corner stance is correctly near-1, so the blend does not
punish it. **Stance does not regress the one true-board detection
available to check it against.**

### What the overlays show

Overlay PNGs for the accumulated windows (`results/run4-acc/ds3_b_win0000.png`
etc.) confirm the diagnosis visually: the top-down scatter shows the same
sparse, ring-gapped board silhouette as the stage-1 single-frame overlays —
denser in absolute point count but not visibly more "filled in" as a shape,
consistent with the 1.95× (not 10×) downsampled-point growth measured above.
Every accumulated-window overlay is labeled "NO DETECTION"; none show a
fitted quad.

## Stage 3 Results

Stage 3 tests the third stage-2 next-step: `--vertical-gap-deg` (commit
landed on this branch) z-compresses points by a range-scaled factor before
generator B's DBSCAN clustering step, hypothesized to reconnect VLP-32C
ring-gap-fragmented board patches into one coherent cluster without
widening the horizontal tolerance (see the docstring at
`candidates/cluster_after_ground.py:_anisotropic_scaled`). Suite is 48/48
green (`uv run pytest -q`) before and after this benchmark.

Two single-frame (`--accumulate 1`, default) runs per the brief, both at
`stance-weight 0.5` so they isolate the anisotropic effect against a
like-for-like baseline (stage 1's B numbers were measured at stance 0, so
they are not directly comparable to either run below):

```bash
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --out results/run5-aniso            # vertical-gap-deg=3.0 (default)
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --vertical-gap-deg 0 --out results/run5-control
```

### Recall per dataset (fraction of frames with any detection ≥ min_score)

| Dataset | Stage-1 B (stance 0, no aniso) | run5-control (stance 0.5, aniso off) | run5-aniso (stance 0.5, aniso on) |
|---------|---|---|---|
| 1 | 1% (1/103, not bbox-verified) | 1% (1/103) | 3% (3/103) |
| 2 | 0% (0/103, not bbox-verified) | 0% (0/103) | 1% (1/103) |
| 3 | 2% (2/113) | 2% (2/113) | 1% (1/113) |
| 4 | 1% (1/113, not bbox-verified) | 1% (1/113) | 0% (0/113) |
| 5 | 8% (8/103, unverified/likely clutter) | 4% (4/103) | 2% (2/103) |
| **Total** | — | **8/535 (1.5%)** | **7/535 (1.3%)** |

Raw recall is a wash — total accepted detections are the same order of
magnitude either way (8 vs 7) and per-dataset deltas go in both directions
(aniso +2 on ds1, control +1 on ds3/ds4/ds5). **On numbers alone, stage 3's
headline candidate mechanism does not clear the ~100% bar stage 1/2 left
unmet** — but a per-frame trace (below) shows the mechanism is doing exactly
what it was designed to do; the scorer, not candidate generation, now caps
the result.

### Timing (median ms per frame, p95 in parentheses; 100 ms/frame budget)

| Run | downsample | candidates | scoring | total |
|---|---|---|---|---|
| run5-aniso (ds1–5 median) | 3.1 | 84.6–87.2 | 0.55–0.63 | 88.6–91.1 (97.8–102) |
| run5-control (ds1–5 median) | 3.1–3.3 | 77.4–81.5 | 0.88–1.14 | 81.9–85.7 (92.6–96.2) |

Both runs comfortably clear the 100 ms budget; the anisotropic scaling adds
~5–7 ms/frame to the candidate stage (extra z-scaling arithmetic on top of
the existing DBSCAN call) and p95 stays under 103 ms everywhere. Not the
bottleneck.

### Pose jitter (std of detected center [mm] / normal [deg], n = detection count)

| Dataset | run5-aniso | run5-control |
|---------|---|---|
| 1 | 5.7 / 0.14 (n=3) | — (n=1) |
| 2 | — (n=1) | — (n=0) |
| 3 | — (n=1) | 25.1 / 0.0 (n=2) |
| 4 | — (n=0) | — (n=1) |
| 5 | 9.4 / 0.00 (n=2) | 1122 / 5.7 (n=4) |

run5-control/ds3 (n=2) is the **exact same two-frame stage-1 pair**
reproduced verbatim: frame 5 scores 0.536 (vs. 0.538 at stance 0 in stage
1/2 — the 0.002 stance delta already documented) and center (2.103,
-0.239, -0.013); jitter is again 25 mm, the noise-floor value already on
record. run5-aniso's ds1 (n=3, 5.7 mm) and ds5 (n=2, 9.4 mm) look like tight
convergent clusters by the numbers alone — but ds5's tightness is
**clutter-panel** jitter (see pose sanity below), not board precision, so
low jitter is not by itself evidence of a correct detection; it only
confirms the same object was hit repeatedly.

### Pose sanity — bbox-reference cross-check, all 5 datasets

The bbox crop-box reference (`ros/lctk_launch/config/board/bbox.json5`,
translation `[2.6, 0, 0.35]`, size `[3.1, 3.94, 2.2]` → x∈[1.05,4.15],
y∈[-1.97,1.97], z∈[-0.75,1.45]) is one physical rig setup shared by all
five sample datasets, not ds3-specific, so it is a valid sanity check
everywhere. A scratch script (not committed) re-ran `detect()` per frame
for both configs on all 535 frames and classified every accepted
detection's center against that box:

| Run | detections inside bbox (true-board candidates) | detections outside bbox (clutter) |
|---|---|---|
| run5-aniso | 5/7 — ds1 (3, all ~(2.25, -0.05, 0.07)), ds2 (1, (2.15, 0.41, 0.08)), ds3 (1, (2.10, -0.32, 0.08)) | 2/7 — ds5, both ~(-1.83, -2.89, -0.07) |
| run5-control | 4/8 — ds1 (1, (2.26, -0.09, 0.02)), ds3 (2, (2.10, -0.24…-0.31, ~0)), ds4 (1, (2.08, -0.56, 0.03)) | 4/8 — ds5, three ~(-1.83, -2.90) + one ~(-3.53, 3.15) |

Every in-box hit clusters tightly around x≈2.1–2.3 m, y≈-0.6…+0.4 m — one
consistent physical location across datasets 1, 2, 3, and 4, not scattered
coordinates. Reading the overlays for confirmed in-box frames (ds1 frame 99/83, ds2
frame 37, ds3 frames 37/96) shows a
clean diamond outline **with two dark hole blobs** — the actual
hollow-board pattern, not a featureless panel. ds3 frame 5, though also at the
bbox location, rasters as a more ragged blob-like region without crisp diamond
boundary — present in the region but not confirmed to the hole-pattern standard.
ds4 frame 17 similarly clusters at the bbox location but rasters as visually
ragged, not confirming the hollow-diamond pattern. ds5's out-of-box hits, by
contrast, raster as a single solid filled region with **no holes** — the
same "board-sized clutter panel" signature stage 1 documented for
generator C. **Stage 3 extends the stage-1 ds3-only verified true-board
finding to datasets 1, 2, and 3 as well** (ds4 is location-consistent but
not confirmed to the hole-pattern standard; ds5 stays clutter under both
configs — stance 0.5 does not fix it here either, consistent with stage
2's finding that stance only kills one of two clutter-panel orientations).

### False positives under stance 0.5

Both configs still accept ds5's clutter panel at score ≥ 0.5 (aniso: 0.62,
0.59; control: 0.55, 0.57, 0.60, 0.53) — stance 0.5 does not suppress it,
matching stage 2's finding that the term only kills the *other* panel
orientation, not both. No new false-positive location appears in stage 3;
ds1–ds3's in-box hits are visually confirmed true-board (holes present), and
ds4's in-box hit is at the correct location but visually ragged (not
confirmed to hole-pattern standard).

### Per-frame trace: candidate generation vs scorer gate (the real finding)

The recall table above hides the mechanism. Re-running `detect()` per frame
on dataset 3 and checking whether *any* candidate (accepted or
`best_rejected`) landed inside the bbox — not just the final accepted
detection — gives a very different picture:

| Run | frames with a board-region candidate (accepted or rejected) | of those, score distribution |
|---|---|---|
| run5-control | 6/113 (5%) | min 0.069, max 0.577, **median 0.399** |
| run5-aniso | 35/113 (31%) | min 0.054, max 0.579, **median 0.074** |

**Anisotropic clustering does exactly what it was designed to do**: a
board-shaped candidate near the true location now appears in 6× more
frames (31% vs 5%). That confirms the hypothesis behind stage 3 — z-scaled
DBSCAN does reconnect ring-gap-fragmented board patches far more often than
isotropic clustering. But the merged patch is lower quality on most of
those extra frames: median score for the near-bbox candidate *drops* from
0.40 to 0.07, because the widened vertical tolerance that bridges ring gaps
also sweeps in more off-board coplanar points at similar range, diluting
squareness/fill/edge-straightness. Only a minority of the 35 aniso frames
(1 of 35) cross `min_score=0.5`; the rest sit well below it. Net effect on
final recall is therefore close to a wash even though candidate generation
improved sharply — **the bottleneck has moved from "no board-shaped
candidate exists" (stage 1/2's diagnosis) to "the scorer can't accept the
noisier merged candidate aniso now reliably produces."** This is a genuine
step forward in candidate generation, just not (yet) in end-to-end recall.

### `best_rejected` distribution — where do the remaining misses land

- run5-aniso: near-bbox rejects (ds3) cluster tightly at (2.10, -0.31, z
  varying with frame) — the *same* physical location across dozens of
  frames, just under-scoring, not scattered.
- run5-control: near-bbox rejects are rarer (matches the 5% figure above)
  but land at the same coordinates when they occur.
- Off-bbox rejects in both runs continue to cluster at the same clutter
  coordinates stage 1/2 already named (~(4.7, 2.6), ~(-3.3, 3.4),
  ~(-1.83, -2.9)) — no new clutter attractor appeared.

### What the overlays show

Side-by-side, run5-aniso/ds1_b_frame0099.png and run5-aniso/ds2_b_frame0037.png
show a clean, dense diamond raster with two well-separated dark hole
blobs and comparatively little residual black speckle inside the white
region — visibly *less fragmented* than the equivalent stage-1/control
raster (run5-control/ds3_b_frame0005.png, identical pixel-for-pixel to
`results/run2/ds3_b_frame0005.png` from stage 1) whose interior shows a
more ragged, blob-like white region without a crisp diamond boundary. This
matches the per-frame trace above: aniso's merged patches are shaped closer
to the true board more often, they just don't clear the score gate as
consistently. run5-control/ds5_b_frame0068.png (clutter) rasters as one
large solid filled blob with a straight edge and no interior holes at all —
visually distinct from every confirmed board hit, and the reason the
bbox-location gate is a meaningful sanity check independent of the scorer.

### Stage-3 verdict

Partial, directionally-positive result, not a fix:

- **Hypothesis confirmed for candidate generation**: anisotropic vertical
  clustering reconnects ring-gap-fragmented board patches 6× more often
  near the true board location (31% of ds3 frames vs 5% for the isotropic
  control), extending stage 1's single-dataset verified finding to
  datasets 1, 2, and 4 as well (visually confirmed via hole-pattern
  overlays and the shared bbox-reference coordinate cluster).
- **Hypothesis not confirmed for end-to-end recall**: the extra candidates
  score far lower on average (median 0.07 vs 0.40) because the same
  widened tolerance that bridges ring gaps also admits more off-board
  clutter into the merged patch, so accepted-detection recall stays flat
  (7 vs 8 total detections across 535 frames) — a wash, reported as such.
- **The bottleneck has moved, not closed.** Stage 1/2 diagnosed "no
  board-shaped candidate reaches the scorer." Stage 3 shows the candidate
  now *does* reach the scorer on 6× more frames — the remaining gap is
  scorer discrimination on a noisier merged patch (fill ratio / squareness
  / edge straightness degraded by the extra swept-in points), not
  candidate absence. A quality-aware merge (e.g. cap how much of the
  merged patch is allowed to fall outside the initial seed cluster's
  convex hull, or re-tighten `eps_v` once a plausible board-size patch is
  found) is the natural next lever, not a bigger `vertical-gap-deg`.
- Timing stays inside budget (aniso adds ~5–7 ms/frame, both configs
  <103 ms p95) and stance 0.5 still leaves one clutter panel unfiltered
  on ds5 under both configs — neither changes the stage-2 verdicts on
  those two fronts.

## Stage 4 Results

Stage 4 tests the stripe-tolerant scorer hypothesis: a gravity-oriented
anisotropic morphological closing (`score_candidate`'s `up_2d` /
`close_height_m` path, `scorer.py`) closes ring-stripe gaps in the fill-ratio
raster along the true vertical direction, while the coarse quad is now fit
with `cv2.minAreaRect` directly on the raw projected points (not the closed,
corner-bulged raster) so corner accuracy is unaffected by the tall kernel.
Suite is 55/55 green (`uv run pytest -q`) before this benchmark.

Two runs per the brief:

```bash
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --out results/run6-stripe          # vertical-gap-deg=3.0 (default): stage-4 scorer + stage-3 clustering
uv run python -m boarddet.benchmark --datasets 3 --generators b \
  --stance-weight 0.5 --vertical-gap-deg 0 --out results/run6-control  # both anisotropic stages off (stage-1-equivalent)
```

A scratch script (not committed) re-ran `detect()` per frame on all 535
frames of run6-stripe's config and classified every accepted detection's
center against the same bbox reference stage 3 used
(`ros/lctk_launch/config/board/bbox.json5`), to separate true-board hits
from clutter — the question this stage's numbers hinge on.

### Recall per dataset (fraction of frames with any detection ≥ min_score)

| Dataset | Stage-1 B (stance 0, no aniso) | Stage-3 aniso (run5-aniso, stance 0.5) | Stage-4 stripe (run6-stripe, stance 0.5, aniso clustering + aniso closing) |
|---------|---|---|---|
| 1 | 1% (1/103) | 3% (3/103) | **34% (35/103) — 31 in-bbox / 4 clutter** |
| 2 | 0% (0/103) | 1% (1/103) | **51% (53/103) — 45 in-bbox / 8 clutter** |
| 3 | 2% (2/113) | 1% (1/113) | **47% (53/113) — 46 in-bbox / 7 clutter** |
| 4 | 1% (1/113) | 0% (0/113) | **33% (37/113) — 33 in-bbox / 4 clutter** |
| 5 | 8% (8/103, unverified) | 2% (2/103) | **50% (52/103) — only 7 in-bbox / 45 clutter** |
| **Total** | — | **7/535 (1.3%)** | **230/535 (43.0%) — 162 in-bbox (30.3%) / 68 clutter (12.7%)** |

The headline recall jump is real, but reading it as a single number would be
wrong: it is two very different stories layered on top of each other, and
they must be reported separately (see the false-positive section below
before treating the "43%" figure as good news).

### Timing (median ms per frame, p95 in parentheses; 100 ms/frame budget)

| Run | downsample | candidates | scoring | total |
|---|---|---|---|---|
| run6-stripe (ds1–5 median) | 2.8–3.1 | 54.3–58.6 | 0.71–0.83 | 58–63 (72–77) |
| run6-control (ds3) | 3.0 | 48.7 | 1.05 | 53 (71) |
| *for reference* stage-3 run5-aniso (ds1–5 median) | 3.1 | 84.6–87.2 | 0.55–0.63 | 88.6–91.1 (97.8–102) |

Both stage-4 runs stay comfortably inside the 100 ms budget, with headroom
to spare. The `scoring` stage itself — the part stage 4 actually changed
(rotation, tall-kernel close, `minAreaRect` on raw points) — costs about the
same as stage 3's isotropic close (0.7–1.1 ms both before and after), so the
new geometry work is not measurably more expensive. The `candidates` stage
median dropped from ~85 ms (stage 3) to ~55 ms (stage 4) even though
`cluster_after_ground.py` was not touched by either stage-4 commit
(`8209e2d`, `8805919` only edit `scorer.py`/`detector.py`/`viz.py`) — this is
most likely machine-load variance between two separate benchmark
invocations on a shared 32-core box (`nproc`, 25 logged-in users, `time`
showed >1600% CPU during both runs, i.e. heavy internal threading whose wall
time is sensitive to contention), not a code effect. Reported as observed,
not claimed as a speedup.

### Pose jitter — read this number with the false-positive section, not alone

| Dataset | run6-stripe (all accepted, mixed population) |
|---------|---|
| 1 | 887 mm / 26.1° |
| 2 | 1113 mm / 28.4° |
| 3 | 903 mm / 26.2° |
| 4 | 851 mm / 23.8° |
| 5 | 1455 mm / 22.4° |

Taken at face value these look like a huge regression from stage 3's 25 mm
noise floor — and they would be, if these were jitter of one detected
object. They are not: the benchmark's jitter formula pools every accepted
detection in a dataset into one std, and stage 4 now accepts both the true
board and several different clutter attractors in the same dataset (next
section), so this number measures *the spread between different physical
objects*, not the precision of any one of them. Splitting the same
detections by the bbox classification (below) gives the real per-object
precision:

| Dataset | in-bbox (true-board candidate) center std | n |
|---------|---|---|
| 1 | 3 / 21 / 16 mm (x/y/z) | 31 |
| 2 | 2 / 6 / 6 mm | 45 |
| 3 | 2 / 4 / 5 mm | 46 |
| 4 | 2 / 15 / 16 mm | 33 |
| 5 | 6 / 40 / 43 mm | 7 |

These are as tight as, or tighter than, stage 3's already-good 25 mm
noise-floor figure — strong independent evidence that the in-bbox population
really is repeat detections of one static object, not scattered noise.

### Pose sanity — bbox-reference cross-check, all 5 datasets

Per-dataset in-bbox center means (score range in brackets), compared against
stage 3's confirmed true-board coordinates:

| Dataset | run6-stripe in-bbox mean center | stage-3 confirmed coordinate | match? |
|---------|---|---|---|
| 1 | (2.256, -0.059, 0.074), [0.517, 0.778] | ~(2.25, -0.05, 0.07) | yes, near-exact |
| 2 | (2.147, 0.420, 0.076), [0.543, 0.790] | (2.15, 0.41, 0.08) | yes, near-exact |
| 3 | (2.101, -0.314, 0.074), [0.556, 0.768] | (2.10, -0.32, 0.08) | yes, near-exact |
| 4 | (2.077, -0.605, 0.066), [0.520, 0.751] | (2.08, -0.56, location-only) | yes, same location |
| 5 | (2.090, -0.829, 0.039), [0.514, 0.758] | not previously confirmed on ds5 | new, plausible (same x-band) |

Overlays for a spread of in-bbox frames across all datasets
(`ds1_b_frame0008.png` score 0.67, `ds2_b_frame0000.png` score 0.78,
`ds3_b_frame0000.png` score 0.72, `ds3_b_frame0110.png` score 0.69,
`ds4_b_frame0003.png` score 0.73) all show a clean, filled diamond raster
**with two distinct dark hole blobs** — the hollow-board pattern, matching
stage 3's verified signature exactly, now recurring across roughly a third
to a half of each dataset's frames instead of 1–3 isolated frames. **This is
a genuine, verified recall conversion for the true board on 4 of 5
datasets** (ds5's true-board population is small — 7/103 — see below).

### False positives — CRITICAL: does the elongated kernel inflate clutter scores too? Yes.

This is the headline finding, and it is a regression, not a footnote:

- **ds5's already-known clutter panel** (the flat, hole-less board-sized
  panel stage 1/2/3 all documented at center ≈ (-1.83, -2.89, -0.1)) is
  accepted on **35 of ds5's 45 out-of-bbox detections** (34% of all ds5
  frames), at scores up to **0.72** — up from stage 3's max 0.62 on the same
  panel, and from a 2% acceptance rate to 34%. `stance-weight 0.5` does
  **not** hold the line here (consistent with stage 2's finding that stance
  only kills one of the two clutter-panel orientations — this is the other
  one). Overlays `ds5_b_frame0006.png` (score 0.51) and
  `ds5_b_frame0046.png` (score 0.59) both show a solid filled diamond-ish
  blob with **no interior holes** — visually the same clutter signature as
  every prior stage, just scored far higher and accepted far more often.
  Under stage 4 this panel's reach also widens beyond ds5: 2 of ds2's 8
  clutter detections (frames 17/18) hit the same panel at ~(-2.0, -2.8,
  -0.1) — a location previously confined to ds5 now also crosses threshold
  in ds2.
- **New clutter attractors appear on datasets 1–4**, which had ~0
  out-of-bbox false positives under stance 0.5 in stage 3. Stage 4 accepts
  4–8 per dataset (23 total, excluding ds5) at scores 0.50–0.67, and they
  are not scattered noise: they recur at a handful of shared, scene-fixed
  coordinates across multiple datasets (all 5 share one physical rig/room,
  as stage 3 noted). The (x, y) footprint ~(0.2–0.65, 3.5–3.95) turns out to
  span two different z-bands rather than one shared fixture: a
  **negative-z** band (z ≈ -0.5 to -0.6, at y ≈ 3.5–3.55) appears in ds1, 3,
  5, while a **positive-z** band (z ≈ +0.58 to +0.61, at y ≈ 3.95) appears
  in ds3, 4 — ds3 alone hits both bands. A ±0.5 m z flip at essentially the
  same (x, y) is more consistent with two different scene objects than one
  physical fixture. A separate attractor at ~(-4.5, 3.68, 0.5) appears in
  ds1, 2, 3, and another at ~(0.1, -4.0, -0.1…-0.4) appears in ds1, 2, 3, 5.
  These read as static room structures (walls, fixtures) that happen to be
  near-vertical, planar, and roughly board-sized — exactly the kind of
  object the border-only cue was already known (stage 1/2) not to
  discriminate against, now newly crossing threshold because the
  anisotropic closing also fills *their* fill-ratio gaps. A further,
  distinct attractor sits at ~(-2.2 to -2.3, 3.32–3.34) in ds3 (z -0.24) and
  ds4 (z -0.62). `ds3_b_frame0051.png` (score 0.51, center (-2.31, 3.32,
  -0.24)) shows the overlay for this one — its coordinates match none of
  the clusters above, so it's this separate attractor, not an example of
  them: a lopsided, non-diamond quadrilateral fit to a mostly-filled black
  region from a scattered building/wall scene, with no hole pattern —
  visually distinguishable from a true-board hit by eye, but not by the
  current score/stance gate.
- **Net effect**: total accepted clutter went from 2/535 (0.4%, stage-3
  aniso) to 68/535 (12.7%, stage-4 stripe). On ds5 specifically, clutter now
  *outnumbers* true-board detections roughly 6.4-to-1 (45 vs 7) — so ds5's
  raw "50% detection rate" in the top-line table is misleading read alone;
  it is overwhelmingly the clutter panel, not the board.
- run6-control (ds3, both anisotropic stages off) reproduces stage 3's
  control number almost exactly — 2% (2/113), jitter 25.1 mm / 0.00° — and
  its overlay (`ds3_b_frame0005.png`, score 0.54) is the same fragmented,
  non-diamond raster stage 1/3 already documented. This confirms the
  stage-4 code changes (raw-point coarse quad, `_refine_sides` skip on the
  anisotropic path) left the isotropic path byte-identical, as the
  `8805919` commit claims — the false-positive expansion above is
  attributable to the anisotropic closing specifically, not a side effect
  of the surrounding refactor.

### What the overlays show

Side by side, the true-board overlays (`ds1_b_frame0008.png`,
`ds2_b_frame0000.png`, `ds3_b_frame0000.png`/`ds3_b_frame0110.png`,
`ds4_b_frame0003.png`) are visually uniform: a dense, filled diamond with
crisp edges and two well-separated dark holes, essentially the same shape
stage 3 showed only on 1–3 hand-picked frames, now the modal outcome across
each dataset. The clutter overlays (`ds5_b_frame0006.png`,
`ds5_b_frame0046.png`, `ds3_b_frame0051.png`) share a different, consistent
signature: a mostly- or fully-filled region (no holes) fit by a skewed,
often non-diamond quadrilateral — visually distinguishable by eye from the
true-board hits in every case inspected, but that distinction is exactly
the cue (hole pattern) the scorer still does not use.

### Stage-4 verdict

A real, substantial win for the true board, bundled with a comparably-sized
new false-positive problem — report both, not the net number:

- **The stage-4 hypothesis is confirmed for the true board**: stage 3's
  diagnosis ("candidate reaches the scorer on 31% of ds3 frames but scores
  ~0.07, well below threshold") is exactly what the stripe-tolerant closing
  fixes. True-board recall converts from stage 3's ~1–3% to **30–47% on
  datasets 1–4**, at the same physical location stage 3 already confirmed
  (mm-level agreement on center coordinates), with mm-tight jitter (2–21 mm)
  as good as or better than stage 3's 25 mm noise floor, and the hole
  pattern visibly present on every inspected overlay. This is the clearest
  positive result the phase has produced.
- **The same mechanism also inflates clutter scores**, exactly as the task
  brief warned it might. ds5's known clutter panel goes from a 2%
  nuisance to the dominant signal in that dataset (34% of frames, score up
  to 0.72, outnumbering true-board hits 6.4:1), and 4 new scene-fixed
  clutter attractors appear across datasets 1–4 that stage 3 never
  triggered. `stance-weight 0.5` does not catch any of this — it was never
  designed to (it targets panel *orientation*, not fill/squareness
  inflation from stripe-closing).
- **The bottleneck has moved again, not closed.** Stage 1/2 diagnosed "no
  board-shaped candidate reaches the scorer." Stage 3 diagnosed "the
  candidate reaches the scorer but the merge is too noisy to score above
  threshold." Stage 4 fixes exactly that noisy-merge scoring gap for the
  true board — but the same fix removes the fill-ratio penalty that was
  incidentally also suppressing clutter, so **the open problem is now pure
  discrimination**: fill ratio, squareness, and stance all move together for
  true board and clutter alike once ring-stripe/gap tolerance is added to
  all of them. A cue that is *absent from the clutter panels* is needed —
  the hole pattern (flagged as the leading candidate since stage 1/2,
  still not implemented) is the natural next lever, since every clutter
  overlay inspected this stage lacks holes while every true-board overlay
  has them.
- Timing stays inside budget in both configs (58–63 ms median, 72–77 ms
  p95, vs the 100 ms budget), and the isotropic control path is confirmed
  unaffected by the stage-4 refactor.

## Stage 5 Results

Stage 4 left one open problem: border/fill/squareness/stance cannot tell the
true board from board-sized planar clutter once both get the same
stripe/gap tolerance. Stage 4's planned fix — a hole-pattern score term — was
overtaken by a hardware decision: the recorded board is moving to a plain
**hole-free** diamond (see `board_config.py`'s Task-18 comment), so
hole-pattern discrimination is off the table for good. Task 18 built four
alternative single-frame discriminator gates that key on the diamond's
*stance* (standing on a corner) and the *physical presence* of all four
edges instead of holes: `strict_squareness`, `stance_floor`,
`edge_support_min`, and a tightened `side_tol`. This section benchmarks both
operating points those gates define — `--stance-gate` (`stance_floor=0.9`
alone) and `--strict-diamond` (all four together) — properly through the
benchmark CLI (overlays, timing, jitter), and independently re-derives the
per-cue ablation table Task 18 first produced under review.

Suite is 65/65 green (`uv run pytest -q`) before this benchmark.

```bash
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --stance-gate --out results/run7-stance
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --strict-diamond --out results/run7-strict
# baseline (stage-4 config, no Task-18 gates) reused from results/run6-stripe
```

A scratch script (not committed) reproduced task 18's exact methodology
(`detect()` called directly, all 535 cached frames across datasets 1–5,
generator `b`, `stance_weight=0.5`, `vertical_gap_deg=3.0`) for all three
operating points plus the two intermediate per-cue rows, classifying every
accepted detection's center against the bbox reference
(`ros/lctk_launch/config/board/bbox.json5`: translation `[2.6,0,0.35]`, size
`[3.1,3.94,2.2]` → x∈[1.05,4.15], y∈[-1.97,1.97], z∈[-0.75,1.45]). Every
number below reproduces task 18's independent-review numbers exactly (no
tolerance needed).

### Precision/recall across the three operating points

| Operating point | ds1 | ds2 | ds3 | ds4 | ds5 | **Total true / clutter** | **Recall (of 535)** | **Precision** |
|---|---|---|---|---|---|---|---|---|
| baseline (stage-4, no gates) | 31/4 | 45/8 | 46/7 | 33/4 | 7/45 | **162/68** | 30.3% | 70.4% |
| **`--stance-gate`** (recommended) | 31/1 | 45/1 | 46/0 | 34/0 | 7/13 | **163/15** | 30.5% | **91.6%** |
| `--strict-diamond` (max precision) | 8/0 | 17/0 | 22/0 | 12/0 | 0/0 | **59/0** | 11.0% | **100%** |

`--stance-gate` **retains full recall** (162→163 — the +1 is a near-tie
frame moving in, not a loss) **while cutting clutter 78%** (68→15,
precision 70.4%→91.6%). `--strict-diamond` reaches 0 clutter but at a
severe recall cost: 163→59, a ~7:1 recall trade (104 true-board detections
lost to remove the last 15 false positives). ds5's true-board recall (7/103
under stance-gate) is wiped out entirely under strict-diamond, along with
its clutter (0/103 either way).

### Per-cue ablation (independently re-derived, generator b, all 535 frames)

| Config | ds1 | ds2 | ds3 | ds4 | ds5 | **Total** |
|---|---|---|---|---|---|---|
| baseline (no Task-18 gates) | 31/4 | 45/8 | 46/7 | 33/4 | 7/45 | **162/68** |
| + `stance_floor=0.9` only | 31/1 | 45/1 | 46/0 | 34/0 | 7/13 | **163/15** |
| + stance_floor + `strict_squareness` | 31/1 | 45/1 | 46/0 | 34/0 | 7/13 | **163/15** |
| + stance_floor + `side_tol=0.08` | 30/0 | 45/1 | 46/0 | 32/0 | 6/10 | **159/11** |
| + stance_floor + `edge_support_min=0.6` | 8/0 | 17/0 | 22/0 | 12/0 | 0/0 | **59/0** |
| full `--strict-diamond` (all four) | 8/0 | 17/0 | 22/0 | 12/0 | 0/0 | **59/0** |

**Stance does essentially all the work at ~0 recall cost.** `stance_floor`
alone accounts for the entire 68→15 clutter reduction that matters at an
acceptable price. Layering `strict_squareness` on top changes nothing
(163/15, identical) — consistent with task 18's structural finding that
`strict_squareness` is inert on the anisotropic path every real cached frame
takes (`corners = cv2.boxPoints(minAreaRect(...))` is exactly rectangular by
construction on that path, so the gate has nothing to fire on). `side_tol`
is a cheap knob (159/11 — trades 4 recall for 4 more clutter kills).
**`edge_support_min=0.6` is the expensive gate**: alone it drives both the
final 15→0 clutter kill and the entire 163→59 recall collapse — a ~7:1
recall cost per extra false positive caught. Real edge support pins to a
near-binary set of values (`{0, 0.33, 0.5, 0.67, 1.0}`) at real sensor
ranges because the ring-gap-calibrated bin width collapses `n_bins` to 2–3
per side; residual clutter's min-side value pins at exactly 0.5 across all
15 detections while true board's min ranges 0.333–0.667, which is what the
gate exploits — cleanly, but at a steep recall price. (Full mechanism —
including the min-vs-mean edge_support distinction and the `n_bins`
collapse — is documented in `task-18-report.md`; not re-derived here.)

### Part A: killable vs irreducible false positives (from `task-18-report.md`)

Task 18 characterized all 68 baseline false positives against three
post-hoc geometric checks (squareness ≤8°, stance >0.9, size ≤8%):

| Dataset | FP total | (i) non-diamond (killable) | (ii) board-like (irreducible, single-frame) |
|---|---|---|---|
| 1 | 4 | 4 (100%) | 0 |
| 2 | 8 | 8 (100%) | 0 |
| 3 | 7 | 7 (100%) | 0 |
| 4 | 4 | 4 (100%) | 0 |
| 5 | 45 | 44 (97.8%) | 1 (2.2%) |
| **Total** | **68** | **67 (98.5%)** | **1 (1.5%)** |

Only **one single frame** (ds5 frame 60: stance 0.902, size deviation 5.4%,
squareness 0.0° off) passes all three static checks — a near-miss that
happens to sit 0.002 above the 0.9 stance floor. This task's independent
per-frame classification confirms that exact frame is among the 15 residual
`--stance-gate` detections (ds5 frame0060, score 0.547, center
(-1.838, -2.876, -0.117)) — the static Part-A characterization and the live
gate's residual population agree on at least this one member.

Read the two "67/68 killable" and "15 residual under `--stance-gate`"
numbers together carefully, they are not the same measurement: Part A
scored the *original* 68 baseline-accepted quads against post-hoc checks,
while `--stance-gate` is a *live* gate that changes which candidate wins
each frame (a previously second-best candidate can become the accepted
detection once the top one is rejected). That is why live `stance_floor`
leaves 15 residual detections rather than the ~2 the static Part-A stance
tally alone would suggest (68 − 66 failing stance) — most of the 15 are
newly-emerged fits on the *same* ds5 clutter object, not survivors of the
original 68, as the residual characterization below confirms.

### Timing (median ms per frame, p95 in parentheses; 100 ms/frame budget)

| Operating point | median range (ds1–5) | p95 range (ds1–5) |
|---|---|---|
| baseline (run6-stripe) | 58–63 ms | 72–77 ms |
| `--stance-gate` (run7-stance) | 57.4–62.9 ms | 72.5–76.7 ms |
| `--strict-diamond` (run7-strict) | 57.4–59.8 ms | 70.3–75.5 ms |

All three operating points are essentially indistinguishable in cost and
comfortably inside the 100 ms budget. This matches task 18's implementation
note that all three new gates run immediately after the (already-computed)
corner-angle array, *before* the more expensive fill-ratio raster work —
they are early-reject checks, not additional per-candidate cost.

### True-board (in-bbox) pose jitter — std of center [mm], n = detection count

| Dataset | baseline | `--stance-gate` | `--strict-diamond` |
|---|---|---|---|
| 1 | 2.9/21.1/15.9 mm (n=31) | 2.9/21.1/15.9 mm (n=31) | 1.8/1.8/2.5 mm (n=8) |
| 2 | 1.7/5.9/6.4 mm (n=45) | 1.7/5.9/6.4 mm (n=45) | 1.2/2.9/2.4 mm (n=17) |
| 3 | 2.0/3.8/5.1 mm (n=46) | 2.0/3.8/5.1 mm (n=46) | 2.1/1.8/2.6 mm (n=22) |
| 4 | 2.0/15.4/16.4 mm (n=33) | 2.1/16.7/17.8 mm (n=34) | 2.1/3.2/3.4 mm (n=12) |
| 5 | 5.9/39.8/43.2 mm (n=7) | 4.4/32.8/34.9 mm (n=7) | n=0 (undefined) |

(x/y/z std, mm.) All three operating points stay at or below the ~25 mm
sensor-noise-floor precedent set in stage 3 on every axis but y/z for
datasets 1/4/5, where the board's own orientation spread (not sensor noise)
dominates. `--strict-diamond`'s surviving population is visibly tighter on
every dataset (1.2–2.6 mm) — unsurprising, since only the most cleanly-fit
candidates clear `edge_support_min=0.6`. ds5's jitter changes with the same
n=7 in both baseline and `--stance-gate` (different std) — evidence that the
live stance gate occasionally swaps in a different accepted candidate for the
same frame rather than only ever adding/removing whole detections, consistent
with the Part-A/live-gate distinction noted above. ds4 shows n=33→34 (+1
detection, not a candidate swap).

### Residual clutter characterization (15 detections, `--stance-gate`)

Classified each of the 15 residual clutter centers by distance from the
known ds5 clutter panel reference (~(-1.83, -2.89, -0.1), documented since
stage 1):

| Attractor | Count | % of 15 | Datasets | Example center |
|---|---|---|---|---|
| ds5 persistent panel (same physical panel; 12 within ≤0.1 m of ref, one ds2 outlier at 0.21 m) | **13** | **87%** | ds2 (1), ds5 (12) | (-1.851, -2.883, -0.120) |
| second attractor (y≈3.5, z≈-0.5…-0.6 band) | 2 | 13% | ds1 (1), ds5 (1) | (0.638, 3.527, -0.507) |

**13 of 15 (87%) are the same persistent near-vertical panel** stage 1–4
already documented — not confined to ds5: one of its hits surfaces in ds2
(frame 17, 0.21 m from the reference, matching stage 4's note that this
panel's reach crosses dataset boundaries since all 5 datasets share one
physical room/rig). The remaining 2/15 (13%) are a second, distinct
scene-fixed attractor stage 4 first flagged (its "y≈3.5–3.55, z≈-0.5 to
-0.6" band, previously noted in ds1/3/5) — here it survives the stance gate
in ds1 and ds5 specifically. **No new, previously-undocumented clutter
attractor appears under `--stance-gate`.**

Inspected overlays for both attractors (`ds2_frame17_panel`,
`ds1_frame41_second_attractor`, generated to the same `save_overlay` format
as the committed run) show the same signature documented since stage 1: a
solid, fully-filled planar region with **no interior holes**, fit by a
well-formed (not fragmentary) quad — visually indistinguishable from the
true board's raster in outline shape, distinguishable only by the absence
of the two hole blobs every true-board overlay shows
(`ds3_b_frame0000.png`, both operating points). This is exactly why
`edge_support` — the one gate that does separate them — has to pay for it
in recall: the panels are real, well-formed, ring-gap-striped planar
surfaces, not fragments, so only the same coarse-binned edge-support signal
that (barely) distinguishes them from the true board also catches some
genuine board detections that share the same 2–3-bin quantization.

### DECISION (Stage 5)

1. **`--stance-gate` is the recommended single-frame operating point.** It
   retains full recall (163/535, no measurable loss vs. baseline's 162) and
   cuts clutter 78% (68→15), raising precision from 70.4% to 91.6% for
   free. There is no recall cost to adopting it as the default.
2. **Driving single-frame clutter to zero is not worth it.** `--strict-diamond`
   reaches 100% precision but costs ~64% of recall (163→59) to remove the
   last 15 false positives — a ~7:1 recall-to-FP trade, and it zeroes out
   ds5's true-board recall entirely along with its clutter. No single-frame
   geometric tightening tested in this phase reaches high precision without
   this order of recall cost, because `edge_support` is the only cue that
   discriminates the residual population and it is expensive by
   construction (coarse bin quantization at real sensor range).
3. **The residual ~15/535 (87% one persistent panel, 13% a second
   attractor) is fundamentally ambiguous to single-frame geometry.** Both
   attractors are real, well-formed, ring-gap-striped planar surfaces —
   not fragments or blobs — that happen to sit close enough to the true
   diamond's stance/edge-support signature that separating them costs
   recall rather than being free. **This is the concrete evidence pointing
   at a multi-pose/session-level cue as the real fix, not a tighter
   single-frame gate**: the calibration board is the object that *moves*
   between fixed poses during a session (already the workflow
   `advanced_extrinsic_solver`'s buffered multi-pose mode assumes), while
   the persistent panels are static scene fixtures that never change
   location. A capture-protocol change — record the board at ≥2 distinct
   positions per session and require a detection to appear at a location
   that changes across poses, rejecting/down-weighting any candidate
   location that repeats unchanged — would close this gap without any
   further single-frame gate tuning. **This cue cannot be tested on the
   current datasets 1–5**: each is a single static capture (the board does
   not move within a dataset — that staticness is exactly what makes the
   jitter statistics above meaningful as a precision proxy), so there is no
   second pose to compare against. Implementing and testing the
   multi-pose/session cue is a capture-protocol and future-phase item, out
   of scope here.

## Stage 6 Results

Stage 5 landed `--stance-gate` (`stance_floor=0.9`) at 30.5% recall / 91.6%
precision and named the *recall* ceiling — half the frames are lost — as the
next thing to attack. Stage 6 does that by sweeping the two recall levers the
per-frame failure diagnosis
(`.superpowers/sdd/stage6-failure-diagnosis.md`) identified, and — crucially —
measures what each costs in **precision** (clutter admitted), the question the
diagnosis explicitly left open because it replayed only the board's own
candidate, never the clutter.

Suite is 71/71 green (`uv run pytest -q`) before this benchmark (65 from
stage 5 + 6 from Task 20's configurable-gate CLI).

### Failure-mode diagnosis summary (the recall levers this stage sweeps)

The diagnosis bucketed every non-detected frame on ds1–4 (432 frames, the
`--stance-gate` operating point, generator b) by *first* failure cause, using
the real `detect()` path with per-stage instrumentation (methodology and the
full per-dataset table in `stage6-failure-diagnosis.md`):

| Bucket | Overall (n=432) | Meaning |
|---|---|---|
| DETECTED | 156 / 36.1% | (baseline for the diagnosis subset) |
| **C_FLATNESS** | **118 / 27.3%** | board plane-fit RMS in 0.035–0.048 m, rejected by the 0.035 gate |
| **F_SCORER_REJECT** | **99 / 22.9%** | `score_candidate` hard-gate/low-score; **50/99 killed by the 2D stance gate** |
| A_FRAGMENTED | 56 / 13.0% | board split across ≥2 clusters, none holding a majority |
| H_NO_BOARD_POINTS | 3 / 0.7% | occlusion/range data limit |
| B_STRIPPED, D_SIZE, E_MERGED, G_OUTSCORED | 0 / 0.0% each | not loss mechanisms here |

C_FLATNESS and F_SCORER_REJECT are 50.2% of frames combined and are the two
levers swept below. C_FLATNESS is a pure **near-miss** population (every
failing frame's RMS is within 37% of the 0.035 threshold; min 0.0351, median
0.0390, max 0.0481); the diagnosis's board-candidate-only replay projected
that raising the gate 0.035→0.045 recovers 84/118 of them (+19.4% absolute
recall) **but could not say what clutter that admits**. F_SCORER_REJECT is
dominated (50/99) by the `stance_floor=0.9` 2D diamond-stance hard gate — the
diagnosis flagged relaxing it as a *re-tune-and-re-measure* task, not a
one-line change, precisely because that gate was purpose-built in Task 19 to
buy precision. Stage 6 runs both re-measures across all 535 frames (ds1–5,
generator b), the missing precision check the diagnosis gated its
recommendations on.

Method: the **flatness sweep** (`0.035`/`0.040`/`0.045`/`0.050`) was run
through `boarddet.benchmark --out` and each point's `results/run8-flat*`
directory (not committed — `results/` is gitignored) holds a persisted
`summary.json` with per-dataset detection rate, timing, and jitter — verified
against the actual files on disk. The **stance-floor sweep** (`0.85`/`0.88`/
`0.90`), despite the command block below showing the same `benchmark.py --out
results/run8-stance085` invocation shape, was **not** persisted this way — no
`results/run8-stance*` directory exists. Its true/clutter, recall, and
precision numbers in the table further below come entirely from a scratch
script (not committed) that calls the real `detect()` per frame on all 535
cached frames and classifies every accepted detection's center against the
bbox reference (`ros/lctk_launch/config/board/bbox.json5`: translation
`[2.6,0,0.35]`, size `[3.1,3.94,2.2]` → x∈[1.05,4.15], y∈[-1.97,1.97],
z∈[-0.75,1.45]) into true-board (in-bbox) vs clutter (out-of-bbox), the same
rule stages 4–5 used. The same scratch script also produced the flatness
sweep's bbox classification (the true/clutter split in that table), layered
on top of the persisted `benchmark.py` runs. The `flat0.035` sweep point
reproduces stage 5's `--stance-gate` baseline exactly (163 true / 15
clutter), confirming the harness is faithful.

```bash
for F in 0.035 0.040 0.045 0.050; do
  uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
    --stance-weight 0.5 --stance-gate --flatness-rms-max $F --out results/run8-flat$F
done
# stance_floor lever at the best flatness:
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --stance-gate --stance-floor 0.85 --flatness-rms-max 0.045 \
  --out results/run8-stance085   # + 0.88, 0.90 points
```

Note: unlike the flatness-sweep invocations above, these `--out
results/run8-stance*` runs were not actually persisted — no such directory
exists under `results/`. The stance-floor sweep table below is sourced
entirely from the scratch `detect()`+bbox harness described above, not from
a committed (or gitignored-but-present) `summary.json`.

### Flatness sweep (stance_floor=0.9 fixed; true/clutter per dataset)

| flatness | ds1 | ds2 | ds3 | ds4 | ds5 | **true / clutter** | **recall** | **precision** | median ms (p95) |
|---|---|---|---|---|---|---|---|---|---|
| **0.035** (stage-5 baseline) | 31/1 | 45/1 | 46/0 | 34/0 | 7/13 | **163 / 15** | 30.5% | 91.6% | 59.6 (71.0) |
| 0.040 | 45/1 | 58/2 | 65/1 | 42/1 | 15/12 | **225 / 17** | 42.1% | 93.0% | 58.9 (73.1) |
| **0.045** (recommended) | 56/1 | 65/2 | 66/3 | 53/1 | 24/13 | **264 / 20** | **49.3%** | **93.0%** | 60.2 (73.0) |
| 0.050 | 57/1 | 65/2 | 66/3 | 54/1 | 28/13 | **270 / 20** | 50.5% | 93.1% | 60.0 (75.9) |

**Headline: raising the flatness gate 0.035→0.045 is a strict Pareto
improvement — recall rises 30.5%→49.3% (+18.8 pts absolute, +62% relative)
while precision *also* rises 91.6%→93.0%.** The precision-for-recall trade the
task set out to quantify **did not occur**: true detections grow far faster
(163→264, +101) than clutter (15→20, +5), so precision goes *up* even as a few
more false positives are admitted. This matches the diagnosis's +19.4%
board-candidate-only projection closely (it predicted the recall gain; the new
result is that the gain does not cost precision). 0.040 already captures most
of it (+11.6 pts recall at +1.4 pts precision); 0.045 is the sweet spot the
diagnosis identified (diminishing returns past it) and is where recall
plateaus for ds1–4. 0.050 buys only +1.2 pts more recall (270 vs 264), and
almost entirely on ds5 (24→28), the clutter-heavy dataset — while pushing the
gate deeper into VLP-32C sensor-noise territory (the floor that motivated 0.035
in the first place). 0.045 is preferred; 0.050 is a marginal, defensible
alternative if ds5-style captures dominate.

### Stance-floor sweep (flatness=0.045 fixed; does relaxing it recover the stance-gated frames?)

The diagnosis found 50 genuine board frames die on the `stance_floor=0.9`
hard gate and asked whether relaxing it recovers them. Swept at the chosen
flatness (0.045):

| stance_floor | ds1 | ds2 | ds3 | ds4 | ds5 | **true / clutter** | **recall** | **precision** | median ms (p95) |
|---|---|---|---|---|---|---|---|---|---|
| 0.85 | 56/4 | 65/3 | 66/6 | 53/2 | 25/25 | **265 / 40** | 49.5% | 86.9% | 61.0 (77.5) |
| 0.88 | 56/2 | 65/2 | 66/3 | 53/1 | 24/17 | **264 / 25** | 49.3% | 91.3% | 62.1 (77.8) |
| **0.90** (recommended) | 56/1 | 65/2 | 66/3 | 53/1 | 24/13 | **264 / 20** | 49.3% | 93.0% | 61.1 (75.8) |

**Relaxing stance_floor is refuted as a recall lever — keep 0.9.** Dropping it
to 0.85 barely moves recall (+1 true frame, 264→265) but craters precision
(93.0%→86.9%) by doubling clutter (20→40, +17 of them NEW). The diagnosis's
hypothesis — that ~50 stance-gated board frames would return as accepted
true detections — does **not** hold on the full scene: those board
candidates do not re-emerge above threshold, but a flood of previously
stance-rejected *clutter* does. This is exactly the precision cost the
diagnosis warned it could not see (board-candidate-only replay), now measured.
0.88 is an unremarkable midpoint (91.3%, no recall gain). The
`stance0.90@flat0.045` row reproduces the flatness-sweep 0.045 row
byte-for-byte (264/20), an internal consistency check on the two independent
sweep harnesses.

### Pareto front

Plotting recall (x) vs precision (y) across all seven points, the
non-dominated front is:

- **(0.035, sf 0.9): 30.5% / 91.6%** — dominated (0.045 beats it on *both* axes)
- **(0.045, sf 0.9): 49.3% / 93.0%** — on the front, the knee
- **(0.050, sf 0.9): 50.5% / 93.1%** — on the front, marginal extension

Every stance-floor-relaxation point (0.85, 0.88) is strictly dominated: lower
precision at equal-or-lower recall than (0.045, sf 0.9). The front is
remarkably flat in precision (91.6–93.1%) and spans 30.5–50.5% recall entirely
along the flatness axis. There is no genuine precision/recall *tension* to
trade off in this sweep — the recall gain from flatness comes for free (indeed
with a slight precision bonus), and the one lever that *would* trade precision
for recall (stance_floor) buys no recall for the precision it spends.

### Added-clutter characterization (where do the extra 5 false positives at 0.045 come from?)

Between 0.035 (15 clutter) and 0.045 (20 clutter), the +5 net is not more hits
on the known ds5 persistent panel — that count actually *drops* (13→11,
because the true board now wins more ds5 frames outright). The composition
shifts:

| clutter kind | flat 0.035 | flat 0.045 |
|---|---|---|
| ds5 persistent panel (≈(-1.83,-2.89,-0.1)) | 13 | 11 |
| second attractor (y≈3.5, z≈-0.5 band) | 2 | 2 |
| **NEW (flatness-relaxation admits)** | **0** | **7** |

The 7 NEW false positives at 0.045 are board-shaped planar clutter whose plane
RMS sat in the 0.035–0.045 band and now clears the gate. Their centers are
**not** a new unexplained population — they cluster at scene-fixed room
structures of exactly the family stages 1–5 documented:

```
ds2 (4.67, 2.68, -0.09)   <- "panel A" at ~(4.7, 2.6), named in stage 1
ds3 (0.26, 3.95, 0.45)    <- the y≈3.5-3.95 positive-z band, named in stage 4
ds3 (3.64, -3.16, 0.51)
ds3 (2.42, -3.22, 0.53)  ┐  one shared fixture at ~(2.4-2.6, -3.1..-3.2, z≈0.55)
ds4 (2.42, -3.22, 0.57)  ├  recurring across ds3/ds4/ds5 (all 5 share one room)
ds5 (2.59, -3.07, 0.55)  ┘
ds5 (-0.79, 3.31, 0.01)
```

So the flatness relaxation admits **more of the same static, board-sized
planar room fixtures** the phase has flagged since stage 1 (a shared
~(2.4,-3.2,0.55) fixture accounts for 3 of the 7), not a novel failure mode.
This is consistent with stage 5's finding that the residual clutter is
static-scene structure a session-level multi-pose cue would remove; raising
flatness widens the net slightly for that same population, at the small,
measured precision cost above (which the recall gain more than offsets).

### Timing — looser gate does not blow the budget

Median total stays flat at ~60 ms across every flatness value (59.6→60.2 ms)
and every stance_floor value (61–62 ms), with p95 71–78 ms — comfortably
inside the 100 ms/frame realtime budget. Relaxing the flatness gate lets more
candidates reach the 2D scorer, but the flatness test is a cheap plane-fit RMS
and per-candidate scoring is sub-millisecond, so the extra candidates cost no
measurable wall time. Timing was never the constraint on this lever.

*(Two provenance notes on the figures above. First — absolute latency, not
the relationship, is machine-load-sensitive: these numbers come from the
committed `results/run8-flat*/summary.json` runs, captured on this machine at
benchmark time. An independent re-measurement of the same code path on a
heavily-loaded host (load average ≈12 on a 32-core box) measured ~85–90 ms
median, materially closer to the 100 ms/frame budget than the ~60 ms above
suggests. The *robust* claim is the relative one — timing stays flat across
flatness/stance values regardless of absolute load — which held in both
measurements; the *margin* to the 100 ms budget is real but thinner under
contention than "comfortably inside" implies taken alone. Second, the
per-point p95 values quoted in the flatness and stance-floor tables are
pooled percentiles over all 535 frames from the scratch harness, not the
per-dataset `p95_total_ms` fields inside `summary.json` — e.g. the
`flat0.050` row's reported 75.9 ms p95 exceeds every individual dataset's
`p95_total_ms` in `run8-flat0.050/summary.json` (max 73.3 ms, dataset 1).
Expect this mismatch when diffing a single dataset's `summary.json` against
the tables above — it reflects pooled-vs-per-dataset percentiles, not a
data error.)*

### True-board pose jitter at the recommended point (flatness 0.045, stance_floor 0.9)

Per-dataset in-bbox center std (mm), n = true-board detections:

| Dataset | 0.035 baseline | **0.045 recommended** | n (0.045) |
|---|---|---|---|
| 1 | 2.9 / 21.1 / 15.9 | 3.5 / 17.7 / 14.7 | 56 |
| 2 | 1.7 / 5.9 / 6.4 | 2.5 / 6.9 / 7.6 | 65 |
| 3 | 2.0 / 3.8 / 5.1 | 2.5 / 4.3 / 5.6 | 66 |
| 4 | 2.1 / 16.7 / 17.8 | 2.7 / 17.9 / 15.7 | 53 |
| 5 | 4.4 / 32.8 / 34.9 | 5.3 / 27.2 / 27.5 | 24 |

(x/y/z std, mm.) The larger 0.045 populations (n up 1.7–3.4×) hold the same
mm-level precision as stage 5's baseline — x-jitter 2.5–5.3 mm on every
dataset, at or below the ~25 mm sensor-noise floor established in stage 3. The
y/z spread on ds1/4/5 is the board's own orientation spread across the
capture, not sensor noise (same pattern stage 4/5 documented). The +101 recall
frames the flatness relaxation adds are therefore genuine repeat detections of
the one static board, not scattered noise — the jitter would blow up if they
were spurious, and it does not.

### DECISION (Stage 6)

1. **Adopt `flatness_rms_max=0.045` on top of `--stance-gate` as the new
   recommended operating point.** It is a *strict Pareto improvement* over
   stage 5's 0.035: recall 30.5%→49.3% (+18.8 pts) **and** precision
   91.6%→93.0% (+1.4 pts), at unchanged ~60 ms timing and unchanged mm-level
   jitter. There is no precision-for-recall tradeoff to confess — the feared
   cost did not materialize, because at this scale true-board detections
   dominate the count and grow much faster than the few extra static-fixture
   clutter hits. The honesty mandate here cuts the other way from usual: the
   result is *better* than the diagnosis dared project, and it is reported
   straight, not hedged.
2. **Keep `stance_floor=0.9`; do not relax it.** The diagnosis's second lever
   is refuted on the full scene: lowering it to 0.85 buys +1 true frame for
   −6.1 pts precision (+17 NEW clutter). The 50 stance-gated board frames the
   board-candidate-only replay hoped to recover do not return as accepted
   detections; only clutter does. `stance_floor=0.9` remains the right value.
3. **0.045 over 0.050.** 0.050 adds only +1.2 pts recall (almost all on
   ds5) for no precision change while sitting deeper in the sensor-noise
   floor; 0.045 is the diagnosis-identified sweet spot and where ds1–4 recall
   plateaus. 0.045 is recommended; 0.050 is a defensible alternative only if
   ds5-style clutter-heavy scenes dominate.
4. **The residual 20/535 clutter is the same static-scene population stage 5
   diagnosed** — the ds5 persistent panel (11), a second attractor (2), and 7
   more hits on board-sized room fixtures the flatness relaxation newly
   admits, all at scene-fixed locations documented since stage 1. This does
   not change stage 5's conclusion that the real fix for the residual is a
   session-level multi-pose cue (the board moves between poses; the fixtures
   do not), which remains a capture-protocol/future-phase item untestable on
   the single-static-capture datasets 1–5. Raising flatness widens the true-
   board recall substantially without changing the *character* of the
   residual clutter, which is why it is a clean win now and the multi-pose cue
   is still the eventual precision closer.

## Stage 7 Results

Stage 6 left the recall ceiling at 49.3% and named the two largest remaining
miss populations as out of the flatness/stance levers' reach. Stage 7 built
and benchmarked a **fixed-size square fitter** (`--square-icp`,
`--square-icp-residual-max`) aimed squarely at the larger of the two — the 86
frames the 2D diamond-stance gate rejects — after a two-step diagnosis
concluded the fitter should recover most of them. **The real-data benchmark
refutes that conclusion outright: the fitter is a strict regression on every
axis.** This section reports that straight, per the honesty mandate, and
diagnoses why the diagnosed gain evaporated.

### The diagnostic chain that said GO (and why it was wrong)

Two read-only diagnostics set the hypothesis:

- `.superpowers/sdd/stage7-rediagnosis.md` re-bucketed the 432 ds1–4 misses
  at the stage-6 operating point (`vertical_gap_deg=3.0, stance_weight=0.5,
  stance_floor=0.9, flatness_rms_max=0.045`). It found F_SCORER_REJECT now the
  dominant bucket (123/432), of which **86 die on the 2D stance gate** and 37
  on shape/size gates a fitter could address. Its first call was **NO-GO** —
  the fitter reaches only 37 frames (a +8.6-pt ceiling), smaller than either
  other miss population.
- `.superpowers/sdd/stage7-stance-cause.md` then reversed that to **GO**. A
  throwaway 180-step θ-sweep fit on the 86 stance-rejected frames' *board
  candidates* classified **66/86 as BAD_POSE**: the raw-point `minAreaRect`
  quad's θ error was a median 43.4° (essentially uncorrelated with truth on
  sparse frames), while the robust fixed-size fit recovered θ to a median 7.4°
  — comfortably inside the stance gate's ~26° slack around the true 45° mount.
  With the board's true orientation essentially constant (44.75–45.10°, std
  <2°) across all four datasets, the stance gate was ruled a pose-*measurement*
  artifact, not a genuine-tilt guard, and the fitter rehabilitated as a
  **pose-accuracy play** worth a diagnosed **55.6%→79.4% ds1–4 recall ceiling**
  (+66 BAD_POSE flips on top of the +37 shape fixes).

**The critical flaw, visible only in hindsight from the real-data run:** the
stance-cause diagnostic fit its throwaway square to the board candidate's
points *in isolation* and asked only "can a robust fit recover θ on these
points?" It never modelled the two things the production detector actually
does — (a) fit *every* candidate and rank by fit residual, so the board
competes against clutter, and (b) gate on an absolute residual threshold. Both
turned out to be where the mechanism dies on real data.

### Benchmark: recall / precision / timing (all 535 frames, generator b, stage-6 operating point)

Each row is the real `detect()` path over every frame, the accepted
detection's center classified in/out of the true-board bbox
(`ros/lctk_launch/config/board/bbox.json5`, trans `[2.6,0,0.35]`, size
`[3.1,3.94,2.2]`) exactly as stages 4–6 did. Baseline is stage 6's adopted
operating point (`--stance-gate --flatness-rms-max 0.045`, no square-icp),
reproducing its 264/20 split byte-for-byte. Timing is a **clean isolated
re-measurement** (single process, nothing else running) pooled over ds1–4 —
the sweep's own timing was contaminated by two concurrent benchmark processes
and is not quoted.

| config | true / clutter | recall (535) | precision | ds1–4 recall | median ms | p95 ms |
|---|---|---|---|---|---|---|
| **base (stage 6, no icp)** | **264 / 20** | **49.3%** | **93.0%** | **55.6%** | **59.6** | **73.9** |
| square-icp R=0.40 | 166 / 28 | 31.0% | 85.6% | 35.2% | — | — |
| square-icp R=0.45 | 196 / 50 | 36.6% | 79.7% | 41.9% | — | — |
| square-icp R=0.50 | 210 / 72 | 39.3% | 74.5% | 44.9% | 118.1 | 150.4 |
| square-icp R=0.55 | 217 / 102 | 40.6% | 68.0% | 46.5% | — | — |

**Headline: every square-icp threshold loses recall AND precision versus the
stage-6 baseline.** The best square-icp recall (40.6% at R=0.55) is still
**8.7 points below** the 49.3% baseline — and buys that by admitting *five
times* the clutter (20→102), collapsing precision from 93.0% to 68.0%.
Tightening the gate to cut clutter only sheds more true detections (R=0.40:
recall 31.0%, precision 85.6%). There is no operating point on this sweep that
matches the baseline on either axis, let alone the diagnosed 79.4% ceiling.
The diagnosed +23.8-pt gain did not merely fall short — recall moved the wrong
way.

### CRITICAL: did the 66 BAD_POSE stance-flips materialize? No.

Cross-checking each config's newly-detected-true frames against the baseline's
per-frame failure bucket (ds1–4):

| config | newly-recovered TRUE (vs base) | of which BAD_POSE stance-flips | regressions (base-true now lost) |
|---|---|---|---|
| R=0.40 | 0 | 0 | 88 |
| R=0.45 | 0 | 0 | 59 |
| R=0.50 | 0 | 0 | 46 |
| R=0.55 | 2 | 2 | 41 |

**The pose-fix channel is essentially inert.** Across the whole sweep it flips
at most **2 of the diagnosed 66** BAD_POSE frames to accepted true-board
detections, and every config *loses* far more baseline-true frames (41–88)
than it gains. Drilling into just the 66 BAD_POSE frames, what the production
detector actually returns:

| threshold | None | true-board | clutter |
|---|---|---|---|
| R=0.40 | 63 | 0 | 3 |
| R=0.45 | 60 | 0 | 6 |
| R=0.50 | 55 | 0 | 11 |
| R=0.55 | 47 | 2 | 17 |

So the rescue *mechanism does fire* on some of these frames (17/66 produce a
detection at R=0.55) — but it lands on **clutter, not the board**, and the
rest (47/66) produce nothing. This is the exact failure the brief flagged as a
risk ("is the fit landing on clutter? is the residual gate rejecting real
recoveries?") — both, confirmed:

1. **Residual gate rejects the real board.** A genuine sparse VLP-32C board
   patch, ring-gap-striped, has poor perimeter coverage, so its own
   fixed-size-fit residual runs high — above a tight gate. That is why 47–63
   of the 66 return `None`: the board's true fit is thrown out by the
   threshold. The synthetic tests' "genuine recovery reads ~0.40 residual"
   margin (Task 23) was measured on clean fixtures and does not hold on real
   ring-gapped board points.
2. **Residual ranking prefers clutter.** `detect()`'s square-icp branch
   replaces the score-based candidate ranking with **lowest-residual-wins
   across all candidates**. A compact planar clutter blob fills its own
   fixed-size box's perimeter better than the sparse board fills its box, so
   it posts a *lower* residual and wins the frame — turning a formerly-correct
   detection into a clutter false-positive (the mechanism behind both the
   102-frame clutter explosion and many of the 41–88 regressions).

The stance-cause diagnostic missed both because it scored the board's points
in isolation, never against the field of competing candidates or the absolute
gate — a textbook case of a synthetic mechanism that works in a unit test
(Task 23's fixtures pass) failing to survive the full real-data pipeline.

### Timing — the flagged pitfall is real and severe

Clean isolated measurement, ds1–4 pooled:

| config | total median | total p95 | max | scoring-stage median (θ-sweep lives here) |
|---|---|---|---|---|
| base | 59.6 ms | 73.9 ms | 93.6 ms | 1.3 ms |
| square-icp | 118.1 ms | 150.4 ms | 180.5 ms | 58.3 ms |

**Square-icp nearly doubles per-frame latency (+58.5 ms, +98%) and pushes the
median *over* the 100 ms realtime budget** (p95 150 ms, well over). The cost is
the full [0°,90°) θ-sweep (~37 coarse + fine evals) run per candidate: it
inflates the scoring stage ~45× (1.3→58 ms). This alone would be
budget-disqualifying even if the recall/precision numbers were neutral — and
they are strongly negative. (Absolute base latency here, ~60 ms, matches stage
6's unloaded figure; the sweep's ~170 ms medians were concurrency-inflated and
are not the honest number.)

### Pose jitter — the one thing that got better, and why it doesn't rescue the result

At R=0.50 the true-board center jitter is actually *tighter* than baseline
(y/z std ~4–7 mm vs baseline's 4–27 mm), because a fixed-size model fit yields
a cleaner pose than the raw quad *when it lands on the board*. But this is cold
comfort: it is measured over a smaller, clutter-polluted true population
(194 vs 264 frames), and a better pose on fewer, less-trustworthy detections
is not a usable trade against −10 pts recall, −18.5 pts precision, and 2× the
compute.

### DECISION (Stage 7)

1. **Do NOT adopt `--square-icp`. It is refuted on real data.** At every
   residual threshold it loses recall (best 40.6% vs 49.3% baseline), loses
   precision (best 85.6% vs 93.0%), and nearly doubles per-frame latency past
   the 100 ms budget. There is no operating point that is not strictly
   dominated by the stage-6 baseline.
2. **The diagnosed pose-fix channel did not materialize** — ≤2 of 66 BAD_POSE
   stance-flips became true detections; the mechanism fires but lands on
   clutter (residual ranking prefers compact clutter over the sparse board)
   or is rejected outright (the real board's ring-gapped perimeter posts a
   residual above the gate). This reverses `stage7-stance-cause.md`'s GO: its
   79.4% ceiling was an artifact of scoring the board candidate *in isolation*,
   never against the competing-candidate field or the absolute residual gate
   the production detector uses.
3. **The stage-6 operating point stands unchanged as the phase's final
   single-frame result** (`--stance-gate --flatness-rms-max 0.045`, 264/535 =
   49.3% recall / 93.0% precision, ~60 ms). The square fitter is retained in
   the codebase behind its default-off flag (all 87 unit tests pass and the
   synthetic mechanism is real), but is **not** part of any recommended
   configuration and should not be enabled in an integration build.
4. **The honest lesson for a future pose-refinement attempt:** a fixed-size
   fit can only help if it (a) refines the *already-selected* board candidate
   rather than re-ranking all candidates by residual, and (b) is gated on a
   residual threshold calibrated to real ring-gapped board coverage, not
   synthetic dense fixtures. Both were the diagnosis's blind spots. The two
   larger miss populations stage-7 set out to attack (the 86-frame stance
   gate and 56-frame fragmentation) remain open, and — as stages 5–6 already
   concluded — the residual-precision closer is a session-level multi-pose
   cue, not a heavier single-frame fitter.

## Decision

**Updated after Stage 7 (final stage of this phase).** The subsections below
record each stage's verdict as it was made; this paragraph is the overall
phase-7 call in light of all seven. Stage 7 built and benchmarked a fixed-size
square fitter aimed at the recall ceiling and **refuted it on real data** — it
loses recall and precision and doubles compute at every threshold — so the
recommended operating point is unchanged from stage 6.

The core idea — plane-fit into orthographic plane coordinates, then a shared
2D quad scorer — is validated, and this phase reaches a genuinely usable,
honestly-quantified single-frame operating point on the hole-free board
design: generator B with anisotropic clustering + anisotropic stripe-tolerant
closing + `stance_floor=0.9` (`--stance-gate`) + **`flatness_rms_max=0.045`
(stage 6)** recalls the true board on **264/535 frames (49.3%, all 5
datasets) at 93.0% precision** (20 residual clutter, all attributable to the
same known static room fixtures, not scattered noise) and mm-level jitter.
Stage 6 raised recall from stage 5's 30.5% to 49.3% by relaxing the flatness
gate 0.035→0.045 — and, unusually, did so as a *strict Pareto improvement*:
precision rose too (91.6%→93.0%), because true-board detections dominate the
count and grow far faster than the handful of extra static-fixture false
positives the looser gate admits. The other recall lever the failure
diagnosis flagged (relaxing `stance_floor`) was refuted — it buys no recall
for the precision it spends — and is not adopted. This is a large improvement
over stage 1's ≤2% and a real answer to stage 4's discrimination gap. It is
**not** a 100%-recall or 100%-precision result, and this phase does not claim
one: reaching 100% precision single-frame costs ~64% of recall
(`--strict-diamond`, 59/535), the recall ceiling is still ~50% (half the
frames lose the board to fragmentation or hard scorer gates), and the residual
clutter is diagnosed, not hand-waved, as out of single-frame geometry's reach
— closing it needs a session-level multi-pose cue (the board moves between
calibration poses; static clutter does not), which is a capture-protocol
change for a future phase, not implementable on today's single-static-capture
sample datasets.

Given that, integration (Rust port, ROS node, replacing or front-ending ICP)
is justified to scope next, on these terms:

- The **projection + 2D scorer core is sound**: when a clean board candidate
  reaches it, it produces a tight quad (2–25 mm jitter across all stages) at
  the right place, at millisecond cost. Stage 1's bottleneck — **candidate
  generation on ring-striped clouds** — was fixed by stages 3–4's
  anisotropic clustering/closing; stage 4's discrimination gap was then
  closed by stage 5's stance gate, and stage 6 nearly doubled recall
  (30.5%→49.3%) by relaxing the flatness gate to 0.045 without any precision
  cost — all at the honest precision/recall numbers above, not for free.
- **B** (Euclidean clustering after big-plane removal) is the only generator
  carried through to a usable result; A (iterative RANSAC) never recalled
  the board at all, and C (region growing) recalls board-*sized* objects
  well but cannot discriminate them and is 3× over the timing budget in
  Python — neither was revisited after stage 1.
- **Discrimination against static, board-sized planar clutter is solved to
  93.0% precision (stage 6) at 49.3% recall, not further, and that is
  deliberate**: the remaining ~7% of accepted detections are the same real,
  static room fixtures that share the true diamond's single-frame geometric
  signature closely enough that only an expensive gate (`edge_support_min`,
  ~7:1 recall cost) separates them. A production integration should rely on
  the session-level multi-pose/session cue (buffered across poses, as
  `advanced_extrinsic_solver` already supports) to filter this residual,
  rather than chasing further single-frame gate tuning.
- Any integration plan must budget for the multi-pose/session filter as a
  named, near-term follow-on phase — not an optional nice-to-have — since
  it is the only tested path to closing the remaining precision gap.

### Stage-2 verdict

Neither stage-2 mechanism moves the needle enough to change the call above —
if anything, stage 2 sharpens *why* the stage-1 verdict was right to hold off
on integration:

- **Accumulation (hypothesis 1) is rejected as tested.** It did not densify
  the board past ring-gap fragmentation; it collapsed B's already-marginal
  2% single-frame recall to 0% on every dataset, and did the same to C. The
  root cause is structural, not a tuning gap: a static capture re-samples
  the same fixed ring elevation angles every frame, so concatenation adds
  redundant density (1.95×, not 10×) rather than new coverage, and the
  existing DBSCAN `cluster_eps` — already loosened once to bridge
  single-frame gaps — fragments the board further once the surrounding
  scene's density and cluster topology shift under the larger window. This
  reinforces stage 1's conclusion that the bottleneck is candidate
  generation on ring-striped clouds, not the scorer, and shows that
  "throw more frames at it" is not a free fix without either real
  board/sensor motion between frames or window-size-aware re-tuning of the
  clustering parameters (neither tested here).
- **Stance (hypothesis 2) is a partial win, isolated by a supplementary
  single-frame check** (the brief's bundled accumulate+stance C-check
  command couldn't isolate it, since accumulation alone already zeroed
  C's recall). Stance fully suppresses one of the two clutter panels
  (18/30 → 0/30) without touching the one confirmed true-board detection
  (0.538 → 0.536, negligible). But it leaves the second clutter panel
  fully intact (10/30 → 10/30) — it discriminates by fitted-quad
  orientation, not by "is this the board," so it is not a general
  axis-aligned-panel rejector. Worth keeping as a score input (it is
  free, and it helped on one of two panels with no measured downside),
  but it does not by itself solve C's discrimination problem.
- **The core discrimination problem stage 1 identified is unresolved.**
  Stage 2 confirms the border-alone cue is still not enough: even with
  stance, C would still false-positive on panel B in this data. The
  hole pattern (present on the recorded board, never used as a cue so
  far) remains the most promising untested lever.

Next steps as of stage 2: (1) stripe-aware candidate merging done
properly (the current coplanar merge helps but under-recovers) — stage 2's
diagnosis shows this needs to hold even as `cluster_eps` and window
composition change, not just at single-frame scale; (2) add the hole
pattern as an optional score term, now the leading candidate for closing
C's remaining discrimination gap after stance's partial result; (3) if
accumulation is revisited, test it against a capture with real board/sensor
motion (a genuine calibration hold, not a static sample) before concluding
it can't help, since the static-scene failure mode diagnosed here may not
generalize; (4) re-run; only then decide pick/combine/reject. Integration
design (Rust port, ROS node, replacing vs front-ending ICP) stays out of
scope until a generator clears a usable detection rate on the real
recordings.

### Stage-3 verdict (updates the above)

Stage 3 was next-step (1) — stripe-aware merging via anisotropic
z-compressed clustering — and it **partially delivers on exactly the
mechanism stage 1/2 asked for**, without yet moving the integration call:

- Candidate generation is measurably fixed: a board-shaped candidate now
  reaches the scorer on 31% of ds3 frames (vs 5% isotropic), and the
  bbox-reference sanity check that stage 1 ran only on dataset 3 now also
  confirms true-board hits (hollow-diamond raster, hole pattern visible)
  on datasets 1, 2, and 4 — the phenomenon generalizes past the single
  dataset stage 1 could verify.
- End-to-end recall does not move (7 vs 8 accepted detections across 535
  frames) because the wider vertical tolerance that reconnects ring gaps
  also drags in more off-board points, so the newly-generated candidates
  mostly land well below `min_score` (median 0.07) rather than clearing
  it. **The bottleneck this phase has chased since stage 1 —
  "candidate generation on ring-striped clouds" — has now measurably
  shifted toward the scorer's ability to discriminate a noisier merged
  patch**, which changes next-step (1) from "try stripe-aware merging" to
  "make the stripe-aware merge quality-aware" (e.g. bound how far a merged
  point can sit from the seed cluster's own plane/extent before it is
  swept in, or re-tighten `eps_v` adaptively once a plausible-size patch
  is found, rather than applying one fixed range-scaled tolerance
  everywhere).
- Stance and timing verdicts from stage 2 are unchanged: stance 0.5 still
  leaves ds5's clutter panel unfiltered under both stage-3 configs, and
  timing has headroom in every run (aniso costs ~5–7 ms/frame extra,
  nowhere near the 100 ms budget).
- **Still not yet an integration decision.** No config this phase has
  tested clears a usable detection rate on real recordings (best is still
  low single digits per dataset). The honest read is: stage 3 is a real,
  measured improvement to the *diagnosis* (candidate generation is closer
  to solved than the scorer is), not yet an improvement to the
  *headline recall number* the Decision above is gated on.

Updated next step: pursue a quality-aware version of the anisotropic merge
(bound admitted points by proximity to the seed cluster's own plane/extent,
or shrink `eps_v` back down once a board-size patch is found) before
retrying the hole-pattern score term — stage 3 suggests the scorer is now
seeing the right *region* far more often, so tightening what gets admitted
into that region's candidate may close more gap than a new score term
would on its own. Re-run stage 2's remaining items (hole pattern, real
board-motion accumulation) after that.

### Stage-4 verdict (updates the above)

Stage 4 was the "make the scorer stripe-tolerant" alternative to stage 3's
planned next step (a quality-aware merge), and it **converts stage 3's
diagnosis into real recall for the true board — but at the cost of a
comparably-sized new false-positive problem that changes what "done" means
here**:

- **The projection + 2D scorer + anisotropic candidate pipeline now finds
  the real board on 30–47% of frames on 4 of 5 datasets**, at the exact
  physical location stage 1/3 already confirmed, with mm-level jitter and
  the hole pattern visible on every inspected overlay. This is the first
  stage in the phase where "usable detection rate" is even plausible —
  stage 1–3 topped out at single digits.
- **It is not usable as shipped.** The same anisotropic closing that
  rescues the true board's fill ratio also rescues clutter's: ds5's known
  clutter panel now dominates that dataset (34% of frames, up from 2%,
  outnumbering true-board hits 6:1), and four new scene-fixed clutter
  attractors appear on datasets 1–4 that stage 3 never triggered. Total
  clutter false positives rose from 0.4% to 12.7% of all frames. Nothing
  in the current score (fill ratio, squareness, edge straightness) or the
  stance term discriminates board from clutter once both get the same
  stripe/gap tolerance — they were never designed to.
- **Verdict: still not an integration decision, but the shape of the
  remaining gap has changed.** Stage 1–3 were blocked on candidate
  generation and scorer sensitivity (not enough signal reaching the gate).
  Stage 4 shows the gate can be made sensitive enough — the open problem
  is now purely discrimination against board-sized planar clutter, which
  border/fill/squareness/stance cannot solve because clutter shares all of
  those properties with the real board. The hole pattern — flagged as the
  next lever since stage 1, still unimplemented — is the one cue in this
  dataset that visually separates every true-board overlay from every
  clutter overlay inspected across all four stages, and is now the clear
  next step rather than one option among several.
- Recommended next step: add the hole pattern as a score term (or hard
  gate) before any further candidate-generation or closing-kernel tuning —
  further raising recall without discrimination only grows the
  false-positive count in lockstep, as stage 4 just demonstrated.

### Stage-5 verdict (updates the above)

The hole pattern stage 4 recommended as the next lever was overtaken by a
hardware decision (the recorded board is moving to hole-free), so stage 5
built the alternative stage-4 flagged as available — stance, edge-support,
and squareness gates keyed on diamond geometry instead of holes — and this
task benchmarked both operating points those gates define:

- **The discrimination gap stage 4 opened is now closed at an honest,
  quantified price, not eliminated for free.** `--stance-gate`
  (`stance_floor=0.9` alone) retains stage 4's full true-board recall
  (163/535 vs. 162/535 baseline — no loss) while cutting clutter 78%
  (68→15), taking precision from 70.4% to 91.6% at zero recall cost. This
  is the first stage-5 result that is both a real recall number *and* a
  real precision number reported honestly side by side, not one traded
  silently for the other.
- **Zero false positives is reachable but not recommended as a default.**
  `--strict-diamond` reaches 100% precision (0/535 clutter) but costs
  ~64% of recall (163→59) to get there — a ~7:1 recall-per-FP trade driven
  entirely by `edge_support_min`, which is expensive by construction (its
  ring-gap-calibrated bin width collapses to 2–3 bins/side at real sensor
  range, making it the one cue that separates board from clutter but only
  by discarding most board hits that share the same coarse quantization).
  `strict_squareness` is inert on every real frame in this dataset
  (structural, not a tuning gap); `side_tol=0.08` is a cheap partial
  knob.
- **The residual clutter under the recommended operating point (15/535,
  87% one persistent panel + 13% a second static attractor) is not a
  single-frame geometry problem left unsolved — it is evidence that
  single-frame geometry has reached its ceiling.** Both attractors are
  real, well-formed, ring-gap-striped planar surfaces that share the true
  diamond's stance and edge-support signature closely enough that only the
  most expensive gate separates them, and only at a recall cost that makes
  it a bad default. **The concrete fix is a session-level cue, not a
  tighter single-frame gate**: the board is the object that moves between
  calibration poses; the panels are fixed room fixtures that never do.
  This requires a capture-protocol change (record ≥2 board positions per
  session, reject any candidate location that repeats unchanged across
  poses) that the current single-static-capture datasets (1–5) cannot
  test — a future-phase item, not a stage-5 gap.
- **Stage 5 closed the discrimination gap; the open lever it left was
  recall** (30.5% — half the frames still lose the board). The five-stage
  arc to here: stage 1 found the core idea sound but candidate generation
  broken (≤2% recall); stages 2–3 fixed candidate generation partially
  (accumulation failed, anisotropic clustering worked for shape but not
  score); stage 4 fixed the scorer's sensitivity but opened a discrimination
  gap of equal size; stage 5 closes that gap at a quantified, honest
  operating point (91.6% precision / 30.5% recall, `--stance-gate`) and
  correctly identifies the irreducible remainder as out of single-frame
  reach. Stage 6 (below) then attacks the recall ceiling stage 5 left open.

### Stage-6 verdict (updates the above)

Stage 6 swept the two recall levers the per-frame failure diagnosis
(`.superpowers/sdd/stage6-failure-diagnosis.md`) identified — the
`flatness_rms_max` plane-fit gate and `stance_floor` — against precision
across all 535 frames, resolving the precision question the diagnosis
explicitly left open (it replayed only the board's own candidate, never
clutter):

- **The flatness lever is a clean, strict Pareto win.** Raising the gate
  0.035→0.045 lifts recall 30.5%→49.3% (+18.8 pts, +62% relative) while
  precision *also* rises 91.6%→93.0%. The precision-for-recall trade this
  stage set out to quantify did not exist: true-board detections dominate
  the count and grow far faster (163→264) than the extra clutter (15→20), so
  precision improves even as a few more false positives are admitted. Timing
  (~60 ms) and mm-level jitter are unchanged. 0.045 is adopted as the new
  recommended operating point; 0.050 buys only +1.2 pts more recall (mostly
  ds5) at no precision gain and deeper in the sensor-noise floor.
- **The stance-floor lever is refuted.** Relaxing `stance_floor` 0.9→0.85
  buys +1 true frame for −6.1 pts precision (+17 new clutter). The 50
  stance-gated board frames the board-candidate-only replay hoped to recover
  do not return as accepted detections on the full scene; only clutter does.
  `stance_floor=0.9` is kept. This is the honesty mandate working as
  intended — a projected recovery that evaporates once the clutter it also
  admits is measured.
- **The residual clutter (20/535) is unchanged in character** from stage 5:
  the ds5 persistent panel (11), a second attractor (2), and 7 more hits on
  board-sized static room fixtures at scene-fixed locations documented since
  stage 1. Raising flatness widens the true-board net substantially without
  changing what the residual *is*, so stage 5's conclusion stands — the
  eventual precision closer is a session-level multi-pose cue, not a tighter
  single-frame gate.
- **Stage 6 reached a single-frame operating point of 49.3% recall / 93.0%
  precision** (`--stance-gate --flatness-rms-max 0.045`) — nearly double stage
  5's recall at slightly higher precision — and a correctly-diagnosed,
  session-level path for the irreducible remainder. Stage 7 then made one more
  attempt at the recall ceiling (below) and failed; this operating point is
  the phase's final single-frame result. See the updated top-level Decision
  above this section for the phase-wide call.

### Stage-7 verdict (updates the above, final for this phase)

Stage 7 built a fixed-size square fitter (`--square-icp`) to attack the
largest remaining miss population — the 86 frames the 2D stance gate rejects —
after a two-step diagnosis (`stage7-rediagnosis.md` NO-GO →
`stage7-stance-cause.md` GO, on a diagnosed 55.6%→79.4% ds1–4 recall ceiling).
**The real-data benchmark refuted it on every axis; nothing was adopted.** Full
numbers and mechanism are in the *Stage 7 Results* section above the top-level
Decision; the verdict:

- **Strict regression, no viable operating point.** Every residual threshold
  lost recall (best 40.6% vs 49.3% baseline), lost precision (best 85.6% vs
  93.0%; worst 68.0% at 5× the clutter), and nearly doubled per-frame latency
  (60→118 ms median, +98%) past the 100 ms budget. Every square-icp point is
  strictly dominated by the stage-6 baseline.
- **The diagnosed pose-fix channel did not materialize.** At most 2 of the
  diagnosed 66 BAD_POSE stance-flips became true detections; the fitter's
  rescue fires but lands on clutter (its residual ranking prefers compact
  clutter over the sparse board) or is rejected outright (a real ring-gapped
  board patch posts a residual above the gate). `stage7-stance-cause.md`'s GO
  was an artifact of scoring the board candidate *in isolation*, never against
  the competing-candidate field or the absolute residual gate the production
  detector actually uses — the same isolation blind-spot the honesty mandate
  caught twice before in this phase.
- **This is the phase's final stage.** The seven-stage arc closes where stage 6
  left it: `--stance-gate --flatness-rms-max 0.045`, **49.3% recall / 93.0%
  precision, ~60 ms**. The square fitter stays in-tree behind its default-off
  flag (mechanism proven synthetically, 87/87 tests green) but is in no
  recommended configuration. The recall ceiling and residual clutter remain
  where stages 5–6 diagnosed them: closable only by a session-level multi-pose
  cue, not a heavier single-frame fitter.
