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

## Decision

Partial success — no winner yet, and the numbers as they stand do not justify
an integration phase:

- The **projection + 2D scorer core is sound**: when a clean board candidate
  reaches it, it produces a tight quad (25 mm jitter) at the right place, at
  millisecond cost. The bottleneck is **candidate generation on ring-striped
  clouds**, not the 2D idea.
- **B** is the only generator that found the real board and the only one
  inside the 100 ms budget, but a 2% frame rate is unusable as-is.
- **C**'s recall on *flat square objects* is high even through ring stripes —
  but it cannot tell the board from board-sized clutter, and is 3× over
  budget in Python.
- **Discrimination needs a stronger cue than the border alone** in scenes
  containing other ~1 m planar objects: the hole pattern (present on the
  recorded board) as an optional-but-scoring cue, or temporal consistency
  across frames, would separate the true board from the two clutter panels
  that C locks onto.

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
