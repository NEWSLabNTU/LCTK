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

| Dataset | A: iterative RANSAC | B: cluster | C: region growing (30-frame cap) |
|---------|--------------------|------------|-------------------|
| 1 | 1% | 1% | 83% |
| 2 | 0% | 0% | 53% |
| 3 | 1% | 2% | 93% |
| 4 | 1% | 1% | 87% |
| 5 | 1% | 8% | 90% |

### Timing (median ms per frame, dataset 3; p95 in parentheses)

| Stage | A | B | C |
|-------|---|---|---|
| downsample (0.03 voxel) | 3.2 | 3.2 | 3.0 |
| candidate generation | 104.7 | 77.5 | 285.2 |
| 2D scoring | 3.9 | 1.1 | 5.0 |
| total | 111.6 (119.7) | 82.4 (91.8) | 293.0 (306.9) |

Timing is stable across datasets (A 109–112 ms, B 82–86 ms, C 293–301 ms
median). **B fits the ~100 ms realtime budget; A is borderline; C is 3×
over** — and C's cost is a pure-Python BFS, so a compiled port would change
its constant dramatically.

### Pose jitter (std of detected center [mm] / normal [deg])

| Dataset | A | B | C |
|---------|---|---|---|
| 1 | — | — | 1468 / 1.4 |
| 2 | — | — | 977 / 6.4 |
| 3 | — | 25 / 0.0 | 1411 / 9.5 |
| 4 | — | — | 1468 / 2.7 |
| 5 | — | 862 / 6.5 | 2090 / 3.4 |

Jitter is only defined where ≥ 2 frames detected. The metre-scale center
jitter is itself a finding: those "detections" are **not the same object from
frame to frame** (see below). B on dataset 3 — the one case verified to be the
real board — jitters by 25 mm, in line with the sensor noise floor.

### What the overlays show (narrative)

- **B's rare detections are the real board.** On dataset 3 every B detection
  sits at the `bbox.json5` reference location (~2.1 m ahead), the raster
  clearly shows the hollow-diamond outline, and center jitter is 25 mm. This
  is the honest headline: **crop-box-free detection of the true board was
  achieved, but only on ~2% of frames.**
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

Next steps if the phase continues: (1) stripe-aware candidate merging done
properly (the current coplanar merge helps but under-recovers); (2) add the
hole pattern as an optional score term; (3) re-run; only then decide
pick/combine/reject. Integration design (Rust port, ROS node, replacing vs
front-ending ICP) stays out of scope until a generator clears a usable
detection rate on the real recordings.
