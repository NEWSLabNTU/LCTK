# boarddet — Phase 7 crop-box-free board detection experiment

Standalone `uv` project (no ROS) exploring whether the LCTK calibration board
can be located in a raw VLP-32C point cloud **without** the manual crop box
(`bbox.json5`) the current `rust/hollow-board-detector` pipeline requires.

The approach: fit a plane to candidate points, project them orthographically
into plane coordinates (distortion-free, metric), rasterize an occupancy
image, and find the board by its square border with OpenCV-style contour /
quad fitting. Three candidate-generation strategies are compared head to
head against the same 2D scorer:

- `src/boarddet/candidates/ransac_iterative.py` — **A**: iterative RANSAC
  multi-plane extraction (velo2cam style).
- `src/boarddet/candidates/cluster_after_ground.py` — **B**: strip large
  planes (ground/walls), then Euclidean-cluster the remainder.
- `src/boarddet/candidates/region_growing.py` — **C**: normal-based region
  growing.

Full design, motivation, and results narrative: see
[`docs/roadmap/phase-7-projection-board-detection.md`](../../docs/roadmap/phase-7-projection-board-detection.md)
in the repo root.

## Running the benchmark

Frames are ingested from the sample-data pcaps
(`ros/lctk_sample_data/data/{1..5}/lidar.pcap`) via `velodyne_decoder` and
cached to `.npz` so repeat runs skip pcap decoding.

```bash
cd experiments/board-detection-2d

# all generators, all 5 datasets, full frames
uv run python -m boarddet.benchmark \
  --datasets 1 2 3 4 5 --generators a b c --side 1.0 --out results/my_run

# single generator, single dataset (fast iteration)
uv run python -m boarddet.benchmark \
  --datasets 3 --generators b --side 1.0 --out results/scratch
```

Output lands in `results/<name>/`: `summary.md` + `summary.json` (detection
rate, per-stage timing, pose jitter per dataset/generator) plus a handful of
overlay PNGs (cloud + fitted quad + raster image) per dataset for visual
spot-checking. `results/` is gitignored — re-run to reproduce.

## Tests

```bash
uv run pytest
```

Covers geometry/scorer/pose primitives, all three candidate generators
against synthetic scenes (`src/boarddet/synth.py`), ingest, and the
benchmark harness itself.
