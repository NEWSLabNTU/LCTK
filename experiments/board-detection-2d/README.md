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
- `src/boarddet/candidates/background_diff.py` — **E**: background / motion
  subtraction. Diffs each frame against a consensus voxel-occupancy
  background, then reuses B's clustering. Validated leave-one-out across the
  five sample datasets at 88.4% recall / 100% precision — see
  [`docs/roadmap/side-track_method-e-background-subtraction.md`](../../docs/roadmap/side-track_method-e-background-subtraction.md).

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

### Method E: leave-one-out cross-dataset validation

```bash
uv run python -m boarddet.benchmark_e_loo \
  --side 1.0 --stance-gate --flatness-rms-max 0.045 \
  --min-sources 3 --isolation --isolation-max-density 0.3 \
  --out results/methodE-ms3-iso
```

Holds out each dataset in turn and builds its background from the other four.
`--min-sources` is the consensus threshold and is load-bearing: 1 (a plain
union) scores 0/535 on this data. See the results doc linked above.

### Recorded bags (two-LiDAR: VLP-32C + solid-state Falcon)

The `TWO_LIDAR_*` bags are gitignored — see
[`ros/lctk_sample_data/bags/README.md`](../../ros/lctk_sample_data/bags/README.md).
Export them once (needs ROS, runs outside this venv):

```bash
source /opt/ros/humble/setup.bash
python3 tools/export_bag_npz.py
```

Then benchmark either sensor. The bag rig has its own reference box per
sensor frame, so `--bbox` is required. The four bags are only two board
positions (`TWO_LIDAR_1`/`2` share one, `3`/`4` the other), so the commands
below pick one bag per position (`TWO_LIDAR_1` and `TWO_LIDAR_3`) for a
clean 2-fold LOO at `--min-sources 1`; the headline results merge each pair
to use all frames — see the results doc. Key flags differ by sensor:

```bash
# VLP-32C: spinning, board at ~9 m -> anisotropic clustering tuned down,
# DBSCAN density lowered so the sparse far-board corners survive
uv run python -m boarddet.benchmark_e_loo --source bag --sensor vlp32 \
  --names TWO_LIDAR_1 TWO_LIDAR_3 --min-sources 1 \
  --side 1.0 --stance-gate --flatness-rms-max 0.045 --vertical-gap-deg 1.0 \
  --cluster-min-points 20 \
  --bbox ../../sessions/twolidar-vlp32-falcon/bbox_vlp32.json5 \
  --out results/bagE-vlp32

# Falcon: solid-state, no rings -> anisotropic clustering OFF; z-forward
# frame -> gravity along +y; fixed-square fitter pins the side length
uv run python -m boarddet.benchmark_e_loo --source bag --sensor falcon \
  --names TWO_LIDAR_1 TWO_LIDAR_3 --min-sources 1 \
  --side 1.0 --stance-gate --flatness-rms-max 0.045 --vertical-gap-deg 0 \
  --square-icp --up-axis 0 1 0 \
  --bbox ../../sessions/twolidar-vlp32-falcon/bbox_falcon.json5 \
  --out results/bagE-falcon
```

Key flags, why they differ by sensor:

- `--vertical-gap-deg` bridges a spinning LiDAR's ring gaps; the Falcon has
  none, so it is off (0), and the pcap default (3.0) must be lowered for the
  VLP's far board.
- `--cluster-min-points` is the foreground DBSCAN core-point density
  (default 30). The VLP board at ~9 m is sampled so sparsely that its corner
  points otherwise drop out as noise and truncate the fitted quad; 20 keeps
  them (recall 79.9% -> 91.2%, precision unchanged).
- `--up-axis` is world-up in the sensor frame for the stance gate. It is
  `0 0 1` for the z-up rigs (default) but `0 1 0` for the z-forward Falcon
  frame — with the wrong axis the stance gate rejects every upright board.
- `--square-icp` pins the board side to `--side` and fits only pose, fixing
  the `minAreaRect` oversize that sinks a dense Falcon board's score
  (recall 92.7% -> 100%); it also re-activates the stance gate, which is why
  the Falcon command must pair it with the correct `--up-axis`.

Add `--save-overlays N` to write N 6-panel Method E overlays per fold into
`--out`.

## Tests

```bash
uv run pytest
```

Covers geometry/scorer/pose primitives, all three candidate generators
against synthetic scenes (`src/boarddet/synth.py`), ingest, and the
benchmark harness itself.

`tests/test_realdata_recall.py` is marked `realdata`: it runs the real LOO
benchmark (`benchmark_e_loo.run_loo`) on the cached sample pcaps at the
`presets.production_config()` operating point and asserts per-fold, pooled,
and precision recall floors -- the only automated guard on the Method E
88.4%-recall headline. It dominates the suite's wall-time (~80s of the
~110s total on this machine) because it decodes real pcap frames instead of
synthetic scenes. Skip it for fast local iteration:

```bash
uv run pytest -m "not realdata"
```

It skips itself (with an explicit reason) when the sample pcaps
(`ros/lctk_sample_data/data/{1..5}/lidar.pcap`) or `velodyne_decoder` are
absent; run the full suite including it before finalizing a change to the
Method E pipeline.
