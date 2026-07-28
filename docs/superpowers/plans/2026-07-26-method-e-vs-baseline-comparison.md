# Method E vs. No-Method-E Comparison — All Datasets — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure board-detection **recall, precision, and execution time** *with* Method E (background subtraction) vs. *without* it, across **every** dataset in the repo — the five pcap sample datasets (VLP-32C, near board) **and** the recorded TWO_LIDAR bags on **both** sensors (VLP-32C far board, Seyond Falcon solid-state) — each run with the arguments appropriate to that sensor/range, and with all generated overlay images preserved.

**Architecture:** The with-Method-E side is already produced by `benchmark_e_loo.py` (leave-one-out cross-dataset background) which natively classifies each accepted detection's centre against a per-rig `bbox.json5` into recall/precision/timing. The without-Method-E side is generator **B** (the same clustering + same 2D scorer Method E reuses, minus the background-subtraction stage), run per-frame. No existing harness runs generator B over bags with the per-sensor knobs and emits recall/precision — `benchmark.py` is pcap-only and reports detection-rate only. So Task 2 adds a dedicated no-E runner `boarddet.benchmark_noe` that mirrors `benchmark_e_loo`'s CLI and output schema exactly (minus `--min-sources`/background), and Task 1 fixes a one-line gap so generator B honours `cluster_min_points` (today only generator E does — without the fix the VLP-bag no-E baseline would silently use the wrong clustering density and the ablation would not be like-for-like). Tasks 3–4 run all scenarios; Task 5 pools everything into one table.

**Tech Stack:** Python 3.11, `uv` project at `experiments/board-detection-2d/`, numpy, opencv, `json5`, pytest. No ROS, no system pip (CLAUDE.md Known Issue 3 — everything stays inside the `uv` venv).

## Global Constraints

- All commands run from `experiments/board-detection-2d/` and are prefixed `uv run` — never system python/pip. Do not run `just build` (this is a standalone `uv` project, not a ROS package).
- Board side length `--side 1.0` on every run.
- Metric definitions identical on both sides: `recall = n_true_board / n_frames`, `precision = n_true_board / n_detections`, where "true board" = detection centre inside the rig's reference box (`BoxRef.contains`). Timing = median of `timings_ms["total"]` per frame.
- **Never delete any generated PNG or `results/` output dir.** `results/` is gitignored, but the overlays are the visual evidence. Use fresh `--out` dirs prefixed `compare-` so nothing pre-existing is overwritten. Every run below passes `--save-overlays 8`.
- Frame caches are all present under `cache/` (datasets 1–5, and `bag_TWO_LIDAR_{1..4}_{vlp32,falcon}`), so no pcap decode and **no ROS bag export** is needed.
- **Per-scenario arguments are load-bearing — do not cross them** (from `experiments/board-detection-2d/README.md` and `docs/roadmap/side-track_method-e-background-subtraction.md`). The three rigs differ by sensor and board range; the table below is the authority. Method E and its no-E counterpart in the same scenario share every gate/tuning flag; the *only* intended difference is background subtraction.

| Scenario | source | names | bbox | vertical-gap-deg | cluster-min-points | up-axis | square-icp | isolation | E min-sources |
|---|---|---|---|---|---|---|---|---|---|
| **pcap** (VLP near) | pcap | 1 2 3 4 5 | `bbox.json5` | 3.0 (default) | 30 (default) | 0 0 1 | off | **on** (0.3) | **3** |
| **VLP-32C bag** (far ~9 m) | bag / vlp32 | TWO_LIDAR_1 TWO_LIDAR_3 | `bbox-vlp.json5` | **1.0** | **20** | 0 0 1 | off | **off** | **1** |
| **Falcon bag** (solid-state ~7.4 m) | bag / falcon | TWO_LIDAR_1 TWO_LIDAR_3 | `bbox-seyond.json5` | **0** | 30 (default) | **0 1 0** | **on** | **off** | **1** |

- Shared across all scenarios: `--side 1.0 --stance-gate --flatness-rms-max 0.045`.
- bbox paths are relative to the run dir: `../../ros/lctk_launch/config/board/{bbox.json5,bbox-vlp.json5,bbox-seyond.json5}` (all three confirmed present).
- Rationale for the per-scenario flags, do not "simplify" them away: `--vertical-gap-deg` bridges spinning-LiDAR ring gaps (3.0 near VLP, 1.0 far VLP so the anisotropic z-compression `2·r·tan(gap)` at 9 m no longer merges the ground, 0 for the ring-less Falcon where z-compression corrupts the dense cloud); `--cluster-min-points 20` keeps the sparse 9 m VLP board's corner points (else the quad truncates); `--up-axis 0 1 0` is world-up in the z-forward Falcon frame (wrong axis → stance gate rejects every upright board); `--square-icp` pins the Falcon board side so `minAreaRect` oversize doesn't sink its score; isolation helps the near pcap board but *hurts* the far bags (their backing structure trips the exterior-band test).
- Expected Method E headlines (for sanity, not exact reproduction): pcap ≈ 88.4% recall / 100% precision; VLP bag ≈ 91.2% / 100% (at cluster-min-points 20); Falcon bag ≈ 100% / 100% (with square-icp + up-axis 0 1 0). No-E baselines expected far lower recall (pcap stage 6 ≈ 49% / stage 8 ≈ 44%; bags much lower still, since B must plane-strip the room instead of diffing it away).

---

### Task 1: Forward `cluster_min_points` to generator B in `detect()`

Today `detect()` passes `board.cluster_min_points` only to generator E; generator B uses the function's hardcoded default of 30. For the VLP-bag no-E baseline to cluster at the same density Method E uses (20), B must honour the config field. Back-compat: `BoardConfig.cluster_min_points` defaults to 30, equal to the generator's own default, so unset behaviour is byte-identical.

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/detector.py:112-113`
- Test: `experiments/board-detection-2d/tests/test_detector.py`

**Interfaces:**
- Consumes: `generate_cluster_after_ground(points, board, *, vertical_gap_deg, cluster_min_points=30)` (already accepts the kwarg — verified at `candidates/cluster_after_ground.py:168`).
- Produces: no signature change to `detect`; generator "b" now respects `board.cluster_min_points`.

- [ ] **Step 1: Write the failing test**

Add to `experiments/board-detection-2d/tests/test_detector.py` (check its top imports include `from boarddet.detector import detect`, `from boarddet.board_config import BoardConfig`, and `from boarddet.synth import make_scene`; add any that are missing):

```python
def test_detect_b_honours_cluster_min_points():
    pts, _ = make_scene(rng=np.random.default_rng(3))
    # Default density detects the synthetic board.
    assert detect(pts, BoardConfig(), generator="b").detection is not None
    # An impossibly high core-point density drops every point as noise, so
    # generator B yields no candidate -> no detection. Proves the field is
    # forwarded (before the fix, B ignored it and still detected).
    starved = BoardConfig(cluster_min_points=10_000_000)
    assert detect(pts, starved, generator="b").detection is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_detector.py::test_detect_b_honours_cluster_min_points -v`
Expected: FAIL on the second assertion — B ignores the field, so a detection is still returned.

- [ ] **Step 3: Forward the field**

In `detector.py`, replace:

```python
    if generator == "b":
        cands = gen(dn, board, vertical_gap_deg=board.vertical_gap_deg)
```

with:

```python
    if generator == "b":
        cands = gen(dn, board, vertical_gap_deg=board.vertical_gap_deg,
                    cluster_min_points=board.cluster_min_points)
```

- [ ] **Step 4: Run test + full suite**

Run: `uv run pytest tests/test_detector.py::test_detect_b_honours_cluster_min_points -v`
Expected: PASS.

Run: `uv run pytest -q`
Expected: whole suite green (change is back-compat; default 30 preserved).

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/detector.py \
        experiments/board-detection-2d/tests/test_detector.py
git commit -m "fix(boarddet): forward cluster_min_points to generator B"
```

---

### Task 2: Add the no-Method-E runner `boarddet.benchmark_noe`

A per-frame generator-B benchmark that mirrors `benchmark_e_loo`'s CLI and output schema (minus `--min-sources` and the background model), so it runs on pcap and bag scenarios with the correct per-sensor flags and emits recall/precision/timing plus overlays. Its per-capture JSON keys match `benchmark_e_loo`'s per-fold keys so Task 5 can pool both uniformly.

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/benchmark_noe.py`
- Test: `experiments/board-detection-2d/tests/test_benchmark_noe.py`

**Interfaces:**
- Consumes: `boarddet.detector.detect`, `boarddet.bbox_ref.{BoxRef, load_bbox}`, `boarddet.ingest.{Frame, load_frames, load_bag_frames}`, `boarddet.viz.save_overlay`, `boarddet.board_config.BoardConfig`.
- Produces: `run_noe(sources, board, out_dir, *, box, save_overlays=0) -> dict` writing `noe_summary.json` with top-level `"captures": {name: {n_frames, n_detections, n_true_board, n_clutter, recall, precision, median_total_ms}}`. `load_sources(kind, names, sensor, max_frames) -> dict[str, list[Frame]]`. `main()` with the flags in the scenario table.

- [ ] **Step 1: Write the runner**

Create `experiments/board-detection-2d/src/boarddet/benchmark_noe.py`:

```python
"""No-Method-E baseline: generator B single-frame recall/precision/timing.

The counterpart to benchmark_e_loo.py for the "without background subtraction"
side of the Method E comparison. Same 2D scorer, same acceptance gates, same
per-rig reference box -- the ONLY difference from Method E is that no
background model is built or diffed (generator "b", not "e"; no min_sources).

Runs per-frame over each named capture (pcap datasets or exported TWO_LIDAR
bags), classifies each accepted detection's centre against the rig's bbox,
and writes noe_summary.json in the same per-capture schema benchmark_e_loo
uses per fold, so tools/compare_methode.py pools both uniformly.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .bbox_ref import BoxRef, load_bbox
from .board_config import BoardConfig
from .detector import detect
from .ingest import Frame, load_bag_frames, load_frames
from .viz import save_overlay

DEFAULT_BBOX_PATH = (Path(__file__).resolve().parents[4]
                     / "ros" / "lctk_launch" / "config" / "board"
                     / "bbox.json5")


def load_sources(kind: str, names: list[str], sensor: str,
                 max_frames: int | None) -> dict[str, list[Frame]]:
    if kind == "pcap":
        return {n: load_frames(int(n), max_frames=max_frames) for n in names}
    if kind == "bag":
        return {n: load_bag_frames(n, sensor, max_frames=max_frames)
                for n in names}
    raise ValueError(f"unknown source kind {kind!r}; expected 'pcap' or 'bag'")


def _overlay_indices(outcomes: list, n: int) -> list[int]:
    """Up to n indices: the first detection, then an even spread."""
    if not outcomes or n <= 0:
        return []
    det = [i for i, o in enumerate(outcomes) if o.detection is not None]
    step = max(1, len(outcomes) // n)
    picks = ([det[0]] if det else []) + list(range(0, len(outcomes), step))
    seen: list[int] = []
    for i in picks:
        if i not in seen:
            seen.append(i)
        if len(seen) >= n:
            break
    return seen


def run_noe(sources: dict[str, list[Frame]], board: BoardConfig,
            out_dir: Path, *, box: BoxRef, save_overlays: int = 0) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    captures: dict[str, dict] = {}
    for name, frames in sources.items():
        outcomes = [detect(f.xyz, board, generator="b") for f in frames]
        dets = [o.detection for o in outcomes if o.detection is not None]
        n_true = sum(1 for d in dets if box.contains(d.center))
        captures[name] = {
            "n_frames": len(outcomes),
            "n_detections": len(dets),
            "n_true_board": n_true,
            "n_clutter": len(dets) - n_true,
            "recall": n_true / len(outcomes) if outcomes else 0.0,
            "precision": (n_true / len(dets)) if dets else None,
            "median_total_ms": float(np.median(
                [o.timings_ms["total"] for o in outcomes]
            )) if outcomes else 0.0,
        }
        for i in _overlay_indices(outcomes, save_overlays):
            save_overlay(frames[i].xyz, outcomes[i],
                         out_dir / f"overlay_{name}_frame{i:04d}.png")
        c = captures[name]
        prec = "n/a" if c["precision"] is None else f"{c['precision']:.1%}"
        print(f"{name}: recall={c['recall']:.1%} true={c['n_true_board']} "
              f"clutter={c['n_clutter']} prec={prec} "
              f"median={c['median_total_ms']:.0f}ms")
    summary = {
        "generator": "b",
        "stance_floor": board.stance_floor,
        "flatness_rms_max": board.flatness_rms_max,
        "vertical_gap_deg": board.vertical_gap_deg,
        "cluster_min_points": board.cluster_min_points,
        "up_axis": list(board.up_axis),
        "square_icp": board.square_icp,
        "isolation": board.isolation,
        "isolation_max_density": board.isolation_max_density,
        "source_labels": list(sources),
        "captures": captures,
    }
    (out_dir / "noe_summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["pcap", "bag"], default="pcap")
    ap.add_argument("--names", nargs="+", default=None,
                    help="captures; default 1..5 (pcap) or "
                         "TWO_LIDAR_1 TWO_LIDAR_3 (bag)")
    ap.add_argument("--sensor", choices=["vlp32", "falcon"], default="vlp32")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--side", type=float, default=1.0)
    ap.add_argument("--stance-gate", action="store_true")
    ap.add_argument("--flatness-rms-max", type=float, default=0.035)
    ap.add_argument("--vertical-gap-deg", type=float, default=3.0)
    ap.add_argument("--cluster-min-points", type=int, default=30)
    ap.add_argument("--square-icp", action="store_true")
    ap.add_argument("--up-axis", type=float, nargs=3, default=(0.0, 0.0, 1.0),
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--isolation", action="store_true")
    ap.add_argument("--isolation-max-density", type=float, default=0.3)
    ap.add_argument("--save-overlays", type=int, default=0)
    ap.add_argument("--bbox", type=Path, default=DEFAULT_BBOX_PATH)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    names = args.names
    if names is None:
        names = (["1", "2", "3", "4", "5"] if args.source == "pcap"
                 else ["TWO_LIDAR_1", "TWO_LIDAR_3"])
    sources = load_sources(args.source, names, args.sensor, args.max_frames)
    board = BoardConfig(
        side_m=args.side,
        stance_floor=0.9 if args.stance_gate else 0.0,
        flatness_rms_max=args.flatness_rms_max,
        vertical_gap_deg=args.vertical_gap_deg,
        cluster_min_points=args.cluster_min_points,
        square_icp=args.square_icp,
        up_axis=tuple(args.up_axis),
        isolation=args.isolation,
        isolation_max_density=args.isolation_max_density,
    )
    run_noe(sources, board, args.out, box=load_bbox(args.bbox),
            save_overlays=args.save_overlays)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Write the test**

Create `experiments/board-detection-2d/tests/test_benchmark_noe.py`:

```python
import json

import numpy as np

from boarddet.benchmark_noe import run_noe
from boarddet.bbox_ref import BoxRef
from boarddet.board_config import BoardConfig
from boarddet.detector import detect
from boarddet.ingest import Frame
from boarddet.synth import make_scene


def _frames(n: int) -> list[Frame]:
    pts, _ = make_scene(rng=np.random.default_rng(16))
    return [Frame(stamp=float(i), xyz=pts,
                  intensity=np.zeros(len(pts), dtype=np.float32),
                  ring=np.zeros(len(pts), dtype=np.uint8))
            for i in range(n)]


def test_run_noe_writes_recall_precision_and_overlays(tmp_path):
    frames = _frames(3)
    center = detect(frames[0].xyz, BoardConfig(), generator="b").detection.center
    box = BoxRef(center=np.asarray(center, dtype=float),
                 half=np.array([0.5, 0.5, 0.5]), rot=np.eye(3))
    summary = run_noe({"synthA": frames}, BoardConfig(), tmp_path,
                      box=box, save_overlays=2)
    cap = summary["captures"]["synthA"]
    assert cap["n_frames"] == 3
    assert cap["n_true_board"] == 3
    assert cap["recall"] == 1.0
    assert cap["precision"] == 1.0
    assert cap["median_total_ms"] > 0
    assert (tmp_path / "noe_summary.json").exists()
    on_disk = json.loads((tmp_path / "noe_summary.json").read_text())
    assert on_disk["captures"]["synthA"]["recall"] == 1.0
    assert len(list(tmp_path.glob("overlay_synthA_*.png"))) >= 1


def test_run_noe_far_box_is_all_clutter(tmp_path):
    frames = _frames(2)
    center = detect(frames[0].xyz, BoardConfig(), generator="b").detection.center
    far = BoxRef(center=np.asarray(center, dtype=float) + 100.0,
                 half=np.array([0.5, 0.5, 0.5]), rot=np.eye(3))
    summary = run_noe({"synthB": frames}, BoardConfig(), tmp_path, box=far)
    cap = summary["captures"]["synthB"]
    assert cap["n_true_board"] == 0
    assert cap["recall"] == 0.0
    assert cap["precision"] == 0.0  # detections exist, none in box
```

- [ ] **Step 3: Run the tests**

Run: `uv run pytest tests/test_benchmark_noe.py -v`
Expected: both PASS.

Run: `uv run pytest -q`
Expected: whole suite green.

- [ ] **Step 4: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/benchmark_noe.py \
        experiments/board-detection-2d/tests/test_benchmark_noe.py
git commit -m "feat(boarddet): no-Method-E generator-B benchmark runner"
```

---

### Task 3: Run the no-Method-E baseline across all scenarios

Generator B (no background subtraction) on every dataset, each with its scenario-appropriate flags. Produces recall/precision/timing JSON + overlays.

**Files:**
- Create (output, gitignored, keep): `results/compare-noE-pcap-stage6/`, `results/compare-noE-pcap-stage8/`, `results/compare-noE-vlp/`, `results/compare-noE-falcon/` (each: `noe_summary.json` + `overlay_*.png`).

**Interfaces:**
- Consumes: `boarddet.benchmark_noe` (Task 2); caches for datasets 1–5 and `TWO_LIDAR_1/3` both sensors.
- Produces: four `noe_summary.json` files consumed by Task 5.

- [ ] **Step 1: Confirm venv + caches**

Run:
```bash
cd experiments/board-detection-2d
uv run python -c "from boarddet.ingest import load_frames, load_bag_frames; print(len(load_frames(3)), len(load_bag_frames('TWO_LIDAR_1','vlp32')), len(load_bag_frames('TWO_LIDAR_1','falcon')))"
```
Expected: three frame counts, no error. A `FileNotFoundError` on a bag means its cache is missing — STOP and export it per `experiments/board-detection-2d/README.md` (needs ROS, outside this venv); do **not** `pip install` anything.

- [ ] **Step 2: pcap baseline, stage 6 (no isolation)**

Run:
```bash
uv run python -m boarddet.benchmark_noe \
  --source pcap --names 1 2 3 4 5 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 \
  --bbox ../../ros/lctk_launch/config/board/bbox.json5 \
  --save-overlays 8 --out results/compare-noE-pcap-stage6
```
Expected: five `1:…5:` lines with `recall=… prec=… median=…ms`; recall near the doc's ~49%.

- [ ] **Step 3: pcap baseline, stage 8 (+ isolation)**

Run:
```bash
uv run python -m boarddet.benchmark_noe \
  --source pcap --names 1 2 3 4 5 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 \
  --isolation --isolation-max-density 0.3 \
  --bbox ../../ros/lctk_launch/config/board/bbox.json5 \
  --save-overlays 8 --out results/compare-noE-pcap-stage8
```
Expected: precision rises toward ~100% vs stage 6.

- [ ] **Step 4: VLP-32C bag baseline (far board tuning, isolation OFF)**

Run:
```bash
uv run python -m boarddet.benchmark_noe \
  --source bag --sensor vlp32 --names TWO_LIDAR_1 TWO_LIDAR_3 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 \
  --vertical-gap-deg 1.0 --cluster-min-points 20 \
  --bbox ../../ros/lctk_launch/config/board/bbox-vlp.json5 \
  --save-overlays 8 --out results/compare-noE-vlp
```
Expected: two `TWO_LIDAR_1/3:` lines. Recall likely low (B must plane-strip the room; no background diff) — that gap is the finding.

- [ ] **Step 5: Falcon bag baseline (ring-less tuning, square-icp, up-axis 0 1 0, isolation OFF)**

Run:
```bash
uv run python -m boarddet.benchmark_noe \
  --source bag --sensor falcon --names TWO_LIDAR_1 TWO_LIDAR_3 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 \
  --vertical-gap-deg 0 --square-icp --up-axis 0 1 0 \
  --bbox ../../ros/lctk_launch/config/board/bbox-seyond.json5 \
  --save-overlays 8 --out results/compare-noE-falcon
```
Expected: two lines; this is the ~270 ms/frame dense-cloud case — slow but should complete.

- [ ] **Step 6: Verify JSON + overlays for one run**

Run:
```bash
uv run python -c "import json; d=json.load(open('results/compare-noE-vlp/noe_summary.json')); print(d['cluster_min_points'], d['vertical_gap_deg'], {k:(v['recall'],v['precision'],v['median_total_ms']) for k,v in d['captures'].items()})"
ls results/compare-noE-*/overlay_*.png | wc -l
```
Expected: `cluster_min_points` is `20` and `vertical_gap_deg` is `1.0` (confirms per-scenario flags took effect and Task 1's forwarding works), plus a positive PNG count. Do not delete the PNGs.

---

### Task 4: Run Method E across all scenarios

The with-Method-E side, using the existing `benchmark_e_loo` with each scenario's arguments and overlays enabled.

**Files:**
- Create (output, gitignored, keep): `results/compare-E-pcap/`, `results/compare-E-vlp/`, `results/compare-E-falcon/` (each: `loo_summary.json` + `overlay_*.png`).

**Interfaces:**
- Consumes: `boarddet.benchmark_e_loo` (unchanged); same caches.
- Produces: three `loo_summary.json` files (per-fold `n_frames`, `n_detections`, `n_true_board`, `recall`, `precision`, `median_total_ms`) consumed by Task 5.

- [ ] **Step 1: pcap Method E (LOO, ms=3, + isolation)**

Run:
```bash
uv run python -m boarddet.benchmark_e_loo \
  --source pcap --names 1 2 3 4 5 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 \
  --min-sources 3 --isolation --isolation-max-density 0.3 \
  --save-overlays 8 --out results/compare-E-pcap
```
Expected: five `held-out ds{1..5}:` lines, `known-clutter-survived=0` on every fold (a nonzero value invalidates that fold — report it if seen), total recall high-80s%.

- [ ] **Step 2: VLP-32C bag Method E (2-fold, ms=1, far tuning)**

Run:
```bash
uv run python -m boarddet.benchmark_e_loo \
  --source bag --sensor vlp32 --names TWO_LIDAR_1 TWO_LIDAR_3 \
  --min-sources 1 --side 1.0 --stance-gate --flatness-rms-max 0.045 \
  --vertical-gap-deg 1.0 --cluster-min-points 20 \
  --bbox ../../ros/lctk_launch/config/board/bbox-vlp.json5 \
  --save-overlays 8 --out results/compare-E-vlp
```
Expected: two folds; recall ≈ 91% at 100% precision (doc's cluster-min-points-20 result).

- [ ] **Step 3: Falcon bag Method E (2-fold, ms=1, ring-less, square-icp, up-axis 0 1 0)**

Run:
```bash
uv run python -m boarddet.benchmark_e_loo \
  --source bag --sensor falcon --names TWO_LIDAR_1 TWO_LIDAR_3 \
  --min-sources 1 --side 1.0 --stance-gate --flatness-rms-max 0.045 \
  --vertical-gap-deg 0 --square-icp --up-axis 0 1 0 \
  --bbox ../../ros/lctk_launch/config/board/bbox-seyond.json5 \
  --save-overlays 8 --out results/compare-E-falcon
```
Expected: two folds; recall ≈ 100% / 100% (doc's square-icp + up-axis 0 1 0 result).

- [ ] **Step 4: Verify JSON + overlays**

Run:
```bash
uv run python -c "import json; d=json.load(open('results/compare-E-vlp/loo_summary.json')); print({k:(v['recall'],v['precision'],v['median_total_ms']) for k,v in d['folds'].items()})"
ls results/compare-E-*/overlay_*.png | wc -l
```
Expected: two folds with numbers; positive PNG count. Do not delete the PNGs.

---

### Task 5: Pool everything into one comparison table

Reads all seven JSONs, computes pooled recall/precision and median timing per run, writes one comparison markdown grouped by scenario. Keeps every image.

**Files:**
- Create: `experiments/board-detection-2d/tools/compare_methode.py` (committed reproducer).
- Create: `experiments/board-detection-2d/results/comparison/summary.md` (gitignored artifact).

**Interfaces:**
- Consumes: the seven summary JSONs. No-E files carry `"captures"`; E files carry `"folds"`; both have the same per-entry keys, so one pooling function reads either.
- Produces: `results/comparison/summary.md` — `Scenario | Configuration | Recall | Precision | Median ms/frame`, and the same table on stdout.

- [ ] **Step 1: Write the pooling script**

Create `experiments/board-detection-2d/tools/compare_methode.py`:

```python
"""Pool the no-Method-E (benchmark_noe) and Method E (benchmark_e_loo) runs
into one recall/precision/timing table across all three rigs. Reproduces the
numbers behind
docs/superpowers/plans/2026-07-26-method-e-vs-baseline-comparison.md."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

RESULTS = Path(__file__).resolve().parents[1] / "results"

# (scenario, configuration label, results-subdir/json-filename)
RUNS = [
    ("pcap 1-5 (VLP near)", "No E - B, stage 6",
     "compare-noE-pcap-stage6/noe_summary.json"),
    ("pcap 1-5 (VLP near)", "No E - B, stage 8 (+iso)",
     "compare-noE-pcap-stage8/noe_summary.json"),
    ("pcap 1-5 (VLP near)", "Method E - ms3 +iso",
     "compare-E-pcap/loo_summary.json"),
    ("VLP-32C bag (~9 m)", "No E - B",
     "compare-noE-vlp/noe_summary.json"),
    ("VLP-32C bag (~9 m)", "Method E - ms1",
     "compare-E-vlp/loo_summary.json"),
    ("Falcon bag (~7.4 m)", "No E - B",
     "compare-noE-falcon/noe_summary.json"),
    ("Falcon bag (~7.4 m)", "Method E - ms1",
     "compare-E-falcon/loo_summary.json"),
]


def pool(path: Path) -> dict:
    d = json.loads(path.read_text())
    entries = d.get("folds") or d.get("captures")
    n_true = sum(e["n_true_board"] for e in entries.values())
    n_det = sum(e["n_detections"] for e in entries.values())
    n_frames = sum(e["n_frames"] for e in entries.values())
    med = float(np.median([e["median_total_ms"] for e in entries.values()]))
    return {
        "recall": n_true / n_frames if n_frames else 0.0,
        "precision": (n_true / n_det) if n_det else None,
        "median_ms": med,
    }


def main() -> None:
    lines = [
        "# Method E vs. no-Method-E - all datasets",
        "",
        "Recall = true-board detections / frames; precision = true-board / all "
        "accepted; both classified against each rig's reference box. Within a "
        "scenario, Method E and its No-E row share every gate and tuning flag; "
        "the only difference is background subtraction.",
        "",
        "| Scenario | Configuration | Recall | Precision | Median ms/frame |",
        "|---|---|---|---|---|",
    ]
    for scenario, label, rel in RUNS:
        p = RESULTS / rel
        if not p.exists():
            lines.append(f"| {scenario} | {label} | MISSING | MISSING | "
                         f"MISSING ({rel}) |")
            continue
        m = pool(p)
        prec = "n/a" if m["precision"] is None else f"{m['precision']:.1%}"
        lines.append(f"| {scenario} | {label} | {m['recall']:.1%} | {prec} | "
                     f"{m['median_ms']:.0f} |")
    text = "\n".join(lines) + "\n"
    out = RESULTS / "comparison"
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it**

Run: `uv run python tools/compare_methode.py`
Expected: prints the seven-row table (no `MISSING` cells) and writes `results/comparison/summary.md`. Within each scenario, the Method E row should show higher recall than its No-E row, at equal-or-better precision, with a higher median ms/frame.

- [ ] **Step 3: Confirm all images survive**

Run:
```bash
find results/compare-noE-pcap-stage6 results/compare-noE-pcap-stage8 \
     results/compare-noE-vlp results/compare-noE-falcon \
     results/compare-E-pcap results/compare-E-vlp results/compare-E-falcon \
     -name '*.png' | wc -l
```
Expected: a positive count; none deleted. Report the count and the seven directory paths.

- [ ] **Step 4: Commit the reproducer**

```bash
git add experiments/board-detection-2d/tools/compare_methode.py
git commit -m "feat(boarddet): Method E vs baseline comparison reproducer"
```

- [ ] **Step 5: Report to the user**

Relay `results/comparison/summary.md` verbatim, plus: the PNG count and the seven overlay directories, a one-line read per scenario of whether Method E's recall gain reproduced at equal/better precision and what it cost in ms/frame, and any fold with `known-clutter-survived > 0`. Do not delete any results dir.

---

## Self-Review

**Spec coverage:**
- "recall, precise [precision], and execution time" → every run emits all three; Task 5 pools them for both sides. ✓
- "all datasets, including pcap and TWO_LIDAR_* bags" → pcap 1–5 (Task 3 Steps 2–3, Task 4 Step 1), VLP-32C bag (Task 3 Step 4, Task 4 Step 2), Falcon bag (Task 3 Step 5, Task 4 Step 3). Both bag sensors covered. ✓
- "arguments … fitting each specific scenario" → the Global-Constraints scenario table fixes per-rig `vertical-gap-deg` / `cluster-min-points` / `up-axis` / `square-icp` / `isolation` / `min-sources`, and Task 1 makes generator B actually honour `cluster_min_points` so the VLP-bag ablation is like-for-like. ✓
- "do not delete images" → Global Constraint + explicit survive-checks (Task 3 Step 6, Task 4 Step 4, Task 5 Step 3); every run uses `--save-overlays 8` into fresh `compare-*` dirs. ✓

**Placeholder scan:** every code step carries complete code; every run step gives the exact command and expected output; no TBD/TODO. ✓

**Type consistency:** no-E `run_noe` writes `captures[name]` with keys `{n_frames, n_detections, n_true_board, n_clutter, recall, precision, median_total_ms}`; E `run_loo` writes `folds[held_out]` with the same keys (verified in `benchmark_e_loo.py`). `compare_methode.pool` reads exactly those keys via `d.get("folds") or d.get("captures")`. `BoardConfig` fields used (`side_m`, `stance_floor`, `flatness_rms_max`, `vertical_gap_deg`, `cluster_min_points`, `square_icp`, `up_axis`, `isolation`, `isolation_max_density`) all exist in `board_config.py`. `BoxRef(center, half, rot)` / `load_bbox` / `load_frames` / `load_bag_frames(bag, sensor)` / `save_overlay(points, outcome, path)` signatures match source. Task 1's `generate_cluster_after_ground(..., cluster_min_points=...)` kwarg confirmed at `candidates/cluster_after_ground.py:168`. ✓
