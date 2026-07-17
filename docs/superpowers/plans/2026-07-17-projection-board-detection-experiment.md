# Projection-Based Board Detection Experiment — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the standalone Python experiment harness specified in `docs/roadmap/phase-7-projection-board-detection.md`: three crop-box-free board-candidate generators feeding one shared 2D quad scorer, benchmarked on the VLP-32C sample pcaps.

**Architecture:** A uv-managed Python package `boarddet` in `experiments/board-detection-2d/`. Pcap frames are decoded once (`velodyne_decoder`) and cached as npz. Each candidate generator (`ransac_iterative`, `cluster_after_ground`, `region_growing`) returns plane candidates; the shared scorer projects a candidate's points into the fitted plane's 2D basis, rasterizes an occupancy image, and fits/scores a metric quad with OpenCV. A benchmark CLI runs every generator over datasets 1–5 and emits markdown tables + overlay PNGs.

**Tech Stack:** Python ≥3.11 (uv-managed), numpy, opencv-python-headless, velodyne-decoder, open3d, matplotlib, pytest.

## Global Constraints

- All work lives under `experiments/board-detection-2d/`; no changes to `ros/` or `rust/`.
- No ROS dependency anywhere in the experiment.
- All Python deps installed only inside the uv venv — never `pip3 install --user` (CLAUDE.md Known Issue 3).
- Board prior is geometry only: diamond (square rotated 45°) with configurable side length, default 1.0 m; holes must never be required.
- `ring` may be stored for diagnostics but must never be used by any algorithm (solid-state compatibility).
- Algorithm code must not read `intensity` (geometry-only decision from brainstorming); it is cached for future diagnostics only.
- Frame cache directory `experiments/board-detection-2d/cache/` and results directory `results/` are gitignored.
- Temporary scratch files go in `$project/tmp/`, not `/tmp/`.
- Format strings use named interpolation (`f"{x}"` style is fine; no positional `%`/`.format`).
- No bare `except Exception: pass`.

---

### Task 1: Project scaffold + pcap ingest with npz cache

**Files:**
- Create: `experiments/board-detection-2d/pyproject.toml` (via `uv init`)
- Create: `experiments/board-detection-2d/.gitignore`
- Create: `experiments/board-detection-2d/src/boarddet/__init__.py`
- Create: `experiments/board-detection-2d/src/boarddet/ingest.py`
- Test: `experiments/board-detection-2d/tests/test_ingest.py`

**Interfaces:**
- Consumes: `ros/lctk_sample_data/data/{1..5}/lidar.pcap` (repo-relative).
- Produces:
  - `Frame` dataclass: `stamp: float`, `xyz: np.ndarray  # (N,3) float32`, `intensity: np.ndarray  # (N,) float32`, `ring: np.ndarray  # (N,) uint8`
  - `load_frames(dataset: int, max_frames: int | None = None) -> list[Frame]` — decodes pcap on first call, caches to `cache/dataset_{n}.npz`, loads cache afterwards.
  - `DATA_DIR`, `CACHE_DIR` module constants.

- [ ] **Step 1: Scaffold the uv project**

```bash
cd /home/aeon/repos/LCTK/experiments  # create dir first: mkdir -p experiments
uv init --lib board-detection-2d --name boarddet --python 3.11
cd board-detection-2d
uv add numpy opencv-python-headless velodyne-decoder open3d matplotlib
uv add --dev pytest
```

Expected: `pyproject.toml` with the deps; `uv run python -c "import velodyne_decoder, open3d, cv2"` succeeds.

- [ ] **Step 2: Add `.gitignore`**

```gitignore
cache/
results/
tmp/
```

- [ ] **Step 3: Verify velodyne_decoder API against a real pcap (spike)**

```bash
uv run python - <<'EOF'
import velodyne_decoder as vd
cfg = vd.Config(model=vd.Model.VLP32C)
gen = vd.read_pcap("../../ros/lctk_sample_data/data/3/lidar.pcap", cfg, as_pcl_structs=True)
stamp, pts = next(iter(gen))
print(type(stamp), pts.dtype.names, pts.shape)
EOF
```

Expected: dtype names include `x, y, z, intensity, ring` (plus `time`). If the installed velodyne-decoder version's API differs (e.g. `read_pcap(path)` without Config, or different field names), note the actual signature and use it in Step 5 — the field-name constants at the top of `ingest.py` are the single place to adjust.

- [ ] **Step 4: Write the failing test**

```python
# tests/test_ingest.py
import numpy as np
from boarddet.ingest import load_frames


def test_load_frames_dataset3_first_frames():
    frames = load_frames(3, max_frames=5)
    assert len(frames) == 5
    f = frames[0]
    assert f.xyz.ndim == 2 and f.xyz.shape[1] == 3
    assert f.xyz.dtype == np.float32
    assert len(f.intensity) == len(f.xyz) == len(f.ring)
    # VLP-32C full rotation at 600 rpm: expect tens of thousands of points
    assert f.xyz.shape[0] > 10_000
    # points are in sensor frame, metres: sane range
    r = np.linalg.norm(f.xyz, axis=1)
    assert r.max() < 200.0


def test_load_frames_uses_cache(tmp_path, monkeypatch):
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    frames1 = ingest.load_frames(3, max_frames=2)
    assert (tmp_path / "dataset_3.npz").exists()
    frames2 = ingest.load_frames(3, max_frames=2)
    np.testing.assert_array_equal(frames1[0].xyz, frames2[0].xyz)
```

- [ ] **Step 5: Run test to verify it fails**

Run: `uv run pytest tests/test_ingest.py -v`
Expected: FAIL with `ModuleNotFoundError` / `ImportError` (no `ingest` module yet).

- [ ] **Step 6: Implement `ingest.py`**

```python
# src/boarddet/ingest.py
"""Decode sample pcaps to per-frame numpy arrays, cached as npz.

ring and intensity are cached for diagnostics only — algorithm code must
never read them (solid-state compatibility / geometry-only prior).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import velodyne_decoder as vd

# Repo-relative anchors; ingest.py sits at src/boarddet/ingest.py.
_PKG_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = _PKG_ROOT.parents[1] / "ros" / "lctk_sample_data" / "data"
CACHE_DIR = _PKG_ROOT / "cache"

# Field names as produced by velodyne_decoder as_pcl_structs=True.
# Single place to adjust if the installed decoder version differs.
_F_X, _F_Y, _F_Z = "x", "y", "z"
_F_INTENSITY = "intensity"
_F_RING = "ring"


@dataclass
class Frame:
    stamp: float
    xyz: np.ndarray        # (N, 3) float32
    intensity: np.ndarray  # (N,) float32 — diagnostics only
    ring: np.ndarray       # (N,) uint8   — diagnostics only


def _decode_pcap(pcap: Path, max_frames: int | None) -> list[Frame]:
    cfg = vd.Config(model=vd.Model.VLP32C)
    frames: list[Frame] = []
    for stamp, pts in vd.read_pcap(str(pcap), cfg, as_pcl_structs=True):
        xyz = np.stack(
            [pts[_F_X], pts[_F_Y], pts[_F_Z]], axis=1
        ).astype(np.float32)
        frames.append(
            Frame(
                stamp=float(stamp.host if hasattr(stamp, "host") else stamp),
                xyz=xyz,
                intensity=pts[_F_INTENSITY].astype(np.float32),
                ring=pts[_F_RING].astype(np.uint8),
            )
        )
        if max_frames is not None and len(frames) >= max_frames:
            break
    return frames


def _cache_path(dataset: int) -> Path:
    return CACHE_DIR / f"dataset_{dataset}.npz"


def _save_cache(path: Path, frames: list[Frame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays: dict[str, np.ndarray] = {
        "stamps": np.array([f.stamp for f in frames], dtype=np.float64)
    }
    for i, f in enumerate(frames):
        arrays[f"xyz_{i}"] = f.xyz
        arrays[f"intensity_{i}"] = f.intensity
        arrays[f"ring_{i}"] = f.ring
    np.savez_compressed(path, **arrays)


def _load_cache(path: Path) -> list[Frame]:
    with np.load(path) as z:
        stamps = z["stamps"]
        return [
            Frame(
                stamp=float(stamps[i]),
                xyz=z[f"xyz_{i}"],
                intensity=z[f"intensity_{i}"],
                ring=z[f"ring_{i}"],
            )
            for i in range(len(stamps))
        ]


def load_frames(dataset: int, max_frames: int | None = None) -> list[Frame]:
    """Load frames for a sample dataset (1-5), decoding + caching on first use."""
    cached = _cache_path(dataset)
    if cached.exists():
        frames = _load_cache(cached)
    else:
        pcap = DATA_DIR / str(dataset) / "lidar.pcap"
        if not pcap.exists():
            raise FileNotFoundError(f"sample pcap not found: {pcap}")
        frames = _decode_pcap(pcap, max_frames=None)
        _save_cache(cached, frames)
    if max_frames is not None:
        frames = frames[:max_frames]
    return frames


if __name__ == "__main__":
    import sys

    ds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    fr = load_frames(ds)
    sizes = [len(f.xyz) for f in fr]
    print(f"dataset {ds}: {len(fr)} frames, "
          f"points/frame min={min(sizes)} max={max(sizes)}")
```

Note: full decode of one pcap (~20 s of data) then compressed save may take ~1 min the first time; subsequent runs hit the cache.

- [ ] **Step 7: Run tests to verify they pass**

Run: `uv run pytest tests/test_ingest.py -v`
Expected: 2 PASS (first run is slow — full decode).

- [ ] **Step 8: Commit**

```bash
cd /home/aeon/repos/LCTK
git add experiments/board-detection-2d
git commit -m "feat(phase-7): scaffold boarddet experiment, pcap ingest with npz cache"
```

---

### Task 2: Synthetic scene generator (test fixture for everything downstream)

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/synth.py`
- Test: `experiments/board-detection-2d/tests/test_synth.py`

**Interfaces:**
- Produces:
  - `SceneTruth` dataclass: `center: np.ndarray  # (3,)`, `normal: np.ndarray  # (3,)`, `corners: np.ndarray  # (4,3) diamond corners top,right,bottom,left`
  - `make_board(side: float, center: np.ndarray, normal: np.ndarray, up_hint: np.ndarray, spacing: float, noise: float, rng) -> tuple[np.ndarray, SceneTruth]` — diamond-shaped planar patch of points.
  - `make_scene(board_side: float = 1.0, board_center=(4.0, 0.5, 0.3), spacing: float = 0.03, noise: float = 0.01, pattern: str = "grid", rng=None) -> tuple[np.ndarray, SceneTruth]` — board + ground plane + wall + a box-shaped clutter cluster, single (M,3) float32 cloud. `pattern="uniform"` uses uniform random sampling instead of a grid (solid-state stand-in).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_synth.py
import numpy as np
from boarddet.synth import make_scene, make_board


def test_make_board_points_lie_on_plane_within_noise():
    rng = np.random.default_rng(0)
    pts, truth = make_board(
        side=1.0,
        center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.2, 0.0]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=0.03,
        noise=0.005,
        rng=rng,
    )
    n = truth.normal / np.linalg.norm(truth.normal)
    d = (pts - truth.center) @ n
    assert np.abs(d).max() < 0.03  # within a few noise sigma
    # diamond diagonal = side * sqrt(2)
    assert np.isclose(
        np.linalg.norm(truth.corners[0] - truth.corners[2]),
        np.sqrt(2.0),
        atol=1e-6,
    )


def test_make_scene_contains_board_and_clutter():
    pts, truth = make_scene(rng=np.random.default_rng(1))
    assert pts.dtype == np.float32
    assert len(pts) > 5_000
    # some points near the board plane, many not
    n = truth.normal
    d = np.abs((pts - truth.center) @ n)
    near = d < 0.02
    assert 200 < near.sum() < len(pts) // 2


def test_uniform_pattern_differs_from_grid():
    g, _ = make_scene(pattern="grid", rng=np.random.default_rng(2))
    u, _ = make_scene(pattern="uniform", rng=np.random.default_rng(2))
    assert g.shape != u.shape or not np.allclose(g[: len(u)], u[: len(g)])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_synth.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `synth.py`**

```python
# src/boarddet/synth.py
"""Synthetic scenes with a known diamond board pose, for tests + benchmarks."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class SceneTruth:
    center: np.ndarray   # (3,)
    normal: np.ndarray   # (3,) unit
    corners: np.ndarray  # (4,3) top, right, bottom, left


def _plane_basis(normal: np.ndarray, up_hint: np.ndarray):
    n = normal / np.linalg.norm(normal)
    v = up_hint - (up_hint @ n) * n  # in-plane "up"
    v = v / np.linalg.norm(v)
    u = np.cross(v, n)
    return u, v, n


def _sample_plane_patch(extent_u, extent_v, spacing, pattern, rng):
    """2D sample coordinates covering [-eu,eu]x[-ev,ev]."""
    if pattern == "grid":
        us = np.arange(-extent_u, extent_u, spacing)
        vs = np.arange(-extent_v, extent_v, spacing)
        uu, vv = np.meshgrid(us, vs)
        return np.stack([uu.ravel(), vv.ravel()], axis=1)
    if pattern == "uniform":
        area = 4.0 * extent_u * extent_v
        count = int(area / spacing**2)
        return rng.uniform(
            [-extent_u, -extent_v], [extent_u, extent_v], size=(count, 2)
        )
    raise ValueError(f"unknown pattern: {pattern}")


def make_board(side, center, normal, up_hint, spacing, noise, rng,
               pattern="grid"):
    u, v, n = _plane_basis(np.asarray(normal, float), np.asarray(up_hint, float))
    center = np.asarray(center, float)
    half_diag = side / np.sqrt(2.0)
    coords = _sample_plane_patch(half_diag, half_diag, spacing, pattern, rng)
    # diamond: |u| + |v| <= half_diag (square rotated 45 deg)
    inside = np.abs(coords[:, 0]) + np.abs(coords[:, 1]) <= half_diag
    coords = coords[inside]
    pts = center + coords[:, :1] * u + coords[:, 1:] * v
    pts = pts + rng.normal(0.0, noise, size=pts.shape) * n
    corners = np.stack([
        center + half_diag * v,   # top
        center + half_diag * u,   # right
        center - half_diag * v,   # bottom
        center - half_diag * u,   # left
    ])
    return pts.astype(np.float32), SceneTruth(center=center, normal=n,
                                              corners=corners)


def make_scene(board_side=1.0, board_center=(4.0, 0.5, 0.3), spacing=0.03,
               noise=0.01, pattern="grid", rng=None):
    rng = rng if rng is not None else np.random.default_rng()
    board_pts, truth = make_board(
        side=board_side,
        center=np.asarray(board_center, float),
        normal=np.array([-1.0, 0.15, 0.05]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=spacing,
        noise=noise,
        rng=rng,
        pattern=pattern,
    )
    parts = [board_pts]
    # ground plane z = -1, 12x12 m
    g = _sample_plane_patch(6.0, 6.0, spacing * 3, pattern, rng)
    ground = np.stack([g[:, 0] + 4.0, g[:, 1], np.full(len(g), -1.0)], axis=1)
    parts.append((ground + rng.normal(0, noise, ground.shape))
                 .astype(np.float32))
    # wall x = 8, 12 m wide, 3 m tall
    w = _sample_plane_patch(6.0, 1.5, spacing * 3, pattern, rng)
    wall = np.stack([np.full(len(w), 8.0), w[:, 0], w[:, 1] + 0.5], axis=1)
    parts.append((wall + rng.normal(0, noise, wall.shape)).astype(np.float32))
    # clutter: box-ish blob (not planar) near the board
    blob = rng.normal([3.0, -2.0, 0.0], [0.3, 0.3, 0.5], size=(800, 3))
    parts.append(blob.astype(np.float32))
    return np.concatenate(parts).astype(np.float32), truth
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_synth.py -v`
Expected: 3 PASS.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): synthetic scene generator with known board truth"
```

---

### Task 3: Plane geometry — fit, basis, projection, downsample

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/geometry.py`
- Test: `experiments/board-detection-2d/tests/test_geometry.py`

**Interfaces:**
- Produces:
  - `PlaneModel` dataclass: `center: np.ndarray  # (3,)`, `normal: np.ndarray  # (3,) unit`, `u: np.ndarray  # (3,)`, `v: np.ndarray  # (3,)` (orthonormal in-plane basis)
  - `fit_plane(points: np.ndarray) -> PlaneModel` — PCA plane through points.
  - `plane_rms(points: np.ndarray, plane: PlaneModel) -> float` — rms out-of-plane distance.
  - `project_to_plane(points: np.ndarray, plane: PlaneModel) -> np.ndarray  # (N,2)`
  - `unproject(coords_2d: np.ndarray, plane: PlaneModel) -> np.ndarray  # (N,3)`
  - `downsample(points: np.ndarray, voxel: float = 0.03) -> np.ndarray` — open3d voxel grid.
  - `extent_2d(coords_2d: np.ndarray) -> float` — max side of the 2D axis-aligned bbox (cheap size gate).
- Consumes: `boarddet.synth` for tests.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_geometry.py
import numpy as np
from boarddet.geometry import (
    fit_plane, plane_rms, project_to_plane, unproject, downsample, extent_2d,
)
from boarddet.synth import make_board


def _board(noise=0.005):
    rng = np.random.default_rng(3)
    return make_board(
        side=1.0, center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.2, 0.1]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=0.02, noise=noise, rng=rng,
    )


def test_fit_plane_recovers_normal():
    pts, truth = _board()
    plane = fit_plane(pts)
    assert abs(plane.normal @ truth.normal) > 0.999
    assert plane_rms(pts, plane) < 0.01
    # basis orthonormal
    for a, b in [(plane.u, plane.v), (plane.u, plane.normal),
                 (plane.v, plane.normal)]:
        assert abs(a @ b) < 1e-9
    assert np.isclose(np.linalg.norm(plane.u), 1.0)


def test_project_unproject_roundtrip():
    pts, _ = _board(noise=0.0)
    plane = fit_plane(pts)
    c2 = project_to_plane(pts, plane)
    back = unproject(c2, plane)
    assert np.abs(back - pts).max() < 1e-5


def test_extent_and_downsample():
    pts, _ = _board(noise=0.0)
    c2 = project_to_plane(pts, fit_plane(pts))
    # diamond of side 1.0: bbox is diagonal x diagonal = sqrt(2) x sqrt(2)
    assert np.isclose(extent_2d(c2), np.sqrt(2.0), atol=0.05)
    dn = downsample(pts, voxel=0.1)
    assert 10 < len(dn) < len(pts)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_geometry.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `geometry.py`**

```python
# src/boarddet/geometry.py
"""Plane fitting and 2D plane-coordinate projection (the chosen projection)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class PlaneModel:
    center: np.ndarray  # (3,)
    normal: np.ndarray  # (3,) unit
    u: np.ndarray       # (3,) in-plane basis
    v: np.ndarray       # (3,) in-plane basis


def fit_plane(points: np.ndarray) -> PlaneModel:
    center = points.mean(axis=0)
    q = points.astype(np.float64) - center
    # smallest singular vector = normal; the two largest span the plane
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    u, v, normal = vt[0], vt[1], vt[2]
    return PlaneModel(center=center, normal=normal, u=u, v=v)


def plane_rms(points: np.ndarray, plane: PlaneModel) -> float:
    d = (points - plane.center) @ plane.normal
    return float(np.sqrt(np.mean(d**2)))


def project_to_plane(points: np.ndarray, plane: PlaneModel) -> np.ndarray:
    q = points - plane.center
    return np.stack([q @ plane.u, q @ plane.v], axis=1)


def unproject(coords_2d: np.ndarray, plane: PlaneModel) -> np.ndarray:
    return (plane.center
            + coords_2d[:, :1] * plane.u
            + coords_2d[:, 1:] * plane.v)


def downsample(points: np.ndarray, voxel: float = 0.03) -> np.ndarray:
    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(points.astype(np.float64)))
    dn = pc.voxel_down_sample(voxel)
    return np.asarray(dn.points, dtype=np.float32)


def extent_2d(coords_2d: np.ndarray) -> float:
    span = coords_2d.max(axis=0) - coords_2d.min(axis=0)
    return float(span.max())
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_geometry.py -v`
Expected: 3 PASS.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): plane fit + 2D plane projection + voxel downsample"
```

---

### Task 4: Shared 2D scorer — rasterize, contour, quad fit, side refit, score

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/board_config.py`
- Create: `experiments/board-detection-2d/src/boarddet/scorer.py`
- Test: `experiments/board-detection-2d/tests/test_scorer.py`

**Interfaces:**
- Produces:
  - `BoardConfig` dataclass: `side_m: float = 1.0`, `side_tol: float = 0.20` (fractional), `min_score: float = 0.5`, `cell_m: float = 0.02`
  - `ScoreResult` dataclass: `score: float`, `corners_2d: np.ndarray  # (4,2) refined, CCW`, `side_lengths: np.ndarray  # (4,)`, `fill_ratio: float`, `angle_err_deg: float`, `raster: np.ndarray  # uint8 occupancy image (debug)`, `origin: np.ndarray  # (2,) raster origin in plane coords`
  - `score_candidate(coords_2d: np.ndarray, board: BoardConfig) -> ScoreResult | None` — None when no plausible quad (too few points, no contour, size out of tolerance).
- Consumes: `BoardConfig` used by all generators and the detector; `ScoreResult.corners_2d` consumed by `pose.py` (Task 5).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_scorer.py
import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.geometry import fit_plane, project_to_plane
from boarddet.scorer import score_candidate
from boarddet.synth import make_board


def _board_2d(side=1.0, noise=0.005, spacing=0.02, seed=4):
    rng = np.random.default_rng(seed)
    pts, truth = make_board(
        side=side, center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.1, 0.05]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=spacing, noise=noise, rng=rng,
    )
    return project_to_plane(pts, fit_plane(pts))


def test_scores_true_board_high():
    res = score_candidate(_board_2d(), BoardConfig(side_m=1.0))
    assert res is not None
    assert res.score > 0.6
    np.testing.assert_allclose(res.side_lengths.mean(), 1.0, atol=0.08)
    assert res.angle_err_deg < 6.0


def test_corner_accuracy_beats_cell_size():
    board = BoardConfig(side_m=1.0)
    res = score_candidate(_board_2d(noise=0.003), board)
    # diamond corners in plane coords: at (+-d, 0), (0, +-d), d = side/sqrt(2)
    d = 1.0 / np.sqrt(2.0)
    expected = np.array([[d, 0], [0, d], [-d, 0], [0, -d]])
    # match each detected corner to nearest expected
    err = [np.linalg.norm(expected - c, axis=1).min() for c in res.corners_2d]
    assert np.mean(err) < board.cell_m  # sub-cell via side refit


def test_rejects_wrong_size():
    assert score_candidate(_board_2d(side=2.5), BoardConfig(side_m=1.0)) is None


def test_rejects_sparse_garbage():
    rng = np.random.default_rng(5)
    junk = rng.uniform(-1, 1, size=(40, 2)).astype(np.float32)
    res = score_candidate(junk, BoardConfig(side_m=1.0))
    assert res is None or res.score < 0.5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_scorer.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `board_config.py` and `scorer.py`**

```python
# src/boarddet/board_config.py
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class BoardConfig:
    side_m: float = 1.0      # diamond (square) side length
    side_tol: float = 0.20   # fractional tolerance on side length
    min_score: float = 0.5   # detector acceptance threshold
    cell_m: float = 0.02     # raster cell size
```

```python
# src/boarddet/scorer.py
"""Shared 2D scorer: occupancy raster -> contour -> quad -> refined corners."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .board_config import BoardConfig

_MIN_POINTS = 60


@dataclass
class ScoreResult:
    score: float
    corners_2d: np.ndarray   # (4,2) refined, CCW order
    side_lengths: np.ndarray  # (4,)
    fill_ratio: float
    angle_err_deg: float
    raster: np.ndarray        # uint8 debug image
    origin: np.ndarray        # (2,) plane coords of raster pixel (0,0)


def _rasterize(coords_2d: np.ndarray, cell: float):
    origin = coords_2d.min(axis=0) - 2 * cell
    ij = np.floor((coords_2d - origin) / cell).astype(np.int32)
    h, w = ij[:, 1].max() + 3, ij[:, 0].max() + 3
    img = np.zeros((h, w), dtype=np.uint8)
    img[ij[:, 1], ij[:, 0]] = 255
    return img, origin


def _px_to_plane(pts_px: np.ndarray, origin: np.ndarray, cell: float):
    return origin + (pts_px + 0.5) * cell


def _refine_sides(coords_2d, quad_plane, cell):
    """TLS line fit per side on raw points near it, intersect adjacent lines."""
    lines = []
    for i in range(4):
        a, b = quad_plane[i], quad_plane[(i + 1) % 4]
        ab = b - a
        length = np.linalg.norm(ab)
        t = (coords_2d - a) @ ab / length**2
        perp = np.abs(np.cross(np.append(ab / length, 0.0),
                               np.c_[coords_2d - a, np.zeros(len(coords_2d))]
                               )[:, 2])
        near = (perp < 2.5 * cell) & (t > 0.1) & (t < 0.9)
        side_pts = coords_2d[near]
        if len(side_pts) < 5:
            return None
        centroid = side_pts.mean(axis=0)
        _, _, vt = np.linalg.svd(side_pts - centroid, full_matrices=False)
        lines.append((centroid, vt[0]))  # point + direction
    corners = []
    for i in range(4):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        m = np.array([d1, -d2]).T
        if abs(np.linalg.det(m)) < 1e-9:
            return None
        s = np.linalg.solve(m, p2 - p1)
        corners.append(p1 + s[0] * d1)
    return np.array(corners)


def score_candidate(coords_2d: np.ndarray,
                    board: BoardConfig) -> ScoreResult | None:
    if len(coords_2d) < _MIN_POINTS:
        return None
    cell = board.cell_m
    img, origin = _rasterize(coords_2d, cell)
    if img.shape[0] > 4000 or img.shape[1] > 4000:
        return None  # candidate far larger than any board
    closed = cv2.morphologyEx(
        img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(
        closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(contour)  # ((cx,cy),(w,h),angle)
    (rw, rh) = rect[1]
    if min(rw, rh) < 3:
        return None
    quad_px = cv2.boxPoints(rect)
    quad_plane = _px_to_plane(quad_px, origin, cell)

    # size gate on the coarse quad
    sides = np.linalg.norm(np.roll(quad_plane, -1, axis=0) - quad_plane,
                           axis=1)
    if not (board.side_m * (1 - 2 * board.side_tol)
            < sides.mean()
            < board.side_m * (1 + 2 * board.side_tol)):
        return None

    refined = _refine_sides(coords_2d, quad_plane, cell)
    corners = refined if refined is not None else quad_plane
    sides = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)

    # angles at each corner
    angs = []
    for i in range(4):
        e1 = corners[(i + 1) % 4] - corners[i]
        e2 = corners[i - 1] - corners[i]
        cosang = e1 @ e2 / (np.linalg.norm(e1) * np.linalg.norm(e2))
        angs.append(np.degrees(np.arccos(np.clip(cosang, -1, 1))))
    angle_err = float(np.mean(np.abs(np.array(angs) - 90.0)))

    # fill ratio: fraction of raster cells inside the quad that are occupied
    mask = np.zeros_like(closed)
    quad_px_int = np.round(
        (corners - origin) / cell - 0.5).astype(np.int32)
    cv2.fillPoly(mask, [quad_px_int], 255)
    inside = mask > 0
    fill = float((closed[inside] > 0).mean()) if inside.any() else 0.0

    side_err = (float(np.std(sides) / np.mean(sides))
                + abs(float(np.mean(sides)) - board.side_m) / board.side_m)
    if abs(float(np.mean(sides)) - board.side_m) > board.side_tol * board.side_m:
        return None
    score = fill * float(np.exp(-4.0 * side_err)) \
        * float(np.exp(-angle_err / 15.0))

    # CCW order
    c = corners.mean(axis=0)
    order = np.argsort(np.arctan2(*(corners - c).T[::-1]))
    corners = corners[order]
    sides = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)

    return ScoreResult(score=float(score), corners_2d=corners,
                       side_lengths=sides, fill_ratio=fill,
                       angle_err_deg=angle_err, raster=closed, origin=origin)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_scorer.py -v`
Expected: 4 PASS. If `test_corner_accuracy_beats_cell_size` fails marginally, inspect whether `_refine_sides` returned None (too few side points) before loosening anything — the refit is the point of the test.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): shared 2D quad scorer with sub-cell corner refit"
```

---

### Task 5: Pose extraction from scored quad

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/pose.py`
- Test: `experiments/board-detection-2d/tests/test_pose.py`

**Interfaces:**
- Produces:
  - `BoardDetection` dataclass: `center: np.ndarray  # (3,)`, `rotation: np.ndarray  # (3,3) columns = board x (toward top corner), board y, board normal`, `corners_3d: np.ndarray  # (4,3)`, `score: float`, `result: ScoreResult`
  - `board_pose(plane: PlaneModel, result: ScoreResult) -> BoardDetection`
- Consumes: `PlaneModel` (Task 3), `ScoreResult` (Task 4).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_pose.py
import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.geometry import fit_plane, project_to_plane
from boarddet.pose import board_pose
from boarddet.scorer import score_candidate
from boarddet.synth import make_board


def test_pose_recovers_truth():
    rng = np.random.default_rng(6)
    pts, truth = make_board(
        side=1.0, center=np.array([4.0, 0.5, 0.3]),
        normal=np.array([-1.0, 0.15, 0.05]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=0.02, noise=0.004, rng=rng,
    )
    plane = fit_plane(pts)
    res = score_candidate(project_to_plane(pts, plane), BoardConfig())
    det = board_pose(plane, res)
    assert np.linalg.norm(det.center - truth.center) < 0.02
    assert abs(det.rotation[:, 2] @ truth.normal) > 0.999
    # rotation is orthonormal, right-handed
    r = det.rotation
    np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-9)
    assert np.linalg.det(r) > 0.99
    # corners_3d match truth corners as a set
    for c in det.corners_3d:
        assert np.linalg.norm(truth.corners - c, axis=1).min() < 0.03
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_pose.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `pose.py`**

```python
# src/boarddet/pose.py
"""Board pose from a scored quad + its plane."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import PlaneModel, unproject
from .scorer import ScoreResult


@dataclass
class BoardDetection:
    center: np.ndarray     # (3,)
    rotation: np.ndarray   # (3,3): cols = board x, board y, normal
    corners_3d: np.ndarray  # (4,3)
    score: float
    result: ScoreResult


def board_pose(plane: PlaneModel, result: ScoreResult) -> BoardDetection:
    corners_3d = unproject(result.corners_2d, plane)
    center = corners_3d.mean(axis=0)
    # board x axis: center -> highest corner (diamond "top"), projected in-plane
    top = corners_3d[np.argmax(corners_3d[:, 2])]
    x = top - center
    x = x - (x @ plane.normal) * plane.normal
    x = x / np.linalg.norm(x)
    n = plane.normal
    y = np.cross(n, x)
    rotation = np.stack([x, y, n], axis=1)
    return BoardDetection(center=center, rotation=rotation,
                          corners_3d=corners_3d,
                          score=result.score, result=result)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_pose.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): board pose extraction from scored quad"
```

---

### Task 6: Candidate interface + generator A (iterative RANSAC)

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/candidates/__init__.py`
- Create: `experiments/board-detection-2d/src/boarddet/candidates/ransac_iterative.py`
- Test: `experiments/board-detection-2d/tests/test_candidates_a.py`

**Interfaces:**
- Produces:
  - `Candidate` dataclass (in `candidates/__init__.py`): `points: np.ndarray  # (N,3) candidate's 3D points`, `plane: PlaneModel`
  - `generate_ransac_iterative(points: np.ndarray, board: BoardConfig, max_planes: int = 8, dist_thresh: float = 0.02, min_inliers: int = 60) -> list[Candidate]`
  - Common gates (in `candidates/__init__.py`): `plausible_board_patch(points_3d, board) -> Candidate | None` — PCA plane fit, flatness gate (`plane_rms < 0.03`), size gate (2D extent in `[0.5*side, 2.2*side*sqrt(2)/2 ...]` — concretely `0.5 * board.side_m <= extent_2d <= 1.8 * board.side_m * sqrt(2)`).
- Consumes: `geometry.fit_plane/plane_rms/project_to_plane/extent_2d`, `BoardConfig`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_candidates_a.py
import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates import plausible_board_patch
from boarddet.candidates.ransac_iterative import generate_ransac_iterative
from boarddet.geometry import downsample
from boarddet.synth import make_scene


def test_finds_board_plane_among_candidates():
    pts, truth = make_scene(rng=np.random.default_rng(7))
    board = BoardConfig(side_m=1.0)
    cands = generate_ransac_iterative(downsample(pts, 0.03), board)
    assert len(cands) >= 1
    # at least one candidate's plane matches the true board plane
    matches = [
        c for c in cands
        if abs(c.plane.normal @ truth.normal) > 0.99
        and abs((c.plane.center - truth.center) @ truth.normal) < 0.05
    ]
    assert matches


def test_gate_rejects_huge_patch():
    rng = np.random.default_rng(8)
    # 6x6 m planar patch: flat but far too large
    g = rng.uniform(-3, 3, size=(4000, 2))
    patch = np.c_[g[:, 0], g[:, 1], rng.normal(0, 0.005, 4000)]
    assert plausible_board_patch(patch.astype(np.float32),
                                 BoardConfig(side_m=1.0)) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_candidates_a.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `candidates/__init__.py` and `ransac_iterative.py`**

```python
# src/boarddet/candidates/__init__.py
"""Candidate generators: full scene -> plausible board plane patches."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..board_config import BoardConfig
from ..geometry import PlaneModel, extent_2d, fit_plane, plane_rms, \
    project_to_plane

_FLATNESS_RMS_MAX = 0.03  # m; above VLP-32C noise floor, below "not a plane"
_MIN_PATCH_POINTS = 60


@dataclass
class Candidate:
    points: np.ndarray  # (N,3)
    plane: PlaneModel


def plausible_board_patch(points_3d: np.ndarray,
                          board: BoardConfig) -> Candidate | None:
    """Gate a 3D patch: enough points, flat, board-sized. None if implausible."""
    if len(points_3d) < _MIN_PATCH_POINTS:
        return None
    plane = fit_plane(points_3d)
    if plane_rms(points_3d, plane) > _FLATNESS_RMS_MAX:
        return None
    ext = extent_2d(project_to_plane(points_3d, plane))
    diag = board.side_m * np.sqrt(2.0)
    if not (0.5 * board.side_m <= ext <= 1.8 * diag):
        return None
    return Candidate(points=points_3d, plane=plane)
```

```python
# src/boarddet/candidates/ransac_iterative.py
"""Approach A: iterative RANSAC plane extraction (velo2cam style)."""
from __future__ import annotations

import numpy as np
import open3d as o3d

from . import Candidate, plausible_board_patch
from ..board_config import BoardConfig


def generate_ransac_iterative(points: np.ndarray, board: BoardConfig,
                              max_planes: int = 8,
                              dist_thresh: float = 0.02,
                              min_inliers: int = 60) -> list[Candidate]:
    remaining = points.astype(np.float64)
    out: list[Candidate] = []
    for _ in range(max_planes):
        if len(remaining) < min_inliers:
            break
        pc = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(remaining))
        _, inlier_idx = pc.segment_plane(
            distance_threshold=dist_thresh, ransac_n=3, num_iterations=500)
        if len(inlier_idx) < min_inliers:
            break
        inliers = remaining[inlier_idx].astype(np.float32)
        cand = plausible_board_patch(inliers, board)
        if cand is not None:
            out.append(cand)
        mask = np.ones(len(remaining), dtype=bool)
        mask[inlier_idx] = False
        remaining = remaining[mask]
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_candidates_a.py -v`
Expected: 2 PASS. Note the known weakness (a RANSAC plane can span board + coplanar clutter); do not tune here — the benchmark measures it.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): candidate gate + generator A (iterative RANSAC)"
```

---

### Task 7: Generator B (Euclidean clustering after big-plane removal)

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/candidates/cluster_after_ground.py`
- Test: `experiments/board-detection-2d/tests/test_candidates_b.py`

**Interfaces:**
- Produces: `generate_cluster_after_ground(points: np.ndarray, board: BoardConfig, big_plane_dist: float = 0.05, big_plane_min_frac: float = 0.15, cluster_eps: float = 0.10, cluster_min_points: int = 30) -> list[Candidate]`
- Consumes: `Candidate`, `plausible_board_patch` (Task 6), open3d `segment_plane` + `cluster_dbscan`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_candidates_b.py
import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates.cluster_after_ground import \
    generate_cluster_after_ground
from boarddet.geometry import downsample
from boarddet.synth import make_scene


def test_finds_board_cluster():
    pts, truth = make_scene(rng=np.random.default_rng(9))
    cands = generate_cluster_after_ground(
        downsample(pts, 0.03), BoardConfig(side_m=1.0))
    matches = [
        c for c in cands
        if abs(c.plane.normal @ truth.normal) > 0.99
        and abs((c.plane.center - truth.center) @ truth.normal) < 0.05
    ]
    assert matches


def test_ground_and_wall_removed_before_clustering():
    pts, _ = make_scene(rng=np.random.default_rng(10))
    cands = generate_cluster_after_ground(
        downsample(pts, 0.03), BoardConfig(side_m=1.0))
    # no candidate should be near-horizontal (the ground)
    for c in cands:
        assert abs(c.plane.normal[2]) < 0.9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_candidates_b.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `cluster_after_ground.py`**

```python
# src/boarddet/candidates/cluster_after_ground.py
"""Approach B: remove large planes, Euclidean-cluster the rest, gate clusters."""
from __future__ import annotations

import numpy as np
import open3d as o3d

from . import Candidate, plausible_board_patch
from ..board_config import BoardConfig
from ..geometry import extent_2d, fit_plane, project_to_plane


def _remove_big_planes(points: np.ndarray, board: BoardConfig,
                       dist: float, min_frac: float) -> np.ndarray:
    """Iteratively strip planes whose inlier patch is far larger than a board."""
    diag = board.side_m * np.sqrt(2.0)
    remaining = points.astype(np.float64)
    for _ in range(6):
        if len(remaining) < 100:
            break
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(remaining))
        _, idx = pc.segment_plane(distance_threshold=dist, ransac_n=3,
                                  num_iterations=300)
        if len(idx) < max(100, int(min_frac * len(remaining))):
            break
        inliers = remaining[idx].astype(np.float32)
        ext = extent_2d(project_to_plane(inliers, fit_plane(inliers)))
        if ext <= 2.0 * diag:
            break  # largest remaining plane is board-scale: stop stripping
        mask = np.ones(len(remaining), dtype=bool)
        mask[idx] = False
        remaining = remaining[mask]
    return remaining.astype(np.float32)


def generate_cluster_after_ground(points: np.ndarray, board: BoardConfig,
                                  big_plane_dist: float = 0.05,
                                  big_plane_min_frac: float = 0.15,
                                  cluster_eps: float = 0.10,
                                  cluster_min_points: int = 30
                                  ) -> list[Candidate]:
    rest = _remove_big_planes(points, board, big_plane_dist,
                              big_plane_min_frac)
    if len(rest) < cluster_min_points:
        return []
    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(rest.astype(np.float64)))
    labels = np.asarray(pc.cluster_dbscan(eps=cluster_eps,
                                          min_points=cluster_min_points))
    out: list[Candidate] = []
    for lbl in np.unique(labels):
        if lbl < 0:
            continue
        cand = plausible_board_patch(rest[labels == lbl], board)
        if cand is not None:
            out.append(cand)
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_candidates_b.py -v`
Expected: 2 PASS.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): generator B (cluster after big-plane removal)"
```

---

### Task 8: Generator C (normal-based region growing)

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/candidates/region_growing.py`
- Test: `experiments/board-detection-2d/tests/test_candidates_c.py`

**Interfaces:**
- Produces: `generate_region_growing(points: np.ndarray, board: BoardConfig, knn: int = 16, angle_deg: float = 12.0, min_region: int = 40) -> list[Candidate]`
- Consumes: `Candidate`, `plausible_board_patch` (Task 6); open3d normal estimation + KDTree. (open3d has no built-in region growing — BFS implemented here.)

- [ ] **Step 1: Write the failing test**

```python
# tests/test_candidates_c.py
import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates.region_growing import generate_region_growing
from boarddet.geometry import downsample
from boarddet.synth import make_scene


def test_finds_board_region():
    pts, truth = make_scene(rng=np.random.default_rng(11))
    cands = generate_region_growing(
        downsample(pts, 0.03), BoardConfig(side_m=1.0))
    matches = [
        c for c in cands
        if abs(c.plane.normal @ truth.normal) > 0.99
        and abs((c.plane.center - truth.center) @ truth.normal) < 0.05
    ]
    assert matches


def test_separates_board_leaning_near_wall():
    # board 0.3 m in front of the wall, tilted 30 deg from wall normal:
    # clustering by distance would merge, normals must separate
    rng = np.random.default_rng(12)
    pts, truth = make_scene(board_center=(7.6, 0.5, 0.3),
                            rng=rng)
    cands = generate_region_growing(
        downsample(pts, 0.03), BoardConfig(side_m=1.0))
    matches = [c for c in cands
               if abs(c.plane.normal @ truth.normal) > 0.99]
    assert matches
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_candidates_c.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `region_growing.py`**

```python
# src/boarddet/candidates/region_growing.py
"""Approach C: grow regions of coherent normals (custom BFS; o3d lacks one)."""
from __future__ import annotations

from collections import deque

import numpy as np
import open3d as o3d

from . import Candidate, plausible_board_patch
from ..board_config import BoardConfig


def generate_region_growing(points: np.ndarray, board: BoardConfig,
                            knn: int = 16, angle_deg: float = 12.0,
                            min_region: int = 40) -> list[Candidate]:
    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(points.astype(np.float64)))
    pc.estimate_normals(
        o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    normals = np.asarray(pc.normals)
    tree = o3d.geometry.KDTreeFlann(pc)
    n_pts = len(points)
    cos_thresh = np.cos(np.radians(angle_deg))

    # precompute neighbor lists once
    neighbors = []
    for i in range(n_pts):
        _, idx, _ = tree.search_knn_vector_3d(pc.points[i], knn)
        neighbors.append(np.asarray(idx[1:]))

    visited = np.zeros(n_pts, dtype=bool)
    out: list[Candidate] = []
    for seed in range(n_pts):
        if visited[seed]:
            continue
        region = [seed]
        visited[seed] = True
        queue = deque([seed])
        while queue:
            cur = queue.popleft()
            for nb in neighbors[cur]:
                if visited[nb]:
                    continue
                if abs(normals[cur] @ normals[nb]) >= cos_thresh:
                    visited[nb] = True
                    region.append(int(nb))
                    queue.append(int(nb))
        if len(region) >= min_region:
            cand = plausible_board_patch(points[np.array(region)], board)
            if cand is not None:
                out.append(cand)
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_candidates_c.py -v`
Expected: 2 PASS. If `test_separates_board_leaning_near_wall` fails because regions merge, tighten `angle_deg` to 8.0 before touching anything else; record the value used.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): generator C (normal region growing)"
```

---

### Task 9: Detector glue with per-stage timing

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/detector.py`
- Test: `experiments/board-detection-2d/tests/test_detector.py`

**Interfaces:**
- Produces:
  - `GENERATORS: dict[str, callable]` — `{"a": generate_ransac_iterative, "b": generate_cluster_after_ground, "c": generate_region_growing}`
  - `DetectOutcome` dataclass: `detection: BoardDetection | None`, `timings_ms: dict[str, float]` (keys: `downsample`, `candidates`, `scoring`, `total`), `n_candidates: int`
  - `detect(points: np.ndarray, board: BoardConfig, generator: str, voxel: float = 0.03) -> DetectOutcome`
- Consumes: everything from Tasks 3–8.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_detector.py
import numpy as np
import pytest
from boarddet.board_config import BoardConfig
from boarddet.detector import GENERATORS, detect
from boarddet.synth import make_scene


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_detects_board_in_synthetic_scene(gen):
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0), generator=gen)
    assert out.detection is not None, f"generator {gen} found nothing"
    assert np.linalg.norm(out.detection.center - truth.center) < 0.05
    assert abs(out.detection.rotation[:, 2] @ truth.normal) > 0.99
    assert out.timings_ms["total"] > 0


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_no_detection_in_boardless_scene(gen):
    rng = np.random.default_rng(14)
    pts, _ = make_scene(rng=rng)
    # strip points near the board plane region entirely: keep clutter only
    keep = pts[:, 0] < 2.0
    out = detect(pts[keep], BoardConfig(side_m=1.0), generator=gen)
    assert out.detection is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_detector.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `detector.py`**

```python
# src/boarddet/detector.py
"""Glue: downsample -> candidate generator -> shared scorer -> best pose."""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .board_config import BoardConfig
from .candidates.cluster_after_ground import generate_cluster_after_ground
from .candidates.ransac_iterative import generate_ransac_iterative
from .candidates.region_growing import generate_region_growing
from .geometry import downsample, project_to_plane
from .pose import BoardDetection, board_pose
from .scorer import score_candidate

GENERATORS = {
    "a": generate_ransac_iterative,
    "b": generate_cluster_after_ground,
    "c": generate_region_growing,
}


@dataclass
class DetectOutcome:
    detection: BoardDetection | None
    timings_ms: dict[str, float]
    n_candidates: int


def detect(points: np.ndarray, board: BoardConfig, generator: str,
           voxel: float = 0.03) -> DetectOutcome:
    gen = GENERATORS[generator]
    t0 = time.perf_counter()
    dn = downsample(points, voxel)
    t1 = time.perf_counter()
    cands = gen(dn, board)
    t2 = time.perf_counter()
    best: BoardDetection | None = None
    for cand in cands:
        res = score_candidate(project_to_plane(cand.points, cand.plane),
                              board)
        if res is None or res.score < board.min_score:
            continue
        det = board_pose(cand.plane, res)
        if best is None or det.score > best.score:
            best = det
    t3 = time.perf_counter()
    return DetectOutcome(
        detection=best,
        timings_ms={
            "downsample": (t1 - t0) * 1e3,
            "candidates": (t2 - t1) * 1e3,
            "scoring": (t3 - t2) * 1e3,
            "total": (t3 - t0) * 1e3,
        },
        n_candidates=len(cands),
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_detector.py -v`
Expected: 6 PASS (3 generators × 2 tests).

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): detector glue with per-stage timing"
```

---

### Task 10: Visualization + benchmark CLI

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/viz.py`
- Create: `experiments/board-detection-2d/src/boarddet/benchmark.py`
- Test: `experiments/board-detection-2d/tests/test_benchmark.py`

**Interfaces:**
- Produces:
  - `viz.save_overlay(points: np.ndarray, outcome: DetectOutcome, path: Path) -> None` — 2-panel PNG: top-down scatter with quad overlay + scorer raster with refined corners.
  - `benchmark.run(datasets: list[int], generators: list[str], board: BoardConfig, max_frames: int | None, out_dir: Path) -> dict` — returns and writes `summary.json` + `summary.md` (detection-rate table, timing table, jitter table) + per-dataset overlay PNGs (first/middle/last detected frame).
  - CLI: `uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators a b c --out results/run1 [--max-frames N] [--side 1.0]`
  - Jitter metric per (dataset, generator): std over detected frames of `center` components (m) and of angle between per-frame board normal and the mean normal (deg).
- Consumes: `ingest.load_frames`, `detector.detect`, `DetectOutcome`.

- [ ] **Step 1: Write the failing test (synthetic — no pcap dependency)**

```python
# tests/test_benchmark.py
import numpy as np
from boarddet.benchmark import summarize
from boarddet.board_config import BoardConfig
from boarddet.detector import detect
from boarddet.synth import make_scene
from boarddet.viz import save_overlay


def test_save_overlay_writes_png(tmp_path):
    pts, _ = make_scene(rng=np.random.default_rng(15))
    out = detect(pts, BoardConfig(), generator="b")
    p = tmp_path / "overlay.png"
    save_overlay(pts, out, p)
    assert p.exists() and p.stat().st_size > 1000


def test_summarize_computes_rates_and_jitter():
    pts, _ = make_scene(rng=np.random.default_rng(16))
    outcomes = [detect(pts, BoardConfig(), generator="b") for _ in range(3)]
    s = summarize(outcomes)
    assert s["detection_rate"] == 1.0
    assert s["jitter_center_mm"] < 1e-6  # identical frames -> zero jitter
    assert s["median_total_ms"] > 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_benchmark.py -v`
Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `viz.py`**

```python
# src/boarddet/viz.py
"""Overlay renders for eyeballing detections."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .detector import DetectOutcome  # noqa: E402


def save_overlay(points: np.ndarray, outcome: DetectOutcome,
                 path: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    ax = axes[0]
    step = max(1, len(points) // 60_000)
    ax.scatter(points[::step, 0], points[::step, 1], s=0.5, c="gray",
               alpha=0.4)
    det = outcome.detection
    if det is not None:
        quad = np.vstack([det.corners_3d, det.corners_3d[:1]])
        ax.plot(quad[:, 0], quad[:, 1], "r-", lw=2)
        ax.plot(det.center[0], det.center[1], "r+", ms=12)
        ax.set_title(f"top-down | score={det.score:.2f}")
    else:
        ax.set_title("top-down | NO DETECTION")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    ax = axes[1]
    if det is not None:
        res = det.result
        ax.imshow(res.raster, cmap="gray", origin="lower")
        cell = (res.corners_2d - res.origin)  # plane coords -> px
        # raster was built with cell size board.cell_m; recover from origin
        # spacing: corners were computed in plane coords
        # plot via px transform stored implicitly: raster px = (c-origin)/cell
        # cell size = extent / raster width is unreliable; recompute:
        # ScoreResult keeps origin; cell_m comes from BoardConfig default.
        px = cell / 0.02
        ax.plot(np.append(px[:, 0], px[0, 0]),
                np.append(px[:, 1], px[0, 1]), "r-", lw=1.5)
        ax.set_title("plane raster + refined quad")
    else:
        ax.axis("off")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
```

Note: `save_overlay` hardcodes the default `cell_m=0.02` for the raster panel; if a non-default `cell_m` is benchmarked, thread it through (add `cell_m` field to `ScoreResult` instead — preferred if touched).

- [ ] **Step 4: Implement `benchmark.py`**

```python
# src/boarddet/benchmark.py
"""Benchmark CLI: all generators x all datasets -> tables + overlays."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .board_config import BoardConfig
from .detector import GENERATORS, DetectOutcome, detect
from .ingest import load_frames
from .viz import save_overlay


def summarize(outcomes: list[DetectOutcome]) -> dict:
    detected = [o for o in outcomes if o.detection is not None]
    rate = len(detected) / len(outcomes) if outcomes else 0.0
    totals = [o.timings_ms["total"] for o in outcomes]
    stage = {
        k: float(np.median([o.timings_ms[k] for o in outcomes]))
        for k in ("downsample", "candidates", "scoring", "total")
    }
    s: dict = {
        "n_frames": len(outcomes),
        "detection_rate": rate,
        "median_total_ms": stage["total"],
        "p95_total_ms": float(np.percentile(totals, 95)) if totals else 0.0,
        "median_stage_ms": stage,
        "median_candidates": float(np.median(
            [o.n_candidates for o in outcomes])) if outcomes else 0.0,
    }
    if len(detected) >= 2:
        centers = np.array([o.detection.center for o in detected])
        normals = np.array([o.detection.rotation[:, 2] for o in detected])
        # sign-align normals to the first
        normals *= np.sign(normals @ normals[0])[:, None]
        mean_n = normals.mean(axis=0)
        mean_n /= np.linalg.norm(mean_n)
        ang = np.degrees(np.arccos(np.clip(normals @ mean_n, -1, 1)))
        s["jitter_center_mm"] = float(centers.std(axis=0).mean() * 1e3)
        s["jitter_normal_deg"] = float(ang.std())
    return s


def _md_tables(all_results: dict) -> str:
    gens = sorted({g for d in all_results.values() for g in d})
    lines = ["# Benchmark results", "", "## Detection rate", "",
             "| Dataset | " + " | ".join(gens) + " |",
             "|---------|" + "---|" * len(gens)]
    for ds in sorted(all_results):
        row = [f"{all_results[ds][g]['detection_rate']:.0%}"
               if g in all_results[ds] else "—" for g in gens]
        lines.append(f"| {ds} | " + " | ".join(row) + " |")
    lines += ["", "## Median total ms (p95)", "",
              "| Dataset | " + " | ".join(gens) + " |",
              "|---------|" + "---|" * len(gens)]
    for ds in sorted(all_results):
        row = []
        for g in gens:
            r = all_results[ds].get(g)
            row.append(f"{r['median_total_ms']:.0f} ({r['p95_total_ms']:.0f})"
                       if r else "—")
        lines.append(f"| {ds} | " + " | ".join(row) + " |")
    lines += ["", "## Jitter: center mm / normal deg", "",
              "| Dataset | " + " | ".join(gens) + " |",
              "|---------|" + "---|" * len(gens)]
    for ds in sorted(all_results):
        row = []
        for g in gens:
            r = all_results[ds].get(g, {})
            if "jitter_center_mm" in r:
                row.append(f"{r['jitter_center_mm']:.1f} / "
                           f"{r['jitter_normal_deg']:.2f}")
            else:
                row.append("—")
        lines.append(f"| {ds} | " + " | ".join(row) + " |")
    return "\n".join(lines) + "\n"


def run(datasets: list[int], generators: list[str], board: BoardConfig,
        max_frames: int | None, out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    all_results: dict = {}
    for ds in datasets:
        frames = load_frames(ds, max_frames=max_frames)
        all_results[ds] = {}
        for g in generators:
            outcomes = [detect(f.xyz, board, generator=g) for f in frames]
            all_results[ds][g] = summarize(outcomes)
            det_idx = [i for i, o in enumerate(outcomes)
                       if o.detection is not None]
            picks = ({det_idx[0], det_idx[len(det_idx) // 2], det_idx[-1]}
                     if det_idx else {0, len(outcomes) // 2,
                                      len(outcomes) - 1})
            for i in sorted(picks):
                save_overlay(frames[i].xyz, outcomes[i],
                             out_dir / f"ds{ds}_{g}_frame{i:04d}.png")
            print(f"dataset {ds} gen {g}: "
                  f"rate={all_results[ds][g]['detection_rate']:.0%} "
                  f"median={all_results[ds][g]['median_total_ms']:.0f}ms")
    (out_dir / "summary.json").write_text(json.dumps(all_results, indent=2))
    (out_dir / "summary.md").write_text(_md_tables(all_results))
    return all_results


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", type=int, nargs="+",
                    default=[1, 2, 3, 4, 5])
    ap.add_argument("--generators", nargs="+", default=list(GENERATORS),
                    choices=list(GENERATORS))
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--side", type=float, default=1.0)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    run(args.datasets, args.generators, BoardConfig(side_m=args.side),
        args.max_frames, args.out)


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `uv run pytest tests/test_benchmark.py -v`
Expected: 2 PASS.

- [ ] **Step 6: Commit**

```bash
git add experiments/board-detection-2d
git commit -m "feat(phase-7): overlay viz + benchmark CLI with md/json summaries"
```

---

### Task 11: Run the benchmark on real data, iterate to first working result, fill phase doc

This task is exploratory by nature: real VLP-32C frames will surface issues the
synthetic tests cannot (ring stripes, board size mismatch with the actual board
in the recordings, clutter). The deliverable is a benchmark run + honest notes,
not perfect detection.

**Files:**
- Modify: `docs/roadmap/phase-7-projection-board-detection.md` (Results section)
- Possibly modify: any `boarddet` module (tuning), each change with its test updated

**Interfaces:**
- Consumes: everything.

- [ ] **Step 1: Smoke-run on dataset 3, few frames**

```bash
cd experiments/board-detection-2d
uv run python -m boarddet.benchmark --datasets 3 --generators b \
  --max-frames 10 --out results/smoke
```

Inspect `results/smoke/*.png`. The recorded board is the existing 1.0 m hollow
diamond — holes will appear in the raster; the scorer's outer-border fit must
still work (fill_ratio will be lower; if the fill term rejects the real board,
lower the fill weight or compute fill on the closed contour area only, and
update `test_scores_true_board_high` accordingly).

**Important unknown:** the true board side length in the recordings. Check
`ros/lctk_launch/config/board/board_detector.json5` (`board_width`, mm) and
pass it as `--side`. If detection still fails, view the overlay PNGs and debug
stage by stage (candidates found? scorer rejecting? why) before touching
thresholds.

- [ ] **Step 2: Full run, all datasets, all generators**

```bash
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 \
  --generators a b c --out results/run1
```

Expected wall time: minutes (C is the slow one — its per-point Python BFS may
be 10–100× slower than A/B; if a full run is impractical, run C with
`--max-frames 30` and note it in the results).

- [ ] **Step 3: Fill the phase doc Results tables**

Copy detection-rate / timing / jitter numbers from `results/run1/summary.md`
into `docs/roadmap/phase-7-projection-board-detection.md`. Add a short
narrative: which generator wins on rate, which on speed, notable failure modes
seen in overlays, and whether the 100 ms realtime budget is met.

- [ ] **Step 4: Solid-state stretch check (synthetic)**

```bash
uv run python - <<'EOF'
import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.detector import GENERATORS, detect
from boarddet.synth import make_scene

for gen in GENERATORS:
    ok = 0
    for seed in range(5):
        pts, truth = make_scene(pattern="uniform",
                                rng=np.random.default_rng(seed))
        out = detect(pts, BoardConfig(), generator=gen)
        if (out.detection is not None
                and np.linalg.norm(out.detection.center - truth.center) < 0.05):
            ok += 1
    print(f"generator {gen}: {ok}/5 uniform-pattern scenes detected")
EOF
```

Record the per-generator result in the phase doc (confirms nothing assumes
ring/grid structure).

- [ ] **Step 5: Commit results**

```bash
git add docs/roadmap/phase-7-projection-board-detection.md \
        experiments/board-detection-2d
git commit -m "docs(phase-7): benchmark results for generators A/B/C on datasets 1-5"
```

---

## Self-Review Notes

- Spec coverage: projection choice (Task 3–4), three generators (6–8), shared scorer (4), pose (5), benchmark protocol incl. jitter + overlays (10–11), solid-state synthetic check (Task 11 Step 4 + `pattern="uniform"` in Task 2), phase-doc results fill (11). Stage-2 ICP comparison is explicitly deferred in the phase doc — no task, by design.
- Real-data unknowns are isolated in Task 11 with explicit debugging guidance rather than pretending thresholds are final.
- Type consistency: `Candidate.points` (3D) → `project_to_plane(cand.points, cand.plane)` in detector; `ScoreResult.corners_2d` → `board_pose`; `DetectOutcome.timings_ms` keys used by `summarize` match detector.

---

# Stage 2 Addendum (2026-07-17): Accumulation + Emit-Best + Diamond-Stance

Approved follow-up after stage-1 results (B finds the true board but recall 0–8%;
border-only scoring passes board-sized flat panels). User confirmed the capture
workflow holds the board static for a few seconds per pose — temporal
accumulation is legitimate.

### Task 12: Frame accumulation + always-emit-best + diamond-stance score term

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/detector.py`
- Modify: `experiments/board-detection-2d/src/boarddet/board_config.py`
- Modify: `experiments/board-detection-2d/src/boarddet/benchmark.py`
- Test: `experiments/board-detection-2d/tests/test_detector.py`, `tests/test_benchmark.py`

**Interfaces:**
- `accumulate_frames(frames: list[Frame], n: int) -> list[np.ndarray]` (in
  `benchmark.py` or a small helper module): chunk consecutive frames into
  windows of n, concatenating xyz. Non-overlapping windows (matches "hold pose
  a few seconds" workflow). Last partial window kept if ≥ n/2 frames.
- `BoardConfig` gains `stance_weight: float = 0.0` (0 = term off; >0 blends).
- `detect(...)` unchanged signature; after `board_pose`, if
  `board.stance_weight > 0` compute stance and blend:
  `score *= (1 - w) + w * stance` where stance =
  `max(|d1 . z|, |d2 . z|)` over the two corner diagonals (unit vectors,
  z = [0,0,1] sensor-up). A diamond standing on its corner has one diagonal
  gravity-aligned → stance ≈ 1; an axis-aligned panel quad → both diagonals
  ~45° → stance ≈ 0.71. Best-candidate selection uses the blended score;
  `DetectOutcome.detection.score` carries it.
- `detect` gains `min_score` override behavior: keep gate, but `DetectOutcome`
  gains `best_rejected: BoardDetection | None` — the best candidate that
  scored below `min_score` (None if none or if a detection was accepted).
  Benchmark records its score so "emit best always" analysis is possible
  without changing the accept semantics.
- Benchmark CLI gains `--accumulate N` (default 1 = stage-1 behavior) and
  `--stance-weight W` (default 0.0). With `--accumulate N`, detection runs
  per window; summarize/jitter operate over windows. Overlay filenames gain
  window index instead of frame index. summary.json gains
  `accumulate`/`stance_weight` echo fields.

**Tests (TDD):**
- stance: build a synthetic scene, run detect with stance_weight=0.5 —
  detection still found, score finite; construct an axis-aligned square quad
  candidate synthetically and assert its stance < diamond stance (unit-test the
  stance function directly — factor it as `_stance(corners_3d) -> float`).
- accumulate_frames: 7 frames, n=3 → windows [3,3] (last 1 < n/2 dropped);
  window xyz length = sum of parts.
- best_rejected: scene with min_score forced to 0.99 → detection None,
  best_rejected not None, best_rejected.score < 0.99.

### Task 13: Stage-2 benchmark + phase doc results

Run on real data, B primarily (A/C one confirming run each):

```bash
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --accumulate 10 --stance-weight 0.0 --out results/run4-acc
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --accumulate 10 --stance-weight 0.5 --out results/run4-acc-stance
uv run python -m boarddet.benchmark --datasets 3 --generators c \
  --accumulate 10 --stance-weight 0.5 --max-frames 60 --out results/run4-c-check
```

Fill a "Stage 2 Results" section in the phase doc: recall per dataset
(windows detected / windows), timing per window (note: window = N frames →
budget is N×100 ms, still realtime for the workflow), jitter, whether
stance kills C's ds3 false positives, best_rejected score distribution
(how close the misses are). Honest narrative + updated Decision section.

---

# Stage 3 Addendum (2026-07-17): Anisotropic (DAC-style) Clustering

Stage-2 diagnosis: recall bottleneck is board-cluster fragmentation in DBSCAN
(fixed eps 0.15 vs multi-cm, range-proportional VLP-32C ring gaps). Related-work
survey: successful sparse-lidar methods never use fixed-eps Euclidean
clustering; the minimal drop-in fix is an elliptical neighborhood whose
vertical tolerance scales with range × vertical angular gap (DAC, Electronics
2021). Gravity-aware but ring-agnostic → solid-state-safe.

### Task 14: Anisotropic clustering in generator B

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/candidates/cluster_after_ground.py`
- Test: `experiments/board-detection-2d/tests/test_candidates_b.py`

**Interfaces:**
- New helper `_anisotropic_scaled(points: np.ndarray, eps_h: float, vertical_gap_deg: float) -> np.ndarray`:
  returns a scaled COPY for clustering only — z compressed per point by
  `eps_h / eps_v(r_i)` where `r_i = sqrt(x²+y²)` (horizontal range) and
  `eps_v(r) = max(eps_h, 2.0 * r * tan(radians(vertical_gap_deg)))`.
  Nearby points share similar range → locally consistent metric. Original
  coordinates untouched downstream (labels map back by index).
- `generate_cluster_after_ground` gains `vertical_gap_deg: float = 3.0`
  (0 disables → stage-2 behavior; VLP-32C worst adjacent-channel spacing ≈3°).
  Apply the scaling to BOTH the main DBSCAN clustering stage and the
  component-split DBSCAN inside `_remove_big_planes` (same fragmentation
  physics). The coplanar stripe-merge stage stays (harmless; may become
  mostly inactive).
- `detect()`/benchmark: pass-through — add `vertical_gap_deg` to BoardConfig
  (default 3.0) and thread it into generator B only (A/C unchanged this stage).
  CLI flag `--vertical-gap-deg` (0 = off).

**Tests (TDD):**
- Synthetic ring-striped board: sample the diamond in horizontal stripes with
  ~4° vertical angular gaps at ~3 m range (gaps ≈ 0.21 m > eps 0.15 so plain
  DBSCAN fragments it). Assert: with `vertical_gap_deg=0` generator B finds NO
  candidate matching the true plane (fragmentation reproduced — discrimination),
  with `vertical_gap_deg=3.5+` it DOES find the board candidate.
- Horizontal separation still tight: two boards side by side 0.5 m apart
  horizontally at same elevation must remain separate clusters under
  anisotropic scaling (no horizontal over-merge).
- Existing suite green (defaults must not break the synthetic-scene tests —
  note scene spacing 0.03 grid: anisotropic scaling at 3° only widens vertical
  tolerance beyond 0.15 for r > ~1.4 m, scene board at 4 m → verify tests
  still pass; if a stage-1 test breaks, investigate before touching it).

### Task 15: Stage-3 benchmark + phase doc

```bash
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --out results/run5-aniso
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --vertical-gap-deg 0 --out results/run5-control
```
(control isolates the anisotropic effect; single-frame, no accumulation.)

Fill "## Stage 3 Results" in the phase doc: recall per dataset vs stage-1/2,
timing, jitter (with n), pose sanity vs bbox reference on ds3, false-positive
check (stance 0.5 active), overlay inspection notes, honest narrative +
Decision update.

---

# Stage 4 Addendum (2026-07-17): Stripe-Tolerant Scorer

Stage-3 finding: anisotropic clustering delivers a board-region candidate to the
scorer on 31% of ds3 frames (vs 5%), but they score ~0.07 — the square 5×5
morphological close cannot bridge multi-cm ring stripes in the occupancy
raster, so fill/contour collapse. Stage 4 mirrors the anisotropic fix in
raster space: close with a vertically-elongated kernel, oriented by gravity.

### Task 16: Gravity-oriented anisotropic closing in the scorer

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/scorer.py`
- Modify: `experiments/board-detection-2d/src/boarddet/detector.py`
- Test: `experiments/board-detection-2d/tests/test_scorer.py`

**Interfaces:**
- `score_candidate(coords_2d, board, up_2d: np.ndarray | None = None, close_height_m: float | None = None) -> ScoreResult | None`
  - `up_2d`: unit 2D vector — direction in plane coords along which ring
    stripes are separated (projection of world +z onto the plane basis).
    None → stage-3 behavior (square kernel, no rotation).
  - `close_height_m`: physical vertical closing reach. None → stage-3 5×5.
- Internals when both provided: rotate coords so `up_2d` → +y (rotation
  matrix from the 2D vector), rasterize in rotated frame, morph close with
  kernel width 3 px, height `ceil(close_height_m / cell) | odd, min 5`,
  contour + minAreaRect in rotated frame, rotate quad corners BACK to
  original plane coords, then side-refit on the ORIGINAL raw coords_2d as
  today. `ScoreResult.raster`/`origin` may be in the rotated frame — add
  field `rot_2d: np.ndarray | None` (2×2) so viz can map corners; viz update
  optional (acceptable: overlay quad drawn from corners_2d in original frame;
  raster panel quad via rot_2d when present).
- Detector: for every candidate, compute `up_2d = normalize([z·u, z·v])`
  from `cand.plane` (skip/None if board plane near-horizontal — norm < 0.2)
  and `close_height_m = 2 * mean(horizontal range of cand.points) *
  tan(radians(board.vertical_gap_deg))`; pass both when
  `board.vertical_gap_deg > 0` (reuses the stage-3 flag; 0 disables both
  anisotropic stages). Applies to ALL generators (scorer stage is shared).
- Fill ratio: unchanged formula (occupied/total inside quad on the CLOSED
  raster) — the elongated kernel is what raises it on striped boards.

**Tests (TDD):**
- Striped 2D fixture: diamond points sampled in horizontal bands (gap ≈
  0.12 m > 5·cell) in a plane frame where stripes are NOT axis-aligned
  (apply a known in-plane rotation to the fixture, pass the matching up_2d).
  Assert: score with (up_2d, close_height_m=0.15) ≥ 2× score without;
  corners still within cell_m of truth (rotation round-trip exact).
- up_2d=None / close_height_m=None → byte-identical ScoreResult to stage 3
  on the standard dense fixture (regression pin).
- Degenerate: near-horizontal plane path (detector skips) — unit-test the
  detector helper that computes up_2d returns None when |proj| < 0.2.
- All existing tests green (defaults preserve old behavior at the scorer
  level; detector now passes the new args when vertical_gap_deg > 0 —
  existing detector tests run on synthetic dense boards where the elongated
  kernel must not change outcomes materially; investigate any flip).

### Task 17: Stage-4 benchmark + phase doc

```bash
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --out results/run6-stripe
uv run python -m boarddet.benchmark --datasets 3 --generators b \
  --stance-weight 0.5 --vertical-gap-deg 0 --out results/run6-control
```

Fill "## Stage 4 Results": recall vs stage-3 (expect the 31% candidate-reach
to convert), score distribution shift on board-region candidates, false-positive
impact (does the elongated kernel inflate clutter scores too? check ds5),
timing, overlays, honest narrative + Decision update.
