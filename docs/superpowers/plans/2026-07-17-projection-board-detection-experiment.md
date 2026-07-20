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

---

# Stage 5 Addendum (2026-07-18): Hole-Free Discrimination

Board will be a plain diamond WITHOUT holes (fabrication ease) — decided by
the user. Stage 4 left the bottleneck at pure discrimination: geometry-only
scorer cannot separate a 1 m diamond board from a 1 m flat background panel,
and the hole pattern (the strongest current cue) is being removed. Stage 5
attacks discrimination with hole-free cues and honestly bounds the
single-frame floor.

Constraint reminder: all cues must be sensor-generic (no ring field) and
gravity-aware at most — the design must survive on a solid-state lidar.

### Task 18: Characterize false positives + strict-diamond discriminator

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/scorer.py`
  (strengthen squareness/size/edge-support terms; expose weights)
- Modify: `experiments/board-detection-2d/src/boarddet/board_config.py`
- Test: `experiments/board-detection-2d/tests/test_scorer.py`

**Part A — characterization (report only, no code):**
Using the committed run6-stripe config, classify each of the 68 stage-4
false-positive detections into:
  (i) non-diamond — fails a *strict* squareness (per-corner angle within
      ±8° of 90°) OR stance (|best diagonal · up| > 0.9, i.e. within ~25° of
      gravity-vertical) OR size (mean side within ±8% of board.side_m);
  (ii) board-like — passes all three yet is not the true board (genuinely
      ambiguous single-frame).
Report the (i)/(ii) split per dataset. This quantifies the killable majority
vs the irreducible core BEFORE tuning.

**Part B — discriminator (implement, targeting the (i) group):**
Add to the scorer an `edge_support` term: for each of the 4 quad sides,
fraction of the side's length that has raw projected points within
`0.5·cell` of the side line (a real board has all 4 edges physically
present; a minAreaRect fit to a blob fragment has 1–2 unsupported sides).
Combine into the score alongside the existing fill/squareness/angle terms.
Expose `BoardConfig`:
  - `strict_squareness: bool = False` (off = stage-4 behavior; on = the ±8°
    angle gate + tightened size band `size_tol` and stance floor)
  - `stance_floor: float = 0.0` (0 = off; e.g. 0.9 = reject quads whose best
    diagonal is > ~25° off vertical — the diamond stands on a corner)
  - `edge_support_min: float = 0.0` (0 = off; e.g. 0.6 = each side ≥60%
    supported)
All new gates DEFAULT OFF so every existing test stays byte-identical; they
activate only when the benchmark sets them.

**Tests (TDD):**
- edge_support: synthetic full diamond → all 4 sides ~1.0; synthetic
  "fragment" (diamond with one side's points deleted) → that side ≈0, term
  rejects at edge_support_min=0.6.
- stance_floor: axis-aligned square quad (diagonals at 45°, |diag·up|≈0.707)
  rejected at stance_floor=0.9; true diamond (one diagonal vertical) passes.
- strict_squareness: a 15°-skewed quad rejected; a clean diamond passes.
- All defaults-off gates: existing suite byte-identical (regression pin).

### Task 19: Stage-5 benchmark + phase doc + honest floor

Task 18's reproduced gate ablation (see task-18-report.md's "Post-review
correction" section) found **two** operating points worth benchmarking, not
one — `--strict-diamond` alone tells only the max-precision/low-recall
half of the story:

```bash
# baseline (stage-4 config) already exists as run6-stripe.

# Operating point 1 (recommended): stance_floor=0.9 alone.
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --stance-gate --out results/run7-stance-gate

# Operating point 2: all four Task 18 gates together (max precision).
uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
  --stance-weight 0.5 --strict-diamond --out results/run7-strict
```
(`--stance-gate` and `--strict-diamond` CLI flags both already exist —
`--stance-gate` sets `stance_floor=0.9` alone; `--strict-diamond` sets
`strict_squareness=True, stance_floor=0.9, edge_support_min=0.6,
size_tol=0.08` together.)

Fill "## Stage 5 Results" with **both** runs side by side as a
precision/recall curve (baseline → stance-gate → strict-diamond), the
Part-A (i)/(ii) split, per-cue ablation (already reproduced once in
task-18-report.md — re-derive independently rather than copying), the
residual board-like-clutter count no single-frame geometry cue removes,
timing. Report plainly that:
- **`--stance-gate` retains full recall** (162→163/535 true-board
  detections — no measurable loss) **while cutting clutter 78%**
  (68→15/535), and is the recommended default operating point for
  single-frame precision/recall trade-offs — not `--strict-diamond`.
- **`--strict-diamond` reaches max precision (0 clutter) at a severe
  recall cost** (163→59/535, ~7:1 recall lost per extra FP caught,
  entirely attributable to `edge_support_min` — `strict_squareness`
  contributes nothing measured on real data, `side_tol=0.08` is cheap).
  ds5's true-board recall (7/103 under stance-gate) is wiped out
  entirely under strict-diamond, along with its clutter.
- The residual ~15 clutter detections under `--stance-gate` are
  dominated by ds5's persistent near-vertical clutter panel (a real,
  well-formed, ring-gap-striped planar surface — not a fragment/blob —
  which is why `edge_support` can eventually catch it but only by also
  catching genuine board detections that share the same coarse-binned
  edge-support signature). This is the concrete evidence pointing at the
  multi-pose/session cue (the panel is static across poses; the board
  moves between them) as the real fix, rather than a tighter single-frame
  geometric gate.

Then a DECISION subsection: state plainly whether single-frame hole-free
detection reaches usable precision at an acceptable recall cost — the
ablation says `--stance-gate` is usable *as a recall-preserving clutter
filter*, but does not fully solve precision (residual ~15/535, ds5-
dominated) without `--strict-diamond`'s much steeper recall cost — or
whether the multi-pose/session cue (board is the object that MOVES between
fixed poses; scene clutter does not) is REQUIRED to close that gap. The
latter is a capture-protocol change (record board at ≥2 positions) and a
future phase, not implementable on the current single-static-capture
datasets. Update the top-level Decision.

---

# Stage 6 Addendum (2026-07-19): Recall Recovery — Flatness Gate + Stance Floor

Stage-6 failure diagnosis (`.superpowers/sdd/stage6-failure-diagnosis.md`,
ds1-4, 432 frames, --stance-gate): recall misses are dominated by
C_FLATNESS (27.3%, a pure near-miss population RMS 0.035-0.048 vs the 0.035
gate) and F_SCORER_REJECT (22.9%, of which ~50% are killed by stance_floor).
Board is never stripped/merged/outscored (0%). The flatness gate
`_FLATNESS_RMS_MAX=0.035` is set right at the bottom of the real board's
planarity distribution — the C-04 "gate below the noise floor" mistake.
Diagnostic replay: raising it to 0.045 recovers 84/118 C_FLATNESS frames
(+19.4% absolute recall through the real downstream scorer/stance). Precision
cost UNMEASURED — a looser flatness gate admits flatter clutter too.

### Task 20: Make the flatness gate configurable + sweep-ready

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/candidates/__init__.py`
  (make `_FLATNESS_RMS_MAX` a `plausible_board_patch` parameter defaulting to
  a `BoardConfig` field)
- Modify: `experiments/board-detection-2d/src/boarddet/board_config.py`
  (`flatness_rms_max: float = 0.035` — default unchanged = current behavior)
- Modify: `experiments/board-detection-2d/src/boarddet/candidates/*.py`
  (thread `board.flatness_rms_max` into the `plausible_board_patch` calls in
  all three generators — the gate is shared)
- Modify: `experiments/board-detection-2d/src/boarddet/benchmark.py`
  (`--flatness-rms-max FLOAT` CLI flag → BoardConfig; echo in summary.json)
- Test: `experiments/board-detection-2d/tests/test_candidates_a.py` (or the
  gate's test home)

**Interfaces:**
- `plausible_board_patch(points_3d, board, flatness_rms_max=None)` — None →
  use `board.flatness_rms_max`; keeps the module constant as the ultimate
  default so nothing silently changes. Default 0.035 → every existing test
  byte-identical.
- CLI default 0.035 (current behavior); the sweep sets it explicitly.

**Tests (TDD):**
- a patch with RMS 0.040 is REJECTED at flatness_rms_max=0.035 but ACCEPTED
  at 0.045 (the gate is actually configurable and discriminates at the new
  value); default-None path uses the config field.
- existing suite byte-identical at the 0.035 default.

### Task 21: Stage-6 precision/recall sweep + phase doc + new-default decision

Sweep the two recall levers against precision on all 5 datasets, generator b:

```bash
# flatness sweep at the recommended stance-gate point:
for F in 0.035 0.040 0.045 0.050; do
  uv run python -m boarddet.benchmark --datasets 1 2 3 4 5 --generators b \
    --stance-weight 0.5 --stance-gate --flatness-rms-max $F \
    --out results/run8-flat$F
done
# stance-floor recall lever at the best flatness (set best F from above):
# (stance_floor is set by --stance-gate=0.9; to sweep it, add a
#  --stance-floor FLOAT override to benchmark.py in Task 20 if not already,
#  OR run the sweep in a scratch script over detect() — document which.)
```

For each (flatness, stance_floor) point compute true-board recall (in-bbox
/535), clutter FP count (out-of-bbox), precision. Classify residual clutter
(is the flatness relaxation admitting NEW clutter attractors, or just more
hits on the known ds5 panel?). Timing check (looser gate = more candidates
reach the scorer = more cost — measure it against the 100 ms budget). Fill
"## Stage 6 Results" in the phase doc: the precision/recall curve over the
sweep, the recommended new operating point (flatness value + stance_floor)
with its precision/recall/timing, and an honest statement of the
precision-for-recall trade. Update the top-level Decision. If the +19%
recall costs unacceptable precision, say so and keep 0.035 — report the
Pareto front, don't force a win.

---

# Stage 7 Addendum (2026-07-19): Generator B + Fixed-Size Square ICP Fitter

Direction (user): combine generator B's crop-box-free candidate segmentation
with a model-fit board fitter (the robustness trick the existing LCTK
crop-box+ICP pipeline uses), to recover the recall our 2D
rasterize-and-fit-shape loses on sparse frames. Board is a PLAIN SQUARE
(diamond, no holes). ICP is compute-heavy — bound it. Quick Python, no ROS.

Design decisions (fixed):
- **Refine-after-quad**: generator B → 2D quad (fast, localizes the easy
  frames + gives an init pose) → fixed-size square fit refines pose and
  rescues quad-rejected candidates. Reuses the whole pipeline; bounds compute.
- **Recall play, not precision**: a flat panel fits a square too, so the fit
  residual won't separate board from panel — stance + multi-pose still own
  discrimination. Success metric is recall gain at acceptable compute.
- **Fixed-size oriented-square fit, NOT literal filled-square ICP.** Literal
  ICP against a filled square stalls: interior points have zero
  point-to-model residual, so an enclosing init is a zero-gradient fixed
  point. Instead fit 3 DOF (center cx,cy, rotation θ) of a square whose side
  is HARD-PINNED to `board.side_m`, minimizing a coverage residual
  (points outside the square + square-edge band not reached by points). The
  known size is the strong low-DOF prior that makes sparse edge points pin
  the pose. Runs on raw plane-projected points — no raster, no contour, so no
  shape-recovery failure.

### Task 22: Re-diagnose recall failures at the stage-6 operating point (GATING)

Read-only analysis (no committed code). At the recommended operating point
(`vertical_gap_deg=3.0, stance_weight=0.5, stance_floor=0.9,
flatness_rms_max=0.045`), reproduce the stage-6 failure-bucket classification
(methodology in `.superpowers/sdd/stage6-failure-diagnosis.md`) over datasets
1–4. Report the bucket distribution NOW (post-stage-6), and specifically
size:
- **F_SCORER_REJECT** (candidate cluster exists + reaches the scorer + 2D fit
  rejected) — the bucket the square fitter can rescue.
- **A_FRAGMENTED** (no single cluster holds enough of the board) — upstream;
  the fitter CANNOT help.
- H_NO_BOARD_POINTS (data limit).

**Go/no-go**: if F_SCORER_REJECT is a substantial share of the remaining
misses, proceed to Task 23. If the remaining misses are dominated by
A_FRAGMENTED / H_NO_BOARD_POINTS, STOP and re-plan (the lever is clustering
or capture, not a fitter) — report that honestly. Write findings to
`.superpowers/sdd/stage7-rediagnosis.md`.

### Task 23: Fixed-size square fitter + refine-after-quad wiring

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/square_fit.py`
- Modify: `experiments/board-detection-2d/src/boarddet/detector.py`
  (rescue/refine path), `board_config.py` (config knobs)
- Test: `experiments/board-detection-2d/tests/test_square_fit.py`,
  `tests/test_detector.py`

**Interfaces:**
- `fit_fixed_square(coords_2d, side, init_center=None, init_theta=None,
  theta_window_deg=20.0) -> SquareFit | None` where
  `SquareFit = {center: (2,), theta: float, residual: float,
  corners_2d: (4,2)}`. Fits 3 DOF (center, θ) of a fixed-side-`side` square
  to the 2D points minimizing the coverage residual; θ searched in a bounded
  window around `init_theta` (from the quad) or, when init is None, from a
  coarse full sweep / PCA principal direction. Center is closed-form per θ
  (align the fixed-size box to the points' robust extent). None if too few
  points.
- Coverage residual (define precisely in impl): mean per-point distance
  *outside* the square (points should lie on the board) plus a coverage
  penalty = fraction of the square's perimeter band with no nearby point
  (the board should reach its own edges). Lower = better; this is both the
  ranking score and the acceptance metric.
- `BoardConfig`: `square_icp: bool = False` (off = stage-6 behavior),
  `square_icp_residual_max: float` (acceptance threshold, tune in Task 24).
- Detector integration (refine-after-quad), when `square_icp` on: for each
  candidate — run the 2D quad as today; if it yields a pose, seed
  `fit_fixed_square` with the quad's center/θ (refine); if the quad is
  rejected, seed from the candidate's PCA/centroid (rescue). Accept a
  detection when the square fit's residual < `square_icp_residual_max` AND
  the stance gate passes on the refined pose. Best candidate ranked by
  (lower) residual. `board_pose` builds the 3D pose from the refined
  `corners_2d` via the candidate's plane (same `unproject` path). All
  existing behavior byte-identical when `square_icp=False`.
- CLI `--square-icp` (+ `--square-icp-residual-max FLOAT`) in benchmark.py,
  echoed in summary.json.

**Tests (TDD):**
- fit_fixed_square recovers a known square's pose from full dense points
  (center/θ within tolerance, residual ~0).
- **sparse rescue**: points sampled only in a few horizontal stripes of the
  square (the sparse-ring failure case) — fit still recovers pose within
  tolerance where a free-size minAreaRect would under-estimate size; assert
  the fixed-size fit's recovered corners match the true square (not the
  shrunk point-extent).
- stall-avoidance: an init square already enclosing the points but rotated
  10° off — the fit corrects θ (proves it's not a filled-square-ICP
  fixed-point stall).
- residual discriminates: a genuine square vs a random blob of the same
  extent → blob residual >> square residual.
- detector: `square_icp=False` byte-identical to stage 6; `square_icp=True`
  finds the board on a synthetic scene.

### Task 24: Stage-7 benchmark + phase doc + decision

Benchmark recall + timing at the stage-6 operating point WITH `--square-icp`
(sweep `--square-icp-residual-max`) vs the stage-6 2D-only baseline, all 5
datasets, generator b. Per point: true-board recall (in-bbox/535), clutter
FP, precision, per-frame timing (measure the ICP compute cost against the
100 ms budget — this is the flagged pitfall), pose jitter (with n). Confirm
the recall gain comes from the F_SCORER_REJECT bucket Task 22 identified.
Fill "## Stage 7 Results": the recall/precision/timing comparison, how much
of the addressable bucket was recovered, the compute cost of ICP-refine,
honest tradeoff, recommended operating point, top-level Decision update. If
the fitter doesn't beat 2D-only, or the compute cost blows the budget, say so.

---

# Stage 8 Addendum (2026-07-20): Isolation / Depth-Discontinuity Discriminator

Stage 7 proved single-frame geometry can't SELECT the sparse board over compact
clutter by shape/residual/stance. Research (stage-7 follow-up survey): the one
unexploited, structurally-sound geometry signal is ISOLATION — a calibration
board is free-standing, so it has no coplanar continuation beyond its edges,
while clutter panels embedded in walls/structure do. Nobody packages this as a
detector; we assemble it. Diagnostic-first (stage-7 lesson: verify the
discriminator on REAL clutter before building).

### Task 25: Isolation-signal diagnostic (GATING, read-only)

For the true board and the known residual-clutter attractors (the ds5
persistent panel ~(-1.83,-2.89,-0.1), the second attractor y≈3.5/z≈-0.5, and
any others in the stage-4/5/7 residual lists), measure an ISOLATION SCORE and
test whether it separates board (isolated) from clutter (embedded).

Isolation score for a candidate (its fitted plane + 2D quad boundary):
- Work in the ORIGINAL voxel-downsampled cloud (BEFORE `_remove_big_planes` —
  the merged neighborhood, not the post-cluster remainder; embedded clutter's
  backing wall may have been stripped by clustering, so we must look at the
  raw cloud).
- Coplanar-continuation test: count/fraction of raw points that are (a)
  within ~2-3 cm of the fitted plane AND (b) just OUTSIDE the fitted quad
  boundary (an in-plane band, e.g. 0.05-0.30 m beyond each edge). Board →~0
  (nothing beyond its edges on its plane); embedded panel →many (the wall
  continues coplanar). Report as an exterior-band coplanar-point density and
  as a per-edge "fraction of the 4 edges with coplanar continuation."
- Secondary (per-ring depth-jump): for rings crossing the candidate, is there
  a range discontinuity at the angular boundary (board→background jump)?
  Report if cheap; the coplanar test is primary.

Deliverable → `.superpowers/sdd/stage8-isolation-diagnosis.md`: the isolation
score distribution for the true board (sample many frames/datasets) vs each
clutter attractor. GO/NO-GO: is there a threshold separating board (isolated)
from the clutter that currently beats us (embedded)? Quantify separation
(e.g. board exterior-density < X, clutter > Y, margin). If the clutter is ALSO
isolated (free-standing objects, not wall-embedded), NO-GO — say so plainly and
we pivot (background subtraction / hardware). Honesty mandate as always.

### Task 26 (if GO): Isolation score as a discrimination dimension

Add an `isolation` term to the detector: for each candidate, compute the
exterior-band coplanar density against the original downsampled cloud; fold
into acceptance (reject candidates with coplanar continuation = embedded) +
ranking. `BoardConfig.isolation: bool=False` (off=byte-identical) +
threshold knob. CLI `--isolation`. Detector must retain/pass the original
pre-cluster downsampled cloud to the scorer (new plumbing). Unit tests:
free-standing synthetic board (isolated, passes) vs a board flush against a
big coplanar wall (embedded, rejected). Byte-identical when off.

### Task 27 (if GO): Stage-8 benchmark + phase doc

Benchmark isolation ON vs the stage-6 operating point, all 5 datasets. Does
it kill the residual clutter (the ds5 panel etc.) while retaining board
recall? Precision/recall/timing. Fill "## Stage 8 Results" + Decision.
Honest: if it doesn't separate on real data, report the null.

---

# Stage 9 Addendum (2026-07-20): Lightweight CNN Board Detector — Feasibility Spike

Idea (user): train a lightweight CNN to locate the board. Assessment: the only
non-overfitting version is a full-scene RANGE-IMAGE detector trained on DIVERSE
SYNTHETIC data (our 5 static scenes would make a real-data-trained CNN memorize
rooms), validated on the real 535 frames as held-out. Patch-classifier on the
plane-projected board is dead (plain board ≡ plain clutter as a uniform square).
Before the full build (PyTorch + synth pipeline + training), a feasibility
spike de-risks two gates.

### Task 28: CNN feasibility spike (GATING, read-only/scratch)

Build a range-image renderer (scratch, not committed production code) and
answer two GO/NO-GO questions with visual evidence.

Range-image definition: rows = pseudo-ring from elevation `atan2(z, hypot(x,y))`
binned to ~32 rows (VLP-32C channels) — geometry-derived, no ring field;
cols = azimuth `atan2(y,x)` binned (e.g. 0.2-0.4° → ~900-1800 cols); pixel
value = range (and optionally a 2nd channel = a depth-discontinuity/edge map).

**Gate 1 — is the board learnable-in-principle in a REAL range image?**
For several real frames where the board is detected (in-bbox, ds1-4), render
the range image, locate the board's pixels (project the known bbox / detected
corners into range-image coords), and assess: does the board appear as a
COHERENT region a CNN could learn — a recognizable patch bounded by depth
discontinuities (the isolation signal, now in image form)? Or is it an
incoherent smear of a handful of pixels? Report the board's typical
pixel-footprint (rows × cols occupied), and whether its border shows the
depth-discontinuity signature in the range image. If the board is too
sparse/incoherent to be a learnable image region → NO-GO (no CNN can find
what isn't visible).

**Gate 2 — is the synth-to-real gap bridgeable?**
Render the same range-image representation for a `synth.py` scene. Compare
real vs synth side by side: board appearance, ring/row structure, sparsity,
clutter, noise. List concretely what synth CAPTURES vs MISSES (real elevation
nonuniformity, real noise, real clutter variety, occlusion, board mounting/
support). Estimate whether domain randomization + realistic ring modeling can
close the gap, or whether synth is so far from real that a synth-trained CNN
would learn artifacts.

**Deliverable**: save side-by-side real-vs-synth range-image PNGs (several
real board frames + synth) to a scratch/results dir; write
`.superpowers/sdd/stage9-cnn-spike.md` with the board pixel-footprint stats,
the two gate assessments, and a combined GO/NO-GO: GO only if (1) the board is
a coherent learnable region in real range images AND (2) the synth gap looks
bridgeable. Honesty mandate — if either gate fails, NO-GO and say why; a CNN
that can't see the board or can't transfer from synth is not worth building.

### Task 29 (if GO): synthetic range-image data pipeline + lightweight CNN
### Task 30 (if GO): train on synth, evaluate recall/precision on real 535 frames
(Detailed only if Task 28 says GO.)

---

# Stage 9 continued: Ray-Based LiDAR Simulator (unblocks the CNN)

Gate-2 failed because `synth.py` grid-samples in object space → range-image
aliasing. Fix: a ray-based VLP-32C simulator casting along the REAL beam angles
(VeloView-VLP-32C.yaml: 32 `vert_correction` elevations + `rot_correction`
azimuth offsets, copied into `experiments/board-detection-2d/VeloView-VLP-32C.yaml`).
Produces realistic range images + point clouds + per-board labels for CNN
training (and better synthetic tests for all prior stages). numpy-only; PyTorch
is later + separate.

### Task 29: Core simulator (sensor + primitives + ray-caster) — gated on Gate-2 re-test

**Files:** new subpackage `experiments/board-detection-2d/src/boarddet/sim/`:
`sensor.py` (beam model), `primitives.py` (rect/box/cylinder + ray-intersect),
`raycast.py` (scene → range image + point cloud). Tests `tests/test_sim.py`.

- `sensor.py`: load the 32 (elevation, az-offset) pairs from the yaml (parse
  once, or hardcode the 32 pairs as a constant with the yaml as provenance);
  `beam_directions(azimuth_steps) -> (32*N, 3)` unit ray dirs; config: azimuth
  resolution (~0.2°), min/max range.
- `primitives.py`: `Rect(center, normal, u_axis, half_u, half_v, holes=[])`
  (holes = list of (center_2d, radius) for hollow boards; empty = plain
  square), `Box(center, R, half_sizes)`, `Cylinder(base, axis, radius,
  height)`. Each: `intersect(ray_origins, ray_dirs) -> t array` (inf where no
  hit; respect bounds/holes). Vectorized over rays.
- `raycast.py`: `render(scene: list[primitive], sensor, noise, dropout) ->
  {range_image: (32,N), points: (M,3), hit_prim_id: ...}`. Nearest valid hit
  per ray; gaussian range noise; angle-dependent dropout (grazing incidence)
  + random dropout; min/max clip. Reuse the spike's range-image layout.
- **Acceptance = Gate-2 re-test**: build a scene approximating a real dataset
  (board ~1m at ~(2.1,-0.2), a ground plane + wall + a couple clutter panels),
  render, and compare the synthetic range image to a real one (save PNG).
  The synth image MUST now look smooth/coherent (no vertical-stripe aliasing),
  the board a coherent near-range region with a clean discontinuity border.
  A unit test asserts basic fidelity (board region present, footprint in the
  ~15-25 row range the spike measured, no all-empty-column aliasing pattern).
  If it still aliases, iterate before proceeding. Report the visual result.

Tests (TDD): ray-rect intersection correctness (known ray hits/misses a
positioned square + respects holes + bounds); ray-box, ray-cylinder;
nearest-hit selection over multiple primitives; dropout/noise statistics;
the Gate-2 fidelity assertion.

### Task 30: Shared range-image renderer + scene generation + labeled dataset

Task-29 review flagged two must-fix-before-training items, baked in here:
- **Row-semantics linchpin**: the sim's range-image rows are real per-channel
  elevations; the spike's real-frame renderer used uniform bins. Train-on-sim /
  eval-on-real would then have DIFFERENT row axes -> CNN fails for a dumb
  reason. Fix: ONE shared renderer used by BOTH sim output and real frames,
  binning every point to its NEAREST REAL CHANNEL (the 32 real elevations from
  the sensor model). Identical row axis for train and eval.
- **Diamond boards**: scenes must orient boards as the real 45deg-rotated
  diamond (1.41 m diagonal extent), not an axis-aligned square.

**Files:** `src/boarddet/sim/range_image.py` (shared renderer),
`src/boarddet/sim/scenegen.py`, `src/boarddet/sim/dataset.py`; tests.

1. `range_image.py`: `to_range_image(points, sensor, azimuth_steps,
   channels=2) -> (H,W,C)` — assign each point to nearest real channel
   (row) + azimuth bin (col); value = range (+ optional discontinuity
   channel). Used by BOTH `raycast.render` output AND real `Frame.xyz`
   (CNN eval). Validate: real-frame range image via this renderer matches
   the sim's row convention exactly; re-do the Gate-2 comparison with a
   DIAMOND board and this shared renderer (honest fidelity, per review #1).
2. `scenegen.py`: `random_scene(rng, cfg) -> (scene, board_poses)` —
   domain-randomized: ground (random z), 2-4 walls, N_board in {1,2,3}
   diamond boards (random pose 2-8 m, orientation facing-ish sensor,
   in-plane rot, side ~1 m +/- jitter, plain or hollow), M panel clutter
   (MIX of embedded-coplanar-with-wall and free-standing -- so the CNN
   learns isolation), K boxes, L cylinders. Randomize sensor tilt/height.
3. `dataset.py`: generate a labeled dataset -> per scene: range image +
   per-board label (pixel mask / bbox in image coords / 3D pose). Dump
   to disk (npz/npy, gitignored). Config for count.

Tests: nearest-channel binning correctness (a point at a known elevation
lands in the right row); real+sim share row axis; scenegen produces valid
scenes with the requested board count + clutter mix; labels align with the
rendered board pixels (a labeled board mask actually covers board hits).

### Tasks 31+: lightweight CNN train-on-synth / eval-on-real
Add PyTorch (uv), small heatmap/U-Net detector, train on the synth dataset,
evaluate recall/precision on the real 535 frames (held-out) via the SHARED
range-image renderer (same row axis). Compare to the geometry pipeline's
44-49%/93-100%.

### Tasks 31+: lightweight CNN train-on-synth / eval-on-real
(Detailed after a labeled dataset exists.)

---

# Stage 9 CNN Training (Tasks 31-33): lightweight U-Net, train on synth, eval on real

Simulator complete + vetted (plain diamonds, facing-with-tilt never edge-on,
varied pose, non-overlapping, >=70% vertical laser coverage, realistic
physics/noise, weighted 0/1/2/3 board counts). Now the model. GPU: RTX 5090
(Blackwell/sm_120) — needs a recent CUDA torch build.

Approved design: range image -> lightweight U-Net -> per-pixel board mask ->
connected components -> per-instance points -> reuse `square_fit.py` for pose.
CNN does SELECTION (the wall geometry couldn't cross), geometry does POSE.

### Task 31: PyTorch dep + data pipeline (GATED on batch sanity)
- Add `torch` (GPU, sm_120/RTX-5090 compatible: try stable cu12x wheel; if it
  won't run on sm_120, use the nightly cu12x build; verify `cuda.is_available()`
  and a GPU matmul actually runs, else document CPU fallback). `uv add torch`.
- `src/boarddet/cnn/data.py`:
  - `SynthBoardDataset(torch.utils.data.Dataset)` — on-the-fly: each item
    generates a random scene (`random_scene` + `render` + `to_range_image`,
    passing the sensor) -> input `(3,32,W)` + target mask `(1,32,W)`.
    Channels: [0] normalized range (r/R_max clipped, 0 at no-return),
    [1] validity (1=return), [2] discontinuity. Target = union of board masks.
    Augmentation: circular azimuth-roll + horizontal flip (both roll input AND
    mask). Include empty scenes (all-zero mask) per the count weights.
  - `real_frame_to_input(frame, sensor, ...)` -> the IDENTICAL input tensor from
    a real `Frame.xyz` via the SAME `to_range_image` + normalization (the
    train/eval consistency linchpin — must match synth exactly).
- GATE: save a PNG of one batch (3 input channels + target-mask overlay) and a
  real-frame input side-by-side with a synth input; confirm channel layout +
  normalization identical and the mask covers board pixels. Tests: shapes,
  mask alignment, synth/real channel-stat parity.

### Task 32: Model + training loop
- `src/boarddet/cnn/model.py`: the lightweight U-Net (enc 16/32/64, dilated
  bottleneck for isolation context, circular width padding, gentle vertical
  pooling, 1-ch sigmoid). Report param count (<~1M target).
- `src/boarddet/cnn/train.py`: Dice+BCE loss, Adam, train on-the-fly synth,
  validate on a fixed held-out synth seed set (IoU / mask precision-recall),
  checkpoint best, log curves. GPU.
- Deliverable: trained checkpoint + synth val metrics. SANITY: if it can't
  learn the synth board mask (val IoU stays low), STOP and debug — don't
  proceed to real eval on a model that didn't learn.

### Task 33: Eval on real + phase doc + decision (THE result)
- `src/boarddet/cnn/eval.py`: checkpoint -> per real frame: input ->
  model -> mask -> threshold -> connected components -> per-instance board
  pixels -> back-project to 3D (row/col->ray via sensor) -> `square_fit.py`
  pose -> accept. Classify at the real board bbox; recall/precision over the
  535 frames vs the geometry baseline (44-49% recall / 93-100% precision).
- Report the synth->real transfer honestly. Fill "## Stage 9 CNN Results" +
  top-level Decision. If it doesn't transfer, report the null straight (as
  stages 2/3/7 did) and diagnose (domain gap? normalization? mask ok but pose
  fails?).

---

# Stage 10 (Tasks 34-36): Hybrid CNN-propose -> geometry-verify

CNN broke the recall ceiling (99.3% on real) but 15% precision (fires on ~4
wall-embedded real clutter fixtures). Research (hybrid-detector literature)
strongly favors: CNN PROPOSES (recall) -> geometric ISOLATION gate VERIFIES
(precision), single pass, verify only proposals (negligible compute; the CNN
REPLACES geometry's expensive scene-wide candidate generation). Robust to
UNSEEN clutter (isolation rejects anything wall-embedded regardless of
training) -- unlike closed-set fixes. Published precedent: VoxelNet-segment +
RANSAC/PCA-verify LiDAR calibration board (MDPI Sensors 2025). Complement with
richer synthetic clutter (raise CNN intrinsic precision, shrink verifier load).

### Task 34: Validate the load-bearing assumption (GATING, read-only)
The isolation verifier's 44% STANDALONE recall is likely a search/localization
artifact, NOT a confirmation-accuracy one. As a GATE on the CNN's already-
localized true-board proposals it only needs a low false-NEGATIVE rate on
genuine boards. Check cheaply BEFORE building:
- Run the CNN on the 535 real frames -> detections (components + back-projected
  3D points). For the IN-BBOX (true-board) detections, compute the stage-8
  isolation density (exterior coplanar continuation, against the pre-strip
  cloud) on their points. What fraction does an isolation threshold ACCEPT
  (keep)? -> the combined-pipeline recall proxy (want ~high).
- For the OUT-OF-BBOX (clutter FP) detections, what fraction does isolation
  REJECT? -> the combined-pipeline precision proxy (want ~high; the ~4 fixtures
  are wall-embedded so should be rejected).
- Report projected combined recall/precision = (CNN recall x isolation-accepts-
  true) / (isolation-rejects-clutter). GO if the projection beats the geometry
  baselines (49.3%/93%, 44.1%/100%) meaningfully; if isolation rejects too many
  true boards (localization-recall confound bites) or fails to reject the CNN's
  specific clutter, diagnose + re-plan. Write .superpowers/sdd/stage10-hybrid-gate.md.

### Task 35 (if GO): Build the single hybrid pipeline
`cnn/hybrid.py`: real frame -> CNN forward -> mask -> components -> per
candidate: back-project -> isolation gate (reject embedded) -> survivors ->
plane + square_fit pose -> accept. Isolation prunes BEFORE the costly square_fit
(fixes Task-33's 89ms). Config knobs (isolation threshold). Tests: on synth +
a held-out check. Optionally regenerate synth with the ~4 real fixture shapes
added to clutter (option c complement) + note.

### Task 36 (if GO): Eval hybrid on real + phase doc + decision
Combined pipeline on 535 real frames: recall/precision/timing vs CNN-alone
(99.3%/15%) and geometry (49.3%/93%, 44.1%/100%). THE result: does CNN-recall x
geometry-precision give high-BOTH in one cheap pass? Fill phase doc + top-level
Decision. Honest if it doesn't.

---

# Stage 10 revised (Tasks 35-36): fix the CNN via richer synthetic clutter

Task 34/34b diagnostics: the isolation hybrid + any cheap geometric verifier
FAIL — the CNN's false positives are a BROAD free-standing-object population
(28% small scatter ~0.2-0.5m = poles/brackets; 72% large, median diagonal
1.71m > the board), none separable from the diamond board by
size/stance/square-residual/isolation. Root cause: synthetic clutter had only
wall-embedded panels + modest boxes/cylinders — NO diverse free-standing
distractors — so the CNN never learned "diamond board vs arbitrary
free-standing object." Fix = option (c), single CNN pass, no inference geometry.

### Task 35: Enrich synthetic clutter with diverse free-standing distractors
`scenegen.py`: broaden the clutter distribution so the CNN sees non-board
free-standing objects across the real size/shape range:
- SMALL scatter distractors: thin vertical poles/brackets, small panels
  (0.1-0.5 m), clusters of a few small objects (mimic the pole/bracket FPs).
- LARGE free-standing structures: big panels/boxes (1.5-2.5 m — the 1.71m
  "other_clutter"), currently uncovered.
- Varied aspect-ratio rectangular (non-square) panels, varied orientation
  (NOT constrained to face the sensor like the board is — clutter can be any
  orientation, incl. edge-on).
- Keep the board the plain diamond (facing-with-tilt, coverage-gated); make
  ONLY the board a diamond. Config knobs for the new distractor types/counts.
Tests: enriched scenes contain the new distractor size range; board still
distinct; existing scene invariants hold. Visual: a gallery showing diverse
clutter. Optionally a light connected-component merge-split fix in the eval
(the mask-merge artifact) — or note it for Task 36.

### Task 36: Retrain on enriched synth + eval on real + decision (THE result)
Retrain BoardUNet on the enriched synth (plain+hollow board mix, diverse
free-standing clutter). Re-run the Task-33 real eval on 535 frames:
recall/precision/timing vs CNN-before (99.3%/15%), geometry (49.3%/93%,
44.1%/100%). Did richer clutter lift precision while keeping ~99% recall?
Fill phase doc "## Stage 10 Results" + top-level Decision. Honest: if precision
doesn't lift enough, report the residual clutter it still fires on and whether
it's now covered/coverable, or whether multi-pose is the remaining fundamental
lever. Save real-pred overlays.
