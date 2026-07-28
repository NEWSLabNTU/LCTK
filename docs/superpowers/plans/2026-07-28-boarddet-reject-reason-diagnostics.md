# boarddet Reject-Reason Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When `detect()` returns no board, surface one structured `RejectReason` naming the furthest-progressing candidate's killer gate, its governing `BoardConfig` param (or `None` if structural), and a normalized margin past the threshold.

**Architecture:** A side-channel collector. A new zero-dependency `reject.py` defines a `Stage` IntEnum and a frozen `RejectReason` dataclass with `upper`/`lower`/`band` builders. Every existing gate keeps its `... | None` return; when an optional `rejects: list[RejectReason] | None` kwarg is threaded in, each reject-site appends a `RejectReason` before returning. `detect()` owns the list, folds it to the max-`stage` entry, and exposes it on `DetectOutcome.reject_reason`. When the kwarg is omitted, everything is byte-identical to today.

**Tech Stack:** Python 3, `uv`, `pytest`, numpy, OpenCV (`cv2`). No new dependencies.

## Global Constraints

- Run everything from `experiments/board-detection-2d/`. Tests: `uv run pytest`.
- **Byte-identical passing path:** when the `rejects` kwarg is omitted, control flow, return values, and detection outputs must be identical to pre-change. Existing tests are the guard — do not modify their assertions except the explicit `is None` → collector updates called out in Task 5.
- `reject.py` has **zero** intra-package imports (imported by `scorer`, `candidates`, `detector` — must not create an import cycle).
- `RejectReason` is `@dataclass(frozen=True)`.
- `margin` is always `>= 0` for a reject. Guard division: if a threshold is `0.0`, `margin = 0.0`.
- Named params in f-strings: `f"{e}"` not `f"{}", e`.
- Never edit files outside `experiments/board-detection-2d/` except this plan's checkboxes.

---

### Task 1: `reject.py` — Stage, RejectReason, builders

**Files:**
- Create: `src/boarddet/reject.py`
- Test: `tests/test_reject.py`

**Interfaces:**
- Consumes: nothing (zero intra-package imports).
- Produces:
  - `class Stage(IntEnum)` with members: `NO_CLUSTERS=0, PATCH_POINTS=1, PATCH_FLATNESS=2, PATCH_EXTENT=3, MIN_POINTS=11, RASTER_SIZE=12, MINAREA_SIZE=13, SIZE_GATE=14, STRICT_SQUARENESS=15, STANCE_2D=16, EDGE_SUPPORT=17, SIDE_ERR=18, SQUARE_FIT=21, MIN_SCORE=22, STANCE_3D=23, ISOLATION=24`.
  - `@dataclass(frozen=True) class RejectReason` with fields `stage: Stage`, `gate: str`, `param: str | None`, `value: float | None`, `threshold: float | tuple[float, float] | None`, `margin: float`.
  - `def upper(stage: Stage, gate: str, param: str | None, value: float, thr: float) -> RejectReason`
  - `def lower(stage: Stage, gate: str, param: str | None, value: float, thr: float) -> RejectReason`
  - `def band(stage: Stage, gate: str, param: str | None, value: float, lo: float, hi: float) -> RejectReason`
  - `def furthest(rejects: list[RejectReason]) -> RejectReason | None` — max `stage`, tie → first seen; `None` on empty list.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_reject.py
import pytest

from boarddet.reject import RejectReason, Stage, band, furthest, lower, upper


def test_stage_bands_order():
    # generation < scorer < detector, monotonic within a path
    assert Stage.PATCH_EXTENT < Stage.MIN_POINTS
    assert Stage.SIDE_ERR < Stage.SQUARE_FIT
    assert Stage.MIN_POINTS < Stage.MIN_SCORE < Stage.ISOLATION


def test_upper_margin():
    r = upper(Stage.PATCH_FLATNESS, "flatness", "flatness_rms_max", 0.07, 0.035)
    assert r.stage is Stage.PATCH_FLATNESS
    assert r.param == "flatness_rms_max"
    assert r.value == 0.07
    assert r.threshold == 0.035
    assert r.margin == pytest.approx(1.0)  # (0.07-0.035)/0.035


def test_lower_margin():
    r = lower(Stage.MIN_SCORE, "min_score", "min_score", 0.4, 0.5)
    assert r.margin == pytest.approx(0.2)  # (0.5-0.4)/0.5


def test_band_margin():
    r = band(Stage.PATCH_EXTENT, "extent", None, 0.2, 0.5, 2.5)
    # dist_outside = 0.5-0.2 = 0.3 ; half-width = (2.5-0.5)/2 = 1.0
    assert r.margin == pytest.approx(0.3)
    assert r.param is None


def test_margin_zero_threshold_guard():
    r = lower(Stage.STANCE_2D, "stance", "stance_floor", 0.0, 0.0)
    assert r.margin == 0.0


def test_furthest_picks_max_stage_first_on_tie():
    a = upper(Stage.PATCH_FLATNESS, "flatness", "flatness_rms_max", 0.07, 0.035)
    b = lower(Stage.MIN_SCORE, "min_score", "min_score", 0.4, 0.5)
    c = lower(Stage.MIN_SCORE, "min_score", "min_score", 0.1, 0.5)
    assert furthest([a, b, c]) is b   # max stage 22, first of the two
    assert furthest([]) is None


def test_frozen():
    r = upper(Stage.SIZE_GATE, "size", "side_tol", 2.0, 1.0)
    with pytest.raises(Exception):
        r.stage = Stage.MIN_POINTS  # frozen
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_reject.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'boarddet.reject'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/boarddet/reject.py
"""Structured reject reasons for the board detector.

Zero intra-package imports on purpose: this module is imported by scorer,
candidates, and detector, and must not create an import cycle. A gate records
*why* it rejected without changing its accept/reject decision or return type;
the collector is a side channel threaded in as an optional `rejects` list.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class Stage(IntEnum):
    # generation band (cluster -> candidate), gapped below the scorer band
    NO_CLUSTERS = 0        # generator emitted zero clusters at all
    PATCH_POINTS = 1       # patch < _MIN_PATCH_POINTS
    PATCH_FLATNESS = 2     # plane_rms > flatness_rms_max
    PATCH_EXTENT = 3       # extent outside [0.5*side, 1.8*diag]
    # scorer band
    MIN_POINTS = 11        # coords < _MIN_POINTS
    RASTER_SIZE = 12       # raster > 4000 px
    MINAREA_SIZE = 13      # minAreaRect side too small / no contour
    SIZE_GATE = 14         # coarse mean side out of 2*side_tol band
    STRICT_SQUARENESS = 15  # max corner angle dev > 8 deg
    STANCE_2D = 16         # 2D diamond stance <= stance_floor
    EDGE_SUPPORT = 17      # min side support < edge_support_min
    SIDE_ERR = 18          # |mean side - side_m| > side_tol*side_m
    # detector band
    SQUARE_FIT = 21        # icp: fit None or residual >= square_icp_residual_max
    MIN_SCORE = 22         # non-icp: det.score < min_score
    STANCE_3D = 23         # icp: 3D stance <= stance_floor
    ISOLATION = 24         # both paths: density > isolation_max_density


@dataclass(frozen=True)
class RejectReason:
    stage: Stage
    gate: str
    param: str | None
    value: float | None
    threshold: float | tuple[float, float] | None
    margin: float


def _safe_div(num: float, den: float) -> float:
    return 0.0 if den == 0 else num / den


def upper(stage: Stage, gate: str, param: str | None,
          value: float, thr: float) -> RejectReason:
    """Gate that rejects when value > thr."""
    return RejectReason(stage, gate, param, float(value), float(thr),
                        max(0.0, _safe_div(float(value) - float(thr), float(thr))))


def lower(stage: Stage, gate: str, param: str | None,
          value: float, thr: float) -> RejectReason:
    """Gate that rejects when value < thr."""
    return RejectReason(stage, gate, param, float(value), float(thr),
                        max(0.0, _safe_div(float(thr) - float(value), float(thr))))


def band(stage: Stage, gate: str, param: str | None,
         value: float, lo: float, hi: float) -> RejectReason:
    """Gate that rejects when value is outside (lo, hi)."""
    v, lo, hi = float(value), float(lo), float(hi)
    dist = (lo - v) if v < lo else (v - hi)
    half = (hi - lo) / 2.0
    return RejectReason(stage, gate, param, v, (lo, hi),
                        max(0.0, _safe_div(dist, half)))


def furthest(rejects: list[RejectReason]) -> RejectReason | None:
    """The reject that reached the highest stage; ties keep the first seen."""
    best: RejectReason | None = None
    for r in rejects:
        if best is None or r.stage > best.stage:
            best = r
    return best
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_reject.py -v`
Expected: PASS (all 7 tests)

- [ ] **Step 5: Commit**

```bash
git add src/boarddet/reject.py tests/test_reject.py
git commit -m "feat(boarddet): reject.py — Stage/RejectReason/margin builders"
```

---

### Task 2: `plausible_board_patch` collects patch-stage rejects

**Files:**
- Modify: `src/boarddet/candidates/__init__.py`
- Test: `tests/test_candidates_a.py` (add cases; do not weaken existing)

**Interfaces:**
- Consumes: `boarddet.reject.{Stage, RejectReason, upper, band}` from Task 1.
- Produces: `plausible_board_patch(points_3d, board, flatness_rms_max=None, rejects: list[RejectReason] | None = None) -> Candidate | None`. Appends to `rejects` (when not None) at its 3 return-None sites: `PATCH_POINTS` (param `None`), `PATCH_FLATNESS` (param `flatness_rms_max`, upper), `PATCH_EXTENT` (param `None`, band over `(0.5*side_m, 1.8*diag)`). `_MIN_PATCH_POINTS` unchanged.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_candidates_a.py  (append)
import numpy as np

from boarddet.board_config import BoardConfig
from boarddet.candidates import Candidate, plausible_board_patch
from boarddet.reject import Stage


def _flat_square(side=1.0, n=40, z=0.0):
    g = np.linspace(-side / 2, side / 2, n)
    xx, yy = np.meshgrid(g, g)
    pts = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, z)])
    return pts


def test_patch_reject_too_few_points():
    board = BoardConfig()
    rejects = []
    out = plausible_board_patch(np.zeros((10, 3)), board, rejects=rejects)
    assert out is None
    assert len(rejects) == 1
    assert rejects[0].stage is Stage.PATCH_POINTS
    assert rejects[0].param is None


def test_patch_reject_not_flat():
    board = BoardConfig()
    pts = _flat_square()
    pts[:, 2] += np.random.default_rng(0).normal(0, 0.2, len(pts))  # thick
    rejects = []
    out = plausible_board_patch(pts, board, rejects=rejects)
    assert out is None
    assert rejects[-1].stage is Stage.PATCH_FLATNESS
    assert rejects[-1].param == "flatness_rms_max"
    assert rejects[-1].margin > 0


def test_patch_reject_wrong_extent():
    board = BoardConfig(side_m=1.0)
    pts = _flat_square(side=0.1)  # far too small in extent
    rejects = []
    out = plausible_board_patch(pts, board, rejects=rejects)
    assert out is None
    assert rejects[-1].stage is Stage.PATCH_EXTENT
    assert rejects[-1].param is None


def test_patch_accept_collects_nothing():
    board = BoardConfig(side_m=1.0)
    pts = _flat_square(side=1.0)
    rejects = []
    out = plausible_board_patch(pts, board, rejects=rejects)
    assert isinstance(out, Candidate)
    assert rejects == []


def test_patch_kwarg_omitted_byte_identical():
    board = BoardConfig(side_m=1.0)
    pts = _flat_square(side=1.0)
    assert isinstance(plausible_board_patch(pts, board), Candidate)
    assert plausible_board_patch(np.zeros((10, 3)), board) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_candidates_a.py -v -k patch`
Expected: FAIL — `plausible_board_patch() got an unexpected keyword argument 'rejects'`

- [ ] **Step 3: Write minimal implementation**

Modify `src/boarddet/candidates/__init__.py`. Add the import at top (after existing imports):

```python
from ..reject import RejectReason, Stage, band, upper
```

Change the signature and the 3 return sites:

```python
def plausible_board_patch(points_3d: np.ndarray, board: BoardConfig,
                          flatness_rms_max: float | None = None,
                          rejects: list[RejectReason] | None = None
                          ) -> Candidate | None:
    """Gate a 3D patch: enough points, flat, board-sized. None if implausible.

    flatness_rms_max=None (default) reads board.flatness_rms_max (Task 20),
    falling back to the module constant if board lacks that attribute --
    board.flatness_rms_max itself defaults to _FLATNESS_RMS_MAX, so the
    default call path is byte-identical to pre-Task-20 behavior.

    rejects, when given, collects a RejectReason at each gate that fires
    (side channel; does not change the accept/reject decision or return type).
    """
    if len(points_3d) < _MIN_PATCH_POINTS:
        if rejects is not None:
            rejects.append(lower_points(len(points_3d)))
        return None
    threshold = flatness_rms_max
    if threshold is None:
        threshold = getattr(board, "flatness_rms_max", _FLATNESS_RMS_MAX)
    plane = fit_plane(points_3d)
    rms = plane_rms(points_3d, plane)
    if rms > threshold:
        if rejects is not None:
            rejects.append(upper(Stage.PATCH_FLATNESS, "flatness",
                                 "flatness_rms_max", rms, threshold))
        return None
    ext = extent_2d(project_to_plane(points_3d, plane))
    diag = board.side_m * np.sqrt(2.0)
    lo, hi = 0.5 * board.side_m, 1.8 * diag
    if not (lo <= ext <= hi):
        if rejects is not None:
            rejects.append(band(Stage.PATCH_EXTENT, "extent", None, ext, lo, hi))
        return None
    return Candidate(points=points_3d, plane=plane)
```

`PATCH_POINTS` is a count gate, not a ratio; add a tiny local builder near the top of the module (below the imports) rather than abusing `lower`:

```python
def lower_points(n: int) -> RejectReason:
    """PATCH_POINTS reject: structural count gate, no tunable param, margin 0."""
    return RejectReason(Stage.PATCH_POINTS, "patch_points", None,
                        float(n), float(_MIN_PATCH_POINTS), 0.0)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_candidates_a.py -v`
Expected: PASS (new patch tests + all pre-existing tests in the file)

- [ ] **Step 5: Commit**

```bash
git add src/boarddet/candidates/__init__.py tests/test_candidates_a.py
git commit -m "feat(boarddet): plausible_board_patch collects patch-stage rejects"
```

---

### Task 3: 4 generators forward the `rejects` kwarg

**Files:**
- Modify: `src/boarddet/candidates/ransac_iterative.py:11` (signature) `:58` (call)
- Modify: `src/boarddet/candidates/cluster_after_ground.py:184` (signature) `:218` (call)
- Modify: `src/boarddet/candidates/region_growing.py:14` (signature) `:45` (call)
- Modify: `src/boarddet/candidates/background_diff.py:25` (signature) `:43` (call)
- Test: `tests/test_candidates_b.py` (add one forwarding case)

**Interfaces:**
- Consumes: `plausible_board_patch(..., rejects=...)` from Task 2.
- Produces: each `generate_*` gains a keyword-only `rejects: list[RejectReason] | None = None` (add to the existing signature; keep other params unchanged) and forwards it: `plausible_board_patch(<pts>, board, rejects=rejects)`. Note `generate_cluster_after_ground` and `generate_background_diff` already have keyword-only params after `*`; add `rejects` there. `generate_ransac_iterative` and `generate_region_growing` take positional args — append `rejects` as a trailing keyword param with default `None`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_candidates_b.py  (append)
import numpy as np

from boarddet.board_config import BoardConfig
from boarddet.candidates.cluster_after_ground import generate_cluster_after_ground
from boarddet.reject import RejectReason


def test_generator_forwards_rejects_kwarg():
    board = BoardConfig()
    # a scene with clusters that fail the patch gate produces patch rejects;
    # here we only assert the kwarg is accepted and the list type is honored.
    rng = np.random.default_rng(0)
    scene = rng.uniform(-1, 1, size=(500, 3))
    rejects: list[RejectReason] = []
    out = generate_cluster_after_ground(
        scene, board, vertical_gap_deg=board.vertical_gap_deg,
        cluster_min_points=board.cluster_min_points, rejects=rejects)
    assert isinstance(out, list)
    # kwarg omitted still works and is byte-identical in shape
    out2 = generate_cluster_after_ground(
        scene, board, vertical_gap_deg=board.vertical_gap_deg,
        cluster_min_points=board.cluster_min_points)
    assert isinstance(out2, list)
    assert len(out) == len(out2)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_candidates_b.py::test_generator_forwards_rejects_kwarg -v`
Expected: FAIL — `generate_cluster_after_ground() got an unexpected keyword argument 'rejects'`

- [ ] **Step 3: Write minimal implementation**

For each generator: add the import `from .reject import RejectReason` if not present (note: generators are in `candidates/`, so it is `from ..reject import RejectReason`), add the param, forward it.

`ransac_iterative.py` — signature at line 11, add trailing param:

```python
def generate_ransac_iterative(points: np.ndarray, board: BoardConfig,
                              rejects: list[RejectReason] | None = None):
```
call at line 58:
```python
            cand = plausible_board_patch(inliers[labels == lbl], board,
                                         rejects=rejects)
```

`cluster_after_ground.py` — signature at line 184 (keyword-only block after `*`), add `rejects`:
```python
                                  rejects: list[RejectReason] | None = None):
```
call at line 218:
```python
        cand = plausible_board_patch(group_pts, board, rejects=rejects)
```

`region_growing.py` — signature at line 14, add trailing param:
```python
def generate_region_growing(points: np.ndarray, board: BoardConfig,
                            rejects: list[RejectReason] | None = None):
```
call at line 45:
```python
            cand = plausible_board_patch(points[np.array(region)], board,
                                         rejects=rejects)
```

`background_diff.py` — signature at line 25 (keyword-only block after `*`), add `rejects`:
```python
                             rejects: list[RejectReason] | None = None):
```
call at line 43:
```python
        cand = plausible_board_patch(fg[labels == lbl], board, rejects=rejects)
```

Add `from ..reject import RejectReason` to each of the four files' imports.

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_candidates_b.py tests/test_candidates_a.py tests/test_candidates_c.py tests/test_candidates_e.py -v`
Expected: PASS (new forwarding test + all existing generator tests unchanged)

- [ ] **Step 5: Commit**

```bash
git add src/boarddet/candidates/ransac_iterative.py src/boarddet/candidates/cluster_after_ground.py src/boarddet/candidates/region_growing.py src/boarddet/candidates/background_diff.py tests/test_candidates_b.py
git commit -m "feat(boarddet): generators forward rejects kwarg to patch gate"
```

---

### Task 4: `score_candidate` collects scorer-stage rejects

**Files:**
- Modify: `src/boarddet/scorer.py:172` (signature) + its return-None sites
- Test: `tests/test_scorer.py` (add cases; do not weaken existing)

**Interfaces:**
- Consumes: `boarddet.reject.{Stage, band, lower, upper}` and a local structural builder.
- Produces: `score_candidate(coords_2d, board, up_2d=None, close_height_m=None, rejects: list[RejectReason] | None = None) -> ScoreResult | None`. Appends at each return-None site: `MIN_POINTS`(param None), `RASTER_SIZE`(param None), `MINAREA_SIZE`(param None), `SIZE_GATE`(param `side_tol`, band), `STRICT_SQUARENESS`(param `strict_squareness`, upper vs 8.0), `STANCE_2D`(param `stance_floor`, lower), `EDGE_SUPPORT`(param `edge_support_min`, lower), `SIDE_ERR`(param `side_tol`, upper vs `side_tol*side_m`). Accept path unchanged.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_scorer.py  (append)
import numpy as np

from boarddet.board_config import BoardConfig
from boarddet.reject import Stage
from boarddet.scorer import ScoreResult, score_candidate


def _square_coords(side=1.0, step=0.02):
    g = np.arange(-side / 2, side / 2, step)
    xx, yy = np.meshgrid(g, g)
    return np.column_stack([xx.ravel(), yy.ravel()])


def test_scorer_reject_min_points():
    board = BoardConfig()
    rejects = []
    out = score_candidate(np.zeros((10, 2)), board, rejects=rejects)
    assert out is None
    assert rejects[-1].stage is Stage.MIN_POINTS
    assert rejects[-1].param is None


def test_scorer_reject_size_gate():
    # a dense square far larger than side_m trips the coarse size gate
    board = BoardConfig(side_m=1.0, side_tol=0.05)
    coords = _square_coords(side=3.0, step=0.03)
    rejects = []
    out = score_candidate(coords, board, rejects=rejects)
    assert out is None
    assert rejects[-1].stage in (Stage.SIZE_GATE, Stage.SIDE_ERR)
    assert rejects[-1].param == "side_tol"


def test_scorer_accept_collects_nothing_and_returns_scoreresult():
    board = BoardConfig(side_m=1.0, side_tol=0.20)
    coords = _square_coords(side=1.0, step=0.02)
    rejects = []
    out = score_candidate(coords, board, rejects=rejects)
    assert isinstance(out, ScoreResult)
    assert rejects == []


def test_scorer_kwarg_omitted_byte_identical():
    board = BoardConfig(side_m=1.0, side_tol=0.20)
    coords = _square_coords(side=1.0, step=0.02)
    assert isinstance(score_candidate(coords, board), ScoreResult)
    assert score_candidate(np.zeros((10, 2)), board) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_scorer.py -v -k "reject or byte_identical or collects_nothing"`
Expected: FAIL — `score_candidate() got an unexpected keyword argument 'rejects'`

- [ ] **Step 3: Write minimal implementation**

Add to `scorer.py` imports:
```python
from .reject import RejectReason, Stage, band, lower, upper
```

Add a structural builder near `_MIN_POINTS`:
```python
def _structural(stage: Stage, gate: str, value: float,
                thr: float) -> RejectReason:
    """A structural gate (no tunable BoardConfig param); margin left at 0."""
    return RejectReason(stage, gate, None, float(value), float(thr), 0.0)
```

Change the signature (line 172) to add `rejects: list[RejectReason] | None = None`. Then at each `return None` site, append before returning. The exact replacements (search for each `return None`):

`len(coords_2d) < _MIN_POINTS`:
```python
    if len(coords_2d) < _MIN_POINTS:
        if rejects is not None:
            rejects.append(_structural(Stage.MIN_POINTS, "min_points",
                                       len(coords_2d), _MIN_POINTS))
        return None
```

`img.shape[0] > 4000 or img.shape[1] > 4000`:
```python
    if img.shape[0] > 4000 or img.shape[1] > 4000:
        if rejects is not None:
            rejects.append(_structural(Stage.RASTER_SIZE, "raster_size",
                                       float(max(img.shape)), 4000.0))
        return None
```

anisotropic `min(rw, rh) < 3 * cell`:
```python
        if min(rw, rh) < 3 * cell:
            if rejects is not None:
                rejects.append(_structural(Stage.MINAREA_SIZE, "minarea_size",
                                           float(min(rw, rh)), 3 * cell))
            return None
```

isotropic `if not contours`:
```python
        if not contours:
            if rejects is not None:
                rejects.append(_structural(Stage.MINAREA_SIZE, "no_contour",
                                           0.0, 3.0))
            return None
```

isotropic `min(rw, rh) < 3`:
```python
        if min(rw, rh) < 3:
            if rejects is not None:
                rejects.append(_structural(Stage.MINAREA_SIZE, "minarea_size",
                                           float(min(rw, rh)), 3.0))
            return None
```

size gate (the `if not (lo < sides.mean() < hi)` block). Rewrite to name bounds:
```python
    lo = board.side_m * (1 - 2 * board.side_tol)
    hi = board.side_m * (1 + 2 * board.side_tol)
    if not (lo < sides.mean() < hi):
        if rejects is not None:
            rejects.append(band(Stage.SIZE_GATE, "size_gate", "side_tol",
                                float(sides.mean()), lo, hi))
        return None
```

`strict_squareness` gate:
```python
    if board.strict_squareness:
        max_ang_dev = float(np.max(np.abs(np.array(angs) - 90.0)))
        if max_ang_dev > 8.0:
            if rejects is not None:
                rejects.append(upper(Stage.STRICT_SQUARENESS, "strict_squareness",
                                     "strict_squareness", max_ang_dev, 8.0))
            return None
```

`stance_floor` 2D gate:
```python
    if board.stance_floor > 0 and up_2d is not None:
        stance = _diamond_stance_2d(corners, up_2d)
        if stance <= board.stance_floor:
            if rejects is not None:
                rejects.append(lower(Stage.STANCE_2D, "stance_2d",
                                     "stance_floor", stance, board.stance_floor))
            return None
```

`edge_support` gate:
```python
    if (board.edge_support_min > 0
            and float(edge_support.min()) < board.edge_support_min):
        if rejects is not None:
            rejects.append(lower(Stage.EDGE_SUPPORT, "edge_support",
                                 "edge_support_min", float(edge_support.min()),
                                 board.edge_support_min))
        return None
```

`side_err` gate (the `abs(mean(sides) - side_m) > side_tol*side_m` return). Rewrite to name the metric:
```python
    side_dev = abs(float(np.mean(sides)) - board.side_m)
    if side_dev > board.side_tol * board.side_m:
        if rejects is not None:
            rejects.append(upper(Stage.SIDE_ERR, "side_err", "side_tol",
                                 side_dev, board.side_tol * board.side_m))
        return None
```

Do not touch the accept path or any computed values feeding the score.

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_scorer.py -v`
Expected: PASS (new reject tests + all existing scorer tests, incl. the byte-identical isotropic pin)

- [ ] **Step 5: Commit**

```bash
git add src/boarddet/scorer.py tests/test_scorer.py
git commit -m "feat(boarddet): score_candidate collects scorer-stage rejects"
```

---

### Task 5: `detect()` folds rejects into `DetectOutcome.reject_reason`

**Files:**
- Modify: `src/boarddet/detector.py` (`DetectOutcome`, `detect` wiring)
- Test: `tests/test_detector.py` (add cases)

**Interfaces:**
- Consumes: `generate_*(..., rejects=...)`, `score_candidate(..., rejects=...)`, `boarddet.reject.{RejectReason, Stage, lower, upper, furthest}`.
- Produces: `DetectOutcome` gains `reject_reason: RejectReason | None = None`. `detect()` builds a `rejects` list, passes it to the generator (all generators) and — non-icp path only — to `score_candidate`; adds `MIN_SCORE`/`ISOLATION` (non-icp) and `SQUARE_FIT`/`STANCE_3D`/`ISOLATION` (icp) reasons on reject; sets `reject_reason = None` when detected, `furthest(rejects)` otherwise, and `NO_CLUSTERS` when `rejects` is empty and no detection.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_detector.py  (append)
# make_scene / detect / BoardConfig are already imported at the top of this
# file (see existing imports: `from boarddet.synth import ... make_scene`);
# add only the reject import.
from boarddet.reject import Stage


def test_detect_no_clusters_reports_no_clusters():
    # too few points to cluster: generator emits nothing, no patch rejects
    board = BoardConfig(side_m=1.0)
    out = detect(np.zeros((3, 3)), board, generator="b")
    assert out.detection is None
    assert out.reject_reason is not None
    assert out.reject_reason.stage is Stage.NO_CLUSTERS


def test_detect_success_has_no_reject_reason():
    # same scene the existing test_detects_board_in_synthetic_scene solves
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0), generator="a")
    assert out.detection is not None
    assert out.reject_reason is None


def test_detect_min_score_reject_names_param():
    # mirrors the existing min_score=0.99 forced-reject test: the board is
    # found geometrically but scored below threshold -> MIN_SCORE reason.
    pts, _ = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0, min_score=0.99), generator="a")
    assert out.detection is None
    assert out.reject_reason is not None
    assert out.reject_reason.stage is Stage.MIN_SCORE
    assert out.reject_reason.param == "min_score"
    assert out.reject_reason.margin > 0
```

The helper is `make_scene(rng=...) -> (pts, truth)` from `boarddet.synth`,
already imported at the top of `test_detector.py`. The `min_score=0.99` case
reuses the exact scenario of the existing `test_high_min_score_rejects` test,
so the geometry is known to reach the `min_score` gate.

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_detector.py -v -k "no_clusters or no_reject_reason"`
Expected: FAIL — `AttributeError: 'DetectOutcome' object has no attribute 'reject_reason'`

- [ ] **Step 3: Write minimal implementation**

In `detector.py`, add the import:
```python
from .reject import RejectReason, Stage, furthest, lower, upper
```

Add the field to `DetectOutcome`:
```python
@dataclass
class DetectOutcome:
    detection: BoardDetection | None
    timings_ms: dict[str, float]
    n_candidates: int
    best_rejected: BoardDetection | None = None
    reject_reason: RejectReason | None = None
```

In `detect()`, create the list before the generator dispatch and thread it in. Replace the generator dispatch block (lines ~112-124) so each call passes `rejects=rejects`:

```python
    rejects: list[RejectReason] = []
    if generator == "b":
        cands = gen(dn, board, vertical_gap_deg=board.vertical_gap_deg,
                    cluster_min_points=board.cluster_min_points,
                    rejects=rejects)
    elif generator == "e":
        if background is None:
            raise ValueError(
                "generator 'e' (background_diff) requires a background "
                "reference; pass detect(..., background=<BackgroundModel>)")
        cands = gen(dn, board, background=background,
                    vertical_gap_deg=board.vertical_gap_deg,
                    cluster_min_points=board.cluster_min_points,
                    rejects=rejects)
    else:
        cands = gen(dn, board, rejects=rejects)
```

Non-icp scorer call (line ~137) — pass `rejects`:
```python
        res = score_candidate(coords, board, up_2d=up_2d,
                              close_height_m=close_height_m, rejects=rejects)
```

icp scorer call (line ~137 is shared; the icp branch reuses `res`). Keep the icp path's `score_candidate` **without** `rejects` — but the current code calls `score_candidate` once, before the `if board.square_icp:` split. To keep icp scorer reasons non-fatal/uncollected, split the call: compute `res` with `rejects=rejects` only on the non-icp path. Restructure minimally:

```python
        up_2d = None
        close_height_m = None
        if board.vertical_gap_deg > 0:
            up_2d = _up_2d(cand.plane, up)
            if up_2d is not None:
                close_height_m = _close_height_m(cand.points, board)
        coords = project_to_plane(cand.points, cand.plane)

        if board.square_icp:
            # scorer reason is non-fatal here (rescued by fit_fixed_square),
            # so do NOT collect it.
            res = score_candidate(coords, board, up_2d=up_2d,
                                  close_height_m=close_height_m)
            seed_center = _quad_center(res) if res is not None \
                else coords.mean(axis=0)
            fit = fit_fixed_square(
                coords, board.side_m, init_center=seed_center,
                init_theta=None)
            if fit is None or fit.residual >= board.square_icp_residual_max:
                if rejects is not None:
                    val = np.inf if fit is None else fit.residual
                    rejects.append(upper(
                        Stage.SQUARE_FIT, "square_fit",
                        "square_icp_residual_max",
                        float(val) if np.isfinite(val) else
                        board.square_icp_residual_max * 2,
                        board.square_icp_residual_max))
                continue
            refined_score = 1.0 / (1.0 + fit.residual)
            ...  # unchanged refined_res construction
            det = board_pose(cand.plane, refined_res)
            det = dataclasses.replace(det, score=refined_score)
            if board.stance_floor > 0:
                stance3d = _stance(det.corners_3d, up)
                if stance3d <= board.stance_floor:
                    rejects.append(lower(Stage.STANCE_3D, "stance_3d",
                                         "stance_floor", stance3d,
                                         board.stance_floor))
                    continue
            if board.isolation:
                density = isolation_density(dn, cand.plane,
                                           det.result.corners_2d)
                if density > board.isolation_max_density:
                    rejects.append(upper(Stage.ISOLATION, "isolation",
                                         "isolation_max_density", density,
                                         board.isolation_max_density))
                    continue
            if fit.residual < best_residual:
                best_residual = fit.residual
                best = det
            continue

        # non-icp path
        res = score_candidate(coords, board, up_2d=up_2d,
                              close_height_m=close_height_m, rejects=rejects)
        if res is None:
            continue
        det = board_pose(cand.plane, res)
        if board.stance_weight > 0:
            stance = _stance(det.corners_3d, up)
            w = board.stance_weight
            blended = res.score * ((1 - w) + w * stance)
            det = dataclasses.replace(det, score=blended)
        if det.score < board.min_score:
            rejects.append(lower(Stage.MIN_SCORE, "min_score", "min_score",
                                 det.score, board.min_score))
            if best_rejected is None or det.score > best_rejected.score:
                best_rejected = det
            continue
        if board.isolation:
            density = isolation_density(dn, cand.plane, det.result.corners_2d)
            if density > board.isolation_max_density:
                rejects.append(upper(Stage.ISOLATION, "isolation",
                                     "isolation_max_density", density,
                                     board.isolation_max_density))
                continue
        if best is None or det.score > best.score:
            best = det
```

**Note:** the current code calls `score_candidate` once *above* the `if board.square_icp:` split. This step moves it inside both branches (icp: no `rejects`; non-icp: with `rejects`). The `up_2d`/`close_height_m`/`coords` computation moves above the split (it already is above). Verify against the current file structure and keep every other line identical.

After the candidate loop, compute the reason:
```python
    if best is not None:
        best_rejected = None
        reject_reason = None
    else:
        reason = furthest(rejects)
        reject_reason = reason if reason is not None else RejectReason(
            Stage.NO_CLUSTERS, "no_clusters", None, None, None, 0.0)
```

Add `reject_reason=reject_reason` to the returned `DetectOutcome(...)`.

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_detector.py tests/test_detector_e.py -v`
Expected: PASS (new cases + all existing detector tests unchanged)

- [ ] **Step 5: Commit**

```bash
git add src/boarddet/detector.py tests/test_detector.py
git commit -m "feat(boarddet): detect() folds furthest reject into DetectOutcome"
```

---

### Task 6: full-suite regression + spec-coverage sweep

**Files:**
- Test: whole suite.

- [ ] **Step 1: Run the full test suite**

Run: `uv run pytest`
Expected: PASS — every pre-existing test unchanged (byte-identical guard holds) plus the new `test_reject.py` and the added cases in `test_candidates_a/b`, `test_scorer`, `test_detector`.

- [ ] **Step 2: Lint**

Run: `uv run ruff check src/boarddet tests`
Expected: clean (fix any unused-import / line-length issues introduced).

- [ ] **Step 3: Manual smoke — reject reason on a real no-detection frame**

Run a single-dataset benchmark generator on a frame known to fail and print the reason, e.g. via a short `uv run python -c` snippet calling `detect(...)` on one cached frame and printing `out.reject_reason`. Confirm the printed stage/param/margin is sensible (a barely-failed frame shows a small margin; an empty scene shows `NO_CLUSTERS`).

- [ ] **Step 4: Commit any lint fixes**

```bash
git add -A
git commit -m "chore(boarddet): lint + regression sweep for reject diagnostics"
```

---

## Self-Review

**Spec coverage:**
- `reject.py` (Stage, RejectReason, margin builders, furthest) → Task 1 ✓
- side-channel kwarg, byte-identical passing path → Tasks 2–5 (guard tests each) ✓
- `plausible_board_patch` 3 sites → Task 2 ✓
- generators forward kwarg → Task 3 ✓
- `score_candidate` 8 sites → Task 4 ✓
- `detect()` fold + `DetectOutcome.reject_reason` + `NO_CLUSTERS` + icp non-fatal scorer reason → Task 5 ✓
- benchmark aggregation / ROS port → explicitly out of scope (spec non-goals) ✓

**Type consistency:** `RejectReason(stage, gate, param, value, threshold, margin)` used identically across Tasks 1–5. `furthest()` name consistent. `rejects: list[RejectReason] | None = None` signature identical in `plausible_board_patch`, all 4 generators, `score_candidate`. `DetectOutcome.reject_reason` name consistent Task 5 ↔ tests.

**Placeholder scan:** one flagged item — Task 5 Step 1 uses `synth_board_scene` as a stand-in for the existing detector-test scene helper; the step explicitly instructs the implementer to mirror the helper already used in `test_detector.py`. Confirm that file's imports at execution time.
