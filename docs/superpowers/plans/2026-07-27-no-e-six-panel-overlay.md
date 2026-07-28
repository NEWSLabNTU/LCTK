# No-E Six-Panel Overlay (+ front/side orientation fix) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the no-Method-E benchmark (`benchmark_noe`) the same 6-panel overlay Method E produces — with RANSAC's big-plane removal as the "background" analog — and fix the front/side view panels to the x=front / y=left / z=up frame convention, consolidating all overlay rendering into a single `viz.py`.

**Architecture:** Extract the 6-panel scaffolding (currently private inside `viz_methode.py`) into `viz.py` as a shared `render_six_panel`, with the front (y-z, horizontal-inverted) and side (x-z) projection panels corrected to the frame convention. `render_methode` (Method E) becomes a thin wrapper computing its foreground via background subtraction; a new `render_noe` computes its "foreground" via generator B's big-plane strip (`big_plane_residual`). `viz_methode.py` is deleted. `save_overlay` (2-panel) stays in `viz.py` for the legacy `benchmark.py`.

**Tech Stack:** Python 3.11, `uv` project at `experiments/board-detection-2d/`, numpy, matplotlib (Agg, headless), open3d (RANSAC), pytest.

## Global Constraints

- All commands run from `experiments/board-detection-2d/`, prefixed `uv run`. Never system python/pip; never `pip3 install --user`; never `just build` (standalone uv project).
- **Frame convention: x = front, y = left, z = up (right-handed, REP-103).** Corrected spatial panels:
  - **Front view** = project onto **y-z** (look along x); horizontal axis **y inverted** so +y (left) renders on the left; vertical z up. Title `"front (y-z)"`.
  - **Side view** = project onto **x-z** (look along y, from +y toward −y); horizontal axis **x**, NOT inverted, so +x (front) points right; vertical z up. Title `"side (x-z)"`.
  - Top-down panels (x-y) are unchanged.
- **Single viz module:** all rendering lives in `viz.py`. Do NOT create `viz_panels.py` or `viz_noe.py`; DELETE `viz_methode.py`. `render_methode` and `render_noe` are functions in `viz.py`.
- **Panel 2 is residual-only** (mirror of E's foreground-diff panel): the residual/foreground layer in blue (`tab:blue`), no separate "removed planes" layer. No-E title `"after big-plane removal (N pts)"`; E title unchanged `"foreground diff (N pts total)"`.
- **The no-E residual must match what the detector actually clustered:** compute it via `big_plane_residual`, which calls `_remove_big_planes` with generator B's own strip params (`_BIG_PLANE_DIST=0.05`, `_BIG_PLANE_MIN_FRAC=0.08`). `_remove_big_planes` seeds RANSAC (`o3d.utility.random.seed(0)`), so the recompute is deterministic and identical to detection's internal strip.
- **Do not delete any generated PNG or `results/` directory.** Overlays regenerate by re-running the benchmarks.
- Fixed layer colors (unchanged from `viz_methode`): raw `"0.6"`, foreground `tab:blue`, best-rejected quad `tab:orange`, bbox `tab:green`, detection quad `tab:red`.

---

### Task 1: `big_plane_residual` helper in `cluster_after_ground.py`

Expose generator B's big-plane strip as a reusable function so the no-E overlay can show the exact residual the detector clustered. Lift the two strip params to module constants (used as the generator's own signature defaults — behavior byte-identical).

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/candidates/cluster_after_ground.py`
- Test: `experiments/board-detection-2d/tests/test_candidates_b.py`

**Interfaces:**
- Consumes: `_remove_big_planes(points, board, dist, min_frac, vertical_gap_deg) -> np.ndarray` (existing, returns `remaining.astype(np.float32)`).
- Produces: module constants `_BIG_PLANE_DIST = 0.05`, `_BIG_PLANE_MIN_FRAC = 0.08`; public `big_plane_residual(points: np.ndarray, board: BoardConfig, vertical_gap_deg: float = 3.0) -> np.ndarray`.

- [ ] **Step 1: Write the failing test**

Add to `experiments/board-detection-2d/tests/test_candidates_b.py` (add imports it needs at the top: `import numpy as np`, `from boarddet.board_config import BoardConfig`, `from boarddet.synth import make_scene`, and `from boarddet.candidates.cluster_after_ground import big_plane_residual, _remove_big_planes, _BIG_PLANE_DIST, _BIG_PLANE_MIN_FRAC` — reuse any already present):

```python
def test_big_plane_residual_is_subset_and_matches_strip():
    pts, _ = make_scene(rng=np.random.default_rng(5))
    board = BoardConfig(side_m=1.0)
    res = big_plane_residual(pts, board, board.vertical_gap_deg)
    # Strip removes the big ground/wall planes -> strictly fewer points.
    assert 0 < len(res) < len(pts)
    # Reproduces the generator's own strip exactly (shared params + seeded
    # RANSAC make this deterministic), so a viz of `res` shows what the
    # detector actually clustered.
    direct = _remove_big_planes(pts, board, _BIG_PLANE_DIST,
                                _BIG_PLANE_MIN_FRAC, board.vertical_gap_deg)
    assert np.array_equal(res, direct)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_candidates_b.py::test_big_plane_residual_is_subset_and_matches_strip -v`
Expected: FAIL — `ImportError: cannot import name 'big_plane_residual'` (and `_BIG_PLANE_DIST`).

- [ ] **Step 3: Add constants + helper, wire generator defaults to the constants**

In `cluster_after_ground.py`, add the two constants near the top of the module (after the imports, before `_anisotropic_scaled`):

```python
# Generator B's big-plane strip params, shared so a viz of the residual
# matches exactly what detection clustered (see big_plane_residual).
_BIG_PLANE_DIST = 0.05
_BIG_PLANE_MIN_FRAC = 0.08
```

Add the public helper (place it just after `_remove_big_planes`'s definition ends, around line 98):

```python
def big_plane_residual(points: np.ndarray, board: BoardConfig,
                       vertical_gap_deg: float = 3.0) -> np.ndarray:
    """Points surviving generator B's big-plane strip -- the 'foreground' its
    clustering step sees, and the no-Method-E analog of a background diff.

    Shares the generator's own strip params (`_BIG_PLANE_DIST`,
    `_BIG_PLANE_MIN_FRAC`); `_remove_big_planes` seeds RANSAC, so this is
    deterministic and identical to what `generate_cluster_after_ground`
    strips internally for the same input and `vertical_gap_deg`.
    """
    return _remove_big_planes(points, board, _BIG_PLANE_DIST,
                              _BIG_PLANE_MIN_FRAC, vertical_gap_deg)
```

In `generate_cluster_after_ground`'s signature, replace the two literal defaults so the generator and the helper share one source of truth:

```python
def generate_cluster_after_ground(points: np.ndarray, board: BoardConfig,
                                  big_plane_dist: float = _BIG_PLANE_DIST,
                                  big_plane_min_frac: float = _BIG_PLANE_MIN_FRAC,
                                  cluster_eps: float = 0.15,
                                  cluster_min_points: int = 30,
                                  vertical_gap_deg: float = 3.0
                                  ) -> list[Candidate]:
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_candidates_b.py::test_big_plane_residual_is_subset_and_matches_strip -v`
Expected: PASS.

- [ ] **Step 5: Run the generator-B suite + full suite (guard the default swap)**

Run: `uv run pytest tests/test_candidates_b.py -q`
Expected: PASS (the `_BIG_PLANE_DIST`/`_BIG_PLANE_MIN_FRAC` values equal the old literals 0.05/0.08, so `generate_cluster_after_ground` is unchanged).

Run: `uv run pytest -q`
Expected: whole suite green.

- [ ] **Step 6: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/candidates/cluster_after_ground.py \
        experiments/board-detection-2d/tests/test_candidates_b.py
git commit -m "feat(boarddet): expose big_plane_residual for no-E overlay"
```

---

### Task 2: Consolidate rendering into `viz.py` + fix front/side orientation

Move the Method E 6-panel renderer out of `viz_methode.py` into `viz.py` as a shared `render_six_panel`, correcting the front and side panels to the frame convention. `render_methode` becomes a thin wrapper. Delete `viz_methode.py`; update its importer and rename its test. `save_overlay` stays untouched.

**Files:**
- Modify (rewrite): `experiments/board-detection-2d/src/boarddet/viz.py`
- Delete: `experiments/board-detection-2d/src/boarddet/viz_methode.py`
- Modify: `experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py:32` (import path)
- Rename + modify: `experiments/board-detection-2d/tests/test_viz_methode.py` → `experiments/board-detection-2d/tests/test_viz.py`

**Interfaces:**
- Consumes: `big_plane_residual` (Task 1) is NOT used here (Task 3 uses it). `DetectOutcome`, `BoardDetection`, `BoxRef`, `BackgroundModel`, `downsample` (existing).
- Produces (all in `viz.py`):
  - `save_overlay(points, outcome, path) -> None` (unchanged).
  - Module tuples `_FRONT = (1, 2, True, "y [m]", "z [m]", "front (y-z)")` and `_SIDE = (0, 2, False, "x [m]", "z [m]", "side (x-z)")` — `(ai, bi, invert_h, xlabel, ylabel, title)`.
  - `render_six_panel(dn, fg, box, outcome, path, panel2_title) -> None`.
  - `render_methode(frame_xyz, board, background, outcome, box, path, voxel=0.03) -> None` (same signature as before, now in `viz.py`).

- [ ] **Step 1: Rewrite `viz.py` with the shared renderer + corrected panels**

Replace the entire contents of `experiments/board-detection-2d/src/boarddet/viz.py` with:

```python
"""Overlay renders for eyeballing detections.

Two renderers live here:
- `save_overlay`  -- the generator-agnostic 2-panel overlay (used by the
  a/b/c benchmark).
- `render_six_panel` + its wrappers `render_methode` / `render_noe` -- the
  full 6-panel pipeline view. The wrappers differ only in how the panel-2
  "foreground" layer is computed (background diff for Method E; RANSAC
  big-plane strip for the no-Method-E baseline).

Frame convention for the spatial panels: x = front, y = left, z = up
(right-handed). The front view projects onto y-z with the horizontal (y)
axis inverted so +y (left) renders on the left; the side view projects onto
x-z with +x (front) to the right. Headless Agg backend throughout.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .background import BackgroundModel  # noqa: E402
from .bbox_ref import BoxRef  # noqa: E402
from .board_config import BoardConfig  # noqa: E402
from .detector import DetectOutcome  # noqa: E402
from .geometry import downsample  # noqa: E402
from .pose import BoardDetection  # noqa: E402


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
        corners = (res.corners_2d @ res.rot_2d.T
                   if res.rot_2d is not None else res.corners_2d)
        px = (corners - res.origin) / res.cell_m
        ax.plot(np.append(px[:, 0], px[0, 0]),
                np.append(px[:, 1], px[0, 1]), "r-", lw=1.5)
        ax.set_title("plane raster + refined quad")
    else:
        ax.axis("off")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


# --- Shared 6-panel renderer -------------------------------------------------

# Fixed layer colors.
_C_RAW = "0.6"
_C_FG = "tab:blue"
_C_CAND = "tab:orange"
_C_BBOX = "tab:green"
_C_DET = "tab:red"

# The 8 corners of a unit box in its own frame, as (sx, sy, sz) signs.
_BOX_SIGNS = np.array([[sx, sy, sz]
                       for sx in (-1, 1) for sy in (-1, 1)
                       for sz in (-1, 1)], dtype=float)
# The 12 edges of that box, as index pairs into _BOX_SIGNS.
_BOX_EDGES = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
              (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]

# How far beyond the reference box, in metres, the spatial panels show.
_ZOOM_MARGIN_M = 4.0

# Spatial projection panels other than the top-down mix, as
# (ai, bi, invert_h, xlabel, ylabel, title). Frame convention x=front,
# y=left, z=up: the front view is y-z with y inverted (left renders left);
# the side view is x-z with +x (front) to the right (not inverted).
_FRONT = (1, 2, True, "y [m]", "z [m]", "front (y-z)")
_SIDE = (0, 2, False, "x [m]", "z [m]", "side (x-z)")


def _box_corners_world(box: BoxRef) -> np.ndarray:
    """(8,3) world-frame corners of the reference box."""
    return box.center + (_BOX_SIGNS * box.half) @ box.rot.T


def _draw_box(ax, corners: np.ndarray, ai: int, bi: int) -> None:
    for i, j in _BOX_EDGES:
        ax.plot([corners[i, ai], corners[j, ai]],
                [corners[i, bi], corners[j, bi]],
                color=_C_BBOX, lw=1.0, alpha=0.9)


def _draw_quad(ax, det: BoardDetection, ai: int, bi: int, color: str) -> None:
    q = np.vstack([det.corners_3d, det.corners_3d[:1]])
    ax.plot(q[:, ai], q[:, bi], color=color, lw=2)


def _scatter(ax, pts: np.ndarray, ai: int, bi: int, color: str,
             s: float, alpha: float) -> None:
    if len(pts) == 0:
        return
    step = max(1, len(pts) // 60_000)
    ax.scatter(pts[::step, ai], pts[::step, bi], s=s, c=color, alpha=alpha)


def _set_limits(ax, box_corners: np.ndarray, ai: int, bi: int,
                invert_h: bool) -> None:
    """Zoom to the box (+margin); invert the horizontal axis when invert_h,
    so a frame direction like +y=left renders on the left instead of right."""
    lo = box_corners[:, [ai, bi]].min(axis=0) - _ZOOM_MARGIN_M
    hi = box_corners[:, [ai, bi]].max(axis=0) + _ZOOM_MARGIN_M
    if invert_h:
        ax.set_xlim(hi[0], lo[0])
    else:
        ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])


def _proj_panel(ax, ai: int, bi: int, invert_h: bool, labels: tuple[str, str],
                title: str, raw, fg, box_corners, outcome: DetectOutcome
                ) -> None:
    """One orthographic projection panel with all layers, zoomed to the box."""
    _scatter(ax, raw, ai, bi, _C_RAW, 2.0, 0.35)
    _scatter(ax, fg, ai, bi, _C_FG, 4.0, 0.8)
    _draw_box(ax, box_corners, ai, bi)
    if outcome.best_rejected is not None:
        _draw_quad(ax, outcome.best_rejected, ai, bi, _C_CAND)
    if outcome.detection is not None:
        _draw_quad(ax, outcome.detection, ai, bi, _C_DET)
    ax.set_aspect("equal")
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_title(title)
    _set_limits(ax, box_corners, ai, bi, invert_h)


def render_six_panel(dn: np.ndarray, fg: np.ndarray, box: BoxRef,
                     outcome: DetectOutcome, path: Path,
                     panel2_title: str) -> None:
    """Render the shared 6-panel pipeline view for one frame. `dn` is the
    downsampled cloud (raw layer), `fg` the pipeline's foreground layer
    (background diff for Method E, big-plane residual for no-E); `panel2_title`
    labels the foreground panel. The other five panels are identical."""
    box_corners = _box_corners_world(box)
    det = outcome.detection
    state = f"score={det.score:.2f}" if det is not None else "NO DETECTION"
    path = Path(path)
    fig, axes = plt.subplots(2, 3, figsize=(19, 11))
    try:
        # Panel 1: raw only, top-down.
        _scatter(axes[0, 0], dn, 0, 1, _C_RAW, 2.0, 0.5)
        _draw_box(axes[0, 0], box_corners, 0, 1)
        axes[0, 0].set_aspect("equal")
        axes[0, 0].set_xlabel("x [m]")
        axes[0, 0].set_ylabel("y [m]")
        axes[0, 0].set_title("raw cloud (top-down)")
        _set_limits(axes[0, 0], box_corners, 0, 1, False)

        # Panel 2: foreground only, top-down. Title carries the layer count.
        _scatter(axes[0, 1], fg, 0, 1, _C_FG, 4.0, 0.9)
        _draw_box(axes[0, 1], box_corners, 0, 1)
        axes[0, 1].set_aspect("equal")
        axes[0, 1].set_xlabel("x [m]")
        axes[0, 1].set_ylabel("y [m]")
        axes[0, 1].set_title(panel2_title)
        _set_limits(axes[0, 1], box_corners, 0, 1, False)

        # Panel 3: mix, top-down.
        _proj_panel(axes[0, 2], 0, 1, False, ("x [m]", "y [m]"),
                    f"mix (top-down) | {state}", dn, fg, box_corners, outcome)

        # Panel 4: front (y-z, horizontal inverted). Panel 5: side (x-z).
        for ax, (ai, bi, invert_h, xl, yl, title) in (
                (axes[1, 0], _FRONT), (axes[1, 1], _SIDE)):
            _proj_panel(ax, ai, bi, invert_h, (xl, yl), title,
                        dn, fg, box_corners, outcome)

        # Panel 6: plane raster + refined quad.
        ax = axes[1, 2]
        if det is not None:
            res = det.result
            ax.imshow(res.raster, cmap="gray", origin="lower")
            corners = (res.corners_2d @ res.rot_2d.T
                       if res.rot_2d is not None else res.corners_2d)
            px = (corners - res.origin) / res.cell_m
            ax.plot(np.append(px[:, 0], px[0, 0]),
                    np.append(px[:, 1], px[0, 1]), color=_C_DET, lw=1.5)
            ax.set_title("plane raster + refined quad")
        else:
            ax.axis("off")
            ax.set_title("plane raster (no detection)")

        fig.suptitle(path.stem, fontsize=12)
        fig.tight_layout()
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=100)
    finally:
        plt.close(fig)


def render_methode(frame_xyz: np.ndarray, board: BoardConfig,
                   background: BackgroundModel, outcome: DetectOutcome,
                   box: BoxRef, path: Path, voxel: float = 0.03) -> None:
    """Method E 6-panel view: foreground = background-diff of the frame."""
    dn = downsample(frame_xyz, voxel)
    fg = background.foreground_points(dn)
    render_six_panel(dn, fg, box, outcome, path,
                     f"foreground diff ({len(fg)} pts total)")
```

- [ ] **Step 2: Delete `viz_methode.py`**

Run: `git rm experiments/board-detection-2d/src/boarddet/viz_methode.py`
Expected: file staged for deletion.

- [ ] **Step 3: Update the Method E benchmark import**

In `experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py`, change line 32:

```python
from .viz_methode import render_methode
```

to:

```python
from .viz import render_methode
```

- [ ] **Step 4: Rename the test and update its import + add the orientation unit test**

Run: `git mv experiments/board-detection-2d/tests/test_viz_methode.py experiments/board-detection-2d/tests/test_viz.py`

In `tests/test_viz.py`, change the import line `from boarddet.viz_methode import render_methode` to `from boarddet.viz import render_methode`, and append this unit test that pins the frame-convention decision as data:

```python
def test_front_side_panel_convention():
    """x=front, y=left, z=up: front view is y-z with the horizontal axis
    inverted (left renders left); side view is x-z, +x (front) to the right."""
    from boarddet.viz import _FRONT, _SIDE
    ai, bi, invert_h, _, _, title = _FRONT
    assert (ai, bi) == (1, 2)      # y-z projection (look along x)
    assert invert_h is True        # +y (left) rendered on the left
    assert "front" in title
    ai, bi, invert_h, _, _, title = _SIDE
    assert (ai, bi) == (0, 2)      # x-z projection (look along y)
    assert invert_h is False       # +x (front) to the right
    assert "side" in title
```

- [ ] **Step 5: Run the viz tests + full suite**

Run: `uv run pytest tests/test_viz.py -v`
Expected: PASS — the three moved render_methode smoke tests (detection / no-detection / empty-foreground) plus `test_front_side_panel_convention`.

Run: `uv run pytest -q`
Expected: whole suite green (`benchmark_e_loo` now imports `render_methode` from `viz`; no stale `viz_methode` import remains).

- [ ] **Step 6: Verify no lingering `viz_methode` references**

Run: `grep -rn "viz_methode" experiments/board-detection-2d/src experiments/board-detection-2d/tests`
Expected: no output (empty). If any line prints, fix that import to `viz` and re-run Step 5.

- [ ] **Step 7: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/viz.py \
        experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py \
        experiments/board-detection-2d/tests/test_viz.py
git rm --cached experiments/board-detection-2d/src/boarddet/viz_methode.py 2>/dev/null || true
git commit -m "refactor(boarddet): unify overlays in viz.py, fix front/side views"
```

---

### Task 3: `render_noe` + wire it into `benchmark_noe`

Add the no-Method-E 6-panel wrapper (foreground = big-plane residual) and switch `benchmark_noe` from the 2-panel `save_overlay` to it.

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/viz.py` (append `render_noe`)
- Modify: `experiments/board-detection-2d/src/boarddet/benchmark_noe.py:25,78`
- Test: `experiments/board-detection-2d/tests/test_viz.py`

**Interfaces:**
- Consumes: `render_six_panel` and `render_methode` (Task 2); `big_plane_residual` (Task 1); `downsample`.
- Produces: `render_noe(frame_xyz, board, outcome, box, path, voxel=0.03) -> None` in `viz.py`.

- [ ] **Step 1: Write the failing test**

Append to `experiments/board-detection-2d/tests/test_viz.py`:

```python
def test_render_noe_writes_valid_png(tmp_path):
    """No-E 6-panel: foreground is generator B's big-plane residual; renders a
    valid PNG on a synthetic scene with a detectable board."""
    from boarddet.synth import make_scene
    from boarddet.viz import render_noe
    pts, _ = make_scene(rng=np.random.default_rng(1))
    board = BoardConfig(side_m=1.0)
    out = detect(pts, board, generator="b")
    box = load_bbox(DEFAULT_BBOX_PATH)
    p = tmp_path / "noe.png"
    render_noe(pts, board, out, box, p)
    assert p.exists()
    assert _png_header(p) == _PNG_MAGIC
    assert p.stat().st_size > 3000


def test_render_noe_handles_no_detection(tmp_path):
    """The None-detection path must render, not crash (empty scene -> no board)."""
    from boarddet.viz import render_noe
    pts = np.random.default_rng(9).normal(scale=0.05, size=(200, 3)).astype(
        np.float32)
    board = BoardConfig(side_m=1.0)
    out = detect(pts, board, generator="b")
    assert out.detection is None
    box = load_bbox(DEFAULT_BBOX_PATH)
    p = tmp_path / "noe_none.png"
    render_noe(pts, board, out, box, p)
    assert p.exists()
    assert _png_header(p) == _PNG_MAGIC
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_viz.py::test_render_noe_writes_valid_png -v`
Expected: FAIL — `ImportError: cannot import name 'render_noe'`.

- [ ] **Step 3: Add `render_noe` to `viz.py`**

Append to `experiments/board-detection-2d/src/boarddet/viz.py`, and add `big_plane_residual` to the imports (with the other boarddet imports at the top of the file):

```python
from .candidates.cluster_after_ground import big_plane_residual  # noqa: E402
```

Then the wrapper (after `render_methode`):

```python
def render_noe(frame_xyz: np.ndarray, board: BoardConfig,
               outcome: DetectOutcome, box: BoxRef, path: Path,
               voxel: float = 0.03) -> None:
    """No-Method-E 6-panel view: foreground = generator B's big-plane residual
    (RANSAC-stripped ground/walls), the crop-free analog of a background diff."""
    dn = downsample(frame_xyz, voxel)
    fg = big_plane_residual(dn, board, board.vertical_gap_deg)
    render_six_panel(dn, fg, box, outcome, path,
                     f"after big-plane removal ({len(fg)} pts)")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_viz.py::test_render_noe_writes_valid_png tests/test_viz.py::test_render_noe_handles_no_detection -v`
Expected: PASS.

- [ ] **Step 5: Switch `benchmark_noe` to the 6-panel renderer**

In `experiments/board-detection-2d/src/boarddet/benchmark_noe.py`, change the import (line 25):

```python
from .viz import save_overlay
```

to:

```python
from .viz import render_noe
```

And in `run_noe`, change the overlay call (the `save_overlay(...)` inside the `_overlay_indices` loop, ~line 78):

```python
        for i in _overlay_indices(outcomes, save_overlays):
            save_overlay(frames[i].xyz, outcomes[i],
                         out_dir / f"overlay_{name}_frame{i:04d}.png")
```

to:

```python
        for i in _overlay_indices(outcomes, save_overlays):
            render_noe(frames[i].xyz, board, outcomes[i], box,
                       out_dir / f"overlay_{name}_frame{i:04d}.png")
```

- [ ] **Step 6: Run the benchmark_noe test + full suite**

Run: `uv run pytest tests/test_benchmark_noe.py -q`
Expected: PASS — `test_run_noe_writes_recall_precision_and_overlays` still finds `overlay_synthA_*.png` (filename unchanged; content now 6-panel).

Run: `uv run pytest -q`
Expected: whole suite green.

- [ ] **Step 7: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/viz.py \
        experiments/board-detection-2d/src/boarddet/benchmark_noe.py \
        experiments/board-detection-2d/tests/test_viz.py
git commit -m "feat(boarddet): 6-panel no-E overlay in benchmark_noe"
```

---

### Task 4: Regenerate the no-E overlays across all scenarios

Re-run the four no-E benchmark scenarios so the on-disk overlays become the new 6-panel figures, and visually confirm one. No code; produces artifacts. (Method E overlays are unaffected in content but gain the front/side fix on their next run — regenerated here too for parity.)

**Files:**
- Overwrite (gitignored, keep): `results/compare-noE-pcap-stage6/`, `results/compare-noE-pcap-stage8/`, `results/compare-noE-vlp/`, `results/compare-noE-falcon/` overlay PNGs; optionally `results/compare-E-*/` for the orientation fix.

**Interfaces:**
- Consumes: `boarddet.benchmark_noe` (Task 3), `boarddet.benchmark_e_loo` (Task 2). Same per-scenario flags as the prior comparison run.

- [ ] **Step 1: Regenerate the four no-E overlay sets**

Run (each writes 6-panel overlays into a fresh dir; the Falcon run is ~1 s/frame, allow several minutes):

```bash
uv run python -m boarddet.benchmark_noe --source pcap --names 1 2 3 4 5 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 \
  --bbox ../../ros/lctk_launch/config/board/bbox.json5 \
  --save-overlays 8 --out results/compare-noE-pcap-stage6-6panel

uv run python -m boarddet.benchmark_noe --source bag --sensor vlp32 \
  --names TWO_LIDAR_1 TWO_LIDAR_3 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 \
  --vertical-gap-deg 1.0 --cluster-min-points 20 \
  --bbox ../../ros/lctk_launch/config/board/bbox-vlp.json5 \
  --save-overlays 8 --out results/compare-noE-vlp-6panel

uv run python -m boarddet.benchmark_noe --source bag --sensor falcon \
  --names TWO_LIDAR_1 TWO_LIDAR_3 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 \
  --vertical-gap-deg 0 --square-icp --up-axis 0 1 0 \
  --bbox ../../ros/lctk_launch/config/board/bbox-seyond.json5 \
  --save-overlays 8 --out results/compare-noE-falcon-6panel
```

Expected: each run prints its per-capture recall/precision (unchanged from the earlier comparison — only the overlay format changed) and writes overlay PNGs. Do not delete the earlier `compare-noE-*` dirs; these `-6panel` dirs are additive.

- [ ] **Step 2: Visually confirm one 6-panel no-E overlay**

Open one written PNG, e.g. `results/compare-noE-pcap-stage6-6panel/overlay_3_frame*.png`, and confirm by eye:
- Panel 2 is titled `"after big-plane removal (N pts)"` and shows the residual (board + any non-stripped clutter) in blue.
- Panel 4 is titled `"front (y-z)"`, Panel 5 `"side (x-z)"`; the board's upright diamond stands on a corner with z up in both, and the front panel's y axis increases leftward.

Report: the confirmed PNG path and that panels read correctly. (This is the human-visible acceptance of the feature.)

- [ ] **Step 3: Confirm images preserved**

Run: `find results/compare-noE-*-6panel -name '*.png' | wc -l`
Expected: a positive count. No prior results dir deleted.

*(No commit — `results/` is gitignored.)*

---

## Self-Review

**Spec coverage:**
- No-E gets a 6-panel overlay → Task 3 (`render_noe` + `benchmark_noe` wiring), verified visually in Task 4. ✓
- RANSAC big-plane strip as the "background"/foreground analog → Task 1 (`big_plane_residual`, params shared with the generator so it matches the detected residual). ✓
- Panel 2 residual-only, correct titles → Task 3 (`"after big-plane removal (N pts)"`) and Task 2 (E keeps `"foreground diff (N pts total)"`). ✓
- Front/side orientation fixed to x=front/y=left/z=up → Task 2 (`_FRONT`/`_SIDE` tuples + `_set_limits` inversion + `test_front_side_panel_convention`), applied to both E and no-E via the shared `render_six_panel`. ✓
- Single viz module; delete `viz_methode.py`; no `viz_panels.py`/`viz_noe.py` → Task 2 (rewrite `viz.py`, `git rm viz_methode.py`, import + test rename, grep guard). ✓
- Keep `save_overlay` for `benchmark.py` → Task 2 (retained verbatim in `viz.py`). ✓
- Don't delete images → Global Constraint; Task 4 writes additive `-6panel` dirs and checks survival. ✓

**Placeholder scan:** every code step carries complete code; every run step gives the exact command + expected result. No TBD/TODO. ✓

**Type consistency:** `render_six_panel(dn, fg, box, outcome, path, panel2_title)` is called by both `render_methode` and `render_noe` with matching arg order. `_FRONT`/`_SIDE` are 6-tuples `(ai, bi, invert_h, xlabel, ylabel, title)`, unpacked identically in `render_six_panel` and the convention test. `big_plane_residual(points, board, vertical_gap_deg)` (Task 1) is called by `render_noe` (Task 3) with `board.vertical_gap_deg`. `render_noe(frame_xyz, board, outcome, box, path)` is called in `benchmark_noe.run_noe`, which has `board` and `box` in scope. `_remove_big_planes` return type (`np.ndarray`) flows through `big_plane_residual` unchanged. ✓
