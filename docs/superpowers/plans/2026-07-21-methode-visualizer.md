# Method E Pipeline Visualizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render the full Method E detection pipeline for one frame as a 6-panel PNG, and have the LOO harness auto-save a spread of them per fold.

**Architecture:** A new `viz_methode.py` module recomputes its own display layers (downsample, background diff) from the frame and the caller's `BackgroundModel`, and reads the accepted/rejected quads off the `DetectOutcome`. A `--save-overlays N` flag on `benchmark_e_loo` calls it for a sampled set of frames per fold. Default off, so existing runs are byte-identical.

**Tech Stack:** Python 3.11, matplotlib (`Agg`, headless), numpy — all already dependencies. `uv` project.

## Context

`boarddet` (`experiments/board-detection-2d/`) is the phase-7 crop-box-free
board detector. Its existing overlay, `viz.py`'s `save_overlay`, shows only
the final detection in 2 panels and predates Method E, so it cannot show the
background-subtraction stages. Design spec:
[docs/superpowers/specs/2026-07-21-methode-visualizer-design.md](../specs/2026-07-21-methode-visualizer-design.md).

The immediate motivating question: on the recorded TWO_LIDAR bags the board
sits at ~9–10 m and is rejected on most frames; front/side projections are
needed to see whether the candidate is the board or the board merged with its
vertical support stand.

Pinned facts from the current code (verified):

- `DetectOutcome` (`detector.py:30-35`): fields `detection: BoardDetection | None`, `timings_ms`, `n_candidates: int`, `best_rejected: BoardDetection | None = None`.
- `BoardDetection` (`pose.py:12-18`): `center: np.ndarray` (3,), `rotation` (3,3), `corners_3d: np.ndarray` (4,3), `score: float`, `result: ScoreResult`.
- `BoxRef` (`bbox_ref.py`): `center` (3,), `half` (3,), `rot` (3,3, box→world); `contains(point) -> bool`.
- `downsample(points, voxel=0.03)` and `project_to_plane` in `geometry.py`; `BackgroundModel.foreground_points(dn)` in `background.py`.
- Existing raster-panel drawing pattern to mirror: `viz.py:38-50` (uses `res.raster`, `res.rot_2d`, `res.corners_2d`, `res.origin`, `res.cell_m`).
- `run_loo`'s per-fold loop (`benchmark_e_loo.py:101-106`): `for held_out in sources: model = build_background(...); outcomes = [detect(f.xyz, board, generator="e", background=model) for f in sources[held_out]]`. `model`, `box`, `board`, and `sources[held_out]` are all in scope for a save block.
- Frame-pick idiom to reuse (`benchmark.py:112-119`): first/mid/last detection index, else first/mid/last frame.

## Global Constraints

- All work in `experiments/board-detection-2d/`, run via `uv` (`uv run pytest`, `uv run python …`). Never plain `python`/`pytest`, never `pip install`.
- matplotlib must stay `Agg` (headless Jetson) — set the backend before importing `pyplot`, exactly as `viz.py:6-9` does.
- Default behavior unchanged: `--save-overlays` defaults to 0 (off); `run_loo`'s returned summary and per-fold numbers are byte-identical when it is 0.
- Do NOT modify `viz.py` or `benchmark.py`.
- The renderer shows only `outcome.detection` (red) and `outcome.best_rejected` (orange) — not a full candidate list (`DetectOutcome` exposes no such list).
- Fixed layer colors: raw `gray`, foreground `blue`, candidate/`best_rejected` `orange`, bbox `green`, detection `red`.
- Commit per task. Work on the current branch; do not create or switch branches.

---

### Task 1: `viz_methode.py` — the 6-panel renderer

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/viz_methode.py`
- Test: `experiments/board-detection-2d/tests/test_viz_methode.py`

**Interfaces:**
- Consumes: `BoardConfig`, `BackgroundModel`, `DetectOutcome`/`BoardDetection`, `BoxRef`, `downsample` (signatures in Context).
- Produces: `render_methode(frame_xyz: np.ndarray, board: BoardConfig, background: BackgroundModel, outcome: DetectOutcome, box: BoxRef, path: Path, voxel: float = 0.03) -> None`.

- [ ] **Step 1: Write the failing tests**

Create `experiments/board-detection-2d/tests/test_viz_methode.py`:

```python
"""Smoke + edge-case tests for the Method E 6-panel renderer. An Agg render
can't be pixel-asserted cheaply, so these pin 'writes a valid non-empty PNG'
and 'never crashes on the None-detection / empty-foreground paths' -- where
this code actually breaks."""
from __future__ import annotations

import numpy as np

from boarddet.background import BackgroundModel
from boarddet.bbox_ref import load_bbox
from boarddet.benchmark_e_loo import DEFAULT_BBOX_PATH
from boarddet.board_config import BoardConfig
from boarddet.detector import detect
from boarddet.geometry import downsample
from boarddet.viz_methode import render_methode


def _png_header(p) -> bytes:
    return p.read_bytes()[:8]


_PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _model(points) -> BackgroundModel:
    m = BackgroundModel(min_sources=1)
    m.observe(downsample(points, 0.03), source=0)
    m.finalize()
    return m


def test_renders_a_detection(tmp_path):
    from boarddet.synth import make_scene
    bg, _ = make_scene(rng=np.random.default_rng(0), include_board=False)
    reveal, _ = make_scene(rng=np.random.default_rng(1))
    board = BoardConfig(side_m=1.0)
    out = detect(reveal, board, generator="e", background=_model(bg))
    assert out.detection is not None
    box = load_bbox(DEFAULT_BBOX_PATH)
    p = tmp_path / "det.png"
    render_methode(reveal, board, _model(bg), out, box, p)
    assert p.exists()
    assert _png_header(p) == _PNG_MAGIC
    assert p.stat().st_size > 3000


def test_renders_without_detection(tmp_path):
    """Background memorizes the board, so the reveal has no foreground and no
    detection -- the None path must render, not crash."""
    from boarddet.synth import make_scene
    scene, _ = make_scene(rng=np.random.default_rng(2))
    board = BoardConfig(side_m=1.0)
    model = _model(scene)  # same scene as background -> nothing new
    out = detect(scene, board, generator="e", background=model)
    assert out.detection is None
    box = load_bbox(DEFAULT_BBOX_PATH)
    p = tmp_path / "nodet.png"
    render_methode(scene, board, model, out, box, p)
    assert p.exists()
    assert _png_header(p) == _PNG_MAGIC


def test_renders_with_empty_foreground(tmp_path):
    """A finalized-but-identical background yields an empty foreground array;
    the renderer must handle a 0-row layer without raising."""
    from boarddet.synth import make_scene
    scene, _ = make_scene(rng=np.random.default_rng(3))
    board = BoardConfig(side_m=1.0)
    model = _model(scene)
    out = detect(scene, board, generator="e", background=model)
    box = load_bbox(DEFAULT_BBOX_PATH)
    p = tmp_path / "empty.png"
    # sanity: foreground really is empty on this identical replay
    assert len(model.foreground_points(downsample(scene, 0.03))) == 0
    render_methode(scene, board, model, out, box, p)
    assert p.exists()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/jetson/LCTK/experiments/board-detection-2d
uv run pytest tests/test_viz_methode.py -q
```
Expected: FAIL — `ModuleNotFoundError: No module named 'boarddet.viz_methode'`

- [ ] **Step 3: Implement the renderer**

Create `experiments/board-detection-2d/src/boarddet/viz_methode.py`:

```python
"""Six-panel render of the full Method E pipeline for one frame.

Separate from viz.py (the generator-agnostic 2-panel overlay): this one
needs the background model and the per-rig bbox, and shows the
background-subtraction stages viz.py cannot. Headless Agg, like viz.py.
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

# Fixed layer colors (design spec).
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


def _box_corners_world(box: BoxRef) -> np.ndarray:
    """(8,3) world-frame corners of the reference box."""
    return box.center + (_BOX_SIGNS * box.half) @ box.rot.T


def _draw_box(ax, corners: np.ndarray, ai: int, bi: int) -> None:
    """Draw the box wireframe projected onto axes (ai, bi) of world coords."""
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


def _proj_panel(ax, ai: int, bi: int, labels: tuple[str, str], title: str,
                raw, fg, box_corners, outcome: DetectOutcome) -> None:
    """One orthographic projection panel with all layers."""
    _scatter(ax, raw, ai, bi, _C_RAW, 0.5, 0.35)
    _scatter(ax, fg, ai, bi, _C_FG, 1.5, 0.8)
    _draw_box(ax, box_corners, ai, bi)
    if outcome.best_rejected is not None:
        _draw_quad(ax, outcome.best_rejected, ai, bi, _C_CAND)
    if outcome.detection is not None:
        _draw_quad(ax, outcome.detection, ai, bi, _C_DET)
    ax.set_aspect("equal")
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_title(title)


def render_methode(frame_xyz: np.ndarray, board: BoardConfig,
                   background: BackgroundModel, outcome: DetectOutcome,
                   box: BoxRef, path: Path, voxel: float = 0.03) -> None:
    dn = downsample(frame_xyz, voxel)
    fg = background.foreground_points(dn)
    box_corners = _box_corners_world(box)

    det = outcome.detection
    state = (f"score={det.score:.2f}" if det is not None else "NO DETECTION")

    fig, axes = plt.subplots(2, 3, figsize=(19, 11))

    # Panel 1: raw only, top-down
    _scatter(axes[0, 0], dn, 0, 1, _C_RAW, 0.5, 0.5)
    _draw_box(axes[0, 0], box_corners, 0, 1)
    axes[0, 0].set_aspect("equal")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].set_title("raw cloud (top-down)")

    # Panel 2: foreground only, top-down
    _scatter(axes[0, 1], fg, 0, 1, _C_FG, 1.5, 0.9)
    _draw_box(axes[0, 1], box_corners, 0, 1)
    axes[0, 1].set_aspect("equal")
    axes[0, 1].set_xlabel("x [m]")
    axes[0, 1].set_ylabel("y [m]")
    axes[0, 1].set_title(f"foreground diff ({len(fg)} pts)")

    # Panel 3: mix, top-down
    _proj_panel(axes[0, 2], 0, 1, ("x [m]", "y [m]"), f"mix (top-down) | {state}",
                dn, fg, box_corners, outcome)

    # Panel 4: front x-z
    _proj_panel(axes[1, 0], 0, 2, ("x [m]", "z [m]"), "front (x-z)",
                dn, fg, box_corners, outcome)

    # Panel 5: side y-z
    _proj_panel(axes[1, 1], 1, 2, ("y [m]", "z [m]"), "side (y-z)",
                dn, fg, box_corners, outcome)

    # Panel 6: plane raster + refined quad (mirrors viz.py:38-50)
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

    fig.suptitle(Path(path).stem, fontsize=12)
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=100)
    plt.close(fig)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/test_viz_methode.py -q
```
Expected: PASS, 3 tests.

- [ ] **Step 5: Full suite, then commit**

```bash
uv run pytest -q
```
Expected: all pass (200 existing + 3 new = 203).

```bash
cd /home/jetson/LCTK
git add experiments/board-detection-2d/src/boarddet/viz_methode.py \
        experiments/board-detection-2d/tests/test_viz_methode.py
git commit -m "feat(boarddet): 6-panel Method E pipeline renderer

Raw / foreground-diff / mix top-downs plus front and side projections plus
the plane raster, with fixed layer colors (raw gray, foreground blue,
best-rejected orange, bbox green, detection red). The front/side views
reveal vertical structure a top-down hides -- e.g. a support stand merged
into a far board. Separate from viz.py because it needs the background
model and the per-rig bbox."
```

---

### Task 2: `--save-overlays` on the LOO harness

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py`
- Test: `experiments/board-detection-2d/tests/test_benchmark_e_loo.py`

**Interfaces:**
- Consumes: `render_methode` (Task 1).
- Produces: `run_loo(..., save_overlays: int = 0)` keyword; CLI `--save-overlays N`. When `N > 0`, writes `overlay_<held_out>_frame<idx>.png` into `out_dir`.

- [ ] **Step 1: Write the failing test**

Add to `experiments/board-detection-2d/tests/test_benchmark_e_loo.py`:

```python
def test_save_overlays_writes_pngs_and_is_off_by_default(tmp_path):
    """save_overlays=0 writes no overlay PNGs; >0 writes at most that many
    per fold, and the returned summary is identical either way."""
    from boarddet.board_config import BoardConfig
    sources = {"A": _frames(1.0), "B": _frames(1.0),
               "C": _frames(1.0), "D": _frames(9.0)}
    board = BoardConfig(side_m=1.0)

    off = tmp_path / "off"
    s0 = loo.run_loo(sources, board, off, box=_BOX, min_sources=2,
                     dilation_radius=0, save_overlays=0)
    assert list(off.glob("overlay_*.png")) == []

    on = tmp_path / "on"
    s1 = loo.run_loo(sources, board, on, box=_BOX, min_sources=2,
                     dilation_radius=0, save_overlays=2)
    pngs = list(on.glob("overlay_*.png"))
    assert len(pngs) > 0
    # at most save_overlays per fold (4 folds x 2)
    assert len(pngs) <= 8
    # numbers unaffected by rendering
    assert s0["folds"] == s1["folds"]
```

`_frames` and `_BOX` already exist in this test file (from earlier tasks).

- [ ] **Step 2: Run the test to verify it fails**

```bash
uv run pytest tests/test_benchmark_e_loo.py::test_save_overlays_writes_pngs_and_is_off_by_default -q
```
Expected: FAIL — `run_loo() got an unexpected keyword argument 'save_overlays'`

- [ ] **Step 3: Implement the save block**

In `benchmark_e_loo.py`, add the import near the top (after the existing `from .detect import` / sibling imports):

```python
from .viz_methode import render_methode
```

Add `save_overlays: int = 0` to `run_loo`'s keyword-only parameters (alongside `background_voxel` / `dilation_radius` / `min_sources`).

Inside the `for held_out in sources:` loop, AFTER `outcomes = [...]` is built and BEFORE the `folds[held_out] = {...}` assignment, insert:

```python
        if save_overlays > 0:
            _save_fold_overlays(sources[held_out], outcomes, board, model,
                                box, out_dir, held_out, save_overlays)
```

Add the helper at module scope (mirrors `benchmark.py:112-119`'s pick idiom,
extended to also surface the best rejection):

```python
def _pick_overlay_indices(outcomes: list, n: int) -> list[int]:
    """Up to n frame indices to render: the first detection, the highest-
    scoring rejection, and an even spread -- deduped, capped at n."""
    picks: list[int] = []
    det_idx = [i for i, o in enumerate(outcomes) if o.detection is not None]
    if det_idx:
        picks.append(det_idx[0])
    rej = [(o.best_rejected.score, i) for i, o in enumerate(outcomes)
           if o.best_rejected is not None]
    if rej:
        picks.append(max(rej)[1])
    if outcomes:
        step = max(1, len(outcomes) // n)
        picks.extend(range(0, len(outcomes), step))
    seen: list[int] = []
    for i in picks:
        if i not in seen:
            seen.append(i)
        if len(seen) >= n:
            break
    return seen


def _save_fold_overlays(frames, outcomes, board, model, box, out_dir,
                        held_out, n) -> None:
    for i in _pick_overlay_indices(outcomes, n):
        render_methode(frames[i].xyz, board, model, outcomes[i], box,
                       out_dir / f"overlay_{held_out}_frame{i:04d}.png")
```

Add the CLI flag in `main()` (next to `--isolation-max-density`):

```python
    ap.add_argument("--save-overlays", type=int, default=0,
                    help="render this many Method E 6-panel overlays per "
                         "fold into --out (0 = off)")
```

and thread it into the `run_loo(...)` call in `main()`: add
`save_overlays=args.save_overlays`.

- [ ] **Step 4: Run the test to verify it passes**

```bash
uv run pytest tests/test_benchmark_e_loo.py -q
```
Expected: PASS (all in that file, including the new one).

- [ ] **Step 5: Full suite, then commit**

```bash
uv run pytest -q
```
Expected: all pass (203 + 1 = 204).

```bash
cd /home/jetson/LCTK
git add experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py \
        experiments/board-detection-2d/tests/test_benchmark_e_loo.py
git commit -m "feat(boarddet): --save-overlays renders Method E overlays per fold

Off by default (0), so every existing run and its summary are unchanged.
When set, saves that many 6-panel overlays per held-out fold -- the first
detection, the best rejection, and an even spread -- into the run's output
directory."
```

---

## Self-review notes

- Spec coverage: 6 panels + colors (Task 1), bbox rotation-aware wireframe (Task 1 `_box_corners_world`/`_draw_box` via `box.rot`), harness `--save-overlays N` default-off (Task 2), tests for detection/no-detection/empty-fg (Task 1) and off-by-default + numbers-unchanged (Task 2). All covered.
- Phase 2 (interactive HTML) is intentionally not in this plan — spec marks it a follow-on.
- Type consistency: `render_methode` signature identical in Task 1 definition, Task 1 tests, and Task 2's `_save_fold_overlays` call site.

## Out of scope

- Interactive HTML (Phase 2 — separate spec/plan).
- Any change to `viz.py`, `benchmark.py`, or the detection numbers.
- A standalone render CLI (the harness flag is the only entry point, per the design decision).
