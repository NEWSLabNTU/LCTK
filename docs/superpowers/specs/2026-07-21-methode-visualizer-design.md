# Method E Pipeline Visualizer — Design

## Problem

Since Method E (background subtraction) landed, `boarddet` has no way to *see*
its pipeline. The existing `viz.py` renders a 2-panel overlay (top-down +
plane raster) of the **final detection only** — it cannot show the stages
Method E added: the raw cloud, the background-subtracted foreground, the
per-rig bbox reference, or a rejected candidate. That gap is concrete: on the
recorded TWO_LIDAR bags the detector rejects the far (~9–10 m) board on most
frames, and deciding *why* (is the candidate the board, or the board merged
with its vertical support stand?) needs a look at the vertical structure a
top-down projection hides. Numbers and ASCII grids are not enough.

## Goals

- Render the full Method E pipeline for one frame so correctness is verifiable by eye.
- Reveal vertical structure (the board-vs-stand question) via front/side projections.
- Reusable and committed — foster future development, not a one-off.
- Zero change to existing behavior when the feature is not requested.

## Non-goals

- Interactive 3D now (that is Phase 2, documented below, not built).
- A standalone render command (the harness drives it; chosen over a separate CLI).
- Rendering every candidate (only the accepted detection and the best rejected one).
- Touching `viz.py` or the pcap `benchmark.py` overlay path.

## Phase 1 — 6-panel PNG renderer, harness-driven (this spec)

### Module

New `src/boarddet/viz_methode.py`, separate from the generator-agnostic
`viz.py`. Matplotlib `Agg` (headless, as `viz.py` already uses). One public
function:

```python
def render_methode(frame_xyz: np.ndarray, board: BoardConfig,
                   background: BackgroundModel, outcome: DetectOutcome,
                   box: BoxRef, path: Path, voxel: float = 0.03) -> None
```

It recomputes the display layers itself so the caller passes only what it
already holds:

- `dn = downsample(frame_xyz, voxel)` — the same cloud `detect()` builds.
- `fg = background.foreground_points(dn)` — the diff result.
- accepted quad ← `outcome.detection` (may be None).
- rejected quad ← `outcome.best_rejected` (may be None).

The full candidate list is deliberately not shown — `DetectOutcome` exposes
only `detection` and `best_rejected`, and those two answer the verification
question without new plumbing.

### Panels

| # | panel | view | layers |
|---|---|---|---|
| 1 | RAW | top-down x–y | full cloud, gray |
| 2 | FOREGROUND | top-down x–y | diff result only, blue |
| 3 | MIX | top-down x–y | raw gray + fg blue + candidate orange + bbox green + detection red |
| 4 | FRONT | x–z | same layers, height axis |
| 5 | SIDE | y–z | same layers, height axis |
| 6 | RASTER | plane | plane raster + refined quad (as `viz.py` panel 2) |

Fixed colors: raw `gray`, foreground `blue`, candidate (`best_rejected`)
`orange`, bbox `green`, detection `red`. Panel titles carry the state: score
when detected, `NO DETECTION` otherwise; fold/frame label in the figure
suptitle.

### bbox drawing

`BoxRef` carries `center` (3,), `half` (3,), `rot` (3×3, box→world). Draw its
wireframe rectangle in each projection by transforming the 8 box corners
(`center + rot @ (±half)`) into world coords and plotting the relevant 2D
face outline per panel. Rotation-aware by construction; the two bag rigs are
axis-aligned so it reduces to an axis-aligned rectangle there.

### Harness wiring

`benchmark_e_loo` gains one flag:

```
--save-overlays N   (default 0 = off; existing runs stay byte-identical)
```

When `N > 0`, `run_loo` saves N overlays per fold into the run's `--out`
directory, named `overlay_<held_out>_frame<idx>.png`. Frame selection reuses
`benchmark.py`'s existing idiom: the first accepted detection, the frame whose
`best_rejected` scores highest, and an even spread across the fold, deduped
and capped at N. `run_loo` already keeps `frame`, `outcome`, and `model` in
scope per iteration, so wiring is the flag plus a save block — no signature
change to `run_loo` beyond the new keyword argument, no change to
`build_background`, `load_sources`, or the detection loop's numbers.

### Testing

`tests/test_viz_methode.py`:

1. A synthetic scene from `make_scene` with a board, a populated
   `BackgroundModel`, and a real `detect(..., generator="e")` outcome →
   `render_methode` writes a PNG whose file size is non-trivial (> a few KB).
2. No-detection case (background memorizes the board so nothing survives) →
   renders without raising, file written.
3. Empty-foreground case (query equals background) → renders without raising.

Pixel-exact assertions are out of scope for an `Agg` render; the tests pin
"produces a valid non-empty image and never crashes on the None/empty paths",
which is where this code will actually break.

## Phase 2 — interactive self-contained HTML (follow-on, not built here)

One standalone `.html` per frame: a rotatable 3D point cloud (three.js
inlined, no server, points downsampled and embedded as JSON), same fixed
layer colors, the bbox and quads as 3D line loops. Opens in any browser,
survives copying, needs no network. To be specced and planned separately once
Phase 1's panels confirm the stage layers are correct — the two share the
layer/colour definitions, so Phase 1 should keep those in one place
`viz_methode` can later export.

## Files

| file | change |
|---|---|
| `src/boarddet/viz_methode.py` | new — `render_methode` + panel/layer helpers |
| `src/boarddet/benchmark_e_loo.py` | `--save-overlays N` flag + per-fold save block |
| `tests/test_viz_methode.py` | new — render smoke + None/empty edge cases |
