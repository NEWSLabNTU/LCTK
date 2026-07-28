# boarddet reject-reason diagnostics — design

**Date:** 2026-07-28
**Component:** `experiments/board-detection-2d` (`boarddet`)
**Status:** approved design, pending implementation plan

## Problem

The 2D board-detection pipeline silently produces no detection on some frames
with no reason reported. There are ~15 gates across candidate generation, the
2D scorer, and the detector, and each rejects by returning bare `None` or
`continue`-ing. When a frame fails, the operator cannot tell **why**, and — the
primary need — cannot tell **whether a mistuned (too-tight) parameter is to
blame** versus a genuine no-board frame.

## Goal

When `detect()` returns no detection, surface a structured, self-diagnosing
reject reason that answers "is a bad parameter to blame?":

- which gate killed the furthest-progressing candidate,
- which `BoardConfig` field (if any) governs that gate,
- the measured metric, the threshold, and a normalized **margin** (how far
  past the bound it fell) — a small margin means "barely failed → suspect the
  param"; a large margin means "genuinely not a board".

Returning the geometry of the near-miss ("most plausible board") is explicitly
**out of scope** — the diagnostic is the reason, not the board.

## Non-goals

- Rendering / visualizing the near-miss cluster or quad. `RejectReason` carries
  metrics only, no points/corners.
- Changing any detection behavior. Every gate keeps its current threshold and
  accept/reject decision; this work only *records* why a reject happened. All
  default-off gates stay default-off; passing frames are byte-identical.
- Instrumenting internals of `fit_fixed_square`, RANSAC, or DBSCAN below the
  `plausible_board_patch` gate.

## Design

### New module: `reject.py`

Zero-dependency module (imported by `scorer`, `detector`, `candidates`) to
avoid an import cycle.

```python
class Stage(IntEnum):
    # generation band (cluster -> candidate), gapped below the scorer band
    NO_CLUSTERS       = 0    # generator emitted zero clusters at all
    PATCH_POINTS      = 1    # cluster < _MIN_PATCH_POINTS
    PATCH_FLATNESS    = 2    # plane_rms > flatness_rms_max
    PATCH_EXTENT      = 3    # extent outside [0.5*side, 1.8*diag]
    # scorer band
    MIN_POINTS        = 11   # coords < _MIN_POINTS (60)
    RASTER_SIZE       = 12   # raster > 4000 px
    MINAREA_SIZE      = 13   # minAreaRect side < 3*cell / no contour
    SIZE_GATE         = 14   # coarse mean side out of 2*side_tol band
    STRICT_SQUARENESS = 15   # max corner angle dev > 8 deg
    STANCE_2D         = 16   # 2D diamond stance <= stance_floor
    EDGE_SUPPORT      = 17   # min side support < edge_support_min
    SIDE_ERR          = 18   # |mean side - side_m| > side_tol*side_m
    # detector band
    SQUARE_FIT        = 21   # icp: fit None or residual >= square_icp_residual_max
    MIN_SCORE         = 22   # non-icp: det.score < min_score
    STANCE_3D         = 23   # icp: 3D stance <= stance_floor
    ISOLATION         = 24   # both paths: density > isolation_max_density

@dataclass(frozen=True)
class RejectReason:
    stage:  Stage
    gate:   str                                    # human name, e.g. "size_gate"
    param:  str | None                             # governing BoardConfig field, or None
    value:  float | None                           # measured metric
    threshold: float | tuple[float, float] | None  # scalar bound or (lo, hi) band
    margin: float                                  # normalized distance past bound, >= 0
```

**Stage ordering** is monotonic within each active path, so "furthest stage
reached" is well-defined:

- generation: 0..3
- non-icp: 11..18 (scorer) -> 22 (min_score) -> 24 (isolation)
- icp: 21 (square_fit) -> 23 (stance_3d) -> 24 (isolation); in the icp path a
  scorer `RejectReason` (stages 11..18) is **non-fatal** (the candidate is
  rescued by `fit_fixed_square`) and is therefore **not** recorded.

A cluster that reached scoring (>= 11) always outranks one that never qualified
(<= 3), which is the intended notion of "most param-suspect".

### `param` attribution table

| Stage | param | tunable |
|---|---|---|
| PATCH_POINTS | `None` | structural (`_MIN_PATCH_POINTS` hardcoded) |
| PATCH_FLATNESS | `flatness_rms_max` | yes |
| PATCH_EXTENT | `None` | geometric (derived from `side_m`) |
| MIN_POINTS | `None` | structural (`_MIN_POINTS` hardcoded) |
| RASTER_SIZE | `None` | structural |
| MINAREA_SIZE | `None` | structural (`3*cell`) |
| SIZE_GATE | `side_tol` | yes |
| STRICT_SQUARENESS | `strict_squareness` | yes (toggle; 8 deg hardcoded) |
| STANCE_2D | `stance_floor` | yes |
| EDGE_SUPPORT | `edge_support_min` | yes |
| SIDE_ERR | `side_tol` | yes |
| SQUARE_FIT | `square_icp_residual_max` | yes |
| MIN_SCORE | `min_score` | yes |
| STANCE_3D | `stance_floor` | yes |
| ISOLATION | `isolation_max_density` | yes |

`param is None` is itself a signal: the failure is structural and no config
change rescues that candidate.

### `margin` definition

`margin` is the normalized amount by which the metric overshot its bound,
always `>= 0` for a rejected candidate:

- upper-bound gate (`value > thr`): `margin = (value - thr) / thr`
- lower-bound gate (`value < thr`, e.g. `min_score`, `stance_floor`):
  `margin = (thr - value) / thr`
- band gate (`value` outside `(lo, hi)`): `margin = dist_outside / ((hi - lo) / 2)`

Small margin => barely failed => suspect the param. Large margin => genuine
non-board.

### Scorer change (`scorer.py`)

`score_candidate` return type `ScoreResult | None` -> **`ScoreResult |
RejectReason`**. Each of its 8 `return None` sites returns a typed
`RejectReason` (stages 11..18) with `param`, `value`, `threshold`, `margin`
filled per the tables above. The accept path is unchanged (still returns a
`ScoreResult` byte-identical to today). Callers switch `res is None` ->
`isinstance(res, RejectReason)`.

### Generation change (`candidates/__init__.py` + 4 generators)

`plausible_board_patch` return type `Candidate | None` -> **`Candidate |
RejectReason`** (stages 1..3). Callers switch `cand is not None` ->
`isinstance(res, Candidate)`.

Each generator (`ransac_iterative`, `cluster_after_ground`, `region_growing`,
`background_diff`) gains **one optional kwarg** `rejects: list[RejectReason] |
None = None`. When provided, patch-level `RejectReason`s are appended; when
omitted (default) behavior and return shape are byte-identical:

```python
res = plausible_board_patch(group_pts, board)
if isinstance(res, Candidate):
    out.append(res)
elif rejects is not None:
    rejects.append(res)
```

### Detector change (`detector.py`)

`DetectOutcome` gains one field:

```python
reject_reason: RejectReason | None = None   # furthest-stage reason; None when detected
```

(`best_rejected` is left exactly as-is — overlays still consume it.)

`detect()`:

1. build a `patch_rejects: list[RejectReason]` and pass it into the generator
   call (`rejects=patch_rejects`), for every generator.
2. define `_consider(reason)`: keep the reason with the max `stage`
   (tie -> first seen).
3. feed all `patch_rejects` into `_consider`.
4. non-icp path: scorer `RejectReason` -> `_consider(res); continue`; the
   `min_score` and `isolation` gates each build and `_consider` their own
   `RejectReason` on reject.
5. icp path: scorer reason ignored (non-fatal); `square_fit` (fit None or
   residual too high), 3D `stance_floor`, and `isolation` each `_consider` a
   `RejectReason` on reject.
6. if a detection is found, `reject_reason = None`.
7. if no candidates *and* no patch rejects were produced,
   `reject_reason = RejectReason(Stage.NO_CLUSTERS, "no_clusters", None, None,
   None, 0.0)`.

### Benchmark change (`benchmark.py` + `benchmark_e_loo.py`)

Aggregate `reject_reason.param` across every no-detection frame into
`summary.md`/`summary.json`: a histogram of furthest-stage killer params (with
a `None`/structural bucket). This is the run-level "which knob is mistuned"
signal — the strongest evidence that a param, not the data, is the cause. Also
record, per killer param, the min/median/max `margin` so a param whose failures
all sit at tiny margins is immediately visible.

## Testing

- `reject.py`: `margin` formula per gate-kind (upper/lower/band) on hand-picked
  values; `Stage` ordering (generation < scorer < detector within a path).
- `scorer.py`: craft `coords_2d` tripping each of the 8 gates; assert returned
  `RejectReason.stage`/`gate`/`param`/`margin`. Accept path still returns
  `ScoreResult`. Keep the existing byte-identical isotropic pin (only the
  return-None sites change).
- `plausible_board_patch`: each of the 3 gates returns the right
  `RejectReason`; accept path returns a `Candidate`. Update `test_candidates_a`
  `is None` / `is not None` asserts to `isinstance`.
- generators: when `rejects=[]` is passed, it collects patch rejects; when the
  kwarg is omitted, output list is unchanged (byte-identical guard).
- `detector.py`: synth scenes where all candidates die at one known gate ->
  assert `outcome.reject_reason` stage/param; a scene that detects -> assert
  `reject_reason is None`; a zero-cluster scene -> `NO_CLUSTERS`.
- benchmark: the param histogram + margin stats are produced and shaped
  correctly on a small forced run.

## Files touched

- new `src/boarddet/reject.py`
- `src/boarddet/scorer.py` (return type + 8 sites)
- `src/boarddet/candidates/__init__.py` (return type + 3 sites)
- `src/boarddet/candidates/{ransac_iterative,cluster_after_ground,region_growing,background_diff}.py`
  (`rejects` kwarg + isinstance branch)
- `src/boarddet/detector.py` (`DetectOutcome.reject_reason`, `_consider`, wiring)
- `src/boarddet/benchmark.py`, `src/boarddet/benchmark_e_loo.py` (histogram)
- tests: `test_reject.py` (new), `test_scorer.py`, `test_candidates_a.py`,
  `test_detector.py`, benchmark test
