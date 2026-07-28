# boarddet reject-reason diagnostics — design (side-channel)

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

When `detect()` returns no detection, surface **one** structured, self-diagnosing
reject reason describing the furthest-progressing candidate's killer gate:

- which gate killed it,
- which `BoardConfig` field (if any) governs that gate,
- the measured metric, the threshold, and a normalized **margin** (how far past
  the bound it fell) — a small margin means "barely failed → suspect the param";
  a large margin means "genuinely not a board".

The diagnostic is the reason, not the board. Returning the near-miss geometry is
out of scope.

## Mechanism: side-channel collector (Approach 1)

No public return types change. An optional accumulator
`rejects: list[RejectReason] | None = None` threads down through `detect()` →
generators → `plausible_board_patch`, and into `score_candidate`. Each gate keeps
returning `... | None`; on reject, `if rejects is not None:
rejects.append(RejectReason(...))`. When the kwarg is omitted (the default), the
control flow, return shape, and outputs are **byte-identical** to today.

This was chosen over changing return types (`ScoreResult | RejectReason`) to keep
the diff additive and the passing path untouched, and because the plain-struct
collector ports cleanly to Rust (`&mut Vec<RejectReason>`) when the diagnostic is
later carried into the ROS `lidar_board_detector`.

## Non-goals

- Rendering / visualizing the near-miss cluster or quad. `RejectReason` carries
  metrics only, no points/corners.
- Changing any detection behavior. Every gate keeps its threshold and accept/
  reject decision; this work only *records* why a reject happened. Default-off
  gates stay default-off; passing frames are byte-identical.
- Instrumenting internals of `fit_fixed_square`, RANSAC, or DBSCAN below the
  `plausible_board_patch` gate.
- Benchmark aggregation / run-level param histogram. Deferred — the stated need
  is the single-frame reason. `RejectReason` is designed to make later
  aggregation trivial, but no benchmark change ships in this pass.
- ROS / Rust port. Deferred; the taxonomy is designed to be portable.

## Design

### New module: `reject.py`

Zero-dependency module (imported by `scorer`, `detector`, `candidates`) to avoid
an import cycle.

```python
class Stage(IntEnum):
    # generation band (cluster -> candidate), gapped below the scorer band
    NO_CLUSTERS       = 0    # generator emitted zero clusters at all
    PATCH_POINTS      = 1    # patch < _MIN_PATCH_POINTS
    PATCH_FLATNESS    = 2    # plane_rms > flatness_rms_max
    PATCH_EXTENT      = 3    # extent outside [0.5*side, 1.8*diag]
    # scorer band
    MIN_POINTS        = 11   # coords < _MIN_POINTS (60)
    RASTER_SIZE       = 12   # raster > 4000 px
    MINAREA_SIZE      = 13   # minAreaRect side too small / no contour
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
  rescued by `fit_fixed_square`) and is therefore **not** collected.

A cluster that reached scoring (>= 11) always outranks one that never qualified
(<= 3) — the intended notion of "most param-suspect".

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

`margin` is the normalized amount by which the metric overshot its bound, always
`>= 0` for a rejected candidate:

- upper-bound gate (`value > thr`): `margin = (value - thr) / thr`
- lower-bound gate (`value < thr`, e.g. `min_score`, `stance_floor`):
  `margin = (thr - value) / thr`
- band gate (`value` outside `(lo, hi)`): `margin = dist_outside / ((hi - lo) / 2)`

`threshold == 0` lower-bound gates (e.g. `stance_floor` when it equals its
metric's own floor) never reach the append site because the gate itself is
guarded by `> 0`; where a zero threshold is still possible, `margin` is defined
as `0.0` to avoid division by zero. Small margin => barely failed => suspect the
param. Large margin => genuine non-board.

### Helper constructors on `reject.py`

To keep call sites terse and consistent, `reject.py` exposes three builders that
compute `margin` from the gate kind:

```python
def upper(stage, gate, param, value, thr) -> RejectReason
def lower(stage, gate, param, value, thr) -> RejectReason
def band(stage, gate, param, value, lo, hi) -> RejectReason
```

### Scorer change (`scorer.py`)

`score_candidate` gains `rejects: list[RejectReason] | None = None`. Its return
type stays `ScoreResult | None`. Each of its `return None` sites, when `rejects`
is not None, appends a typed `RejectReason` (stages 11..18) before returning.
The accept path is unchanged (still returns a `ScoreResult` byte-identical to
today).

### Generation change (`candidates/__init__.py` + 4 generators)

`plausible_board_patch` gains `rejects: list[RejectReason] | None = None`; return
type stays `Candidate | None`. Its 3 `return None` sites append stages 1..3 when
`rejects` is not None.

Each generator (`ransac_iterative`, `cluster_after_ground`, `region_growing`,
`background_diff`) gains the same optional kwarg and forwards it into its
`plausible_board_patch(...)` call:

```python
cand = plausible_board_patch(group_pts, board, rejects=rejects)
if cand is not None:
    out.append(cand)
```

When omitted (default) behavior and return shape are byte-identical.

### Detector change (`detector.py`)

`DetectOutcome` gains one field:

```python
reject_reason: RejectReason | None = None   # furthest-stage reason; None when detected
```

(`best_rejected` is left exactly as-is — overlays still consume it.)

`detect()`:

1. build a `rejects: list[RejectReason] = []` and pass it into the generator call
   (`rejects=rejects`), for every generator.
2. non-icp path: pass `rejects=rejects` into `score_candidate`; the `min_score`
   and `isolation` gates each append their own `RejectReason` on reject.
3. icp path: do **not** pass `rejects` into `score_candidate` (scorer reason is
   non-fatal, the candidate is rescued by `fit_fixed_square`); `square_fit` (fit
   None or residual too high), 3D `stance_floor`, and `isolation` each append a
   `RejectReason` on reject.
4. after the loop, fold `rejects` to the entry with the max `stage` (tie -> first
   seen) via a small helper `_furthest(rejects)`.
5. if a detection is found, `reject_reason = None`.
6. if the fold is over an empty list (no candidates *and* no rejects),
   `reject_reason = RejectReason(Stage.NO_CLUSTERS, "no_clusters", None, None,
   None, 0.0)`.

## Testing

- `reject.py`: `upper`/`lower`/`band` margin formulas on hand-picked values;
  `Stage` ordering (generation < scorer < detector within a path).
- `scorer.py`: craft `coords_2d` tripping each of the scorer gates with
  `rejects=[]` passed; assert the collected `RejectReason.stage`/`gate`/`param`/
  `margin`. Accept path still returns `ScoreResult`. Byte-identical guard: with
  the kwarg omitted, the existing scorer tests still pass unchanged.
- `plausible_board_patch`: each of the 3 gates appends the right `RejectReason`
  when `rejects=[]`; accept path returns a `Candidate`; kwarg omitted -> output
  identical.
- generators: when `rejects=[]` is passed, patch rejects are collected; when the
  kwarg is omitted, the output list is byte-identical (guard test).
- `detector.py`: synth scenes where all candidates die at one known gate ->
  assert `outcome.reject_reason` stage/param; a scene that detects ->
  `reject_reason is None`; a zero-cluster scene -> `NO_CLUSTERS`.

## Files touched

- new `src/boarddet/reject.py`
- `src/boarddet/scorer.py` (`rejects` kwarg + append at each return-None site)
- `src/boarddet/candidates/__init__.py` (`rejects` kwarg + 3 sites)
- `src/boarddet/candidates/{ransac_iterative,cluster_after_ground,region_growing,background_diff}.py`
  (`rejects` kwarg forwarded)
- `src/boarddet/detector.py` (`DetectOutcome.reject_reason`, `_furthest`, wiring)
- tests: `test_reject.py` (new), `test_scorer.py`, `test_candidates_a.py`,
  `test_detector.py`
