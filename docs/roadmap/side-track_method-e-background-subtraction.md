# Side Track: Method E — Motion / Background Subtraction

Implements strategy **E** of [`side-track_auto-bounding-box.md`](side-track_auto-bounding-box.md)
as generator `"e"` in the `boarddet` experiment
(`experiments/board-detection-2d/`), and validates it on the real sample
recordings by **leave-one-out cross-dataset background construction**.

Status: 🟢 validated on real data. Recommended operating point reaches
**88.4% recall at 100% precision**, against
[phase-7](phase-7-projection-board-detection.md)'s best prior single-frame
results of 49.3%/93.0% (stage 6) and 44.1%/100% (stage 8).

---

## Motivation

Phase 7 closed after eight stages with recall stuck at ~44–50%, and diagnosed
the cause as fundamental rather than a tuning gap. From its Decision section:

> closing it needs a session-level multi-pose cue (the board moves between
> calibration poses; static clutter does not), which is a capture-protocol
> change for a future phase, **not implementable on today's
> single-static-capture sample datasets**.

The premise of this side track is that the last clause is wrong. The five
sample datasets **already are** five board poses in one shared static room:

- Stage 3's pose-sanity check established `bbox.json5`'s reference box as
  "one physical rig setup shared by all five sample datasets, not
  ds3-specific."
- Static clutter attractors recur at identical scene-fixed coordinates across
  datasets — `~(-1.83,-2.89)` in ds1/3/5, `~(4.7,2.6)`, `~(-3.3,3.4)`.
- But each dataset places the board somewhere different (stage 4 pose sanity):

  | Dataset | 1 | 2 | 3 | 4 | 5 |
  |---|---|---|---|---|---|
  | board centre (x, y, z) | (2.256, −0.059, 0.074) | (2.147, **+0.420**, 0.076) | (2.101, −0.314, 0.074) | (2.077, −0.605, 0.066) | (2.090, −0.829, 0.039) |

Hold out dataset K, accumulate a background from the other four, and K's board
is the one thing that background has never seen.

**This is not a literal single-session multi-pose buffer** (what
`advanced_extrinsic_solver` maintains). These are five independent captures of
one room — a related but distinct instance of the same persistent-vs-transient
occupancy cue. The distinction matters when reading the numbers below: nothing
here demonstrates that a within-session buffer performs the same way.

## Method

`BackgroundModel` (`src/boarddet/background.py`) accumulates voxel occupancy
**per source**, then collapses sources into a consensus background: a voxel
counts as background only once `>= min_sources` distinct sources have seen it.
Query-time dilation (a 3×3×3 neighbour stencil at `dilation_radius=1`) absorbs
voxel-boundary aliasing, so range noise nudging a static point across a cell
edge does not make it read as newly-occupied.

Why per-source consensus rather than a plain union: each board is seen by
exactly one dataset while the room is seen by all of them, so a threshold above
1 drops every contributor's own board out of the background while keeping the
room. Without it the contributors' boards land in the background and suppress
the held-out board wherever the positions overlap — and they do overlap, since
the five board centres span only ~1.25 m in y while a 1 m diamond is 1.414 m
across its diagonal.

`generate_background_diff` (`src/boarddet/candidates/background_diff.py`) diffs
the frame, then reuses generator B's range-scaled anisotropic DBSCAN and the
shared `plausible_board_patch` gate. It runs **no** `_remove_big_planes` stage:
ground and walls are background by construction and are already gone, which
also drops B's most expensive stage. The anisotropic scaling is still required
— a revealed board is sampled through the same VLP-32C rings, so ring-gap
fragmentation survives the diff untouched.

All runs below use the stage-6 operating point (`--stance-gate
--flatness-rms-max 0.045`) so the comparison is like-for-like:

```bash
uv run python -m boarddet.benchmark_e_loo --datasets 1 2 3 4 5 --side 1.0 \
  --stance-gate --flatness-rms-max 0.045 --min-sources 3 \
  --isolation --isolation-max-density 0.3 --out results/methodE-ms3-iso
```

## Sensor-pose sanity check

LOO assumes all five captures share one sensor pose, so that a world coordinate
means the same physical spot in every dataset. The harness tests this itself:
`n_known_clutter_survived` counts accepted detections landing on the documented
static attractors, which a background built from four other datasets must
suppress.

**It passed (0 survivors) at every threshold except `min_sources=4`.** Every
number below is conditioned on that; the `min_sources=4` row is reported but
must not be trusted (see the ablation).

## Consensus-threshold ablation

All 535 frames, five folds, stage-6 operating point, isolation off:

| `min_sources` | recall | precision | clutter | known-clutter survived | median ms/frame |
|---|---|---|---|---|---|
| 1 (plain union) | 0/535 = **0.0%** | — | 1 | 0 | 85–103 |
| 2 | 204/535 = 38.1% | 99.51% | 1 | 0 | 80–110 |
| **3** | 473/535 = **88.4%** | 99.79% | 1 | 0 | 90–117 |
| 4 | 493/535 = 92.1% | 99.40% | 3 | **2** ⚠ | 128–136 |

Per fold (held-out dataset), same runs:

| `min_sources` | ds1 | ds2 | ds3 | ds4 | ds5 |
|---|---|---|---|---|---|
| 1 | 0% | 0% | 0% | 0% | 0% |
| 2 | 99.0% | 99.0% | **0%** | **0%** | **0%** |
| 3 | 99.0% | 100% | 99.1% | 99.1% | 42.7% |
| 4 | 99.0% | 100% | 99.1% | 99.1% | 62.1% |

Three findings, none of them tuning noise:

1. **The plain union is not merely worse, it is total failure (0/535).** With
   `min_sources=1` all four contributors' boards enter the background, and
   between them they cover every held-out board position. This is the single
   clearest result here: consensus is not an optimisation of Method E on this
   data, it is what makes Method E work at all.

2. **`min_sources=2` fails exactly where the geometry predicts.** It recovers
   only ds1 and ds2 — the two *outermost* board positions (y = −0.059 and
   +0.420). ds3, ds4 and ds5 are each sandwiched between two neighbours at
   Δy = 0.224–0.291 m, far inside the board's 1.414 m diagonal, so two
   contributors cover the held-out board, it reaches the 2-vote threshold, and
   it is absorbed into the background. Raising the threshold to 3 requires
   three overlapping boards, which no position has, and all three folds
   recover.

3. **`min_sources=4`'s higher recall is not usable.** It buys +3.7 pts of
   recall but its background shrinks to ~6.9k voxels (vs ~23k at threshold 3),
   too thin to suppress the documented static panels — `n_known_clutter_
   survived` goes to 2, meaning the sanity assumption the whole method rests on
   no longer holds. It also costs ~40 ms/frame more, pushing well past budget.
   Rejected.

## Recommended operating point

`--min-sources 3 --stance-gate --flatness-rms-max 0.045 --isolation
--isolation-max-density 0.3`:

| fold | true board | frames | recall | clutter | median ms |
|---|---|---|---|---|---|
| ds1 | 102 | 103 | 99.0% | 0 | 102 |
| ds2 | 103 | 103 | 100.0% | 0 | 108 |
| ds3 | 112 | 113 | 99.1% | 0 | 100 |
| ds4 | 112 | 113 | 99.1% | 0 | 101 |
| ds5 | 44 | 103 | 42.7% | 0 | 119 |
| **total** | **473** | **535** | **88.4%** | **0** | — |

Stage 8's isolation gate removes the single residual false positive at **zero**
recall cost (473 true both with and without it), taking precision 99.79% →
**100%** for ~10 ms/frame. It is worth keeping here, unlike in stage 8 where it
cost 5 points of recall.

Comparison against the phase's prior best, same stance/flatness settings:

| configuration | recall | precision |
|---|---|---|
| stage 6 (`--stance-gate --flatness-rms-max 0.045`) | 49.3% | 93.0% |
| stage 8 (+ isolation 0.3) | 44.1% | 100% |
| **Method E, ms=3 + isolation** | **88.4%** | **100%** |

This is a **strict Pareto improvement over both** — nearly double stage 6's
recall at stage 8's perfect precision. It is the first configuration in this
phase to break the ~44–50% recall ceiling, and it does so via precisely the
mechanism the phase-7 Decision named as the only path to it.

## Caveats

1. **Not a within-session multi-pose buffer.** See Motivation. Five independent
   captures of one room is a related but distinct setting; nothing here shows a
   single-session buffer behaves the same.

2. **ds5 is a genuine partial failure at 42.7%, not noise.** ds5's board sits at
   the extreme edge of the spread (y = −0.829) with ds4 only 0.224 m away, so
   even at threshold 3 much of its footprint is covered. Its fold is also the
   slowest (117–119 ms) and carried the only false positive in the ms=3 run.
   Board-position overlap is mitigated by consensus, not eliminated, and ds5 is
   where that shows.

3. **The result depends on board positions differing between captures.** That is
   true of this data by luck, not by protocol. A real deployment wanting these
   numbers must actually move the board between captures — which is the
   capture-protocol change phase 7 already identified, now with a measured
   payoff attached.

4. **Timing sits at the edge of the 100 ms budget.** Medians are 100–119 ms at
   the recommended point, versus ~60 ms for stage 6. Method E drops B's
   plane-strip stage but pays it back in the diff and in clustering a larger
   surviving foreground. Not disqualifying for offline calibration, but it is
   no longer comfortably inside budget.

5. **The deployment workflow (empty room, then board walked in) remains
   unvalidated.** No sample capture has a board-absent period. `BackgroundModel`
   supports it directly — `observe()` warmup frames, then `finalize()` — but
   nothing here exercises that path, so intra-dataset warmup mode was
   deliberately not built.

6. **A person or mount near a contributor's board** adds background occupancy
   there. No mitigation was needed: the flatness gate crops non-planar
   candidates, verified by a dedicated test.

## Verdict

Adopt for the static-mount scenario. Method E is the first mechanism in phase 7
to move recall past the ceiling stages 5–8 all converged on, and it does it
without trading precision away — 88.4%/100% against stage 6's 49.3%/93.0% and
stage 8's 44.1%/100%.

The load-bearing insight is the **consensus threshold**, not background
subtraction as such: a plain-union background scores 0/535 on this data, and
threshold 2 recovers only the two outermost board positions. Any future use of
this generator must set `min_sources` from the geometry of the capture set
(enough votes that no held-out object is covered by that many contributors),
and must check the background is still dense enough to suppress known static
clutter — the harness's `n_known_clutter_survived` is there for exactly that,
and it is what disqualifies the otherwise-tempting `min_sources=4`.
