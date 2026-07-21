"""Leave-one-out cross-dataset validation for generator "e".

The five sample datasets share one static physical room and sensor mount --
`bbox.json5`'s single reference box is "one physical rig setup shared by all
five sample datasets" (phase-7 doc, Stage 3 pose sanity), and the same
clutter attractors recur at the same coordinates across datasets -- but each
places the calibration board somewhere different (phase-7 doc :782-793).

So holding out dataset K and accumulating a consensus background from the
other four turns their shared room into "background" while K's own board
survives the diff. This is the phase-7 Decision section's diagnosed
session-level multi-pose cue, realized on data that already exists.

It is NOT a literal single-session multi-pose buffer (what
`advanced_extrinsic_solver` does): these are five independent captures of one
room, a related but distinct instance of the same persistent-vs-transient
occupancy cue. Report it as such.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .background import BackgroundModel
from .board_config import BoardConfig
from .detector import detect
from .ingest import Frame, load_frames

# ros/lctk_launch/config/board/bbox.json5: translation [2.6, 0, 0.35],
# size [3.1, 3.94, 2.2]. The same true-board reference stages 4-8 used.
_BBOX_CENTER = np.array([2.6, 0.0, 0.35])
_BBOX_HALF = np.array([3.1, 3.94, 2.2]) / 2.0

# Static clutter attractors documented across stages 1-8. A background built
# from four other datasets MUST suppress these; if it does not, the shared
# room / shared sensor pose assumption LOO rests on is broken, and every
# other number in the fold is suspect.
_KNOWN_CLUTTER_XY = np.array([[-1.83, -2.89], [4.7, 2.6], [-3.3, 3.4]])
_KNOWN_CLUTTER_TOL = 0.5  # m, in the xy plane


def in_bbox(center: np.ndarray) -> bool:
    return bool(np.all(np.abs(np.asarray(center) - _BBOX_CENTER) <= _BBOX_HALF))


def near_known_clutter(center: np.ndarray) -> bool:
    d = np.linalg.norm(_KNOWN_CLUTTER_XY - np.asarray(center)[:2], axis=1)
    return bool((d <= _KNOWN_CLUTTER_TOL).any())


def build_background(all_frames: dict[int, list[Frame]], held_out: int,
                     voxel: float, dilation_radius: int,
                     min_sources: int) -> BackgroundModel:
    """One source per contributing dataset -- that per-source split is what
    makes the >=min_sources consensus drop each contributor's own board."""
    model = BackgroundModel(voxel=voxel, dilation_radius=dilation_radius,
                            min_sources=min_sources)
    for ds, frames in all_frames.items():
        if ds == held_out:
            continue
        model.observe(np.concatenate([f.xyz for f in frames], axis=0),
                      source=ds)
    model.finalize()
    return model


def run_loo(datasets: list[int], board: BoardConfig, out_dir: Path, *,
            max_frames: int | None = None, background_voxel: float = 0.06,
            dilation_radius: int = 1, min_sources: int = 2) -> dict:
    # Each fold contributes every dataset except the held-out one, so a
    # consensus threshold above that count can never be met: the background
    # would finalize EMPTY, every point would read as foreground, and the
    # fold would report a meaninglessly high recall from a detector that is
    # no longer doing background subtraction at all. Fail loudly instead --
    # this is the same class of silent-acceptance bug as C-04.
    n_contributors = len(datasets) - 1
    if n_contributors < min_sources:
        raise ValueError(
            f"min_sources={min_sources} is unreachable with {len(datasets)} "
            f"datasets: each fold has only {n_contributors} contributing "
            f"source(s), so the background would be empty and every point "
            f"would count as foreground. Use at least {min_sources + 1} "
            f"datasets, or lower --min-sources.")
    out_dir.mkdir(parents=True, exist_ok=True)
    all_frames = {ds: load_frames(ds, max_frames=max_frames)
                  for ds in datasets}
    folds: dict[int, dict] = {}
    for held_out in datasets:
        model = build_background(all_frames, held_out, background_voxel,
                                 dilation_radius, min_sources)
        outcomes = [detect(f.xyz, board, generator="e", background=model)
                    for f in all_frames[held_out]]
        dets = [o.detection for o in outcomes if o.detection is not None]
        n_true = sum(1 for d in dets if in_bbox(d.center))
        folds[held_out] = {
            "n_frames": len(outcomes),
            "n_detections": len(dets),
            "n_true_board": n_true,
            "n_clutter": len(dets) - n_true,
            "recall": n_true / len(outcomes) if outcomes else 0.0,
            "precision": (n_true / len(dets)) if dets else None,
            "n_known_clutter_survived": sum(
                1 for d in dets if near_known_clutter(d.center)),
            "background_voxels": model.n_voxels,
            "n_contributing_sources": model.n_sources,
            "median_total_ms": float(np.median(
                [o.timings_ms["total"] for o in outcomes]
            )) if outcomes else 0.0,
        }
        f = folds[held_out]
        print(f"held-out ds{held_out}: recall={f['recall']:.1%} "
              f"true={f['n_true_board']} clutter={f['n_clutter']} "
              f"known-clutter-survived={f['n_known_clutter_survived']} "
              f"bg_voxels={f['background_voxels']} "
              f"median={f['median_total_ms']:.0f}ms")
    summary = {
        "min_sources": min_sources,
        "background_voxel": background_voxel,
        "dilation_radius": dilation_radius,
        "stance_floor": board.stance_floor,
        "flatness_rms_max": board.flatness_rms_max,
        "isolation": board.isolation,
        "isolation_max_density": board.isolation_max_density,
        "folds": folds,
    }
    (out_dir / "loo_summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--side", type=float, default=1.0)
    ap.add_argument("--background-voxel", type=float, default=0.06,
                    help="background occupancy cell size, metres")
    ap.add_argument("--dilation-radius", type=int, default=1,
                    help="query-time neighbour radius absorbing voxel-"
                         "boundary aliasing (0 = off, reproduces the bug)")
    ap.add_argument("--min-sources", type=int, default=2,
                    help="a voxel is background only if this many "
                         "contributing datasets saw it; 1 = plain union")
    ap.add_argument("--stance-gate", action="store_true",
                    help="stage-6 operating point: stance_floor=0.9")
    ap.add_argument("--flatness-rms-max", type=float, default=0.035,
                    help="stage 6 adopted 0.045")
    ap.add_argument("--isolation", action="store_true",
                    help="stage-8 isolation gate")
    ap.add_argument("--isolation-max-density", type=float, default=0.3)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    board = BoardConfig(
        side_m=args.side,
        stance_floor=0.9 if args.stance_gate else 0.0,
        flatness_rms_max=args.flatness_rms_max,
        isolation=args.isolation,
        isolation_max_density=args.isolation_max_density,
    )
    run_loo(args.datasets, board, args.out, max_frames=args.max_frames,
            background_voxel=args.background_voxel,
            dilation_radius=args.dilation_radius,
            min_sources=args.min_sources)


if __name__ == "__main__":
    main()
