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
`lidar_to_camera_solver` does): these are five independent captures of one
room, a related but distinct instance of the same persistent-vs-transient
occupancy cue. Report it as such.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .background import BackgroundModel
from .bbox_ref import BoxRef, load_bbox
from .board_config import BoardConfig
from .detector import detect
from .ingest import Frame, load_bag_frames, load_frames
from .viz import render_methode

# The pcap rig's reference, used by stages 3-8 and Method E. Other rigs
# (e.g. the recorded TWO_LIDAR bags) supply their own via --bbox; theirs live
# in sessions/twolidar-vlp32-falcon/.
DEFAULT_BBOX_PATH = (Path(__file__).resolve().parents[4]
                     / "sessions" / "sample3-hollow-velodyne"
                     / "bbox.json5")

# Static clutter attractors documented across stages 1-8. A background built
# from four other datasets MUST suppress these; if it does not, the shared
# room / shared sensor pose assumption LOO rests on is broken, and every
# other number in the fold is suspect.
_KNOWN_CLUTTER_XY = np.array([[-1.83, -2.89], [4.7, 2.6], [-3.3, 3.4]])
_KNOWN_CLUTTER_TOL = 0.5  # m, in the xy plane


def near_known_clutter(center: np.ndarray) -> bool:
    d = np.linalg.norm(_KNOWN_CLUTTER_XY - np.asarray(center)[:2], axis=1)
    return bool((d <= _KNOWN_CLUTTER_TOL).any())


def build_background(all_frames: dict[str, list[Frame]], held_out: str,
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


def load_sources(kind: str, names: list[str], sensor: str,
                 max_frames: int | None) -> dict[str, list[Frame]]:
    """Load frames for each named capture. `kind` selects the reader:
    "pcap" for the sample datasets 1-5, "bag" for exported TWO_LIDAR bags
    (see tools/export_bag_npz.py). Labels are the names as given, so folds
    are readable in the output either way."""
    if kind == "pcap":
        return {n: load_frames(int(n), max_frames=max_frames) for n in names}
    if kind == "bag":
        return {n: load_bag_frames(n, sensor, max_frames=max_frames)
                for n in names}
    raise ValueError(f"unknown source kind {kind!r}; expected 'pcap' or 'bag'")


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
    # Overlays are a debug aid -- a render failure on one frame must never
    # abort the benchmark before its summary is written. Isolate each call.
    for i in _pick_overlay_indices(outcomes, n):
        try:
            render_methode(frames[i].xyz, board, model, outcomes[i], box,
                           out_dir / f"overlay_{held_out}_frame{i:04d}.png")
        except Exception as e:  # noqa: BLE001 -- debug output, never fatal
            print(f"  overlay {held_out} frame {i} failed to render: {e}")


def run_loo(sources: dict[str, list[Frame]], board: BoardConfig,
            out_dir: Path, *, box: BoxRef, background_voxel: float = 0.06,
            dilation_radius: int = 1, min_sources: int = 2,
            save_overlays: int = 0) -> dict:
    # Each fold contributes every source except the held-out one, so a
    # consensus threshold above that count can never be met: the background
    # would finalize EMPTY, every point would read as foreground, and the
    # fold would report a meaninglessly high recall from a detector that is
    # no longer doing background subtraction at all. Fail loudly instead --
    # this is the same class of silent-acceptance bug as C-04.
    n_contributors = len(sources) - 1
    if n_contributors < min_sources:
        raise ValueError(
            f"min_sources={min_sources} is unreachable with {len(sources)} "
            f"captures: each fold has only {n_contributors} contributing "
            f"source(s), so the background would be empty and every point "
            f"would count as foreground. Use at least {min_sources + 1} "
            f"captures, or lower --min-sources.")
    out_dir.mkdir(parents=True, exist_ok=True)
    folds: dict[str, dict] = {}
    for held_out in sources:
        model = build_background(sources, held_out, background_voxel,
                                 dilation_radius, min_sources)
        outcomes = [detect(f.xyz, board, generator="e", background=model)
                    for f in sources[held_out]]
        if save_overlays > 0:
            _save_fold_overlays(sources[held_out], outcomes, board, model,
                                box, out_dir, held_out, save_overlays)
        dets = [o.detection for o in outcomes if o.detection is not None]
        n_true = sum(1 for d in dets if box.contains(d.center))
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
        "source_labels": list(sources),
        "folds": folds,
    }
    (out_dir / "loo_summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["pcap", "bag"], default="pcap",
                    help="pcap = sample datasets 1-5; bag = exported "
                         "TWO_LIDAR bags (run tools/export_bag_npz.py first)")
    ap.add_argument("--names", nargs="+", default=None,
                    help="capture names; defaults to 1..5 for pcap and "
                         "TWO_LIDAR_1..4 for bag")
    ap.add_argument("--sensor", choices=["vlp32", "falcon"], default="vlp32",
                    help="bag sources only; falcon is solid-state, so "
                         "consider --vertical-gap-deg 0")
    ap.add_argument("--vertical-gap-deg", type=float, default=3.0,
                    help="anisotropic clustering tolerance; 3.0 suits the "
                         "VLP-32C's ring gaps, 0 disables it for a "
                         "solid-state sensor with no ring structure")
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
    ap.add_argument("--square-icp", action="store_true",
                    help="refine each candidate with the fixed-side square "
                         "fitter (pins side=side_m, spends DOF on pose). "
                         "Fixes the minAreaRect oversize that sinks a dense "
                         "board's score, and re-activates the stance gate -- "
                         "pair with a correct --up-axis on a z-forward rig")
    ap.add_argument("--up-axis", type=float, nargs=3, default=(0.0, 0.0, 1.0),
                    metavar=("X", "Y", "Z"),
                    help="world-up direction in the sensor frame for the "
                         "stance gate; (0 0 1) for a z-up rig (pcap, VLP "
                         "bag), (0 1 0) for the z-forward Falcon")
    ap.add_argument("--cluster-min-points", type=int, default=30,
                    help="DBSCAN core-point density for generator E's "
                         "foreground clustering; lower it for a far, sparsely "
                         "sampled board (the ~9 m VLP bag wants 20)")
    ap.add_argument("--flatness-rms-max", type=float, default=0.035,
                    help="stage 6 adopted 0.045")
    ap.add_argument("--isolation", action="store_true",
                    help="stage-8 isolation gate")
    ap.add_argument("--isolation-max-density", type=float, default=0.3)
    ap.add_argument("--save-overlays", type=int, default=0,
                    help="render this many Method E 6-panel overlays per "
                         "fold into --out (0 = off)")
    ap.add_argument("--bbox", type=Path, default=DEFAULT_BBOX_PATH,
                    help="true-board reference box (bbox.json5 schema); "
                         "each recording rig has its own")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    names = args.names
    if names is None:
        names = (["1", "2", "3", "4", "5"] if args.source == "pcap"
                 else ["TWO_LIDAR_1", "TWO_LIDAR_2", "TWO_LIDAR_3",
                       "TWO_LIDAR_4"])
    sources = load_sources(args.source, names, args.sensor, args.max_frames)
    board = BoardConfig(
        side_m=args.side,
        stance_floor=0.9 if args.stance_gate else 0.0,
        flatness_rms_max=args.flatness_rms_max,
        isolation=args.isolation,
        isolation_max_density=args.isolation_max_density,
        vertical_gap_deg=args.vertical_gap_deg,
        cluster_min_points=args.cluster_min_points,
        square_icp=args.square_icp,
        up_axis=tuple(args.up_axis),
    )
    run_loo(sources, board, args.out, box=load_bbox(args.bbox),
            background_voxel=args.background_voxel,
            dilation_radius=args.dilation_radius,
            min_sources=args.min_sources,
            save_overlays=args.save_overlays)


if __name__ == "__main__":
    main()
