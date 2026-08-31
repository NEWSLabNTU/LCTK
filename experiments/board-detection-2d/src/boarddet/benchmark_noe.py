"""No-Method-E baseline: generator B single-frame recall/precision/timing.

The counterpart to benchmark_e_loo.py for the "without background subtraction"
side of the Method E comparison. Same 2D scorer, same acceptance gates, same
per-rig reference box -- the ONLY difference from Method E is that no
background model is built or diffed (generator "b", not "e"; no min_sources).

Runs per-frame over each named capture (pcap datasets or exported TWO_LIDAR
bags), classifies each accepted detection's centre against the rig's bbox,
and writes noe_summary.json in the same per-capture schema benchmark_e_loo
uses per fold, so tools/compare_methode.py pools both uniformly.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .bbox_ref import BoxRef, load_bbox
from .board_config import BoardConfig
from .detector import detect
from .ingest import Frame, load_bag_frames, load_frames
from .viz import render_noe

DEFAULT_BBOX_PATH = (Path(__file__).resolve().parents[4]
                     / "sessions" / "sample3-hollow-velodyne"
                     / "bbox.json5")


def load_sources(kind: str, names: list[str], sensor: str,
                 max_frames: int | None) -> dict[str, list[Frame]]:
    if kind == "pcap":
        return {n: load_frames(int(n), max_frames=max_frames) for n in names}
    if kind == "bag":
        return {n: load_bag_frames(n, sensor, max_frames=max_frames)
                for n in names}
    raise ValueError(f"unknown source kind {kind!r}; expected 'pcap' or 'bag'")


def _overlay_indices(outcomes: list, n: int) -> list[int]:
    """Up to n indices: the first detection, then an even spread."""
    if not outcomes or n <= 0:
        return []
    det = [i for i, o in enumerate(outcomes) if o.detection is not None]
    step = max(1, len(outcomes) // n)
    picks = ([det[0]] if det else []) + list(range(0, len(outcomes), step))
    seen: list[int] = []
    for i in picks:
        if i not in seen:
            seen.append(i)
        if len(seen) >= n:
            break
    return seen


def run_noe(sources: dict[str, list[Frame]], board: BoardConfig,
            out_dir: Path, *, box: BoxRef, save_overlays: int = 0) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    captures: dict[str, dict] = {}
    for name, frames in sources.items():
        outcomes = [detect(f.xyz, board, generator="b") for f in frames]
        dets = [o.detection for o in outcomes if o.detection is not None]
        n_true = sum(1 for d in dets if box.contains(d.center))
        captures[name] = {
            "n_frames": len(outcomes),
            "n_detections": len(dets),
            "n_true_board": n_true,
            "n_clutter": len(dets) - n_true,
            "recall": n_true / len(outcomes) if outcomes else 0.0,
            "precision": (n_true / len(dets)) if dets else None,
            "median_total_ms": float(np.median(
                [o.timings_ms["total"] for o in outcomes]
            )) if outcomes else 0.0,
        }
        for i in _overlay_indices(outcomes, save_overlays):
            render_noe(frames[i].xyz, board, outcomes[i], box,
                       out_dir / f"overlay_{name}_frame{i:04d}.png")
        c = captures[name]
        prec = "n/a" if c["precision"] is None else f"{c['precision']:.1%}"
        print(f"{name}: recall={c['recall']:.1%} true={c['n_true_board']} "
              f"clutter={c['n_clutter']} prec={prec} "
              f"median={c['median_total_ms']:.0f}ms")
    summary = {
        "generator": "b",
        "stance_floor": board.stance_floor,
        "flatness_rms_max": board.flatness_rms_max,
        "vertical_gap_deg": board.vertical_gap_deg,
        "cluster_min_points": board.cluster_min_points,
        "up_axis": list(board.up_axis),
        "square_icp": board.square_icp,
        "isolation": board.isolation,
        "isolation_max_density": board.isolation_max_density,
        "source_labels": list(sources),
        "captures": captures,
    }
    (out_dir / "noe_summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["pcap", "bag"], default="pcap")
    ap.add_argument("--names", nargs="+", default=None,
                    help="captures; default 1..5 (pcap) or "
                         "TWO_LIDAR_1 TWO_LIDAR_3 (bag)")
    ap.add_argument("--sensor", choices=["vlp32", "falcon"], default="vlp32")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--side", type=float, default=1.0)
    ap.add_argument("--stance-gate", action="store_true")
    ap.add_argument("--flatness-rms-max", type=float, default=0.035)
    ap.add_argument("--vertical-gap-deg", type=float, default=3.0)
    ap.add_argument("--cluster-min-points", type=int, default=30)
    ap.add_argument("--square-icp", action="store_true")
    ap.add_argument("--up-axis", type=float, nargs=3, default=(0.0, 0.0, 1.0),
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--isolation", action="store_true")
    ap.add_argument("--isolation-max-density", type=float, default=0.3)
    ap.add_argument("--save-overlays", type=int, default=0)
    ap.add_argument("--bbox", type=Path, default=DEFAULT_BBOX_PATH)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    names = args.names
    if names is None:
        names = (["1", "2", "3", "4", "5"] if args.source == "pcap"
                 else ["TWO_LIDAR_1", "TWO_LIDAR_3"])
    sources = load_sources(args.source, names, args.sensor, args.max_frames)
    board = BoardConfig(
        side_m=args.side,
        stance_floor=0.9 if args.stance_gate else 0.0,
        flatness_rms_max=args.flatness_rms_max,
        vertical_gap_deg=args.vertical_gap_deg,
        cluster_min_points=args.cluster_min_points,
        square_icp=args.square_icp,
        up_axis=tuple(args.up_axis),
        isolation=args.isolation,
        isolation_max_density=args.isolation_max_density,
    )
    run_noe(sources, board, args.out, box=load_bbox(args.bbox),
            save_overlays=args.save_overlays)


if __name__ == "__main__":
    main()
