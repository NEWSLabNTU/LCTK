"""Benchmark CLI: all generators x all datasets -> tables + overlays."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .board_config import BoardConfig
from .detector import GENERATORS, DetectOutcome, detect
from .ingest import Frame, load_frames
from .viz import save_overlay


def accumulate_frames(frames: list[Frame], n: int) -> list[np.ndarray]:
    """Chunk consecutive frames into non-overlapping windows of n, xyz-concat.

    Matches the "hold the board static for a few seconds" workflow: a
    calibration pose spans several frames, so accumulating them thickens a
    single scan into a denser one instead of averaging across poses. The
    trailing partial window is kept only if it has at least n/2 frames;
    n=1 reproduces stage-1 (one window per frame) exactly.
    """
    if n < 1:
        raise ValueError(f"n must be >= 1, got {n}")
    windows: list[np.ndarray] = []
    for i in range(0, len(frames), n):
        chunk = frames[i:i + n]
        if len(chunk) >= n / 2:
            windows.append(np.concatenate([f.xyz for f in chunk], axis=0))
    return windows


def summarize(outcomes: list[DetectOutcome]) -> dict:
    detected = [o for o in outcomes if o.detection is not None]
    rate = len(detected) / len(outcomes) if outcomes else 0.0
    totals = [o.timings_ms["total"] for o in outcomes]
    stage = {
        k: float(np.median([o.timings_ms[k] for o in outcomes]))
        for k in ("downsample", "candidates", "scoring", "total")
    }
    s: dict = {
        "n_frames": len(outcomes),
        "detection_rate": rate,
        "median_total_ms": stage["total"],
        "p95_total_ms": float(np.percentile(totals, 95)) if totals else 0.0,
        "median_stage_ms": stage,
        "median_candidates": float(np.median(
            [o.n_candidates for o in outcomes])) if outcomes else 0.0,
    }
    if len(detected) >= 2:
        centers = np.array([o.detection.center for o in detected])
        normals = np.array([o.detection.rotation[:, 2] for o in detected])
        # sign-align normals to the first
        normals *= np.sign(normals @ normals[0])[:, None]
        mean_n = normals.mean(axis=0)
        mean_n /= np.linalg.norm(mean_n)
        ang = np.degrees(np.arccos(np.clip(normals @ mean_n, -1, 1)))
        s["jitter_center_mm"] = float(centers.std(axis=0).mean() * 1e3)
        s["jitter_normal_deg"] = float(ang.std())
    return s


def _md_tables(all_results: dict) -> str:
    gens = sorted({g for d in all_results.values() for g in d})
    lines = ["# Benchmark results", "", "## Detection rate", "",
             "| Dataset | " + " | ".join(gens) + " |",
             "|---------|" + "---|" * len(gens)]
    for ds in sorted(all_results):
        row = [f"{all_results[ds][g]['detection_rate']:.0%}"
               if g in all_results[ds] else "—" for g in gens]
        lines.append(f"| {ds} | " + " | ".join(row) + " |")
    lines += ["", "## Median total ms (p95)", "",
              "| Dataset | " + " | ".join(gens) + " |",
              "|---------|" + "---|" * len(gens)]
    for ds in sorted(all_results):
        row = []
        for g in gens:
            r = all_results[ds].get(g)
            row.append(f"{r['median_total_ms']:.0f} ({r['p95_total_ms']:.0f})"
                       if r else "—")
        lines.append(f"| {ds} | " + " | ".join(row) + " |")
    lines += ["", "## Jitter: center mm / normal deg", "",
              "| Dataset | " + " | ".join(gens) + " |",
              "|---------|" + "---|" * len(gens)]
    for ds in sorted(all_results):
        row = []
        for g in gens:
            r = all_results[ds].get(g, {})
            if "jitter_center_mm" in r:
                row.append(f"{r['jitter_center_mm']:.1f} / "
                           f"{r['jitter_normal_deg']:.2f}")
            else:
                row.append("—")
        lines.append(f"| {ds} | " + " | ".join(row) + " |")
    return "\n".join(lines) + "\n"


def run(datasets: list[int], generators: list[str], board: BoardConfig,
        max_frames: int | None, out_dir: Path, accumulate: int = 1) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    all_results: dict = {}
    label = "win" if accumulate > 1 else "frame"
    for ds in datasets:
        frames = load_frames(ds, max_frames=max_frames)
        windows = accumulate_frames(frames, accumulate)
        all_results[ds] = {}
        for g in generators:
            outcomes = [detect(w, board, generator=g) for w in windows]
            all_results[ds][g] = summarize(outcomes)
            det_idx = [i for i, o in enumerate(outcomes)
                       if o.detection is not None]
            picks = ({det_idx[0], det_idx[len(det_idx) // 2], det_idx[-1]}
                     if det_idx else {0, len(outcomes) // 2,
                                      len(outcomes) - 1})
            for i in sorted(picks):
                save_overlay(windows[i], outcomes[i],
                             out_dir / f"ds{ds}_{g}_{label}{i:04d}.png")
            print(f"dataset {ds} gen {g}: "
                  f"rate={all_results[ds][g]['detection_rate']:.0%} "
                  f"median={all_results[ds][g]['median_total_ms']:.0f}ms")
    summary = {
        "accumulate": accumulate,
        "stance_weight": board.stance_weight,
        "vertical_gap_deg": board.vertical_gap_deg,
        "strict_squareness": board.strict_squareness,
        "stance_floor": board.stance_floor,
        "edge_support_min": board.edge_support_min,
        "side_tol": board.side_tol,
        "datasets": all_results,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    (out_dir / "summary.md").write_text(_md_tables(all_results))
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", type=int, nargs="+",
                    default=[1, 2, 3, 4, 5])
    ap.add_argument("--generators", nargs="+", default=list(GENERATORS),
                    choices=list(GENERATORS))
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--side", type=float, default=1.0)
    ap.add_argument("--accumulate", type=int, default=1,
                    help="frames per window (1 = stage-1 behavior)")
    ap.add_argument("--stance-weight", type=float, default=0.0,
                    help="diamond-stance score blend weight (0 = off)")
    ap.add_argument("--vertical-gap-deg", type=float, default=3.0,
                    help="generator B anisotropic vertical clustering "
                         "tolerance, LiDAR ring-gap degrees (0 = off)")
    ap.add_argument("--strict-diamond", action="store_true",
                    help="enable Task 18's hole-free strict-diamond "
                         "discriminator gates together: strict_squareness, "
                         "stance_floor=0.9, edge_support_min=0.6, "
                         "size_tol=0.08 (all off by default)")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    board_kwargs: dict = dict(
        side_m=args.side, stance_weight=args.stance_weight,
        vertical_gap_deg=args.vertical_gap_deg,
    )
    if args.strict_diamond:
        board_kwargs.update(strict_squareness=True, stance_floor=0.9,
                            edge_support_min=0.6, side_tol=0.08)
    run(args.datasets, args.generators, BoardConfig(**board_kwargs),
        args.max_frames, args.out, accumulate=args.accumulate)


if __name__ == "__main__":
    main()
