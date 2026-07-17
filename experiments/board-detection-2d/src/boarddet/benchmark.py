"""Benchmark CLI: all generators x all datasets -> tables + overlays."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .board_config import BoardConfig
from .detector import GENERATORS, DetectOutcome, detect
from .ingest import load_frames
from .viz import save_overlay


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
        max_frames: int | None, out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    all_results: dict = {}
    for ds in datasets:
        frames = load_frames(ds, max_frames=max_frames)
        all_results[ds] = {}
        for g in generators:
            outcomes = [detect(f.xyz, board, generator=g) for f in frames]
            all_results[ds][g] = summarize(outcomes)
            det_idx = [i for i, o in enumerate(outcomes)
                       if o.detection is not None]
            picks = ({det_idx[0], det_idx[len(det_idx) // 2], det_idx[-1]}
                     if det_idx else {0, len(outcomes) // 2,
                                      len(outcomes) - 1})
            for i in sorted(picks):
                save_overlay(frames[i].xyz, outcomes[i],
                             out_dir / f"ds{ds}_{g}_frame{i:04d}.png")
            print(f"dataset {ds} gen {g}: "
                  f"rate={all_results[ds][g]['detection_rate']:.0%} "
                  f"median={all_results[ds][g]['median_total_ms']:.0f}ms")
    (out_dir / "summary.json").write_text(json.dumps(all_results, indent=2))
    (out_dir / "summary.md").write_text(_md_tables(all_results))
    return all_results


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", type=int, nargs="+",
                    default=[1, 2, 3, 4, 5])
    ap.add_argument("--generators", nargs="+", default=list(GENERATORS),
                    choices=list(GENERATORS))
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--side", type=float, default=1.0)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    run(args.datasets, args.generators, BoardConfig(side_m=args.side),
        args.max_frames, args.out)


if __name__ == "__main__":
    main()
