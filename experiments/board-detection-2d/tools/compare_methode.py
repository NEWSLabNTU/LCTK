"""Pool the no-Method-E (benchmark_noe) and Method E (benchmark_e_loo) runs
into one recall/precision/timing table across all three rigs. Reproduces the
numbers behind
docs/superpowers/plans/2026-07-26-method-e-vs-baseline-comparison.md."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

RESULTS = Path(__file__).resolve().parents[1] / "results"

# (scenario, configuration label, results-subdir/json-filename)
RUNS = [
    ("pcap 1-5 (VLP near)", "No E - B, stage 6",
     "compare-noE-pcap-stage6/noe_summary.json"),
    ("pcap 1-5 (VLP near)", "No E - B, stage 8 (+iso)",
     "compare-noE-pcap-stage8/noe_summary.json"),
    ("pcap 1-5 (VLP near)", "Method E - ms3 +iso",
     "compare-E-pcap/loo_summary.json"),
    ("VLP-32C bag (~9 m)", "No E - B",
     "compare-noE-vlp/noe_summary.json"),
    ("VLP-32C bag (~9 m)", "Method E - ms1",
     "compare-E-vlp/loo_summary.json"),
    ("Falcon bag (~7.4 m)", "No E - B",
     "compare-noE-falcon/noe_summary.json"),
    ("Falcon bag (~7.4 m)", "Method E - ms1",
     "compare-E-falcon/loo_summary.json"),
]


def pool(path: Path) -> dict:
    d = json.loads(path.read_text())
    entries = d.get("folds") or d.get("captures")
    n_true = sum(e["n_true_board"] for e in entries.values())
    n_det = sum(e["n_detections"] for e in entries.values())
    n_frames = sum(e["n_frames"] for e in entries.values())
    med = float(np.median([e["median_total_ms"] for e in entries.values()]))
    return {
        "recall": n_true / n_frames if n_frames else 0.0,
        "precision": (n_true / n_det) if n_det else None,
        "median_ms": med,
    }


def main() -> None:
    lines = [
        "# Method E vs. no-Method-E - all datasets",
        "",
        "Recall = true-board detections / frames; precision = true-board / all "
        "accepted; both classified against each rig's reference box. Within a "
        "scenario, Method E and its No-E row share every gate and tuning flag; "
        "the only difference is background subtraction.",
        "",
        "| Scenario | Configuration | Recall | Precision | Median ms/frame |",
        "|---|---|---|---|---|",
    ]
    for scenario, label, rel in RUNS:
        p = RESULTS / rel
        if not p.exists():
            lines.append(f"| {scenario} | {label} | MISSING | MISSING | "
                         f"MISSING ({rel}) |")
            continue
        m = pool(p)
        prec = "n/a" if m["precision"] is None else f"{m['precision']:.1%}"
        lines.append(f"| {scenario} | {label} | {m['recall']:.1%} | {prec} | "
                     f"{m['median_ms']:.0f} |")
    text = "\n".join(lines) + "\n"
    out = RESULTS / "comparison"
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
