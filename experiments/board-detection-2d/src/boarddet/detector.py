"""Glue: downsample -> candidate generator -> shared scorer -> best pose."""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .board_config import BoardConfig
from .candidates.cluster_after_ground import generate_cluster_after_ground
from .candidates.ransac_iterative import generate_ransac_iterative
from .candidates.region_growing import generate_region_growing
from .geometry import downsample, project_to_plane
from .pose import BoardDetection, board_pose
from .scorer import score_candidate

GENERATORS = {
    "a": generate_ransac_iterative,
    "b": generate_cluster_after_ground,
    "c": generate_region_growing,
}


@dataclass
class DetectOutcome:
    detection: BoardDetection | None
    timings_ms: dict[str, float]
    n_candidates: int


def detect(points: np.ndarray, board: BoardConfig, generator: str,
           voxel: float = 0.03) -> DetectOutcome:
    gen = GENERATORS[generator]
    t0 = time.perf_counter()
    dn = downsample(points, voxel)
    t1 = time.perf_counter()
    cands = gen(dn, board)
    t2 = time.perf_counter()
    best: BoardDetection | None = None
    for cand in cands:
        res = score_candidate(project_to_plane(cand.points, cand.plane),
                              board)
        if res is None or res.score < board.min_score:
            continue
        det = board_pose(cand.plane, res)
        if best is None or det.score > best.score:
            best = det
    t3 = time.perf_counter()
    return DetectOutcome(
        detection=best,
        timings_ms={
            "downsample": (t1 - t0) * 1e3,
            "candidates": (t2 - t1) * 1e3,
            "scoring": (t3 - t2) * 1e3,
            "total": (t3 - t0) * 1e3,
        },
        n_candidates=len(cands),
    )
