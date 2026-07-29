"""Named BoardConfig presets.

BoardConfig's dataclass defaults are frozen "stage-6 byte-identical" for the
pinned test suite. The production operating point lives here instead, as the
single source of truth the benchmarks, real-data regression tests, and the
ROS port all consume. Rationale for each flag is in
docs/roadmap/phase-7-projection-board-detection.md (recommended operating
point) and docs/roadmap/side-track_method-e-background-subtraction.md.
"""
from __future__ import annotations

from .board_config import BoardConfig


def production_config(side_m: float = 1.0,
                      up_axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
                      cluster_min_points: int = 30) -> BoardConfig:
    """The recommended operating point for real VLP-32C frames.

    - square_icp: fixed-side square fit (raw minAreaRect angle is near-random
      on sparse frames; median error 43 deg -> 7 deg).
    - stance_floor=0.9: reject non-diamond-stance panels.
    - isolation: reject embedded (wall-continuation) clutter.
    - flatness_rms_max=0.045: stage-6 recall recovery, still above the
      ~0.031 m VLP-32C noise floor.
    - up_axis / cluster_min_points: per-rig (z-forward Falcon -> (0,1,0);
      far/sparse board -> 20).
    """
    return BoardConfig(
        side_m=side_m,
        up_axis=up_axis,
        cluster_min_points=cluster_min_points,
        square_icp=True,
        stance_floor=0.9,
        isolation=True,
        flatness_rms_max=0.045,
    )
