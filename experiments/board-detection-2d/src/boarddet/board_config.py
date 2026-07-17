from __future__ import annotations

from dataclasses import dataclass


@dataclass
class BoardConfig:
    side_m: float = 1.0      # diamond (square) side length
    side_tol: float = 0.20   # fractional tolerance on side length
    min_score: float = 0.5   # detector acceptance threshold
    cell_m: float = 0.02     # raster cell size
    # Diamond-stance score term weight. 0 (default) disables the term and
    # score is the raw scorer output; >0 blends in `_stance` (detector.py)
    # to prefer candidates standing on a corner (gravity-aligned diagonal)
    # over axis-aligned flat panels that would otherwise false-positive.
    stance_weight: float = 0.0
    # Worst-case adjacent-channel vertical spacing of the LiDAR, in degrees.
    # Generator B (cluster_after_ground) uses this to widen its DBSCAN
    # vertical tolerance with range (anisotropic/DAC-style clustering) so
    # ring gaps don't fragment a single physical surface into several
    # clusters. VLP-32C worst adjacent-channel spacing is ~3 deg. 0 disables
    # the anisotropic scaling (plain isotropic DBSCAN).
    vertical_gap_deg: float = 3.0
