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
    # Plane-fit RMS gate in plausible_board_patch (candidates/__init__.py):
    # a 3D patch flatter than this (in meters) is accepted as board-shaped.
    # Default matches the module's own _FLATNESS_RMS_MAX -- current
    # stage-1..6 behavior is unchanged. Task 20 exposes this as a CLI/config
    # field so Task 21's recall sweep can raise it (stage6 diagnosis found
    # 0.045 recovers +19% recall on real VLP-32C near-misses at 0.035-0.048).
    flatness_rms_max: float = 0.035
    # --- Task 18: hole-free "strict diamond" discriminator gates ---
    # The recorded board is becoming a plain (hole-free) diamond, so
    # hole-pattern discrimination is off the table; these gates instead
    # exploit the diamond's *stance* (square standing on a corner) and the
    # fact that a real board's edges are physically present all the way
    # around, unlike a minAreaRect fit to a flat-panel/clutter fragment.
    # ALL default off/0 so stage-4 (run6-stripe) behavior, and every test
    # pinned against it, stays byte-identical; `--strict-diamond` (Task 19)
    # turns all four on together (see benchmark.py).
    strict_squareness: bool = False   # reject if any corner angle > 8 deg off 90
    stance_floor: float = 0.0         # 0 = off; e.g. 0.9 = reject if best
                                      # diagonal is > ~25 deg off vertical
    edge_support_min: float = 0.0     # 0 = off; e.g. 0.6 = each of the 4
                                      # sides must be >=60% backed by raw
                                      # points near the side line
    # --- Task 23: fixed-size square fitter (refine-after-quad) -----------
    # The 2D quad's angle (raw-point minAreaRect) is near-random on sparse
    # frames (stage7-stance-cause.md: median 43 deg error), which is what
    # makes the stance gate wrongly reject an upright board. A fixed-side
    # (side_m pinned, not fit from data) square fit spends its DOF purely on
    # pose (center, theta) and recovers a median 7 deg error instead. When
    # on, the detector refines (or, if the quad was rejected, rescues) each
    # candidate's pose with `square_fit.fit_fixed_square` and judges the
    # stance gate on the REFINED pose -- see detector.py. Off by default so
    # square_icp=False reproduces stage-6 behavior byte-identical.
    square_icp: bool = False
    # Acceptance threshold on the square fit's coverage residual (lower is
    # better, ~0 is an exact fully-covered fit); Task 24 tunes this against
    # real data. Chosen here as a sane starting point: dense/well-covered
    # synthetic boards fit near 0, a same-extent non-square blob's coverage
    # penalty alone pushes residual well past this.
    square_icp_residual_max: float = 0.35
    # Bounded search window (+/- this many degrees) fit_fixed_square uses
    # around the quad's (or, on rescue, the PCA/centroid seed's) theta.
    square_icp_theta_window_deg: float = 20.0
