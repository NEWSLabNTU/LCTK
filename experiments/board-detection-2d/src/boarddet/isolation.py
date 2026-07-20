"""Exterior-band coplanar-continuation isolation score (Task 26).

Discriminates a free-standing board (isolated: no coplanar surface continues
past its fitted edges) from clutter that is actually a patch cut from a
larger surface (embedded: the backing plane continues past the fitted
quad). This module faithfully reproduces the Stage 8 gating diagnostic
(`.superpowers/sdd/stage8-isolation-diagnosis.md`): coplanar tolerance
0.03 m, exterior band 0.05-0.30 m beyond the fitted quad's edges, density =
exterior-band-coplanar point count / quad perimeter (points per metre).

Must be computed against the frame's full voxel-downsampled cloud BEFORE
`_remove_big_planes` strips anything (`detect()`'s `dn`) -- that is where an
embedded panel's backing wall still survives; the winning candidate's own
`.points` (already stripped/clustered) never see it.
"""
from __future__ import annotations

import numpy as np
from matplotlib.path import Path as MplPath

from .geometry import PlaneModel, project_to_plane

# m. Matches the Stage 8 diagnostic exactly -- see module docstring.
COPLANAR_TOL = 0.03
BAND_LO, BAND_HI = 0.05, 0.30


def _segment_dists(pts: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Point-to-segment distance in 2D: `pts` (N,2) vs segment a->b (2,)."""
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return np.linalg.norm(pts - a, axis=1)
    t = np.clip(((pts - a) @ ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(pts - proj, axis=1)


def isolation_density(dn: np.ndarray, plane: PlaneModel,
                      corners_2d: np.ndarray) -> float:
    """Exterior-band coplanar density, in points per metre of quad perimeter.

    `dn` is the frame's full pre-strip voxel-downsampled cloud (the same
    cloud generator B receives, before its internal `_remove_big_planes`).
    `plane` and `corners_2d` are the winning candidate's fitted plane and its
    (4,2) quad corners expressed in that plane's (u, v) basis (see
    `ScoreResult.corners_2d`); corners need only be in consecutive boundary
    order (CW or CCW), matching how `_seg_dists`/`MplPath` walk the edges.

    Free-standing board -> ~0 (no coplanar points beyond the quad's own
    edges). Clutter embedded in a larger coplanar surface -> high (the
    backing surface's own points fall coplanar and just past the edge).
    """
    coords2d = project_to_plane(dn, plane)
    resid = (dn - plane.center) @ plane.normal
    coplanar = np.abs(resid) < COPLANAR_TOL

    edges = [(corners_2d[i], corners_2d[(i + 1) % 4]) for i in range(4)]
    dists = np.stack(
        [_segment_dists(coords2d, a, b) for a, b in edges], axis=1)
    min_dist = dists.min(axis=1)

    inside = MplPath(corners_2d).contains_points(coords2d)
    band = (~inside) & (min_dist >= BAND_LO) & (min_dist <= BAND_HI)
    ext_mask = band & coplanar

    count = int(ext_mask.sum())
    perimeter = float(sum(np.linalg.norm(b - a) for a, b in edges))
    return count / perimeter if perimeter > 1e-9 else 0.0
