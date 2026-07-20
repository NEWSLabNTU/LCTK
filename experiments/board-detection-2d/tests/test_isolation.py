"""TDD for Task 26's isolation_density: free-standing vs embedded discrimination.

Must reproduce the Stage 8 diagnostic's core asymmetry
(`.superpowers/sdd/stage8-isolation-diagnosis.md`): a free-standing board has
~0 exterior-band coplanar density (89% of true detections measure exactly
0), while a board embedded in (flush against) a larger coplanar surface
always measures meaningfully above 0 (residual clutter's minimum was 0.485
pts/m; a 0.3 gate separates the two)."""
from __future__ import annotations

import numpy as np

from boarddet.geometry import PlaneModel, unproject
from boarddet.isolation import isolation_density

# A simple square quad (side 1.0) centered at the plane origin, in (u, v)
# plane coords -- consecutive boundary order (not required to be a diamond;
# isolation_density only needs a 4-corner boundary walked edge by edge).
_HALF = 0.5
_CORNERS_2D = np.array([
    [-_HALF, -_HALF], [_HALF, -_HALF], [_HALF, _HALF], [-_HALF, _HALF],
])
# Axis-aligned plane: normal = world x, so (u, v) = (world y, world z).
_PLANE = PlaneModel(
    center=np.array([4.0, 0.0, 0.3]),
    normal=np.array([1.0, 0.0, 0.0]),
    u=np.array([0.0, 1.0, 0.0]),
    v=np.array([0.0, 0.0, 1.0]),
)


def _grid_2d(lo: float, hi: float, spacing: float) -> np.ndarray:
    xs = np.arange(lo, hi, spacing)
    xx, yy = np.meshgrid(xs, xs)
    return np.stack([xx.ravel(), yy.ravel()], axis=1)


def _to_world(coords2d: np.ndarray, noise: float,
             rng: np.random.Generator) -> np.ndarray:
    pts = unproject(coords2d, _PLANE)
    pts = pts + rng.normal(0.0, noise, size=(len(pts), 1)) * _PLANE.normal
    return pts.astype(np.float32)


def test_free_standing_board_has_near_zero_density():
    """Points ONLY on the board itself, nothing beyond its edges -> density
    must be exactly (or essentially) 0."""
    rng = np.random.default_rng(0)
    board_2d = _grid_2d(-_HALF, _HALF, 0.02)
    dn = _to_world(board_2d, noise=0.003, rng=rng)
    density = isolation_density(dn, _PLANE, _CORNERS_2D)
    assert density == 0.0


def test_embedded_board_flush_against_big_wall_has_high_density():
    """Same board, PLUS a large coplanar wall extending well beyond the
    board's footprint (the board is really just a patch of the wall) ->
    density must be far above the Stage 8 gate threshold (~0.3)."""
    rng = np.random.default_rng(1)
    board_2d = _grid_2d(-_HALF, _HALF, 0.02)
    wall_2d = _grid_2d(-1.2, 1.2, 0.02)
    dn = _to_world(np.concatenate([board_2d, wall_2d]), noise=0.003, rng=rng)
    density = isolation_density(dn, _PLANE, _CORNERS_2D)
    assert density > 0.3, f"expected embedded density >> 0.3, got {density}"


def test_free_standing_passes_and_embedded_fails_the_0_3_gate():
    """The exact discrimination this task wires into the detector: a 0.3
    pts/m gate must pass the free-standing case and reject the embedded
    one -- matching the Stage 8 diagnostic's chosen operating point."""
    rng = np.random.default_rng(2)
    threshold = 0.3
    board_2d = _grid_2d(-_HALF, _HALF, 0.02)

    free_dn = _to_world(board_2d, noise=0.003, rng=rng)
    free_density = isolation_density(free_dn, _PLANE, _CORNERS_2D)
    assert free_density <= threshold

    wall_2d = _grid_2d(-1.2, 1.2, 0.02)
    embedded_dn = _to_world(
        np.concatenate([board_2d, wall_2d]), noise=0.003, rng=rng)
    embedded_density = isolation_density(embedded_dn, _PLANE, _CORNERS_2D)
    assert embedded_density > threshold
