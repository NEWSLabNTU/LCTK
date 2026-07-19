"""Fixed-size square model fit: refine (or rescue) a candidate's pose by
fitting a KNOWN-side square's 3 DOF (center, theta) directly to its 2D
points, instead of trusting the 2D quad's (raw-point minAreaRect) angle.

Why this exists (see .superpowers/sdd/stage7-stance-cause.md): on sparse
frames the quad's angle is essentially uncorrelated with truth (median error
43 deg), which is what makes the stance gate wrongly reject an upright board.
A square fit that KNOWS the true side length ahead of time doesn't need to
infer size from the data at all, so it can spend its degrees of freedom
purely on getting theta right -- median error drops to ~7 deg.

Deliberately NOT filled-square ICP: an ICP-style fit that only pulls its
model's occupied interior onto matched points has zero residual gradient
whenever the model already encloses every point (an "enclosing init" is a
fixed point -- interior points contribute nothing, since they're already
"inside"). The coverage residual below charges for BOTH (a) points that fall
outside the fixed-size square (it shouldn't be smaller than the data implies)
and (b) perimeter band it doesn't reach (it shouldn't be larger than the data
implies, either) -- so an over-large or mis-rotated enclosing box is still
penalized and the search has a gradient to follow.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

_MIN_POINTS = 20
# Trim this much off each end when reading the points' own extent per axis
# (min/max would work on clean data; a couple of percent of headroom guards
# against a single stray outlier setting the whole box).
_EXTENT_PCTL = 2.0
_N_BINS_PER_SIDE = 10   # perimeter coverage-band bins per side (matches the
                        # stance-cause.md throwaway diagnostic's own choice)
_BAND_FRAC = 0.06       # coverage-band half-width, as a fraction of `side`
_COARSE_STEPS = 37      # steps across the full [0, 90) deg sweep (~2.5 deg)
_COARSE_REFINE_WINDOW_DEG = 4.0  # window for the post-coarse-sweep refine
_RESOLUTION_DEG = 0.25  # target step spacing for any windowed theta search
_LOCALIZE_RADIUS_FACTOR = 1.5  # x `side`, around init_center


@dataclass
class SquareFit:
    center: np.ndarray      # (2,) plane coords, same frame as coords_2d
    theta: float            # radians; determined mod 90 deg (4-fold symmetry)
    residual: float         # coverage residual, lower is better; ~0 is exact
    corners_2d: np.ndarray  # (4,2), cyclic order (not necessarily CCW)


def _rot2d(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _coverage_residual(rel: np.ndarray, side: float) -> float:
    """`rel` is points in the square's own axis-aligned frame, already
    centered on the candidate center (so the square occupies
    [-side/2, side/2]^2). Lower is better; 0 is an exact, fully-covered fit.
    """
    half = side / 2.0
    # Term 1: mean per-point distance outside the square. A real board's
    # points should all lie ON the board, so any point beyond the boundary
    # is a direct sign the box is mis-placed/rotated (this is what gives the
    # search a gradient even when the box already encloses everything --
    # see the module docstring).
    d_out = np.hypot(np.maximum(np.abs(rel[:, 0]) - half, 0.0),
                     np.maximum(np.abs(rel[:, 1]) - half, 0.0))
    mean_outside = float(d_out.mean())

    # Term 2: fraction of the perimeter's binned bands with no nearby point.
    # A real board's edge is a physical boundary scanned along its whole
    # length, so a correctly-posed box should have points near ~all of its
    # own perimeter, not just wherever the data happens to be densest.
    corners_sq = half * np.array([[1.0, 1.0], [-1.0, 1.0],
                                  [-1.0, -1.0], [1.0, -1.0]])
    band = _BAND_FRAC * side
    misses = 0
    for i in range(4):
        a, b = corners_sq[i], corners_sq[(i + 1) % 4]
        ab = b - a
        length = float(np.linalg.norm(ab))
        ab_hat = ab / length
        r = rel - a
        t = (r @ ab_hat) / length
        perp = np.abs(r[:, 0] * ab_hat[1] - r[:, 1] * ab_hat[0])
        near = (perp < band) & (t > -0.02) & (t < 1.02)
        bin_idx = np.clip((t[near] * _N_BINS_PER_SIDE).astype(np.int64), 0,
                          _N_BINS_PER_SIDE - 1)
        covered = len(np.unique(bin_idx))
        misses += _N_BINS_PER_SIDE - covered
    coverage_penalty = misses / (4 * _N_BINS_PER_SIDE)

    return mean_outside / side + coverage_penalty


def _fit_at_theta(coords_2d: np.ndarray, side: float, theta: float):
    """Closed-form center (robust per-axis extent midpoint) + coverage
    residual for a fixed theta. Returns (residual, center_world, corners_world).
    """
    rot = _rot2d(theta)
    p_sq = coords_2d @ rot  # world -> square-local (unrotate by theta)
    lo = np.percentile(p_sq, _EXTENT_PCTL, axis=0)
    hi = np.percentile(p_sq, 100.0 - _EXTENT_PCTL, axis=0)
    center_sq = (lo + hi) / 2.0
    rel = p_sq - center_sq

    residual = _coverage_residual(rel, side)

    half = side / 2.0
    corners_sq = half * np.array([[1.0, 1.0], [-1.0, 1.0],
                                  [-1.0, -1.0], [1.0, -1.0]])
    center_world = center_sq @ rot.T
    corners_world = (corners_sq + center_sq) @ rot.T
    return residual, center_world, corners_world


def _best_over(coords_2d: np.ndarray, side: float, thetas: np.ndarray):
    best_residual = np.inf
    best = None
    for theta in thetas:
        residual, center, corners = _fit_at_theta(coords_2d, side, theta)
        if residual < best_residual:
            best_residual = residual
            best = (theta, center, corners)
    theta, center, corners = best
    return SquareFit(center=center, theta=float(theta),
                     residual=float(best_residual), corners_2d=corners)


def fit_fixed_square(coords_2d: np.ndarray, side: float,
                     init_center: np.ndarray | None = None,
                     init_theta: float | None = None,
                     theta_window_deg: float = 20.0) -> SquareFit | None:
    """Fit a fixed-side-`side` square's center + theta to `coords_2d`.

    `init_theta=None`: no orientation prior at all -- coarse full sweep over
    [0, 90 deg) (4-fold symmetry: a square looks the same every 90 deg, so
    that range is the whole period), then a finer windowed refine around the
    coarse winner. Used both directly (dense-fixture tests) and as the
    fallback inside `init_theta`-less callers.

    `init_theta` given: search is a bounded window (+/- theta_window_deg)
    around it instead of the full range -- this is the normal "refine the
    quad's angle" call, and is what actually recovers pose on sparse frames
    (see module docstring): the quad's angle is a noisy prior, not a
    trustworthy one, so the window must be wide enough to correct it (Task
    23's brief default, 20 deg, comfortably covers the quad's typical error
    after the stance gate's own ~26 deg slack is accounted for) while still
    being narrow enough to search efficiently and not have to reproduce the
    full-sweep case.

    `init_center` given: points farther than 1.5*`side` away are dropped
    before fitting, so unrelated clutter elsewhere in a candidate's point
    cloud (e.g. from a generously-sized generator patch) can't bias the
    robust-extent center estimate away from the seeded region. When absent,
    all of `coords_2d` is used as-is.

    Returns None if too few points remain to fit.
    """
    coords_2d = np.asarray(coords_2d, dtype=np.float64)
    if len(coords_2d) < _MIN_POINTS:
        return None

    work = coords_2d
    if init_center is not None:
        init_center = np.asarray(init_center, dtype=np.float64)
        radius = _LOCALIZE_RADIUS_FACTOR * side
        near = np.linalg.norm(coords_2d - init_center, axis=1) < radius
        if near.sum() >= _MIN_POINTS:
            work = coords_2d[near]

    if len(work) < _MIN_POINTS:
        return None

    if init_theta is None:
        coarse_thetas = np.linspace(0.0, np.pi / 2.0, _COARSE_STEPS,
                                    endpoint=False)
        coarse_fit = _best_over(work, side, coarse_thetas)
        lo = coarse_fit.theta - np.radians(_COARSE_REFINE_WINDOW_DEG)
        hi = coarse_fit.theta + np.radians(_COARSE_REFINE_WINDOW_DEG)
    else:
        lo = init_theta - np.radians(theta_window_deg)
        hi = init_theta + np.radians(theta_window_deg)

    n_steps = max(9, int(round((hi - lo) / np.radians(_RESOLUTION_DEG))) + 1)
    fine_thetas = np.linspace(lo, hi, n_steps)
    return _best_over(work, side, fine_thetas)
