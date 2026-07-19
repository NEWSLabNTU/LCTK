import cv2
import numpy as np
from boarddet.square_fit import SquareFit, fit_fixed_square


def _rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _theta_mod90_error_deg(a, b):
    """Circular distance between two angles that are only meaningful mod
    90 deg (a square's 4-fold symmetry), in degrees. Max possible is 45."""
    d = abs(a - b) % (np.pi / 2)
    d = min(d, np.pi / 2 - d)
    return float(np.degrees(d))


def _square_corners(side, center, theta):
    half = side / 2.0
    corners_sq = half * np.array([[1.0, 1.0], [-1.0, 1.0],
                                  [-1.0, -1.0], [1.0, -1.0]])
    return corners_sq @ _rot2d(theta).T + center


def _nearest_corner_errors(corners_a, corners_b):
    return np.array([
        np.linalg.norm(corners_b - c, axis=1).min() for c in corners_a
    ])


def _filled_square_points(side, center, theta, spacing=0.02, noise=0.002,
                          seed=1):
    rng = np.random.default_rng(seed)
    half = side / 2.0
    xs = np.arange(-half, half, spacing)
    ys = np.arange(-half, half, spacing)
    xx, yy = np.meshgrid(xs, ys)
    local = np.stack([xx.ravel(), yy.ravel()], axis=1)
    world = local @ _rot2d(theta).T + center
    return (world + rng.normal(0.0, noise, world.shape))


def _sparse_stripe_square(side=1.0, theta_true=np.radians(37.0),
                          center_true=None, n_stripes=7, tip_margin=0.08,
                          stripe_width=0.02, spacing=0.01, noise=0.002,
                          seed=3):
    """Sparse-ring failure case: a filled square sampled only in a handful
    of stripes along its OWN local y axis (before rotating to world by
    `theta_true`), with a margin excluded at both ends -- modelling a LiDAR
    ring gap that happens to fall exactly on the board's own top/bottom
    edges, so the true y-extent is never directly observed at all. A
    free-size fit (e.g. cv2.minAreaRect) can only ever be as large as what
    it sees, so it necessarily comes back short of the true side along that
    axis; the fixed-size fit's side is pinned, so it isn't limited that way.
    Stripes are defined in the object's own (pre-rotation) frame, not
    world-y, so the starved axis is deterministic regardless of theta_true.
    """
    if center_true is None:
        center_true = np.array([0.4, -0.15])
    half = side / 2.0
    rng = np.random.default_rng(seed)
    xs = np.arange(-half, half, spacing)
    ys = np.arange(-half, half, spacing)
    xx, yy = np.meshgrid(xs, ys)
    local = np.stack([xx.ravel(), yy.ravel()], axis=1)
    lo, hi = -half + tip_margin, half - tip_margin
    stripe_centers = np.linspace(lo, hi, n_stripes)
    keep = np.zeros(len(local), dtype=bool)
    for yc in stripe_centers:
        keep |= np.abs(local[:, 1] - yc) < stripe_width
    local = local[keep]
    world = local @ _rot2d(theta_true).T + center_true
    world = world + rng.normal(0.0, noise, world.shape)
    return world, center_true, theta_true


def _disk_points(side, center, n=400, seed=9):
    """Filled disk inscribed in a `side`x`side` box: same overall bounding
    extent as the square (touches +-side/2 along each axis at its poles)
    but never reaches the square's own corners -- a shape discriminator
    that isolates the coverage-band term (the "outside" term is ~0 for
    both, since the disk never exceeds the box)."""
    rng = np.random.default_rng(seed)
    r = (side / 2.0) * np.sqrt(rng.uniform(0.0, 1.0, n))
    ang = rng.uniform(0.0, 2 * np.pi, n)
    local = np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)
    return local + center


# --- dense recovery ------------------------------------------------------

def test_recovers_dense_square_pose_no_init():
    side = 1.0
    theta_true = np.radians(27.0)
    center_true = np.array([0.3, -0.2])
    pts = _filled_square_points(side, center_true, theta_true)
    fit = fit_fixed_square(pts, side)
    assert fit is not None
    assert isinstance(fit, SquareFit)
    assert np.linalg.norm(fit.center - center_true) < 0.02
    assert _theta_mod90_error_deg(fit.theta, theta_true) < 3.0
    assert fit.residual < 0.05
    true_corners = _square_corners(side, center_true, theta_true)
    errs = _nearest_corner_errors(fit.corners_2d, true_corners)
    assert errs.max() < 0.03


# --- sparse rescue: the core value ---------------------------------------

def test_sparse_stripe_rescue_matches_true_square_not_shrunk_extent():
    side = 1.0
    pts, center_true, theta_true = _sparse_stripe_square(side=side)

    # Sanity: the observed points are really short of the true half-side
    # reach along the object's own (starved) y axis -- confirms the fixture
    # is genuinely testing recovery from missing data, not a vacuous no-op.
    rel = (pts - center_true) @ _rot2d(theta_true)
    naive_half_extent_y = np.abs(rel[:, 1]).max()
    assert naive_half_extent_y < side / 2.0 - 0.03, (
        "fixture didn't actually starve the tip region -- test is vacuous"
    )
    # A genuine free-size fit (cv2.minAreaRect, the same primitive
    # score_candidate's coarse quad uses) on these exact points comes back
    # measurably undersized -- this is the failure mode the fixed-size fit
    # must NOT reproduce.
    rect = cv2.minAreaRect(pts.astype(np.float32))
    naive_sides = np.linalg.norm(
        np.roll(cv2.boxPoints(rect), -1, axis=0) - cv2.boxPoints(rect),
        axis=1)
    assert naive_sides.mean() < 0.97 * side, (
        "fixture's minAreaRect isn't actually undersized -- test is vacuous"
    )

    fit = fit_fixed_square(pts, side)
    assert fit is not None
    assert np.linalg.norm(fit.center - center_true) < 0.03
    assert _theta_mod90_error_deg(fit.theta, theta_true) < 5.0

    true_corners = _square_corners(side, center_true, theta_true)
    errs = _nearest_corner_errors(fit.corners_2d, true_corners)
    assert errs.max() < 0.04, (
        f"recovered corners should match the TRUE full-size square, "
        f"not a shrunk point-extent box: errs={errs}"
    )
    fit_sides = np.linalg.norm(
        np.roll(fit.corners_2d, -1, axis=0) - fit.corners_2d, axis=1)
    np.testing.assert_allclose(fit_sides, side, atol=1e-9)


# --- stall-avoidance: not filled-square ICP -------------------------------

def test_stall_avoidance_corrects_enclosing_but_misrotated_init():
    """An init that already ENCLOSES every point (correct center, side)
    but is rotated 10 deg off must still be corrected -- a filled-square
    ICP that only pulls occupied model cells onto points has zero gradient
    once its box already encloses everything (see square_fit.py's module
    docstring), so recovering here proves this fit isn't that."""
    side = 1.0
    theta_true = np.radians(12.0)
    center_true = np.array([-0.1, 0.25])
    pts = _filled_square_points(side, center_true, theta_true, spacing=0.015)
    bad_init_theta = theta_true + np.radians(10.0)

    fit = fit_fixed_square(pts, side, init_center=center_true,
                           init_theta=bad_init_theta, theta_window_deg=20.0)
    assert fit is not None
    err = _theta_mod90_error_deg(fit.theta, theta_true)
    assert err < 2.0, f"theta didn't correct from the 10 deg-off init: {err}"
    assert err < _theta_mod90_error_deg(bad_init_theta, theta_true)


# --- pose-accuracy: mirrors the diagnosed 43 deg -> 7 deg gain ------------

def test_pose_accuracy_recovers_truth_from_quad_like_bad_init():
    """Deliberately-wrong init_theta, comparable in size to the diagnosed
    real quad's ~43 deg median error (stage7-stance-cause.md) -- the fit
    must still land within ~10 deg of truth. Window is 45 deg (not the
    20 deg detector default): since theta is only meaningful mod 90 deg,
    the max possible circular distance from ANY init to the truth is 45
    deg, so a 45 deg window is the mathematically-necessary size to
    guarantee the search range covers truth regardless of how bad the
    quad's angle is -- this test is about the fit's accuracy given a wide
    enough search, not about tuning the detector's default window (that is
    a separate, Task-24-tunable knob)."""
    side = 1.0
    theta_true = np.radians(8.0)
    center_true = np.array([0.15, 0.05])
    pts = _filled_square_points(side, center_true, theta_true, spacing=0.02)
    bad_init_theta = theta_true + np.radians(41.0)  # ~worst-case mod-90 error

    fit = fit_fixed_square(pts, side, init_center=center_true,
                           init_theta=bad_init_theta, theta_window_deg=45.0)
    assert fit is not None
    err = _theta_mod90_error_deg(fit.theta, theta_true)
    assert err < 10.0, f"theta error {err} deg exceeds the ~10 deg target"


# --- residual discrimination ----------------------------------------------

def test_residual_discriminates_square_from_same_extent_blob():
    side = 1.0
    center = np.array([0.0, 0.0])
    square_pts = _filled_square_points(side, center, 0.0, spacing=0.02)
    disk_pts = _disk_points(side, center)

    square_fit = fit_fixed_square(square_pts, side)
    disk_fit = fit_fixed_square(disk_pts, side)

    assert square_fit is not None
    assert disk_fit is not None
    assert disk_fit.residual > 2.0 * square_fit.residual, (
        f"square residual {square_fit.residual} vs disk residual "
        f"{disk_fit.residual} -- not discriminating"
    )


# --- no-regress guard ------------------------------------------------------

def test_no_regress_good_init_theta_stays_put():
    """When the quad is already good (dense, well-sampled board), the fit
    must not wander away from a correct init -- Task 23's caveat (28/240
    already-DETECTED frames where a from-scratch robust fit disagreed with
    a good quad by >15 deg): seeded with the TRUE (i.e. already-correct)
    theta, the refine must stay close to it, not drift."""
    side = 1.0
    theta_true = np.radians(45.0)
    center_true = np.array([0.2, 0.1])
    pts = _filled_square_points(side, center_true, theta_true, spacing=0.02)

    fit = fit_fixed_square(pts, side, init_center=center_true,
                           init_theta=theta_true, theta_window_deg=20.0)
    assert fit is not None
    err = _theta_mod90_error_deg(fit.theta, theta_true)
    assert err < 2.0, f"fit moved a correct init away by {err} deg"
    true_corners = _square_corners(side, center_true, theta_true)
    errs = _nearest_corner_errors(fit.corners_2d, true_corners)
    assert errs.max() < 0.03


# --- degenerate input -------------------------------------------------------

def test_returns_none_for_too_few_points():
    pts = np.zeros((5, 2))
    assert fit_fixed_square(pts, 1.0) is None
