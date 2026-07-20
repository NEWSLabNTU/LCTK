import numpy as np
import pytest
from boarddet.board_config import BoardConfig
from boarddet.candidates import Candidate
from boarddet.detector import GENERATORS, _stance, _up_2d, detect
from boarddet.geometry import PlaneModel, fit_plane, project_to_plane, unproject
from boarddet.synth import SceneTruth, _plane_basis, make_scene


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_detects_board_in_synthetic_scene(gen):
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0), generator=gen)
    assert out.detection is not None, f"generator {gen} found nothing"
    assert np.linalg.norm(out.detection.center - truth.center) < 0.05
    assert abs(out.detection.rotation[:, 2] @ truth.normal) > 0.99
    assert out.timings_ms["total"] > 0


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_no_detection_in_boardless_scene(gen):
    rng = np.random.default_rng(14)
    pts, _ = make_scene(rng=rng)
    # strip points near the board plane region entirely: keep clutter only
    keep = pts[:, 0] < 2.0
    out = detect(pts[keep], BoardConfig(side_m=1.0), generator=gen)
    assert out.detection is None


def test_stance_diamond_beats_axis_aligned():
    # Diamond standing on a corner: one diagonal gravity (z) aligned.
    diamond = np.array([
        [0.0, 0.0, 1.0],   # top (gravity-aligned corner)
        [1.0, 0.0, 0.0],   # right
        [0.0, 0.0, -1.0],  # bottom (gravity-aligned corner)
        [-1.0, 0.0, 0.0],  # left
    ])
    # Axis-aligned square panel (upright, sides horizontal/vertical): both
    # diagonals sit at ~45 deg off gravity.
    flat = np.array([
        [0.5, 0.0, 0.5],    # top-right
        [-0.5, 0.0, 0.5],   # top-left
        [-0.5, 0.0, -0.5],  # bottom-left
        [0.5, 0.0, -0.5],   # bottom-right
    ])
    assert _stance(flat) < _stance(diamond)
    assert _stance(diamond) > 0.99
    assert 0.6 < _stance(flat) < 0.8


def test_detect_with_stance_weight_still_finds_board():
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0, stance_weight=0.5),
                 generator="a")
    assert out.detection is not None
    assert np.isfinite(out.detection.score)
    assert np.linalg.norm(out.detection.center - truth.center) < 0.05


def test_best_rejected_populated_when_min_score_too_high():
    pts, _ = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0, min_score=0.99), generator="a")
    assert out.detection is None
    assert out.best_rejected is not None
    assert out.best_rejected.score < 0.99


def test_best_rejected_none_when_detection_accepted():
    pts, _ = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0), generator="a")
    assert out.detection is not None
    assert out.best_rejected is None


def test_up_2d_none_for_near_horizontal_plane():
    """A near-horizontal plane (normal ~= world z) has u, v spanning a
    near-horizontal patch, so world +z projects to ~zero in-plane -- no
    privileged "up" stripe direction to rotate onto. _up_2d must signal
    this (None) so the detector falls back to the isotropic kernel rather
    than rotating by a near-arbitrary, noise-dominated direction."""
    horizontal = PlaneModel(center=np.zeros(3), normal=np.array([0.0, 0.0, 1.0]),
                            u=np.array([1.0, 0.0, 0.0]),
                            v=np.array([0.0, 1.0, 0.0]))
    assert _up_2d(horizontal) is None


def _match_corner_errors(corners_a, corners_b):
    """Nearest-neighbour per-corner distance from each row of `corners_a` to
    the closest row of `corners_b` (both (4,3))."""
    return np.array([
        np.linalg.norm(corners_b - c, axis=1).min() for c in corners_a
    ])


def test_anisotropic_path_corners_match_isotropic_on_default_scene():
    """Reproduces the reviewer's corner-accuracy regression: the stage-4
    anisotropic path's tall gravity-oriented closing kernel bulges the
    diamond's pointed corners in the occupancy raster, and (pre-fix) the
    coarse quad was fit to that bulged raster -- ~4x corner-accuracy
    regression at vertical_gap_deg=3.0 on this exact scene (0.03 m -> 0.12
    m). With the coarse quad fit to raw points instead, the anisotropic
    path must not degrade corners relative to the isotropic path on a
    dense, well-observed board."""
    pts, truth = make_scene(rng=np.random.default_rng(13))
    board_iso = BoardConfig(side_m=1.0, vertical_gap_deg=0.0)
    board_aniso = BoardConfig(side_m=1.0, vertical_gap_deg=3.0)
    out_iso = detect(pts, board_iso, generator="a")
    out_aniso = detect(pts, board_aniso, generator="a")
    assert out_iso.detection is not None
    assert out_aniso.detection is not None
    cell = board_aniso.cell_m
    errs = _match_corner_errors(out_aniso.detection.corners_3d,
                                out_iso.detection.corners_3d)
    assert errs.max() < cell, (
        f"anisotropic corners diverge from isotropic by up to {errs.max():.4f} m "
        f"(cell_m {cell}): {errs}"
    )


def test_long_range_anisotropic_detection_has_accurate_corners():
    """Long-range guard: the pre-fix bug produced an ACCEPTED detection
    (score > min_score) at range ~10 m with corners off by ~0.5 m on a 1 m
    board (center_err stayed ~0.01 m because center averages the bulge out
    across all four corners). Any detection returned here must have
    genuinely accurate corners; silently accepting a badly-fit quad is
    worse than rejecting it outright."""
    board = BoardConfig(side_m=1.0, vertical_gap_deg=3.0)
    pts, truth = make_scene(board_center=(10.0, 0.5, 0.3),
                            rng=np.random.default_rng(13))
    out = detect(pts, board, generator="a")
    if out.detection is None:
        return  # no detection is an acceptable outcome
    cell = board.cell_m
    errs = _match_corner_errors(out.detection.corners_3d, truth.corners)
    assert errs.max() < 2.0 * cell, (
        f"accepted long-range detection has corner error up to "
        f"{errs.max():.4f} m (2*cell_m {2 * cell}): {errs}"
    )


# --- Task 23: fixed-size square fitter (refine-after-quad) ---------------

def _rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _make_clipped_diamond(side, center, normal, up_hint, depth, spacing,
                          noise, rng):
    """A dense diamond board (see synth.make_board) with TWO OPPOSITE
    corners of its underlying square clipped off -- models a real partial
    occlusion (e.g. a mounting bracket, or grazing-angle self-shadowing on a
    diamond-mounted board) that removes an entire physical corner region,
    not just a ring-gap.

    This is what actually fools cv2.minAreaRect's angle: it's driven purely
    by the convex hull's extreme points, so sparse-but-still-corner-complete
    sampling (e.g. missing whole LiDAR rings, still spanning the full
    extent) does NOT fool it -- verified empirically while building this
    fixture, ring-gap sparsity alone left minAreaRect's angle error under
    1 deg even at very low point counts. Only removing an entire corner
    REGION (not just individual points near it) changes which convex-hull
    edge direction minAreaRect's rotating-calipers search prefers.

    `depth` is the half-side fraction kept along the (1,1)-diagonal of the
    UNDERLYING axis-aligned square (before the +45 deg rotation that turns
    it into make_board's diamond shape): depth ~0.32-0.34 is the measured
    critical band where cv2.minAreaRect's own angle flips from ~0 deg error
    to the ~45 deg (worst-case mod-90) error, while `fit_fixed_square`'s
    full sweep still recovers the true theta cleanly (its coverage residual
    stays a stable ~0.40 -- a real perimeter-coverage gap from the missing
    corners, not evidence of a bad fit; see board_config.py's
    square_icp_residual_max comment and test_square_fit.py's
    test_residual_gate_discriminates_board_recovery_from_clutter_blob).
    """
    u, v, n = _plane_basis(np.asarray(normal, float), np.asarray(up_hint, float))
    center = np.asarray(center, float)
    half = side / 2.0
    xs = np.arange(-half, half, spacing)
    ys = np.arange(-half, half, spacing)
    xx, yy = np.meshgrid(xs, ys)
    sq_local = np.stack([xx.ravel(), yy.ravel()], axis=1)
    d1 = np.array([1.0, 1.0]) / np.sqrt(2.0)
    sq_local = sq_local[np.abs(sq_local @ d1) < depth]
    # make_board's diamond IS the underlying square rotated 45 deg in (u,v)
    # (diamond corners sit on the u/v axes; see make_board's own docstring).
    local = sq_local @ _rot2d(np.radians(45.0)).T
    pts = center + local[:, :1] * u + local[:, 1:] * v
    pts = pts + rng.normal(0.0, noise, size=(len(pts), 1)) * n
    half_diag = side / np.sqrt(2.0)
    corners = np.stack([
        center + half_diag * v, center + half_diag * u,
        center - half_diag * v, center - half_diag * u,
    ])
    return pts.astype(np.float32), SceneTruth(center=center, normal=n,
                                              corners=corners)


def _theta_mod90_err_deg(corners_a, corners_b):
    """Circular mod-90 deg error (square's 4-fold symmetry) between the
    in-plane orientations of two coplanar 4-corner (4,3) arrays. Fits ITS
    OWN common plane from `corners_a` (any 4 coplanar points determine a
    plane) so the comparison doesn't depend on either corner set's own
    originating candidate/plane fit -- just a consistent shared frame to
    measure both thetas in."""
    plane = fit_plane(corners_a)

    def theta(corners):
        c2 = project_to_plane(corners, plane)
        c = c2.mean(axis=0)
        v = c2[0] - c
        return np.arctan2(v[1], v[0]) - np.pi / 4.0

    d = abs(theta(corners_a) - theta(corners_b)) % (np.pi / 2)
    d = min(d, np.pi / 2 - d)
    return float(np.degrees(d))


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_square_icp_off_byte_identical_to_stage6(gen):
    """Regression pin: adding the square_icp knob must not perturb the
    default (off) path at all -- explicit BoardConfig(square_icp=False)
    must reproduce the exact same scored outcome as a plain BoardConfig()
    call (which predates this task) on the standard scene."""
    pts, _ = make_scene(rng=np.random.default_rng(13))
    out_default = detect(pts, BoardConfig(side_m=1.0), generator=gen)
    out_explicit_off = detect(pts, BoardConfig(side_m=1.0, square_icp=False),
                              generator=gen)
    assert (out_default.detection is None) == (
        out_explicit_off.detection is None)
    if out_default.detection is not None:
        np.testing.assert_array_equal(out_default.detection.center,
                                      out_explicit_off.detection.center)
        np.testing.assert_array_equal(out_default.detection.rotation,
                                      out_explicit_off.detection.rotation)
        np.testing.assert_array_equal(out_default.detection.corners_3d,
                                      out_explicit_off.detection.corners_3d)
        assert out_default.detection.score == out_explicit_off.detection.score


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_square_icp_detects_board_in_synthetic_scene(gen):
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0, square_icp=True), generator=gen)
    assert out.detection is not None, f"generator {gen} found nothing"
    assert np.linalg.norm(out.detection.center - truth.center) < 0.05
    assert abs(out.detection.rotation[:, 2] @ truth.normal) > 0.99


def test_square_icp_does_not_regress_an_already_detected_scene():
    """Task 23's caveat (stage7-stance-cause.md): a robust from-scratch fit
    disagreed with an already-good quad by >15 deg on 28/240 already-
    DETECTED frames. Turning square_icp on must never break a scene that
    was already cleanly detected -- both must still find the board, pose
    must stay close between the two paths, AND (post full-sweep fix)
    in-plane theta specifically must not have wandered: the full [0,90 deg)
    sweep is only really "safe" if it reliably re-finds an already-correct
    quad's theta rather than occasionally drifting to some other coverage-
    residual local minimum."""
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out_off = detect(pts, BoardConfig(side_m=1.0, square_icp=False),
                     generator="a")
    out_on = detect(pts, BoardConfig(side_m=1.0, square_icp=True),
                    generator="a")
    assert out_off.detection is not None
    assert out_on.detection is not None
    assert np.linalg.norm(out_on.detection.center - truth.center) < 0.05
    assert abs(out_on.detection.rotation[:, 2] @ truth.normal) > 0.99
    assert np.linalg.norm(
        out_on.detection.center - out_off.detection.center) < 0.02
    theta_err = _theta_mod90_err_deg(out_on.detection.corners_3d,
                                     out_off.detection.corners_3d)
    assert theta_err < 3.0, (
        f"square_icp's full sweep moved an already-good quad's theta by "
        f"{theta_err:.2f} deg relative to the quad-only path")


def test_square_icp_flips_stance_rejected_quad_to_accepted_rescue():
    """THE test that proves square_icp's value (Task 23 fix): a candidate
    whose raw-point quad angle is bad enough (cv2.minAreaRect itself picks a
    totally different-oriented, non-square box -- see
    _make_clipped_diamond) that score_candidate's OWN internal stance gate
    rejects it outright (res is None in detect()'s non-square_icp branch).

    square_icp=False must NOT produce any detection at all here (not just
    "loses to a better candidate" -- the fixture has exactly one candidate,
    injected directly via a stubbed generator, so a None here is a genuine
    fail-closed, verified by construction rather than assumed).

    square_icp=True -- now always calling fit_fixed_square with
    init_theta=None (full mod-90 sweep) instead of seeding a narrow window
    from the untrustworthy quad angle -- must recover the true pose: theta
    within ~10 deg mod-90, stance passing, at the true location.
    """
    rng = np.random.default_rng(21)
    side = 1.0
    center = np.array([4.0, 0.5, 0.3])
    normal = np.array([-1.0, 0.15, 0.05])
    up_hint = np.array([0.0, 0.0, 1.0])
    pts, truth = _make_clipped_diamond(side, center, normal, up_hint,
                                       depth=0.34, spacing=0.008,
                                       noise=0.004, rng=rng)

    def fake_gen(dn, board, **kwargs):
        return [Candidate(points=pts, plane=fit_plane(pts))]

    GENERATORS["_clipped_diamond"] = fake_gen
    try:
        board_off = BoardConfig(side_m=side, vertical_gap_deg=3.0,
                                stance_floor=0.9, square_icp=False)
        board_on = BoardConfig(side_m=side, vertical_gap_deg=3.0,
                               stance_floor=0.9, square_icp=True)
        out_off = detect(pts, board_off, generator="_clipped_diamond")
        out_on = detect(pts, board_on, generator="_clipped_diamond")
    finally:
        del GENERATORS["_clipped_diamond"]

    # OFF: fails closed. score_candidate's own stance gate rejects the raw
    # quad (res is None), so the candidate is skipped before a BoardDetection
    # is ever built -- no detection AND no best_rejected.
    assert out_off.detection is None, (
        "quad-only path should have failed on this fixture's bad quad angle "
        "-- fixture is not exercising the intended failure mode")
    assert out_off.best_rejected is None

    # ON: the full sweep recovers the true pose and passes stance.
    assert out_on.detection is not None, (
        "square_icp=True should rescue the stance-rejected candidate")
    det = out_on.detection
    center_err = float(np.linalg.norm(det.center - truth.center))
    theta_err = _theta_mod90_err_deg(det.corners_3d, truth.corners)
    assert center_err < 0.05, f"center error {center_err:.4f} m too large"
    assert theta_err < 10.0, f"theta error {theta_err:.2f} deg too large"
    assert abs(det.rotation[:, 2] @ truth.normal) > 0.99
    assert _stance(det.corners_3d) > 0.9, "rescued pose must pass stance too"


def test_up_2d_present_for_vertical_plane():
    """Sanity check on the other side of the gate: a vertical plane (world
    z lies entirely in-plane) must return a unit vector, not None."""
    vertical = PlaneModel(center=np.zeros(3), normal=np.array([1.0, 0.0, 0.0]),
                          u=np.array([0.0, 1.0, 0.0]),
                          v=np.array([0.0, 0.0, 1.0]))
    up = _up_2d(vertical)
    assert up is not None
    np.testing.assert_allclose(np.linalg.norm(up), 1.0)
    np.testing.assert_allclose(up, [0.0, 1.0], atol=1e-12)


# --- Task 26: isolation (exterior coplanar-continuation) discriminator ---

@pytest.mark.parametrize("gen", list(GENERATORS))
def test_isolation_off_byte_identical_to_stage6(gen):
    """Regression pin: adding the isolation knob must not perturb the
    default (off) path at all -- explicit BoardConfig(isolation=False) must
    reproduce the exact same scored outcome as a plain BoardConfig() call
    (which predates this task) on the standard scene."""
    pts, _ = make_scene(rng=np.random.default_rng(13))
    out_default = detect(pts, BoardConfig(side_m=1.0), generator=gen)
    out_explicit_off = detect(pts, BoardConfig(side_m=1.0, isolation=False),
                              generator=gen)
    assert (out_default.detection is None) == (
        out_explicit_off.detection is None)
    if out_default.detection is not None:
        np.testing.assert_array_equal(out_default.detection.center,
                                      out_explicit_off.detection.center)
        np.testing.assert_array_equal(out_default.detection.rotation,
                                      out_explicit_off.detection.rotation)
        np.testing.assert_array_equal(out_default.detection.corners_3d,
                                      out_explicit_off.detection.corners_3d)
        assert out_default.detection.score == out_explicit_off.detection.score


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_isolation_on_still_detects_free_standing_board(gen):
    """`make_scene`'s board is free-standing (ground/wall/blob clutter all
    sit spatially and spectrally away from the board's own fitted plane),
    so turning isolation on must not cost this detection: the exterior band
    around its quad has no coplanar clutter to trip the gate."""
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0, isolation=True), generator=gen)
    assert out.detection is not None, f"generator {gen} found nothing"
    assert np.linalg.norm(out.detection.center - truth.center) < 0.05
    assert abs(out.detection.rotation[:, 2] @ truth.normal) > 0.99


def test_isolation_rejects_board_sized_patch_of_a_big_wall():
    """THE test that proves isolation's value: a "board" that is really
    just a board-sized patch cut out of a much larger coplanar wall (an
    embedded panel, like the ds5 residual clutter in
    stage8-isolation-diagnosis.md) must score/stance/etc. fine on its own
    patch of points (that's the whole problem -- nothing else catches it)
    but isolation must reject it once the full pre-strip cloud (which still
    contains the wall's continuation beyond the patch) is consulted.

    Uses a stubbed generator returning exactly one Candidate (the small
    patch) so the outcome is attributable purely to the isolation gate, not
    generator/candidate selection.
    """
    rng = np.random.default_rng(7)
    plane = PlaneModel(center=np.array([4.0, 0.0, 0.3]),
                       normal=np.array([1.0, 0.0, 0.0]),
                       u=np.array([0.0, 1.0, 0.0]),
                       v=np.array([0.0, 0.0, 1.0]))
    half = 0.5
    xs = np.arange(-1.2, 1.2, 0.02)
    xx, yy = np.meshgrid(xs, xs)
    wall_2d = np.stack([xx.ravel(), yy.ravel()], axis=1)
    wall_pts = unproject(wall_2d, plane)
    wall_pts = (wall_pts + rng.normal(0.0, 0.003, size=(len(wall_pts), 1))
               * plane.normal).astype(np.float32)
    patch_mask = (np.abs(wall_2d[:, 0]) < half) & (np.abs(wall_2d[:, 1]) < half)
    patch_pts = wall_pts[patch_mask]

    def fake_gen(dn, board, **kwargs):
        return [Candidate(points=patch_pts, plane=fit_plane(patch_pts))]

    GENERATORS["_wall_patch"] = fake_gen
    try:
        board = BoardConfig(side_m=1.0, vertical_gap_deg=0.0)
        out_off = detect(wall_pts, board, generator="_wall_patch")
        out_on = detect(
            wall_pts,
            BoardConfig(side_m=1.0, vertical_gap_deg=0.0, isolation=True),
            generator="_wall_patch")
    finally:
        del GENERATORS["_wall_patch"]

    assert out_off.detection is not None, (
        "fixture's patch should score/stance fine on its own -- the whole "
        "point is that nothing but isolation catches this")
    assert out_on.detection is None, (
        "isolation should reject a board-sized patch of a much larger "
        "coplanar wall")
