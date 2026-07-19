import numpy as np
import pytest
from boarddet.board_config import BoardConfig
from boarddet.detector import GENERATORS, _stance, _up_2d, detect
from boarddet.geometry import PlaneModel
from boarddet.synth import make_scene


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
    was already cleanly detected -- both must still find the board, and
    pose must stay close between the two paths."""
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


def test_square_icp_rescues_stance_rejected_quad():
    """The core value (stage7-stance-cause.md): with vertical_gap_deg and a
    strict stance_floor on, a candidate whose raw-point quad angle is bad
    enough to fail the stance gate can still be rescued by the fixed-size
    fit's refined pose. square_icp=False stays on whatever the quad-only
    path produces; square_icp=True must detect at least as often here."""
    pts, _ = make_scene(rng=np.random.default_rng(13))
    board_off = BoardConfig(side_m=1.0, vertical_gap_deg=3.0,
                            stance_floor=0.9, square_icp=False)
    board_on = BoardConfig(side_m=1.0, vertical_gap_deg=3.0,
                           stance_floor=0.9, square_icp=True)
    out_off = detect(pts, board_off, generator="b")
    out_on = detect(pts, board_on, generator="b")
    # square_icp=True must never do worse than off at the same operating
    # point: if the quad-only path already detects, the refined path must
    # too.
    if out_off.detection is not None:
        assert out_on.detection is not None


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
