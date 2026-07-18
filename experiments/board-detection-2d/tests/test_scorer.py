import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.geometry import fit_plane, project_to_plane
from boarddet.scorer import score_candidate
from boarddet.synth import make_board


def _board_2d(side=1.0, noise=0.005, spacing=0.02, seed=4, holes=None,
              pattern="grid"):
    rng = np.random.default_rng(seed)
    pts, truth = make_board(
        side=side, center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.1, 0.05]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=spacing, noise=noise, rng=rng, holes=holes, pattern=pattern,
    )
    return project_to_plane(pts, fit_plane(pts))


# Matches ros/lctk_launch/config/board/board_detector.json5: hole_radius
# 150mm, hole_center_shift 200mm, 3 of the 4 possible corner holes punched
# (the recorded board is a hollow diamond, not a solid one).
_REAL_HOLES = [((0.2, 0.2), 0.15), ((0.2, -0.2), 0.15), ((-0.2, 0.2), 0.15)]


def test_scores_true_board_high():
    res = score_candidate(_board_2d(), BoardConfig(side_m=1.0))
    assert res is not None
    assert res.score > 0.6
    np.testing.assert_allclose(res.side_lengths.mean(), 1.0, atol=0.08)
    assert res.angle_err_deg < 6.0


def test_scores_hollow_board_high():
    """The recorded board has 3 punched holes (fill_ratio << 1 even for a
    perfect, fully-observed board), so the fill term must not tank the score
    below min_score for an otherwise-perfect fit."""
    res = score_candidate(_board_2d(holes=_REAL_HOLES), BoardConfig(side_m=1.0))
    assert res is not None
    assert res.fill_ratio < 0.85  # holes measurably reduce fill...
    assert res.score > 0.5        # ...but the fit must still clear min_score


def test_scores_sparse_hollow_board_above_min_score():
    """Reproduces what dataset 3 frame 5 actually looked like: a hollow
    board seen through VLP-32C ring gaps has fill_ratio ~0.44-0.45 (holes
    plus real sparsity, not just holes) with an otherwise good outer-border
    fit. Before the sqrt(fill)/loosened side_err weighting (this task), this
    scenario scored ~0.39 and was silently rejected by min_score=0.5."""
    coords = _board_2d(spacing=0.05, noise=0.02, holes=_REAL_HOLES,
                       pattern="uniform")
    res = score_candidate(coords, BoardConfig(side_m=1.0))
    assert res is not None
    assert 0.3 < res.fill_ratio < 0.6  # matches the real observed range
    assert res.score > 0.5


def test_corner_accuracy_beats_cell_size():
    board = BoardConfig(side_m=1.0)
    res = score_candidate(_board_2d(noise=0.003), board)
    assert res is not None
    d = 1.0 / np.sqrt(2.0)  # half-diagonal
    c = res.corners_2d.mean(axis=0)
    # board is centred at the projection origin
    assert np.linalg.norm(c) < board.cell_m
    # each corner sits at radius d from the centroid (rotation-invariant)
    radii = np.linalg.norm(res.corners_2d - c, axis=1)
    assert np.abs(radii - d).max() < board.cell_m
    # diagonals are orthogonal
    d1 = res.corners_2d[2] - res.corners_2d[0]
    d2 = res.corners_2d[3] - res.corners_2d[1]
    cosang = abs(d1 @ d2) / (np.linalg.norm(d1) * np.linalg.norm(d2))
    assert cosang < 0.02


def test_rejects_wrong_size():
    assert score_candidate(_board_2d(side=2.5), BoardConfig(side_m=1.0)) is None


def test_rejects_sparse_garbage():
    rng = np.random.default_rng(5)
    junk = rng.uniform(-1, 1, size=(150, 2)).astype(np.float32)
    res = score_candidate(junk, BoardConfig(side_m=1.0))
    assert res is None or res.score < 0.5


# --- Task 16: gravity-oriented anisotropic closing ---------------------

def _rotate_2d(pts, theta):
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T


def _striped_diamond_2d(side=1.0, spacing=0.02, band_gap=0.12,
                        band_width=0.02, noise=0.003, theta=0.15, seed=7):
    """Diamond sampled in horizontal (v-axis) bands separated by a real gap
    of ~band_gap (> 5*cell_m for the default 0.02 cell), modelling ring-gap
    striping, then rotated by `theta` so the stripe-separation axis is NOT
    aligned with either raster axis (the scenario the isotropic 5x5 square
    kernel cannot bridge, and the whole reason score_candidate needs a
    caller-supplied up_2d rather than assuming +y).

    Returns (coords_2d, up_2d, truth_corners) where up_2d is the rotated
    +y direction (world "up" projected into this frame) and truth_corners
    are the diamond's 4 true corners, both already rotated to match
    coords_2d's frame.
    """
    rng = np.random.default_rng(seed)
    half_diag = side / np.sqrt(2.0)
    us = np.arange(-half_diag, half_diag, spacing)
    vs_all = np.arange(-half_diag, half_diag, spacing)
    # Keep only rows landing in a thin band once per band_gap period: real,
    # sizeable (> 5*cell) empty gaps between stripes, not just a sparse grid.
    vs = vs_all[np.mod(vs_all + half_diag, band_gap) < band_width]
    uu, vv = np.meshgrid(us, vs)
    coords = np.stack([uu.ravel(), vv.ravel()], axis=1)
    inside = np.abs(coords[:, 0]) + np.abs(coords[:, 1]) <= half_diag
    coords = coords[inside]
    coords = coords + rng.normal(0.0, noise, coords.shape)
    truth_corners = np.array([
        [0.0, half_diag], [half_diag, 0.0], [0.0, -half_diag], [-half_diag, 0.0],
    ])
    up_2d = np.array([-np.sin(theta), np.cos(theta)])  # rotated world +y
    return (_rotate_2d(coords.astype(np.float64), theta), up_2d,
            _rotate_2d(truth_corners, theta))


def test_anisotropic_closing_beats_isotropic_on_striped_board():
    """Ring-gap-striped board, sampled with stripes NOT axis-aligned in
    this candidate's plane frame (the general case -- a board's stance is
    whatever it is, not aligned to the scorer's raster axes). The isotropic
    5x5 kernel can't bridge a > 5-cell gap; the anisotropic path, told the
    true up_2d, rotates the gap onto +y and bridges it with an elongated
    kernel, then rotates the recovered quad back."""
    board = BoardConfig(side_m=1.0)
    coords, up_2d, truth_corners = _striped_diamond_2d()

    without = score_candidate(coords, board)
    with_up = score_candidate(coords, board, up_2d=up_2d,
                              close_height_m=0.15)

    assert without is not None
    assert with_up is not None
    assert with_up.score >= 2.0 * without.score
    assert with_up.rot_2d is not None

    # Rotation round-trip must be exact: refined corners land back near the
    # true (rotated) diamond corners, in the ORIGINAL (input) plane frame.
    cell = board.cell_m
    for truth in truth_corners:
        d = np.linalg.norm(with_up.corners_2d - truth, axis=1)
        assert d.min() < 3.0 * cell, (
            f"corner {truth} not matched within 3*cell_m "
            f"(closest {d.min():.4f}, cell_m {cell})"
        )


def test_anisotropic_coarse_quad_exact_on_rotated_striped_board():
    """Tight rotation-exactness check (Task 16 corner-accuracy fix): with an
    un-bulged coarse quad -- fit directly to the raw, rotated, striped point
    set rather than to a raster closed with the tall gravity-oriented
    kernel -- corners must land within 1*cell_m of truth, not the old 3x
    slack that was only needed to tolerate the bulge. A wider tolerance here
    would silently let a bulge-sized error back in.
    """
    board = BoardConfig(side_m=1.0)
    coords, up_2d, truth_corners = _striped_diamond_2d()
    res = score_candidate(coords, board, up_2d=up_2d, close_height_m=0.15)
    assert res is not None
    assert res.rot_2d is not None
    cell = board.cell_m
    for truth in truth_corners:
        d = np.linalg.norm(res.corners_2d - truth, axis=1)
        assert d.min() < 1.0 * cell, (
            f"corner {truth} not matched within 1*cell_m "
            f"(closest {d.min():.4f}, cell_m {cell})"
        )


def test_isotropic_path_byte_identical_to_pre_task16():
    """Regression pin: up_2d=None/close_height_m=None must reproduce the
    exact stage-3 (pre-Task-16) numbers on the standard dense fixture --
    golden values captured from the isotropic code path (unchanged by this
    task; verified by diffing against `git show HEAD:.../scorer.py` prior
    to this task's edits) before Task 16 introduced the rotated-frame path.
    """
    board = BoardConfig(side_m=1.0)
    res = score_candidate(_board_2d(), board, up_2d=None, close_height_m=None)
    assert res is not None
    assert res.rot_2d is None

    np.testing.assert_allclose(res.score, 0.9317028458569921, rtol=1e-12)
    np.testing.assert_allclose(
        res.corners_2d,
        [[-0.19057871555303893, -0.6667600824310329],
         [0.6631198394527915, -0.1903388436985557],
         [0.19084213898897115, 0.6637913606888861],
         [-0.6605612139952075, 0.18894285706611078]],
        rtol=1e-12)
    np.testing.assert_allclose(
        res.side_lengths,
        [0.9776392072408057, 0.9760044223272387,
         0.9748685915883694, 0.9762740749943098],
        rtol=1e-12)
    np.testing.assert_allclose(res.fill_ratio, 0.9975786924939467, rtol=1e-12)
    np.testing.assert_allclose(res.angle_err_deg, 0.2985387632596037,
                               rtol=1e-12)
    np.testing.assert_allclose(
        res.origin, [-0.7053198348638247, -0.70531901544243], rtol=1e-12)
    assert res.cell_m == 0.02
    # Whole-raster checksum in lieu of embedding the full (71, 71) array.
    assert res.raster.shape == (71, 71)
    assert int(res.raster.sum()) == 649485


# --- Task 18: hole-free strict-diamond discriminator --------------------
#
# The board is losing its holes (becoming a plain diamond), so hole-pattern
# discrimination is off the table. These gates instead exploit (a) the
# diamond's *stance* -- one diagonal ~vertical, standing on a corner -- and
# (b) `edge_support`: a real board's 4 edges are all physically scanned,
# while a fragment/blob fit only has 1-2. All three gates default off
# (BoardConfig()) so they cannot perturb the pinned byte-identical test
# above; that test, unmodified, is this task's defaults-off regression pin.
#
# Fixture note: `score_candidate`'s corner fit is *structurally* rectangular
# (minAreaRect on the anisotropic path; a rectangular fallback whenever
# `_refine_sides` can't find >=5 raw points per side on the isotropic path),
# so a densely FILLED non-rectangular shape (e.g. a rhombus) doesn't survive
# as a skewed quad -- the coarse rectangle-vs-truth mismatch starves refine
# of side points and it silently falls back to an exact-90 deg rectangle
# (confirmed empirically; see task-18-report.md). Perimeter-only sampling
# with a single corner nudged sideways keeps refine's per-side search bands
# populated (the other 3 sides are untouched, near-exact), so the quad it
# recovers is genuinely skewed -- this is what the squareness/edge-support
# fixtures below use, rather than a filled rhombus.

def _diamond_perimeter_2d(side=1.0, points_per_side=300, thickness=0.004,
                          corner_margin=0.03, seed=21, skip_side=None,
                          top_shift=0.0):
    """Diamond boundary only (no interior fill): points sampled along each
    side's length (trimmed corner_margin short of each true corner, small
    perpendicular noise). `skip_side` omits one side's points entirely
    (edge_support fragment fixture); `top_shift` nudges the top corner
    sideways, skewing the two sides that meet there (squareness fixture).
    Corners returned in top/right/bottom/left order."""
    half_diag = side / np.sqrt(2.0)
    corners = np.array([
        [0.0 + top_shift, half_diag], [half_diag, 0.0],
        [0.0, -half_diag], [-half_diag, 0.0],
    ])
    rng = np.random.default_rng(seed)
    parts = []
    for i in range(4):
        if i == skip_side:
            continue
        a, b = corners[i], corners[(i + 1) % 4]
        ts = np.linspace(corner_margin, 1 - corner_margin, points_per_side)
        pts = a + ts[:, None] * (b - a)
        ab = b - a
        normal = np.array([-ab[1], ab[0]]) / np.linalg.norm(ab)
        pts = pts + normal * rng.normal(0.0, thickness, size=(len(pts), 1))
        parts.append(pts)
    return np.concatenate(parts, axis=0), corners


def _filled_square_2d(side=1.0, spacing=0.01, noise=0.001, seed=12):
    """Dense filled axis-aligned square (NOT standing on a corner): both
    diagonals sit at 45 deg off vertical -- the flat-panel-clutter shape the
    stance gate exists to reject."""
    rng = np.random.default_rng(seed)
    half = side / 2.0
    xs = np.arange(-half, half, spacing)
    ys = np.arange(-half, half, spacing)
    xx, yy = np.meshgrid(xs, ys)
    coords = np.stack([xx.ravel(), yy.ravel()], axis=1)
    coords = coords + rng.normal(0.0, noise, coords.shape)
    return coords


_UP_2D = np.array([0.0, 1.0])


def test_edge_support_full_diamond_all_sides_near_one():
    board = BoardConfig(side_m=1.0)
    coords, _ = _diamond_perimeter_2d()
    res = score_candidate(coords, board, up_2d=_UP_2D, close_height_m=0.15)
    assert res is not None
    assert res.edge_support is not None
    assert res.edge_support.min() > 0.9, res.edge_support


def test_edge_support_fragment_missing_side_near_zero():
    board = BoardConfig(side_m=1.0)
    fragment, _ = _diamond_perimeter_2d(skip_side=0)
    res = score_candidate(fragment, board, up_2d=_UP_2D, close_height_m=0.15)
    assert res is not None
    assert res.edge_support is not None
    # 3 sides still fully backed at 1.0; the missing side reads well below
    # them (bin width is coarsened for real ring-gap tolerance -- see
    # _edge_support's docstring -- so "empty" isn't exactly 0, but it's
    # clearly separated from the other 3 and below edge_support_min=0.6).
    assert res.edge_support.min() < 0.6, res.edge_support
    assert (res.edge_support > 0.9).sum() >= 3, res.edge_support


def test_edge_support_min_rejects_fragment_not_full_diamond():
    board_gate = BoardConfig(side_m=1.0, edge_support_min=0.6)
    fragment, _ = _diamond_perimeter_2d(skip_side=0)
    full, _ = _diamond_perimeter_2d()

    assert score_candidate(fragment, board_gate, up_2d=_UP_2D,
                           close_height_m=0.15) is None
    res_full = score_candidate(full, board_gate, up_2d=_UP_2D,
                               close_height_m=0.15)
    assert res_full is not None
    assert res_full.score > 0.0


def test_stance_floor_rejects_axis_aligned_square():
    """Axis-aligned square: diagonals at 45 deg off vertical,
    |diag . up| ~= cos(45) ~= 0.707 -- must be rejected at stance_floor=0.9."""
    board = BoardConfig(side_m=1.0, stance_floor=0.9)
    coords = _filled_square_2d()
    res = score_candidate(coords, board, up_2d=_UP_2D, close_height_m=0.15)
    assert res is None


def test_stance_floor_passes_true_diamond():
    """True diamond standing on a corner: top diagonal is exactly vertical
    (|diag . up| ~= 1.0) -- must pass stance_floor=0.9."""
    board = BoardConfig(side_m=1.0, stance_floor=0.9)
    coords, _ = _diamond_perimeter_2d()
    res = score_candidate(coords, board, up_2d=_UP_2D, close_height_m=0.15)
    assert res is not None


def test_stance_floor_skipped_when_up_2d_none():
    """Near-horizontal-plane candidates have no meaningful "stand on a
    corner" direction (detector._up_2d returns None there); the stance gate
    must be skipped rather than guess, so an axis-aligned square is not
    rejected by stance_floor alone when up_2d is None."""
    board = BoardConfig(side_m=1.0, stance_floor=0.9)
    coords = _filled_square_2d()
    res = score_candidate(coords, board, up_2d=None, close_height_m=None)
    assert res is not None


def test_strict_squareness_rejects_skewed_quad():
    """Diamond with its top corner nudged 0.32m sideways: `_refine_sides`
    (the only score_candidate path that can report a non-rectangular quad;
    see the fixture-note comment above) recovers a genuinely skewed quad
    here, ~13-14 deg off 90 at two corners -- clears the +-8 deg gate."""
    board = BoardConfig(side_m=1.0, strict_squareness=True)
    coords, _ = _diamond_perimeter_2d(top_shift=0.32)
    res = score_candidate(coords, board)
    assert res is None


def test_strict_squareness_passes_clean_diamond():
    board = BoardConfig(side_m=1.0, strict_squareness=True)
    coords, _ = _diamond_perimeter_2d()
    res = score_candidate(coords, board)
    assert res is not None
    assert res.angle_err_deg < 6.0


def test_strict_diamond_gates_default_off_no_regression():
    """Sanity check that a plain BoardConfig() (all Task 18 gates off)
    reproduces the pre-Task-18 accept/reject behaviour on the fixtures this
    task introduces -- the byte-identical pin above already covers the
    pre-existing dense-board fixture; this extends it to the new ones."""
    board = BoardConfig(side_m=1.0)
    fragment, _ = _diamond_perimeter_2d(skip_side=0)
    square = _filled_square_2d()
    skewed, _ = _diamond_perimeter_2d(top_shift=0.32)

    assert score_candidate(fragment, board, up_2d=_UP_2D,
                           close_height_m=0.15) is not None
    assert score_candidate(square, board, up_2d=_UP_2D,
                           close_height_m=0.15) is not None
    assert score_candidate(skewed, board) is not None
