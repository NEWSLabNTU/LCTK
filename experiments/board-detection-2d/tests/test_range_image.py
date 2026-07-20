"""Tests for the shared nearest-real-channel range-image renderer (Task 30,
review must-fix #1): sim and real data MUST land on the identical row axis,
or a synth-trained CNN evaluated on real data fails for a dumb, structural
reason. See `src/boarddet/sim/range_image.py`'s module docstring for the
full row/column semantics this locks down."""
from __future__ import annotations

import numpy as np

from boarddet.sim.primitives import Rect
from boarddet.sim.range_image import (
    azimuth_to_col,
    elevation_to_row,
    points_to_rows_cols_ranges,
    sim_frame_to_range_image,
    to_range_image,
)
from boarddet.sim.raycast import render
from boarddet.sim.sensor import N_LASERS, Vlp32cSensor

# ---------------------------------------------------------------------------
# nearest-channel row binning correctness
# ---------------------------------------------------------------------------


def test_point_at_known_channel_elevation_lands_in_expected_row():
    sensor = Vlp32cSensor()
    for row in (0, 5, 16, 31):
        elev = sensor.elevations[row]
        r = 3.0  # arbitrary horizontal range
        x = r * np.cos(elev)
        z = r * np.sin(elev)
        point = np.array([[x, 0.0, z]])
        row_out, _, _ = points_to_rows_cols_ranges(point, sensor, azimuth_steps=360)
        assert row_out[0] == row


def test_row_binning_ties_break_toward_nearer_channel():
    sensor = Vlp32cSensor()
    # a point exactly between rows 10 and 11 must land on whichever is closer
    e_lo, e_hi = sensor.elevations[10], sensor.elevations[11]
    just_below_mid = e_lo + 0.49 * (e_hi - e_lo)
    just_above_mid = e_lo + 0.51 * (e_hi - e_lo)
    for elev, expected in ((just_below_mid, 10), (just_above_mid, 11)):
        row = elevation_to_row(np.array([elev]), sensor.elevations)
        assert row[0] == expected


def test_elevation_to_row_never_out_of_range():
    sensor = Vlp32cSensor()
    extreme = np.array([-10.0, -np.pi / 2, 0.0, np.pi / 2, 10.0])
    rows = elevation_to_row(extreme, sensor.elevations)
    assert np.all((rows >= 0) & (rows < N_LASERS))


# ---------------------------------------------------------------------------
# azimuth column binning
# ---------------------------------------------------------------------------


def test_azimuth_straight_ahead_lands_on_center_column():
    n_cols = 360
    col = azimuth_to_col(np.array([0.0]), n_cols)
    assert col[0] == n_cols // 2


def test_azimuth_wraps_correctly_near_pi_seam():
    n_cols = 360
    col_pos = azimuth_to_col(np.array([np.pi - 1e-6]), n_cols)
    col_neg = azimuth_to_col(np.array([-np.pi + 1e-6]), n_cols)
    # both must land adjacent to column 0 (the wrap seam), not far apart
    assert min(col_pos[0], n_cols - col_pos[0]) <= 1
    assert min(col_neg[0], n_cols - col_neg[0]) <= 1


# ---------------------------------------------------------------------------
# to_range_image: shape, NaN handling, channels
# ---------------------------------------------------------------------------


def test_to_range_image_shape_and_row_axis():
    sensor = Vlp32cSensor()
    points = np.array([[3.0, 0.0, 0.0]])
    image, grid = to_range_image(points, sensor, azimuth_steps=180)
    assert image.shape == (N_LASERS, 180, 1)
    assert grid.n_rows == N_LASERS
    assert np.array_equal(grid.row_elevations, sensor.elevations)


def test_to_range_image_empty_points_is_all_nan():
    sensor = Vlp32cSensor()
    image, _ = to_range_image(np.zeros((0, 3)), sensor, azimuth_steps=90)
    assert np.all(np.isnan(image))


def test_to_range_image_two_channel_discontinuity():
    sensor = Vlp32cSensor()
    row = 10
    elev = sensor.elevations[row]
    step = 2.0 * np.pi / 360
    # two adjacent columns, near vs far range -> a discontinuity spike
    near = np.array([2.0 * np.cos(elev), 0.0, 2.0 * np.sin(elev)])
    far_az = step  # one column over from straight-ahead
    far = np.array([8.0 * np.cos(elev) * np.cos(far_az),
                    8.0 * np.cos(elev) * np.sin(far_az),
                    8.0 * np.sin(elev)])
    image, grid = to_range_image(np.stack([near, far]), sensor,
                                 azimuth_steps=360,
                                 channels=("range", "discontinuity"))
    c_near = azimuth_to_col(np.array([0.0]), 360)[0]
    c_far = azimuth_to_col(np.array([far_az]), 360)[0]
    assert np.isclose(image[row, c_near, 0], 2.0, atol=1e-6)
    assert np.isclose(image[row, c_far, 0], 8.0, atol=1e-6)
    if c_far == c_near + 1:
        assert np.isclose(image[row, c_far, 1], 6.0, atol=1e-6)


def test_to_range_image_nearest_return_wins_per_cell():
    sensor = Vlp32cSensor()
    row = 5
    elev = sensor.elevations[row]
    near = np.array([2.0 * np.cos(elev), 0.0, 2.0 * np.sin(elev)])
    far = np.array([9.0 * np.cos(elev), 0.0, 9.0 * np.sin(elev)])
    image, _ = to_range_image(np.stack([far, near]), sensor, azimuth_steps=360)
    col = azimuth_to_col(np.array([0.0]), 360)[0]
    assert np.isclose(image[row, col, 0], 2.0, atol=1e-6)


# ---------------------------------------------------------------------------
# sim + real share the row axis (the must-fix linchpin)
# ---------------------------------------------------------------------------


def test_sim_and_arbitrary_points_share_identical_row_axis():
    """A sim scene and a real point cloud, run through the SAME renderer,
    must produce identical channel-elevation row axes -- not just the same
    row COUNT, but the exact same angles, since they use the same sensor."""
    sensor = Vlp32cSensor()
    sim_points = np.array([[3.0, 0.5, 0.1]])
    real_like_points = np.array([[5.0, -1.0, 0.3], [2.0, 0.2, -0.2]])
    _, grid_sim = to_range_image(sim_points, sensor, azimuth_steps=200)
    _, grid_real = to_range_image(real_like_points, sensor, azimuth_steps=200)
    assert grid_sim.n_rows == grid_real.n_rows == N_LASERS
    assert np.array_equal(grid_sim.row_elevations, grid_real.row_elevations)


def test_rebinning_simframe_points_reproduces_exact_row_assignment():
    """The load-bearing guarantee: a ray's true elevation is EXACTLY
    `sensor.elevations[row]` by construction, so re-deriving row from a
    rendered SimFrame's own 3D points (as a real Frame.xyz-consumer would
    have to) must recover the identical row for every single point --
    zero ambiguity, not an approximation."""
    sensor = Vlp32cSensor()
    board = Rect(center=(3.0, 0.0, 0.2), normal=(-1.0, 0.1, 0.05),
                u_axis=(0.0, 1.0, 0.0), half_u=0.5, half_v=0.5)
    ground = Rect(center=(0.0, 0.0, -1.1), normal=(0.0, 0.0, 1.0),
                 u_axis=(1.0, 0.0, 0.0), half_u=20.0, half_v=20.0)
    frame = render([ground, board], sensor, azimuth_step_deg=0.25,
                  rng=np.random.default_rng(0))
    rows_rebinned, _, _ = points_to_rows_cols_ranges(frame.points, sensor,
                                                      frame.n_cols)
    assert np.array_equal(rows_rebinned, frame.rows)


def test_rebinning_simframe_column_shift_is_constant_per_row():
    """Columns DO differ from SimFrame.cols (see module docstring): each
    ray's true azimuth includes its laser's own rot_correction offset,
    which this renderer bakes into the point geometry but SimFrame.cols
    (nominal firing index) does not. That shift must be a single constant
    per row (same laser -> same offset for every column), never a
    per-ray shuffle -- otherwise the "shared renderer" would be
    introducing aliasing of its own."""
    sensor = Vlp32cSensor()
    board = Rect(center=(3.0, 0.0, 0.2), normal=(-1.0, 0.1, 0.05),
                u_axis=(0.0, 1.0, 0.0), half_u=0.5, half_v=0.5)
    ground = Rect(center=(0.0, 0.0, -1.1), normal=(0.0, 0.0, 1.0),
                 u_axis=(1.0, 0.0, 0.0), half_u=20.0, half_v=20.0)
    frame = render([ground, board], sensor, azimuth_step_deg=0.25,
                  rng=np.random.default_rng(0))
    _, cols_rebinned, _ = points_to_rows_cols_ranges(frame.points, sensor,
                                                      frame.n_cols)
    n = frame.n_cols
    diff = (cols_rebinned - frame.cols + n // 2) % n - n // 2
    for row in np.unique(frame.rows):
        mask = frame.rows == row
        assert len(np.unique(diff[mask])) == 1, (
            f"row {row}'s column shift is not constant: {np.unique(diff[mask])}"
        )
    # bounded by the sensor's max azimuth offset / column step
    max_offset = np.abs(sensor.az_offsets).max()
    step = 2.0 * np.pi / n
    assert np.abs(diff).max() <= np.ceil(max_offset / step) + 1


def test_sim_frame_to_range_image_matches_simframe_range_image_up_to_row_shift():
    """Per-row: the shared renderer's range values for a row equal
    SimFrame.range_image's row values, just circularly rolled by that
    row's constant column shift (see
    test_rebinning_simframe_column_shift_is_constant_per_row) -- i.e. the
    two really do "agree", modulo the documented, bounded, per-row-constant
    azimuth-offset effect."""
    sensor = Vlp32cSensor()
    board = Rect(center=(3.0, 0.0, 0.2), normal=(-1.0, 0.1, 0.05),
                u_axis=(0.0, 1.0, 0.0), half_u=0.5, half_v=0.5)
    ground = Rect(center=(0.0, 0.0, -1.1), normal=(0.0, 0.0, 1.0),
                 u_axis=(1.0, 0.0, 0.0), half_u=20.0, half_v=20.0)
    frame = render([ground, board], sensor, azimuth_step_deg=0.25,
                  rng=np.random.default_rng(0))
    image, grid = sim_frame_to_range_image(frame, sensor)
    assert grid.n_cols == frame.n_cols
    assert np.array_equal(grid.row_elevations, sensor.elevations)

    n = frame.n_cols
    _, cols_rebinned, _ = points_to_rows_cols_ranges(frame.points, sensor, n)
    diff = (cols_rebinned - frame.cols + n // 2) % n - n // 2

    checked_rows = 0
    for row in np.unique(frame.rows):
        mask = frame.rows == row
        shift = int(diff[mask][0])  # constant within the row (asserted elsewhere)
        native_row = frame.range_image[row]
        shared_row = image[row, :, 0]
        rolled = np.roll(shared_row, -shift)
        valid = ~np.isnan(native_row) & ~np.isnan(rolled)
        assert valid.sum() > 0
        assert np.allclose(native_row[valid], rolled[valid], atol=1e-3)
        checked_rows += 1
    assert checked_rows > 0
