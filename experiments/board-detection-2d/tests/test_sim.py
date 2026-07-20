"""Tests for the ray-based VLP-32C simulator (Task 29): primitives,
sensor beam model, and the render()/nearest-hit resolve -- including the
Gate-2 fidelity re-test (see stage9-cnn-spike.md / task-29-brief.md)."""
from __future__ import annotations

import numpy as np
import pytest

from boarddet.sim.primitives import Box, Cylinder, Rect
from boarddet.sim.raycast import render
from boarddet.sim.sensor import N_LASERS, Vlp32cSensor

# ---------------------------------------------------------------------------
# Rect
# ---------------------------------------------------------------------------


def test_rect_ray_hits_at_known_distance():
    rect = Rect(center=(5.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
               u_axis=(0.0, 1.0, 0.0), half_u=1.0, half_v=1.0)
    origins = np.array([[0.0, 0.0, 0.0]])
    dirs = np.array([[1.0, 0.0, 0.0]])
    t, cosang = rect.intersect(origins, dirs)
    assert np.isclose(t[0], 5.0)
    assert np.isclose(cosang[0], 1.0)  # straight-on


def test_rect_ray_outside_bounds_misses():
    rect = Rect(center=(5.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
               u_axis=(0.0, 1.0, 0.0), half_u=1.0, half_v=1.0)
    origins = np.array([[0.0, 5.0, 0.0]])  # aimed well outside the rect's y extent
    dirs = np.array([[1.0, 0.0, 0.0]])
    t, _ = rect.intersect(origins, dirs)
    assert np.isinf(t[0])


def test_rect_ray_through_hole_misses():
    rect = Rect(center=(5.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
               u_axis=(0.0, 1.0, 0.0), half_u=1.0, half_v=1.0,
               holes=[((0.0, 0.0), 0.3)])
    origins = np.array([[0.0, 0.0, 0.0]])
    dirs = np.array([[1.0, 0.0, 0.0]])  # aimed dead-center at the hole
    t, _ = rect.intersect(origins, dirs)
    assert np.isinf(t[0])
    # just outside the hole radius still hits the rect
    origins2 = np.array([[0.0, 0.5, 0.0]])
    t2, _ = rect.intersect(origins2, dirs)
    assert np.isclose(t2[0], 5.0)


def test_rect_ray_parallel_to_plane_misses():
    rect = Rect(center=(5.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
               u_axis=(0.0, 1.0, 0.0), half_u=1.0, half_v=1.0)
    origins = np.array([[2.0, 0.0, 0.0]])
    dirs = np.array([[0.0, 1.0, 0.0]])  # runs along the plane, never reaches it
    t, _ = rect.intersect(origins, dirs)
    assert np.isinf(t[0])


def test_rect_ray_behind_origin_misses():
    """A rect behind the ray origin (t < 0) must not register as a hit."""
    rect = Rect(center=(-5.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
               u_axis=(0.0, 1.0, 0.0), half_u=1.0, half_v=1.0)
    origins = np.array([[0.0, 0.0, 0.0]])
    dirs = np.array([[1.0, 0.0, 0.0]])
    t, _ = rect.intersect(origins, dirs)
    assert np.isinf(t[0])


# ---------------------------------------------------------------------------
# Box
# ---------------------------------------------------------------------------


def test_box_ray_hits_near_face():
    box = Box(center=(5.0, 0.0, 0.0), R=np.eye(3), half_sizes=(1.0, 1.0, 1.0))
    origins = np.array([[0.0, 0.0, 0.0]])
    dirs = np.array([[1.0, 0.0, 0.0]])
    t, cosang = box.intersect(origins, dirs)
    assert np.isclose(t[0], 4.0)  # enters at x=4 (5 - half_size)
    assert np.isclose(cosang[0], 1.0)


def test_box_ray_missing_extent_misses():
    box = Box(center=(5.0, 0.0, 0.0), R=np.eye(3), half_sizes=(1.0, 1.0, 1.0))
    origins = np.array([[0.0, 0.0, 0.0]])
    dirs = np.array([[0.0, 0.0, 1.0]])  # vertical: x stays 0, never enters [4,6]
    t, _ = box.intersect(origins, dirs)
    assert np.isinf(t[0])


def test_box_ray_rotated():
    """A box rotated 45 deg about z; ray still hits its near corner-on face."""
    theta = np.pi / 4
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    box = Box(center=(5.0, 0.0, 0.0), R=R, half_sizes=(1.0, 1.0, 1.0))
    origins = np.array([[0.0, 0.0, 0.0]])
    dirs = np.array([[1.0, 0.0, 0.0]])
    t, _ = box.intersect(origins, dirs)
    assert np.isfinite(t[0])
    assert t[0] < 5.0


# ---------------------------------------------------------------------------
# Cylinder
# ---------------------------------------------------------------------------


def test_cylinder_ray_hits_side():
    cyl = Cylinder(base=(5.0, -1.0, 0.0), axis=(0.0, 0.0, 1.0),
                   radius=0.3, height=2.0)
    origins = np.array([[0.0, -1.0, 0.0]])
    dirs = np.array([[1.0, 0.0, 0.0]])
    t, cosang = cyl.intersect(origins, dirs)
    assert np.isclose(t[0], 4.7, atol=1e-6)
    assert np.isclose(cosang[0], 1.0, atol=1e-6)  # radial-on hit


def test_cylinder_ray_above_height_misses():
    cyl = Cylinder(base=(5.0, -1.0, 0.0), axis=(0.0, 0.0, 1.0),
                   radius=0.3, height=2.0)
    origins = np.array([[0.0, -1.0, 3.0]])  # above the pole's top (z in [0,2])
    dirs = np.array([[1.0, 0.0, 0.0]])
    t, _ = cyl.intersect(origins, dirs)
    assert np.isinf(t[0])


def test_cylinder_ray_missing_radius():
    cyl = Cylinder(base=(5.0, -1.0, 0.0), axis=(0.0, 0.0, 1.0),
                   radius=0.3, height=2.0)
    origins = np.array([[0.0, -5.0, 1.0]])  # far off to the side
    dirs = np.array([[1.0, 0.0, 0.0]])
    t, _ = cyl.intersect(origins, dirs)
    assert np.isinf(t[0])


# ---------------------------------------------------------------------------
# nearest-hit (render's resolve across multiple primitives)
# ---------------------------------------------------------------------------


def test_render_nearest_hit_wins_over_farther_occluded_primitive():
    sensor = Vlp32cSensor()
    row0 = int(np.argmin(np.abs(sensor.elevations)))  # the near-horizontal beam
    near = Rect(center=(2.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
               u_axis=(0.0, 1.0, 0.0), half_u=2.0, half_v=2.0)
    far = Rect(center=(10.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
              u_axis=(0.0, 1.0, 0.0), half_u=5.0, half_v=5.0)
    frame = render([near, far], sensor, azimuth_steps=360)
    col0 = int(np.argmin(np.abs(frame.azimuths)))  # the straight-ahead (+x) column
    value = frame.range_image[row0, col0]
    assert np.isfinite(value)
    assert value < 3.0  # picked the near rect, not the far (occluded) one


# ---------------------------------------------------------------------------
# sensor
# ---------------------------------------------------------------------------


def test_sensor_beam_directions_are_unit_vectors():
    sensor = Vlp32cSensor()
    grid = sensor.beam_directions(azimuth_steps=180)
    norms = np.linalg.norm(grid.directions, axis=1)
    assert np.allclose(norms, 1.0, atol=1e-9)


def test_sensor_has_32_distinct_elevations_sorted_ascending():
    sensor = Vlp32cSensor()
    assert len(sensor.elevations) == N_LASERS
    assert len(set(np.round(sensor.elevations, 8))) == N_LASERS
    assert np.all(np.diff(sensor.elevations) > 0)


def test_sensor_beam_grid_rows_match_sorted_elevation_rank():
    sensor = Vlp32cSensor()
    grid = sensor.beam_directions(azimuth_steps=90)
    assert grid.n_rows == N_LASERS
    assert np.array_equal(grid.row_elevations, sensor.elevations)
    # elevation actually realized by a ray must match its row's nominal elevation
    elev_realized = np.arcsin(grid.directions[:, 2])
    assert np.allclose(elev_realized, sensor.elevations[grid.rows], atol=1e-9)


# ---------------------------------------------------------------------------
# render: single board, noise-free unprojection + dropout monotonicity
# ---------------------------------------------------------------------------


def _board_scene():
    board = Rect(center=(3.0, 0.0, 0.2), normal=(-1.0, 0.1, 0.05),
                u_axis=(0.0, 1.0, 0.0), half_u=0.5, half_v=0.5)
    ground = Rect(center=(0.0, 0.0, -1.1), normal=(0.0, 0.0, 1.0),
                 u_axis=(1.0, 0.0, 0.0), half_u=20.0, half_v=20.0)
    return [ground, board], board


def test_render_single_board_gives_coherent_region_and_exact_unprojection():
    sensor = Vlp32cSensor()
    scene, board = _board_scene()
    board_idx = len(scene) - 1
    frame = render(scene, sensor, azimuth_step_deg=0.25)

    board_mask = frame.hit_prim_id == board_idx
    assert board_mask.sum() > 50  # a coherent filled region, not a scatter

    board_pts = frame.points[board_mask]
    d = np.abs((board_pts - np.asarray(board.center)) @ board.normal)
    assert d.max() < 1e-4  # zero-noise render: exact ray-plane intersection


def test_render_dropout_reduces_point_count_monotonically():
    sensor = Vlp32cSensor()
    scene, _ = _board_scene()
    counts = []
    for rate in (0.0, 0.3, 0.7, 0.95):
        frame = render(scene, sensor, azimuth_step_deg=0.5,
                       dropout_random=rate,
                       rng=np.random.default_rng(42))
        counts.append(len(frame.points))
    assert counts == sorted(counts, reverse=True)
    assert counts[-1] < counts[0]


def test_render_min_max_range_clips_out_of_window_hits():
    sensor = Vlp32cSensor(min_range=0.9, max_range=5.0)
    far_rect = Rect(center=(50.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
                    u_axis=(0.0, 1.0, 0.0), half_u=10.0, half_v=10.0)
    frame = render([far_rect], sensor, azimuth_steps=360)
    assert len(frame.points) == 0


# ---------------------------------------------------------------------------
# Gate-2 fidelity re-test
# ---------------------------------------------------------------------------


def _gate2_scene():
    """Approximates real dataset ds3 (see stage9-cnn-spike.md): board at the
    real measured bbox location, a ground plane, a back wall, and clutter."""
    board = Rect(center=(2.6, -0.2, 0.35), normal=(-1.0, 0.1, 0.05),
                u_axis=(0.0, 1.0, 0.0), half_u=0.5, half_v=0.5)
    ground = Rect(center=(0.0, 0.0, -1.1), normal=(0.0, 0.0, 1.0),
                 u_axis=(1.0, 0.0, 0.0), half_u=20.0, half_v=20.0)
    wall = Rect(center=(8.0, 0.0, 0.5), normal=(-1.0, 0.0, 0.0),
               u_axis=(0.0, 1.0, 0.0), half_u=6.0, half_v=2.5)
    clutter1 = Box(center=(3.0, -2.0, -0.5), R=np.eye(3),
                   half_sizes=(0.3, 0.3, 0.6))
    clutter2 = Cylinder(base=(4.0, 1.5, -1.1), axis=(0.0, 0.0, 1.0),
                        radius=0.15, height=1.3)
    scene = [ground, wall, clutter1, clutter2, board]
    return scene, len(scene) - 1


def test_gate2_board_footprint_and_no_column_aliasing():
    sensor = Vlp32cSensor()
    scene, board_idx = _gate2_scene()
    frame = render(scene, sensor, azimuth_step_deg=0.25,
                  range_noise_std=0.01, dropout_grazing=0.1,
                  dropout_random=0.01, rng=np.random.default_rng(0))

    mask = frame.hit_prim_id == board_idx
    assert mask.sum() > 200
    rows, cols = frame.rows[mask], frame.cols[mask]
    row_span = int(rows.max() - rows.min() + 1)
    # spike's real-frame measurement: 8/8 frames landed on exactly 21 rows
    # (see stage9-cnn-spike.md); this asserts the sim lands in that
    # neighbourhood, not the aliasing failure mode's degenerate footprint.
    assert 15 <= row_span <= 25, f"board row footprint {row_span} outside ~15-25"

    r0, r1, c0, c1 = rows.min(), rows.max(), cols.min(), cols.max()
    region = frame.range_image[r0:r1 + 1, c0:c1 + 1]
    empty_col_frac = np.isnan(region).all(axis=0).mean()
    # the aliasing failure mode (synth.py's object-space grid sampling)
    # produces a striping pattern with ~50% fully-empty columns in exactly
    # this kind of window; a ray-per-pixel render should show essentially none.
    assert empty_col_frac < 0.1, (
        f"empty-column fraction {empty_col_frac:.3f} in the board region "
        "looks like vertical-stripe aliasing, not a coherent ray-cast region"
    )
