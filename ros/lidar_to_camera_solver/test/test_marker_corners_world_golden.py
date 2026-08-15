"""The cross-language contract: where the ArUco marker corners are, in the world.

`board_geometry.compute_multi_marker_corners` and the Rust
`BoardModel::multi_marker_corners` must agree corner-for-corner, because the published
board pose is ``T_sensor<-board`` and the solver feeds board-local marker coordinates
into it. The convention therefore appears on both sides of one product, and half of a
disagreement is *silent*: a 45-degree in-plane rotation leaves the reprojection error
low, because the 2x2 marker grid is symmetric.

`fixtures/board/marker_corners_world.golden.json` is that contract. It is the same file
`rust/hollow-board-config/tests/marker_layout_golden.rs` asserts against, it is keyed by
ArUco marker **id** — the binding whose corruption produces the silent quarter-turn —
and it states **world** positions at a stated physical mounting.

World, not board-local, for a reason that is the whole point of this test: the board
model's local frame was redefined (corner-aligned, origin at the plate centre), so local
coordinates change by construction and cannot pin anything across that change. The
physical board did not move. This fixture must pass **byte-identical before and after**
the port; re-baselining it would verify nothing.
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest
from lidar_to_camera_solver.board_geometry import compute_multi_marker_corners

# Positional tolerance, metres. The fixture carries exact decimal geometry, so the only
# error in play is f64 round-off through a rotation and a translation.
TOL_M = 1e-9

GOLDEN_PATH = (
    Path(__file__).resolve().parents[3]
    / "fixtures"
    / "board"
    / "marker_corners_world.golden.json"
)

CORNER_NAMES = ["right", "top", "left", "bottom"]


@pytest.fixture(scope="module")
def golden():
    return json.loads(GOLDEN_PATH.read_text())


def mounting_axes(golden):
    """The stated physical mounting, as the board frame's three axis directions.

    Same handedness rule the Rust test uses: (left, up, normal) is right-handed, and
    the board frame's columns are exactly [+X toward the left corner, +Y toward the top
    corner, +Z along the normal].
    """
    normal = np.array(golden["mounting"]["normal"], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    up = np.array(golden["mounting"]["up_diagonal"], dtype=np.float64)
    up /= np.linalg.norm(up)
    left = np.cross(up, normal)
    left /= np.linalg.norm(left)
    return left, up, normal


def board_to_world(golden):
    """Rotation and translation taking board-local coordinates to world."""
    left, up, normal = mounting_axes(golden)
    rotation = np.column_stack([left, up, normal])
    translation = np.array(golden["mounting"]["plate_center"], dtype=np.float64)
    return rotation, translation


def pattern_config(golden, toward_left_mm=0.0, toward_top_mm=-353.5533905932738):
    """The printed pattern exactly as `config/aruco/aruco_pattern.json5` states it.

    The default placement is that file's measured value: the paper sits in the plate's
    lower quarter with its top corner at the plate centre. It is quoted here as the
    config states it rather than derived, because it is a measurement.
    """
    return {
        "marker_ids": golden["pattern"]["marker_ids"],
        "board_size": f"{golden['marker_paper_size_m'] * 1000.0}mm",
        "board_border_size": f"{golden['pattern']['board_border_size_m'] * 1000.0}mm",
        "num_squares_per_side": golden["pattern"]["num_squares_per_side"],
        "marker_square_size_ratio": golden["pattern"]["marker_square_size_ratio"],
        "paper_placement": {
            "toward_left_corner": f"{toward_left_mm}mm",
            "toward_top_corner": f"{toward_top_mm}mm",
        },
    }


def world_corners(golden, config):
    rotation, translation = board_to_world(golden)
    local = compute_multi_marker_corners(config)
    return {
        marker_id: [
            rotation @ np.asarray(c, dtype=np.float64) + translation for c in corners
        ]
        for marker_id, corners in local.items()
    }


def test_marker_corners_match_the_world_golden_keyed_by_marker_id(golden):
    config = pattern_config(golden)
    computed = world_corners(golden, config)

    assert sorted(computed) == sorted(int(k) for k in golden["markers"]), (
        "one corner set per marker id"
    )

    for marker_id, expected_corners in golden["markers"].items():
        actual_corners = computed[int(marker_id)]
        assert len(actual_corners) == len(expected_corners)
        for name, actual, expected in zip(
            CORNER_NAMES, actual_corners, expected_corners
        ):
            error = float(np.linalg.norm(actual - np.asarray(expected)))
            assert error < TOL_M, (
                f"marker {marker_id} {name}: got {actual.tolist()}, "
                f"expected {expected} (error {error:e} m)"
            )


def test_the_plate_centre_is_the_board_frames_origin(golden):
    """The corner-aligned frame puts the origin at the plate centre, so the marker
    paper's centre sits at the stated placement offset from the origin — not ~707 mm
    away at a plate corner, which is what the edge-aligned frame produced."""
    config = pattern_config(golden)
    local = compute_multi_marker_corners(config)

    all_corners = np.array([c for corners in local.values() for c in corners])
    paper_centre = all_corners.mean(axis=0)

    assert paper_centre[0] == pytest.approx(0.0, abs=1e-9)
    assert paper_centre[1] == pytest.approx(-0.3535533905932738, abs=1e-9)
    assert paper_centre[2] == pytest.approx(0.0, abs=1e-12)


def test_marker_corners_follow_the_stated_paper_placement(golden):
    """The paper's position on the plate is a **measurement**, not a derivation from the
    plate's width. Sliding the stated placement must slide every marker corner by
    exactly the same world vector — otherwise the field could be plumbed through and
    silently ignored, which is the failure mode that leaves Rust and Python disagreeing
    while every other test still passes.
    """
    shift_left = 0.03
    shift_up = 0.1

    baseline = world_corners(golden, pattern_config(golden))
    shifted = world_corners(
        golden,
        pattern_config(
            golden,
            toward_left_mm=shift_left * 1000.0,
            toward_top_mm=-353.5533905932738 + shift_up * 1000.0,
        ),
    )

    left, up, _ = mounting_axes(golden)
    expected_shift = up * shift_up + left * shift_left

    for marker_id, was_corners in baseline.items():
        for name, was, now in zip(CORNER_NAMES, was_corners, shifted[marker_id]):
            error = float(np.linalg.norm(now - (was + expected_shift)))
            assert error < TOL_M, (
                f"marker {marker_id} {name} after sliding the paper: "
                f"moved by {(now - was).tolist()}, expected {expected_shift.tolist()}"
            )


def test_the_markers_lie_on_the_paper_diagonal_to_the_plate(golden):
    """Convention sensitivity: the paper's edges run at 45 degrees to the board frame's
    axes, because the plate is hung as a diamond and the sheet is glued square to its
    edges. A test built only from rotation-invariant quantities (inter-corner distances,
    dot products, reprojection residuals) cannot see an in-plane relabelling at all,
    which is exactly the defect this whole phase is about.
    """
    config = pattern_config(golden)
    local = compute_multi_marker_corners(config)

    corners = local[golden["pattern"]["marker_ids"][0]]
    bottom = np.asarray(corners[3])
    left = np.asarray(corners[2])

    edge = left - bottom
    assert edge[2] == pytest.approx(0.0, abs=1e-12), "the paper is flat on the plate"
    # A paper edge runs along a bisector of the board frame's axes: |x| == |y|.
    assert abs(edge[0]) == pytest.approx(abs(edge[1]), abs=1e-12)
    assert abs(edge[0]) == pytest.approx(
        np.linalg.norm(edge) / math.sqrt(2.0), abs=1e-12
    )
