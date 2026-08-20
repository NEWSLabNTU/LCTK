"""`board_geometry` is the camera side of a cross-language contract.

The Rust `hollow-board-config` crate and this module both answer "where are the ArUco
marker corners on the board?", and the published board pose (`T_sensor<-board`) puts
that answer on both sides of one product. A disagreement is therefore partly *silent*:
the 2x2 grid is symmetric, so an in-plane 45-degree error still solves cleanly with a
low reprojection error.

These tests pin the properties that make the contract checkable at all. The contract
itself — the world positions — is asserted in `test_marker_corners_world_golden.py`.
"""

import subprocess
import sys

import pytest

BOARD_SIZE_MM = "500mm"


def test_module_imports_without_rclpy():
    """The geometry must be testable without a ROS graph.

    If `board_geometry` ever grows an `rclpy` import, every assertion about the board's
    frame convention starts requiring a running node, and in practice stops being run.
    """
    # A fresh interpreter: a sibling test in this same session imports the node, which
    # would leave `rclpy` in this process's `sys.modules` regardless.
    probe = (
        "import sys;"
        "import lidar_to_camera_solver.board_geometry as g;"
        "print(f'LCTK_BOARD_GEOMETRY_RCLPY={\"rclpy\" in sys.modules}')"
    )
    result = subprocess.run(
        [sys.executable, "-c", probe], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    sentinel_lines = [
        line
        for line in result.stdout.splitlines()
        if line.startswith("LCTK_BOARD_GEOMETRY_RCLPY=")
    ]
    assert sentinel_lines == ["LCTK_BOARD_GEOMETRY_RCLPY=False"], (
        "importing board_geometry pulled in rclpy"
    )


@pytest.mark.parametrize(
    "text,meters",
    [
        ("500mm", 0.5),
        ("10mm", 0.01),
        ("1.5m", 1.5),
        ("-353.5533905932738mm", -0.3535533905932738),
        ("0.25", 0.25),
    ],
)
def test_parse_dimension_reads_the_configs_units(text, meters):
    """`aruco_pattern.json5` states lengths as strings; a mis-parse silently rescales
    the whole board. Negative values matter: `paper_placement` uses one."""
    from lidar_to_camera_solver.board_geometry import parse_dimension

    assert parse_dimension(text) == pytest.approx(meters, abs=1e-12)


def test_fewer_than_four_marker_ids_is_a_clear_error():
    """M-09: the 2x2 layout indexes marker_ids[0..3]."""
    from lidar_to_camera_solver.board_geometry import compute_multi_marker_corners

    config = {
        "board_size": BOARD_SIZE_MM,
        "board_border_size": "10mm",
        "marker_square_size_ratio": 0.8,
        "num_squares_per_side": 2,
        "marker_ids": [1, 2, 3],
    }
    with pytest.raises(ValueError, match="at least 4 marker_ids"):
        compute_multi_marker_corners(config)
