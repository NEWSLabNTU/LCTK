"""Camera-side evidence for target-owned marker corner geometry."""

import json
from pathlib import Path

import numpy as np
import pytest
from lctk_target import load_target

ROOT = Path(__file__).resolve().parents[3]
TARGET_FIXTURES = ROOT / "fixtures" / "targets"
GOLDEN_PATH = TARGET_FIXTURES / "marker_corners_world.golden.json"
TARGETS = ROOT / "ros" / "lctk_launch" / "config" / "targets"
CORNER_NAMES = ["right", "top", "left", "bottom"]
TOL_M = 1e-9


@pytest.fixture(scope="module")
def golden():
    return json.loads(GOLDEN_PATH.read_text(encoding="utf-8"))


def board_to_world(golden):
    rotation = np.column_stack(
        [
            golden["mounting"]["local_x_toward_left"],
            golden["mounting"]["local_y_toward_top"],
            golden["mounting"]["local_z_normal"],
        ]
    )
    return rotation, np.asarray(golden["mounting"]["plate_center"], dtype=np.float64)


@pytest.mark.parametrize("target_name", ["solid_600_aruco_1", "hollow_1000_aruco_4"])
def test_target_marker_corners_match_shared_world_golden(golden, target_name):
    target = load_target(TARGETS / f"{target_name}_v1.json5")
    rotation, translation = board_to_world(golden)
    expected_target = golden["targets"][target_name]

    assert list(target.marker_corners_by_id) == expected_target["marker_ids"]
    for marker_id, expected_corners in expected_target["markers"].items():
        actual_corners = [
            rotation @ np.asarray(corner, dtype=np.float64) + translation
            for corner in target.marker_corners_by_id[int(marker_id)]
        ]
        for name, actual, expected in zip(
            CORNER_NAMES, actual_corners, expected_corners
        ):
            error = float(np.linalg.norm(actual - np.asarray(expected)))
            assert error < TOL_M, (
                f"{target_name} marker {marker_id} {name}: error {error:e} m"
            )


def test_solid_marker_is_exactly_480_mm():
    target = load_target(TARGETS / "solid_600_aruco_1_v1.json5")
    assert (
        target.fiducial.paper_side_um - 2 * target.fiducial.outer_border_um == 480_000
    )
    assert target.fiducial.marker_ids == (24,)
    assert len(target.marker_corners_by_id[24]) == 4


def test_hollow_manifest_preserves_four_marker_order():
    target = load_target(TARGETS / "hollow_1000_aruco_4_v1.json5")
    assert tuple(target.marker_corners_by_id) == (696, 64, 306, 195)
    assert all(len(corners) == 4 for corners in target.marker_corners_by_id.values())
