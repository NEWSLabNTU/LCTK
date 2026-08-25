"""Static contract tests for Phase-8 target manifests and detector presets."""

import runpy
from pathlib import Path
from unittest.mock import patch

import json5
import pytest

CONFIG = Path(__file__).parent.parent / "config"
PACKAGE_ROOT = CONFIG.parent
TARGETS = CONFIG / "targets"
BOARD = CONFIG / "board"

PRESETS = {
    "hollow_1000": {"velodyne", "seyond"},
    "solid_600": {"velodyne", "seyond"},
}
REMOVED_PHYSICAL_KEYS = {
    "board_width",
    "hole_radius",
    "hole_center_shift",
    "side_m",
}
REQUIRED_TUNING_KEYS = {
    "detection_mode",
    "foreground_method",
    "cluster_eps",
    "cluster_min_points",
    "up_axis",
    "stance_floor",
    "square_icp_residual_max",
    "sensor_up_axis",
}
TARGET_KEYS = {
    "schema_version",
    "target_id",
    "revision",
    "board_frame_convention",
    "plate",
    "fiducial",
    "lidar_orientation_reference",
}
FIDUCIAL_KEYS = {
    "kind",
    "dictionary",
    "marker_ids",
    "paper_side",
    "paper_center",
    "outer_border",
    "cells_per_side",
    "marker_fill_ratio",
    "border_bits",
}


def load_json5(path: Path):
    with path.open(encoding="utf-8") as source:
        return json5.load(source)


@pytest.mark.parametrize(
    ("filename", "target_id", "surface_keys", "orientation_keys"),
    [
        (
            "hollow_1000_aruco_4_v1.json5",
            "hollow_1000_aruco_4",
            {"kind", "circular_cutouts"},
            {"kind"},
        ),
        (
            "solid_600_aruco_1_v1.json5",
            "solid_600_aruco_1",
            {"kind"},
            {"kind", "local_axis"},
        ),
    ],
)
def test_target_manifest_has_exact_schema(
    filename, target_id, surface_keys, orientation_keys
):
    manifest = load_json5(TARGETS / filename)

    assert set(manifest) == TARGET_KEYS
    assert manifest["schema_version"] == 1
    assert manifest["revision"] == 1
    assert manifest["target_id"] == target_id
    assert manifest["board_frame_convention"] == "corner_aligned_plate_center_v1"
    assert set(manifest["plate"]) == {"side", "surface"}
    assert set(manifest["plate"]["surface"]) == surface_keys
    assert set(manifest["fiducial"]) == FIDUCIAL_KEYS
    assert set(manifest["fiducial"]["paper_center"]) == {
        "toward_left_corner",
        "toward_top_corner",
    }
    assert set(manifest["lidar_orientation_reference"]) == orientation_keys

    cutouts = manifest["plate"]["surface"].get("circular_cutouts", [])
    for cutout in cutouts:
        assert set(cutout) == {"center", "radius"}
        assert set(cutout["center"]) == {"x", "y"}


def test_setup_installs_target_manifests_and_nested_presets(monkeypatch):
    monkeypatch.chdir(PACKAGE_ROOT)
    with patch("setuptools.setup"):
        setup_module = runpy.run_path(str(PACKAGE_ROOT / "setup.py"))

    installed = {
        (Path(destination) / Path(source).name).as_posix()
        for destination, sources in setup_module["get_data_files"]()
        for source in sources
    }
    expected = {
        "share/lctk_launch/config/targets/hollow_1000_aruco_4_v1.json5",
        "share/lctk_launch/config/targets/solid_600_aruco_1_v1.json5",
        "share/lctk_launch/config/board/hollow_1000/velodyne.json5",
        "share/lctk_launch/config/board/hollow_1000/seyond.json5",
        "share/lctk_launch/config/board/solid_600/velodyne.json5",
        "share/lctk_launch/config/board/solid_600/seyond.json5",
    }
    assert expected <= installed


def test_target_preset_matrix_is_complete_and_geometry_free():
    for target_directory, sensors in PRESETS.items():
        found = {path.stem for path in (BOARD / target_directory).glob("*.json5")}
        assert found == sensors

        for sensor in sensors:
            path = BOARD / target_directory / f"{sensor}.json5"
            preset = load_json5(path)
            assert REMOVED_PHYSICAL_KEYS.isdisjoint(preset)
            assert REQUIRED_TUNING_KEYS <= preset.keys()
            assert preset["detection_mode"] == "bbox_free"


def test_solid_presets_are_explicitly_experimental():
    for sensor in PRESETS["solid_600"]:
        text = (BOARD / "solid_600" / f"{sensor}.json5").read_text(encoding="utf-8")
        assert "EXPERIMENTAL" in text
        assert "field-validated" in text


def test_hollow_presets_preserve_the_current_sensor_operating_values():
    legacy = {
        "velodyne": load_json5(BOARD / "board_detector_velodyne.json5"),
        "seyond": load_json5(BOARD / "board_detector_seyond.json5"),
    }
    for sensor, expected in legacy.items():
        preset = load_json5(BOARD / "hollow_1000" / f"{sensor}.json5")
        for key, value in expected.items():
            if key not in REMOVED_PHYSICAL_KEYS:
                assert preset[key] == value
