#!/usr/bin/env python3
"""Test script for calibration config parser.

Usage:
    # Requires ROS environment to be sourced
    source install/setup.bash
    python3 ros/lctk_launch/test/test_config_parser.py
"""

import sys
from pathlib import Path

import pytest

# Add the package to path for standalone testing
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from lctk_launch.config_parser import CalibrationConfigParser, parse_config
    from lctk_launch.session import SessionError
except ImportError as e:
    print(f"Error: {e}")
    print()
    print("This test requires the ROS environment to be sourced:")
    print("  source install/setup.bash")
    print("  python3 ros/lctk_launch/test/test_config_parser.py")
    sys.exit(1)


def test_sample_data_config():
    """Test parsing the sample_data.yaml config."""
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "sample_data.yaml"
    )
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # Basic assertions
    assert len(pipeline.lidar_board_detectors) == 1, "Expected 1 board detector"
    assert len(pipeline.aruco_locators) == 1, "Expected 1 aruco locator"
    assert len(pipeline.lidar_camera_solvers) == 1, "Expected 1 lidar-camera solver"
    assert len(pipeline.lidar_lidar_solvers) == 0, "Expected 0 lidar-lidar solvers"

    # Calibration plan assertions
    assert pipeline.calibration_plan is not None, "Expected calibration plan"
    assert pipeline.calibration_plan.reference_frame == "top_lidar"
    assert len(pipeline.calibration_plan.tree_edges) == 1
    assert len(pipeline.calibration_plan.validation_edges) == 0


def test_vehicle_config():
    """Test parsing the vehicle.yaml config (multi-sensor)."""
    config_path = Path(__file__).parent.parent / "config" / "examples" / "vehicle.yaml"
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # Calibration plan assertions
    assert pipeline.calibration_plan is not None, "Expected calibration plan"
    assert pipeline.calibration_plan.reference_frame == "L1"
    assert len(pipeline.calibration_plan.tree_edges) == 5
    assert len(pipeline.calibration_plan.validation_edges) == 0


@pytest.mark.parametrize(
    ("example_name", "expected_target_id"),
    [
        ("sample_data.yaml", "hollow_1000_aruco_4"),
        ("seyond_left.yaml", "hollow_1000_aruco_4"),
        ("seyond_right.yaml", "hollow_1000_aruco_4"),
        ("two_lidar.yaml", "hollow_1000_aruco_4"),
        ("vehicle.yaml", "hollow_1000_aruco_4"),
        ("solid_600_handheld.yaml", "solid_600_aruco_1"),
    ],
)
def test_maintained_examples_select_their_target(example_name, expected_target_id):
    """W5-D: every maintained example parses to the target it now names via
    target_config/detector_config, not the legacy translation.

    Expressed per example rather than branching on the filename inside the
    body, so this is also the guard against relabelling: the five examples
    that already recorded data against the hollow board must keep resolving
    to hollow_1000_aruco_4 -- the new solid_600_handheld.yaml example is the
    only one that resolves to solid_600_aruco_1. A cutover bug that pointed
    an existing example at the wrong target_config would either point it at
    the wrong physical board (invalidating every recording made against it)
    or fail this test outright.
    """
    config_path = Path(__file__).parent.parent / "config" / "examples" / example_name

    pipeline = CalibrationConfigParser(str(config_path)).parse()

    identities = {
        detector.target_identity for detector in pipeline.lidar_board_detectors
    }
    assert identities
    assert {identity.target_id for identity in identities} == {expected_target_id}


def test_two_lidar_config():
    """Test parsing the two_lidar.yaml config."""
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "two_lidar.yaml"
    )
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # Basic assertions
    assert len(pipeline.lidar_board_detectors) == 2, "Expected 2 board detectors"
    assert len(pipeline.aruco_locators) == 0, "Expected 0 aruco locators"
    assert len(pipeline.lidar_camera_solvers) == 0, "Expected 0 lidar-camera solvers"
    assert len(pipeline.lidar_lidar_solvers) == 1, "Expected 1 lidar-lidar solver"

    # Device dicts populated
    assert len(pipeline.lidars) == 2
    assert "top_lidar" in pipeline.lidars
    assert "front_lidar" in pipeline.lidars
    assert len(pipeline.cameras) == 0

    # Calibration plan assertions
    assert pipeline.calibration_plan is not None
    assert pipeline.calibration_plan.reference_frame == "top_lidar"
    assert len(pipeline.calibration_plan.tree_edges) == 1
    assert len(pipeline.calibration_plan.validation_edges) == 0


def test_sample_data_node_parity():
    """Verify sample_data.yaml produces nodes matching what the XML would generate.

    The XML path (lidar_camera_calibration.launch.xml) produced:
    - 1 aruco_locator in calibration/aruco_locator namespace
    - 1 lidar_board_detector in calibration/lidar_board_detector namespace
    - 1 extrinsic_solver in calibration/extrinsic_solver namespace

    The config-driven path should produce equivalent nodes (possibly different
    namespaces, but same count and correct topic wiring).
    """
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "sample_data.yaml"
    )
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # 1 aruco locator
    assert len(pipeline.aruco_locators) == 1
    locator = pipeline.aruco_locators[0]
    assert locator.camera_name == "front_center"
    assert locator.image_topic == "/sensing/camera/front_center/image_raw"

    # 1 board detector
    assert len(pipeline.lidar_board_detectors) == 1
    detector = pipeline.lidar_board_detectors[0]
    assert detector.lidar_name == "top_lidar"
    assert detector.pointcloud_topic == "/sensing/lidar/top/pointcloud_raw"

    # 1 lidar-camera solver
    assert len(pipeline.lidar_camera_solvers) == 1
    solver = pipeline.lidar_camera_solvers[0]
    assert solver.lidar_name == "top_lidar"
    assert solver.camera_name == "front_center"
    assert solver.parent_frame == "velodyne_top"
    assert solver.child_frame == "camera_front_center"

    # Correct topic wiring: solver subscribes to detector/locator output topics
    assert solver.board_detections_topic == detector.output_topic
    assert solver.aruco_detections_topic == locator.output_topic
    assert solver.camera_topic == locator.image_topic

    # Device dicts populated
    assert len(pipeline.lidars) == 1
    assert len(pipeline.cameras) == 1
    assert (
        pipeline.lidars["top_lidar"].pointcloud_topic
        == "/sensing/lidar/top/pointcloud_raw"
    )
    assert (
        pipeline.cameras["front_center"].image_topic
        == "/sensing/camera/front_center/image_raw"
    )


def test_two_lidar_node_parity():
    """Verify two_lidar.yaml produces correct nodes for two-lidar calibration.

    Expected:
    - 2 lidar_board_detectors (one per lidar)
    - 1 lidar-lidar solver
    - 0 aruco locators (no cameras)
    """
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "two_lidar.yaml"
    )
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # 2 board detectors
    assert len(pipeline.lidar_board_detectors) == 2
    detector_names = {d.lidar_name for d in pipeline.lidar_board_detectors}
    assert detector_names == {"top_lidar", "front_lidar"}

    # 0 aruco locators
    assert len(pipeline.aruco_locators) == 0

    # 1 lidar-lidar solver
    assert len(pipeline.lidar_lidar_solvers) == 1
    solver = pipeline.lidar_lidar_solvers[0]
    assert {solver.lidar1_name, solver.lidar2_name} == {"top_lidar", "front_lidar"}
    # Frame ids come from two_lidar.yaml, which describes the real two-LiDAR rig
    # (VLP-32C + Seyond Falcon). The recorded bags publish exactly these frames.
    frames = {
        solver.lidar1_name: solver.lidar1_frame,
        solver.lidar2_name: solver.lidar2_frame,
    }
    assert frames["top_lidar"] == "velodyne"
    assert frames["front_lidar"] == "seyond"

    # Correct topic wiring: solver subscribes to detector output topics
    detector_by_lidar = {d.lidar_name: d for d in pipeline.lidar_board_detectors}
    assert (
        solver.lidar1_detections_topic
        == detector_by_lidar[solver.lidar1_name].output_topic
    )
    assert (
        solver.lidar2_detections_topic
        == detector_by_lidar[solver.lidar2_name].output_topic
    )

    # 0 lidar-camera solvers
    assert len(pipeline.lidar_camera_solvers) == 0


def test_solid_600_handheld_config():
    """solid_600_handheld.yaml parses to the single-pair pipeline it claims,
    with the tighter sync window its hand-held (moving) board requires.

    100ms is what every hollow example uses for a board on a tripod;
    50ms/100/reject_new is deliberately tighter here because the board can
    move between a camera frame and a LiDAR sweep in a way a stationary
    board cannot.
    """
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "solid_600_handheld.yaml"
    )
    # Not pytest.skip: this example ships in the repo, so a missing file is a
    # deleted maintained example, not an absent optional fixture. Skipping
    # would hide the deletion behind a green run.
    assert config_path.exists(), f"maintained example missing: {config_path}"

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    assert len(pipeline.lidar_board_detectors) == 1
    assert len(pipeline.aruco_locators) == 1
    assert len(pipeline.lidar_camera_solvers) == 1
    assert len(pipeline.lidar_lidar_solvers) == 0

    assert pipeline.sync is not None
    assert pipeline.sync.tolerance_ms == 50.0
    assert pipeline.sync.queue_size == 100
    assert pipeline.sync.drop_policy == "reject_new"


def test_two_lidar_per_lidar_detector_config_override_reaches_the_right_node():
    """two_lidar.yaml's per-LiDAR detector_config override on front_lidar
    reaches only front_lidar's detector; top_lidar keeps the marker-level
    preset.

    two_lidar.yaml is the one maintained example where a per-device override
    and a marker-level preset coexist (front_lidar overrides to the seyond
    preset; top_lidar has no override and falls back to the marker's
    velodyne preset). Getting this backwards would silently give the
    solid-state LiDAR (seyond/Falcon) a spinning-LiDAR (velodyne) operating
    point, or vice versa -- a real miscalibration, not just a wiring slip.
    """
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "two_lidar.yaml"
    )
    assert config_path.exists(), f"maintained example missing: {config_path}"

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    detector_by_lidar = {
        detector.lidar_name: detector for detector in pipeline.lidar_board_detectors
    }
    assert detector_by_lidar["front_lidar"].detector_config.endswith(
        "hollow_1000/seyond.json5"
    )
    assert detector_by_lidar["top_lidar"].detector_config.endswith(
        "hollow_1000/velodyne.json5"
    )


def test_bbox_config_omitted_at_marker_level_parses(tmp_path):
    """A marker used by a lidar no longer requires bbox_config at parse
    time.

    bbox_config is only read by lidar_board_detector when its detector
    tuning file selects detection_mode=bbox; under bbox_free (what every
    maintained board config ships) it is loaded and discarded. The parser
    treats detector_config as an opaque path and does not read
    detection_mode out of it, so it cannot know which mode applies -- the
    rule now lives solely in lidar_board_detector
    (ros/lidar_board_detector/src/main.rs), which raises a clear,
    node-specific error when detection_mode=bbox and no bbox_file was
    supplied.
    """
    targets = Path(__file__).parent.parent / "config" / "targets"
    config_text = f"""
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /lidar/points
      frame_id: velodyne_top
  cameras:
    front_center:
      image_topic: /camera/image_raw
      frame_id: camera_front_center

markers:
  calibration_board:
    target_config: {targets / "hollow_1000_aruco_4_v1.json5"}
    detector_config: /tmp/detector.json5
    # bbox_config intentionally omitted
    pairs:
      - [top_lidar, front_center]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    config_path = tmp_path / "missing_bbox.yaml"
    config_path.write_text(config_text)

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    assert len(pipeline.lidar_board_detectors) == 1
    assert pipeline.lidar_board_detectors[0].bbox_config is None


TARGETS = Path(__file__).parent.parent / "config" / "targets"


def _write_new_schema_config(tmp_path, first_target, second_target=None):
    """Write a parser-only config; tuning/bbox files need not exist to parse."""
    second_marker = ""
    if second_target is not None:
        second_marker = f"""
  target_b:
    target_config: {second_target}
    detector_config: /tmp/detector-b.json5
    bbox_config: /tmp/bbox-b.json5
    aruco_detector_config: /tmp/aruco-detector-a.json5
    pairs:
      - [lidar_b, camera]
"""
    config_path = tmp_path / "new_schema.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    lidar_a:
      pointcloud_topic: /lidar/a
      frame_id: lidar_a_frame
    lidar_b:
      pointcloud_topic: /lidar/b
      frame_id: lidar_b_frame
  cameras:
    camera:
      image_topic: /camera/image
      frame_id: camera_frame
markers:
  target_a:
    target_config: {first_target}
    detector_config: /tmp/detector-a.json5
    bbox_config: /tmp/bbox-a.json5
    aruco_detector_config: /tmp/aruco-detector-a.json5
    pairs:
      - [lidar_a, camera]
{second_marker}

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    )
    return config_path


def test_new_target_schema_parses_without_ros_startup(tmp_path):
    config_path = _write_new_schema_config(
        tmp_path, TARGETS / "solid_600_aruco_1_v1.json5"
    )

    pipeline = CalibrationConfigParser(str(config_path)).parse()

    assert (
        pipeline.lidar_board_detectors[0].target_identity.target_id
        == "solid_600_aruco_1"
    )
    assert pipeline.lidar_board_detectors[0].detector_config == "/tmp/detector-a.json5"
    assert pipeline.aruco_locators[0].target_config.endswith(
        "solid_600_aruco_1_v1.json5"
    )


def test_different_paths_with_same_target_identity_share_sensor(tmp_path):
    equivalent_target = tmp_path / "same_target_different_format.json5"
    equivalent_target.write_text(
        """// Key order, comments, and equivalent units are not semantic.
{
  lidar_orientation_reference: { local_axis: "+y", kind: "mounting_up" },
  fiducial: {
    border_bits: 1,
    marker_fill_ratio: 1.0,
    cells_per_side: 1,
    outer_border: "0.060m",
    paper_center: { toward_top_corner: "0mm", toward_left_corner: "0mm" },
    paper_side: "0.600m",
    marker_ids: [24],
    dictionary: "DICT_5X5_1000",
    kind: "square_aruco_grid",
  },
  plate: { surface: { kind: "solid" }, side: "0.600m" },
  board_frame_convention: "corner_aligned_plate_center_v1",
  revision: 1,
  target_id: "solid_600_aruco_1",
  schema_version: 1,
}
"""
    )
    config_path = _write_new_schema_config(
        tmp_path,
        TARGETS / "solid_600_aruco_1_v1.json5",
        equivalent_target,
    )

    pipeline = CalibrationConfigParser(str(config_path)).parse()

    assert len(pipeline.aruco_locators) == 1
    assert (
        pipeline.lidar_board_detectors[0].target_identity
        == pipeline.lidar_board_detectors[1].target_identity
    )


def test_different_target_identities_on_one_sensor_reject(tmp_path):
    config_path = _write_new_schema_config(
        tmp_path,
        TARGETS / "solid_600_aruco_1_v1.json5",
        TARGETS / "hollow_1000_aruco_4_v1.json5",
    )

    with pytest.raises(
        ValueError, match="Camera Target Identities|Sensor 'camera'.*different"
    ):
        CalibrationConfigParser(str(config_path)).parse()


def test_legacy_marker_schema_rejected(tmp_path):
    """W5-E1: the marker-level legacy schema (type/board_config/aruco_config)
    that W5-D used to silently translate into an explicit hollow target is
    now rejected outright, with migration guidance. There is no automatic
    translation left in the parser -- see `_parse_markers`.
    """
    config_path = tmp_path / "legacy.yaml"
    config_path.write_text(
        """
devices:
  lidars:
    lidar:
      pointcloud_topic: /lidar
      frame_id: lidar_frame
markers:
  board:
    type: hollow_board
    board_config: /tmp/legacy-board.json5
    aruco_config: /tmp/legacy-aruco.json5
    bbox_config: /tmp/bbox.json5
    pairs:
      - [lidar, lidar]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    )

    with pytest.raises(ValueError, match="Marker 'board' sets retired schema key"):
        CalibrationConfigParser(str(config_path)).parse()


def test_marker_rejects_mixed_legacy_and_new_schema(tmp_path):
    config_path = tmp_path / "mixed.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    lidar:
      pointcloud_topic: /lidar
      frame_id: lidar_frame
markers:
  board:
    type: hollow_board
    board_config: /tmp/legacy-board.json5
    target_config: {TARGETS / "solid_600_aruco_1_v1.json5"}
    detector_config: /tmp/detector.json5
    pairs:
      - [lidar, lidar]
"""
    )

    with pytest.raises(ValueError, match="Marker 'board' sets retired schema key"):
        CalibrationConfigParser(str(config_path)).parse()


def test_lidar_rejects_mixed_board_and_detector_overrides(tmp_path):
    config_path = tmp_path / "mixed_lidar.yaml"
    config_path.write_text(
        """
devices:
  lidars:
    lidar:
      pointcloud_topic: /lidar
      frame_id: lidar_frame
      board_config: /tmp/legacy-board.json5
      detector_config: /tmp/detector.json5
markers: {}
"""
    )

    with pytest.raises(
        ValueError, match="LiDAR 'lidar' sets retired schema key 'board_config'"
    ):
        CalibrationConfigParser(str(config_path)).parse()


def test_lidar_device_board_config_alone_rejected(tmp_path):
    """A LiDAR device carrying only 'board_config' (no 'detector_config') is
    refused.

    This is the most dangerous silent-acceptance case: with no
    detector_config present to collide with, the device would otherwise
    parse cleanly and only diverge from the marker's tuning at runtime,
    against the wrong detector config, with no error anywhere.
    """
    config_path = tmp_path / "device_only_legacy.yaml"
    config_path.write_text(
        """
devices:
  lidars:
    lidar:
      pointcloud_topic: /lidar
      frame_id: lidar_frame
      board_config: /tmp/legacy-board.json5
markers: {}
"""
    )

    with pytest.raises(
        ValueError, match="LiDAR 'lidar' sets retired schema key 'board_config'"
    ):
        CalibrationConfigParser(str(config_path)).parse()


@pytest.mark.parametrize(
    ("device_override", "marker_fields", "expected_match"),
    [
        (
            "board_config: /tmp/device-legacy-board.json5",
            """target_config: {solid_target}
    detector_config: /tmp/marker-detector.json5""",
            "LiDAR 'lidar' sets retired schema key 'board_config'",
        ),
        (
            "detector_config: /tmp/device-detector.json5",
            """type: hollow_board
    board_config: /tmp/marker-legacy-board.json5
    aruco_config: /tmp/marker-legacy-aruco.json5""",
            "Marker 'board' sets retired schema key",
        ),
    ],
)
def test_device_and_marker_schema_mismatch_rejected(
    tmp_path, device_override, marker_fields, expected_match
):
    """A LiDAR device and the marker it pairs with choose the legacy or new
    schema independently -- whichever side uses the retired schema is
    refused, regardless of what the other side uses.

    This used to be guarded by a dedicated cross-level `marker_type` check
    that compared the device's and marker's schema choice against each
    other and raised a single "Sensor '...' ... marker '...'" message
    naming both. That guard, and the fields it used to name (`marker_type`
    among them), no longer exist -- `_parse_devices` and `_parse_markers`
    each now independently refuse retired keys on their own level. Rewritten
    (not folded into the two single-level rejection tests above) to keep
    the coverage that a device/marker disagreement -- one side legacy, the
    other new -- is refused, not merely that each side is refused in
    isolation.
    """
    config_path = tmp_path / "cross_level_mixed.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    lidar:
      pointcloud_topic: /lidar
      frame_id: lidar_frame
      {device_override}
markers:
  board:
    {marker_fields.format(solid_target=TARGETS / "solid_600_aruco_1_v1.json5")}
    bbox_config: /tmp/bbox.json5
    pairs:
      - [lidar, lidar]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    )

    with pytest.raises(ValueError, match=expected_match):
        CalibrationConfigParser(str(config_path)).parse()


def _write_sync_only_config(tmp_path: Path, sync_yaml: str) -> Path:
    """Write a config with empty devices/markers, isolating `sync:` validation.

    `_parse_sync` runs before the reference_frame/planner checks that would
    otherwise require a real device+pair setup (a config with no pairs makes
    `compute_plan` raise "No calibration pairs defined"), so a config with
    empty devices/markers reaches sync validation directly and nothing else.
    `sync_yaml` is substituted verbatim -- pass `""` to omit the `sync:` key
    entirely, for the "missing section" refusal.
    """
    config_path = tmp_path / "sync_only.yaml"
    config_path.write_text(
        f"""
devices: {{}}
markers: {{}}
{sync_yaml}
"""
    )
    return config_path


def test_sync_section_missing_rejected(tmp_path):
    """A config with no `sync:` key at all is refused, naming the section
    and the three required keys -- there is no mode-derived fallback.
    """
    config_path = _write_sync_only_config(tmp_path, "")

    with pytest.raises(ValueError, match="Missing required 'sync' section"):
        CalibrationConfigParser(str(config_path)).parse()


@pytest.mark.parametrize(
    ("sync_yaml", "expected_missing"),
    [
        ("sync:\n  queue_size: 100\n  drop_policy: reject_new\n", "tolerance_ms"),
        ("sync:\n  tolerance_ms: 100\n  drop_policy: reject_new\n", "queue_size"),
        ("sync:\n  tolerance_ms: 100\n  queue_size: 100\n", "drop_policy"),
        ("sync: {}\n", "tolerance_ms"),
    ],
)
def test_sync_section_missing_individual_key_rejected(
    tmp_path, sync_yaml, expected_missing
):
    """A `sync:` section present but missing one (or all) of its three
    required keys is refused, naming the missing key(s).
    """
    config_path = _write_sync_only_config(tmp_path, sync_yaml)

    with pytest.raises(ValueError) as error:
        CalibrationConfigParser(str(config_path)).parse()
    message = str(error.value)
    assert "missing required key" in message
    assert expected_missing in message


@pytest.mark.parametrize(
    "raw_value",
    ["0", "-5", "inf", "Infinity", "1e400", "nan", "banana"],
)
def test_sync_tolerance_ms_rejects_invalid_values(tmp_path, raw_value):
    """`sync.tolerance_ms` refuses 0, negatives, and anything that is not a
    finite positive number.

    "inf", "Infinity", "1e400" and "nan" are all written unquoted here on
    purpose: PyYAML's safe loader does not recognize any of them as a float
    literal (it requires a leading `.` for infinity/nan, and a signed
    exponent with a decimal point for scientific notation), so each reaches
    the parser as a plain Python `str`. `float()` still turns every one of
    them into a real `inf`/`nan` -- which is exactly the case
    `math.isfinite` exists to catch, since a bare `value <= 0` comparison
    would let `inf` through (it is `> 0`) and `nan` fails every comparison.
    """
    config_path = _write_sync_only_config(
        tmp_path,
        f"sync:\n  tolerance_ms: {raw_value}\n  queue_size: 100\n  drop_policy: reject_new\n",
    )

    with pytest.raises(
        ValueError, match="sync.tolerance_ms must be a finite, strictly positive"
    ):
        CalibrationConfigParser(str(config_path)).parse()


@pytest.mark.parametrize(
    "raw_value",
    ["0", "-3", "true", "10.5"],
)
def test_sync_queue_size_rejects_invalid_values(tmp_path, raw_value):
    """`sync.queue_size` refuses 0, negatives, non-integers, and `bool`
    (Python's `bool` is an `int` subclass, so `True` would otherwise pass as
    a queue size of 1 without an explicit guard).
    """
    config_path = _write_sync_only_config(
        tmp_path,
        f"sync:\n  tolerance_ms: 100\n  queue_size: {raw_value}\n  drop_policy: reject_new\n",
    )

    with pytest.raises(ValueError, match="sync.queue_size must be a positive integer"):
        CalibrationConfigParser(str(config_path)).parse()


def test_sync_drop_policy_rejects_unknown_value(tmp_path):
    """`sync.drop_policy` accepts only 'reject_new' or 'drop_oldest', naming
    both the bad value and the two valid ones.
    """
    config_path = _write_sync_only_config(
        tmp_path,
        "sync:\n  tolerance_ms: 100\n  queue_size: 100\n  drop_policy: drop_everything\n",
    )

    with pytest.raises(ValueError) as error:
        CalibrationConfigParser(str(config_path)).parse()
    message = str(error.value)
    assert "sync.drop_policy must be one of" in message
    assert "reject_new" in message
    assert "drop_oldest" in message
    assert "drop_everything" in message


def test_sync_section_valid_parses(tmp_path):
    """A valid, non-default `sync:` section is carried through onto
    `PipelineConfig.sync` unchanged -- not replaced by any mode-derived
    preset (there is no such preset left in the parser to replace it with).
    """
    config_path = tmp_path / "sync_valid.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    lidar:
      pointcloud_topic: /lidar
      frame_id: lidar_frame
markers:
  board:
    target_config: {TARGETS / "hollow_1000_aruco_4_v1.json5"}
    detector_config: /tmp/detector.json5
    bbox_config: /tmp/bbox.json5
    pairs:
      - [lidar, lidar]

sync:
  tolerance_ms: 250
  queue_size: 5
  drop_policy: drop_oldest
"""
    )

    pipeline = CalibrationConfigParser(str(config_path)).parse()

    assert pipeline.sync is not None
    assert pipeline.sync.tolerance_ms == 250.0
    assert pipeline.sync.queue_size == 5
    assert pipeline.sync.drop_policy == "drop_oldest"


# --- Sessions: the `data:` section and $(session-dir) in a config ----------


def write_session(tmp_path, data_block, devices_block, name="rig"):
    directory = tmp_path / name
    directory.mkdir(parents=True, exist_ok=True)
    target = "$(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5"
    detector = "$(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5"
    (directory / "session.yaml").write_text(
        f"""
name: {name}
{data_block}
{devices_block}
markers:
  calibration_board:
    target_config: {target}
    detector_config: {detector}
    pairs:
      - [top, front_center]
sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
""",
        encoding="utf-8",
    )
    return directory / "session.yaml"


def make_pcap_dir(tmp_path, name="data"):
    directory = tmp_path / name
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "lidar.pcap").write_bytes(b"")
    (directory / "video.avi").write_bytes(b"")
    return directory


def test_pcap_avi_derives_device_topics(tmp_path):
    make_pcap_dir(tmp_path / "rig")
    manifest = write_session(
        tmp_path,
        "data:\n  kind: pcap_avi\n  dir: $(session-dir)/data\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n",
    )
    pipeline = parse_config(str(manifest))
    assert (
        pipeline.lidars["top"].pointcloud_topic == "/sensing/lidar/top/pointcloud_raw"
    )
    assert (
        pipeline.cameras["front_center"].image_topic
        == "/sensing/camera/front_center/image_raw"
    )


def test_pcap_avi_refuses_a_stated_topic(tmp_path):
    """Accepting one would reinstate the two-sources-of-truth bug the manifest
    exists to remove."""
    make_pcap_dir(tmp_path / "rig")
    manifest = write_session(
        tmp_path,
        "data:\n  kind: pcap_avi\n  dir: $(session-dir)/data\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "      pointcloud_topic: /my/topic\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n",
    )
    with pytest.raises((ValueError, SessionError), match="derived"):
        parse_config(str(manifest))


def test_live_requires_a_stated_topic(tmp_path):
    manifest = write_session(
        tmp_path,
        "data:\n  kind: live\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n",
    )
    with pytest.raises((ValueError, SessionError), match="pointcloud_topic"):
        parse_config(str(manifest))


def test_session_dir_resolves_in_a_marker_path(tmp_path):
    make_pcap_dir(tmp_path / "rig")
    bbox = tmp_path / "rig" / "bbox.json5"
    bbox.write_text("{}", encoding="utf-8")
    manifest = write_session(
        tmp_path,
        "data:\n  kind: pcap_avi\n  dir: $(session-dir)/data\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n",
    )
    text = manifest.read_text(encoding="utf-8").replace(
        "    pairs:", "    bbox_config: $(session-dir)/bbox.json5\n    pairs:"
    )
    manifest.write_text(text, encoding="utf-8")
    pipeline = parse_config(str(manifest))
    assert pipeline.lidar_board_detectors[0].bbox_config == str(bbox)


def test_a_config_without_a_data_section_still_parses(tmp_path):
    """calibrate.launch.py must keep working against plain configs and live rigs."""
    manifest = write_session(
        tmp_path,
        "",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "      pointcloud_topic: /points\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n"
        "      image_topic: /image\n",
    )
    pipeline = parse_config(str(manifest))
    assert pipeline.data is None
    assert pipeline.lidars["top"].pointcloud_topic == "/points"


def write_bag(tmp_path, topics, name="bag"):
    """A minimal rosbag2 directory: only metadata.yaml is read for verification."""
    import yaml

    bag = tmp_path / name
    bag.mkdir(parents=True, exist_ok=True)
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "topics_with_message_count": [
                        {"topic_metadata": {"name": topic}} for topic in topics
                    ]
                }
            }
        ),
        encoding="utf-8",
    )
    return bag


def test_a_bag_session_naming_a_topic_the_bag_lacks_is_refused_at_parse_time(tmp_path):
    """M-26, caught before a node starts.

    `two_lidar.yaml` named /velodyne_points while TWO_LIDAR_1 records
    /lidar/vlp32/velodyne_points. The pipeline launched cleanly and sat silent
    forever. Verifying against metadata.yaml at parse time turns that into a
    refusal that names the fix.
    """
    (tmp_path / "rig").mkdir(parents=True, exist_ok=True)
    write_bag(tmp_path / "rig", ["/lidar/vlp32/velodyne_points"])
    manifest = write_session(
        tmp_path,
        "data:\n  kind: bag\n  path: $(session-dir)/bag\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "      pointcloud_topic: /velodyne_points\n"
        "  cameras:\n    front_center:\n      frame_id: cam\n"
        "      image_topic: /image\n",
    )
    with pytest.raises((ValueError, SessionError)) as excinfo:
        parse_config(str(manifest))
    message = str(excinfo.value)
    assert "does not publish /velodyne_points" in message
    assert "It records: /lidar/vlp32/velodyne_points" in message


def test_a_bag_session_whose_topics_match_parses(tmp_path):
    (tmp_path / "rig").mkdir(parents=True, exist_ok=True)
    write_bag(tmp_path / "rig", ["/lidar/vlp32/velodyne_points", "/image"])
    manifest = write_session(
        tmp_path,
        "data:\n  kind: bag\n  path: $(session-dir)/bag\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "      pointcloud_topic: /lidar/vlp32/velodyne_points\n"
        "  cameras:\n    front_center:\n      frame_id: cam\n"
        "      image_topic: /image\n",
    )
    pipeline = parse_config(str(manifest))
    assert pipeline.data.kind == "bag"
    assert pipeline.lidars["top"].pointcloud_topic == "/lidar/vlp32/velodyne_points"


if __name__ == "__main__":
    import inspect

    tests = [
        obj
        for name, obj in sorted(globals().items())
        if name.startswith("test_") and inspect.isfunction(obj)
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            print(f"  PASS: {test.__name__}")
            passed += 1
        except Exception as e:  # noqa: BLE001 - hand-rolled runner: reporting per-test failure is the point
            print(f"  FAIL: {test.__name__}: {e}")
            failed += 1

    print()
    print(f"{passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)
