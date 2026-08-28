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
    from lctk_launch.config_parser import CalibrationConfigParser
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


def test_hollow_board_missing_bbox_config_parses(tmp_path):
    """A hollow_board marker used by a lidar no longer requires bbox_config
    at parse time.

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
    config_text = """
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
    type: hollow_board
    board_config: /tmp/board.json5
    aruco_config: /tmp/aruco.json5
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
    assert pipeline.aruco_locators[0].aruco_config is None
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
    marker_ids: [1],
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


def test_legacy_schema_translates_to_explicit_hollow_target(tmp_path):
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

    pipeline = CalibrationConfigParser(str(config_path)).parse()

    assert (
        pipeline.lidar_board_detectors[0].target_identity.target_id
        == "hollow_1000_aruco_4"
    )
    assert pipeline.lidar_board_detectors[0].target_config.endswith(
        "hollow_1000_aruco_4_v1.json5"
    )


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

    with pytest.raises(ValueError, match="mixes legacy"):
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
        ValueError, match="both legacy 'board_config' and new 'detector_config'"
    ):
        CalibrationConfigParser(str(config_path)).parse()


@pytest.mark.parametrize(
    ("device_override", "marker_fields", "expected_fields"),
    [
        (
            "board_config: /tmp/device-legacy-board.json5",
            """target_config: {solid_target}
    detector_config: /tmp/marker-detector.json5""",
            ("board_config", "target_config", "detector_config"),
        ),
        (
            "detector_config: /tmp/device-detector.json5",
            """type: hollow_board
    board_config: /tmp/marker-legacy-board.json5
    aruco_config: /tmp/marker-legacy-aruco.json5""",
            ("detector_config", "type", "board_config", "aruco_config"),
        ),
    ],
)
def test_lidar_rejects_cross_level_schema_mix(
    tmp_path, device_override, marker_fields, expected_fields
):
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

    with pytest.raises(ValueError) as error:
        CalibrationConfigParser(str(config_path)).parse()
    message = str(error.value)
    assert "Sensor 'lidar'" in message
    assert "marker 'board'" in message
    assert all(field in message for field in expected_fields)


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
