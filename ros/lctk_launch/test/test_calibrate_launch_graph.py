"""Graph contracts for the config-driven calibration launch."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest
from launch_ros.actions import Node

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "calibrate.launch.py"
TARGETS_ROOT = PACKAGE_ROOT / "config" / "targets"

# The maintained configs are now session manifests at the repo root, one
# directory per calibration run, rather than loose files under
# config/examples/. `session.yaml` is a normal calibration config plus a
# `data:` section, so everything this module asserts about a maintained
# config still applies unchanged.
SESSIONS_ROOT = PACKAGE_ROOT.parents[1] / "sessions"
MANIFEST_NAME = "session.yaml"


def _session(name: str) -> Path:
    """The manifest of one shipped session, by directory name."""
    return SESSIONS_ROOT / name / MANIFEST_NAME


# Keep source-tree execution consistent with the existing lctk_launch tests.
sys.path.insert(0, str(PACKAGE_ROOT))


def _write_new_schema_detector_config(tmp_path: Path) -> Path:
    """Write a new-schema (target_config/detector_config) marker config.

    Mirrors ``test_config_parser._write_new_schema_config``'s shape: only the
    Target Definition manifest (``target_config``) needs to exist on disk to
    parse -- ``detector_config``/``bbox_config`` are opaque paths as far as
    the parser and this launch file are concerned. The maintained configs
    under ``sessions/`` moved onto this same new schema in W5-D; this test
    still writes its own file into pytest's tmp_path rather than reading one
    of them, so it stays independent of exactly which target/detector preset
    a given maintained session happens to select.

    Deliberately a two-LiDAR pairing (no camera), like
    ``sessions/twolidar-vlp32-falcon``, so the tests built on it exercise
    detector routing and nothing else. The LiDAR+camera case has its own
    helper, ``_write_new_schema_camera_config`` below.
    """

    target_config = TARGETS_ROOT / "solid_600_aruco_1_v1.json5"
    # Never opened by the parser or this launch file -- just opaque path
    # strings that must round-trip unchanged. Named "not-a-real-file" and
    # placed under tmp_path (never /tmp/, per CLAUDE.md) so no reader
    # mistakes them for files this test depends on existing.
    detector_config = tmp_path / "not-a-real-file-detector-tuning.json5"
    bbox_config = tmp_path / "not-a-real-file-bbox.json5"
    config_path = tmp_path / "new_schema.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /velodyne_points
      frame_id: velodyne
    front_lidar:
      pointcloud_topic: /iv_points
      frame_id: seyond

markers:
  calibration_target:
    target_config: {target_config}
    detector_config: {detector_config}
    bbox_config: {bbox_config}
    pairs:
      - [top_lidar, front_lidar]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    )
    return config_path


def _write_new_schema_detector_config_no_bbox(tmp_path: Path) -> Path:
    """Same shape as ``_write_new_schema_detector_config`` but omits
    ``bbox_config`` entirely.

    config_parser no longer requires ``bbox_config`` (it is only read by
    lidar_board_detector when detector tuning selects
    ``detection_mode=bbox``; bbox_free -- what every maintained board config
    ships -- never reads it). This exercises that a config without it still
    generates a graph, and that ``bbox_file`` is simply absent from the
    detector's params rather than present-as-``None`` (which would trip
    launch_ros's eager ``normalize_parameters`` at ``Node()`` construction
    time).
    """

    target_config = TARGETS_ROOT / "solid_600_aruco_1_v1.json5"
    detector_config = tmp_path / "not-a-real-file-detector-tuning.json5"
    config_path = tmp_path / "new_schema_no_bbox.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /velodyne_points
      frame_id: velodyne
    front_lidar:
      pointcloud_topic: /iv_points
      frame_id: seyond

markers:
  calibration_target:
    target_config: {target_config}
    detector_config: {detector_config}
    pairs:
      - [top_lidar, front_lidar]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    )
    return config_path


def _write_new_schema_camera_config(tmp_path: Path) -> Path:
    """Write a new-schema (target_config/detector_config) marker config
    pairing a LiDAR with a camera.

    This is the config C2 unblocks: before this piece, generating a node
    graph for it raised, because ``calibrate.launch.py`` still passed
    ``aruco_config_file`` unconditionally to both camera-side nodes
    (``aruco_locator_node`` and ``lidar_to_camera_solver``), and a new-schema
    marker leaves that field ``None``. launch_ros's ``Node()`` normalizes --
    and rejects ``None`` -- parameter values eagerly at construction time, so
    the failure happened while ``generate_nodes`` was still building the
    node list, not at node startup. C2 makes both nodes take ``target_config``
    instead and omit ``aruco_config_file`` entirely under this schema.
    """

    target_config = TARGETS_ROOT / "solid_600_aruco_1_v1.json5"
    # Never opened by the parser or this launch file -- just opaque path
    # strings that must round-trip unchanged. Placed under tmp_path (never
    # /tmp/, per CLAUDE.md).
    detector_config = tmp_path / "not-a-real-file-detector-tuning.json5"
    bbox_config = tmp_path / "not-a-real-file-bbox.json5"
    config_path = tmp_path / "new_schema_camera.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /sensing/lidar/top/pointcloud_raw
      frame_id: velodyne_top
  cameras:
    front_center:
      image_topic: /sensing/camera/front_center/image_raw
      frame_id: camera_front_center

markers:
  calibration_target:
    target_config: {target_config}
    detector_config: {detector_config}
    bbox_config: {bbox_config}
    pairs:
      - [top_lidar, front_center]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    )
    return config_path


def _write_new_schema_shared_camera_config(tmp_path: Path) -> Path:
    """Write a new-schema config where one camera is paired with two
    different LiDARs against the same marker.

    Exercises the graph-level "one locator per camera" invariant:
    `config_parser.py` dedupes `cameras_needed` by name (~line 570), so two
    calibration pairs that share a camera must still yield exactly one
    `aruco_locator_node`, not two.
    """

    target_config = TARGETS_ROOT / "solid_600_aruco_1_v1.json5"
    detector_config = tmp_path / "not-a-real-file-detector-tuning.json5"
    bbox_config = tmp_path / "not-a-real-file-bbox.json5"
    config_path = tmp_path / "shared_camera.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /sensing/lidar/top/pointcloud_raw
      frame_id: velodyne_top
    front_lidar:
      pointcloud_topic: /sensing/lidar/front/pointcloud_raw
      frame_id: velodyne_front
  cameras:
    front_center:
      image_topic: /sensing/camera/front_center/image_raw
      frame_id: camera_front_center

markers:
  calibration_target:
    target_config: {target_config}
    detector_config: {detector_config}
    bbox_config: {bbox_config}
    pairs:
      - [top_lidar, front_center]
      - [front_lidar, front_center]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    )
    return config_path


def _write_new_schema_two_lidar_detector_override_config(tmp_path: Path) -> Path:
    """Write a new-schema two-LiDAR marker config with a per-LiDAR
    ``detector_config`` override on one of the two LiDARs.

    New-schema successor of the deleted legacy two-LiDAR fixture: the
    per-device override branch in ``_derive_pipeline``
    (``lidar.detector_config_override or marker.detector_config``) still
    needs launch-graph-level coverage now that no maintained config doubles
    as its fixture in this file -- ``sessions/twolidar-vlp32-falcon`` covers
    the same shape, but this fixture keeps that coverage independent of
    exactly what that maintained session currently contains, and of whether
    its gitignored recording is present on this machine.

    Points at real repo files (rather than opaque tmp_path strings) so the
    two LiDARs' resulting ``detector_config`` params are distinguishable by
    filename: top_lidar takes the marker-level ``velodyne.json5``, and
    front_lidar's own override selects ``seyond.json5``.
    """

    target_config = TARGETS_ROOT / "hollow_1000_aruco_4_v1.json5"
    marker_detector_config = (
        PACKAGE_ROOT / "config" / "board" / "hollow_1000" / "velodyne.json5"
    )
    front_lidar_override = (
        PACKAGE_ROOT / "config" / "board" / "hollow_1000" / "seyond.json5"
    )
    config_path = tmp_path / "two_lidar_detector_override.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /velodyne_points
      frame_id: velodyne
    front_lidar:
      pointcloud_topic: /iv_points
      frame_id: seyond
      detector_config: {front_lidar_override}

markers:
  calibration_board:
    target_config: {target_config}
    detector_config: {marker_detector_config}
    pairs:
      - [top_lidar, front_lidar]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
"""
    )
    return config_path


class _LaunchContext:
    """Minimal launch context for evaluating LaunchConfiguration values."""

    def __init__(self, config_file: Path):
        self.launch_configurations = {
            "config_file": str(config_file),
            "debug_mode": "false",
            "log_level": "info",
            "mode": "offline",
            "solver_mode": "continuous",
            "enable_overlay": "false",
            "enable_judge": "false",
        }

    def perform_substitution(self, substitution):
        return substitution.perform(self)


@pytest.fixture(scope="module")
def calibrate_launch() -> ModuleType:
    """Load the launch file directly; it is not a Python package module."""

    spec = importlib.util.spec_from_file_location("calibrate_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _nodes_for_package(nodes, package: str) -> list[Node]:
    return [
        node
        for node in nodes
        if isinstance(node, Node) and vars(node)["_Node__package"] == package
    ]


def _resolve(value):
    """Resolve a launch substitution stored in a Node's private graph data."""

    if isinstance(value, tuple):
        assert len(value) == 1
        value = value[0]
    if hasattr(value, "perform"):
        value = value.perform(None)
    if isinstance(value, str):
        # launch_ros serializes string parameter values as YAML documents.
        value = value.removesuffix("\n...\n")
    return value


def _parameters(node: Node) -> dict:
    parameters = {}
    for parameter_set in vars(node)["_Node__parameters"]:
        for key, value in parameter_set.items():
            parameters[_resolve(key)] = _resolve(value)
    return parameters


def _remappings(node: Node) -> dict[str, str]:
    return {
        _resolve(source): _resolve(destination)
        for source, destination in vars(node)["_Node__remappings"]
    }


def _namespace(node: Node) -> str:
    """Return the namespace a generated Node was constructed with.

    This is the same string `generate_nodes` built (e.g.
    ``calibration/{lidar_name}_{marker_name}``), not a launch substitution,
    so it needs no `_resolve`.
    """

    return vars(node)["_Node__node_namespace"]


def test_two_lidar_lidar_graph_routes_each_identity_and_detector_override(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """New-schema two-LiDAR graph routes both detector identities exactly,
    and each LiDAR's per-device ``detector_config`` override reaches its own
    detector node rather than the marker-level file both would otherwise
    share.

    New-schema successor of the deleted
    ``test_legacy_lidar_lidar_graph_routes_each_identity``. Still the ONLY
    launch-graph-level test proving a per-LiDAR detector override reaches
    the correct detector node: test_config_parser.py's
    ``test_two_lidar_per_lidar_detector_config_override_reaches_the_right_node``
    checks the parser's dataclasses, not the generated Node parameters.
    """

    config_path = _write_new_schema_two_lidar_detector_override_config(tmp_path)
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    solvers = _nodes_for_package(nodes, "lidar_to_lidar_solver")
    assert len(detectors) == 2
    assert len(solvers) == 1

    # Each LiDAR's own detector_config override must reach its own
    # detector, not the marker-level file both would otherwise share.
    detector_configs = {
        _namespace(detector): _parameters(detector)["detector_config"]
        for detector in detectors
    }
    assert detector_configs["calibration/top_lidar_calibration_board"].endswith(
        "hollow_1000/velodyne.json5"
    )
    assert detector_configs["calibration/front_lidar_calibration_board"].endswith(
        "hollow_1000/seyond.json5"
    )

    remappings = _remappings(solvers[0])
    assert remappings == {
        "lidar1_target_identity": "/calibration/top_lidar_calibration_board/target_identity",
        "lidar2_target_identity": "/calibration/front_lidar_calibration_board/target_identity",
    }


def test_new_schema_detector_gets_target_config_and_omits_legacy_keys(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """A new-schema marker routes target_config/detector_config to the node
    and must not carry the legacy board_detector_file/aruco_pattern_file keys
    at all -- select_config_source in lidar_board_detector's main.rs refuses
    to start if both sources are present, and a present-but-None value is
    still "present" to it.
    """

    config_path = _write_new_schema_detector_config(tmp_path)
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    # One detector per lidar in the pair (top_lidar, front_lidar).
    assert len(detectors) == 2

    for detector in detectors:
        params = _parameters(detector)

        assert params["target_config"].endswith("solid_600_aruco_1_v1.json5")
        assert params["detector_config"].endswith(
            "not-a-real-file-detector-tuning.json5"
        )
        assert "board_detector_file" not in params
        assert "aruco_pattern_file" not in params
        assert params["bbox_file"].endswith("not-a-real-file-bbox.json5")
        assert None not in params.values()


def test_new_schema_detector_without_bbox_config_omits_bbox_file(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """A marker that omits bbox_config still generates its graph without
    raising, and the detector's params carry no bbox_file key at all.

    config_parser's old "bbox_config is mandatory" guard is gone; the rule
    now lives solely in lidar_board_detector, conditional on
    detection_mode. A present-but-None bbox_file would trip launch_ros's
    eager Node() parameter normalization before any node started -- see
    commit eb58770 for the same failure mode on the camera-side nodes.
    """

    config_path = _write_new_schema_detector_config_no_bbox(tmp_path)
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    assert len(detectors) == 2

    for detector in detectors:
        params = _parameters(detector)
        assert "bbox_file" not in params
        assert None not in params.values()


def test_new_schema_camera_graph_generates_without_raising(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """Regression test for the eager-``normalize_parameters`` failure C1
    documented and C2 fixes: launch_ros's ``Node()`` validates parameter
    values at construction time, so a new-schema LiDAR+camera config used to
    raise ``TypeError: Unexpected type for parameter value None`` while
    ``generate_nodes`` was still building the node list -- before any node
    ever started. A future ``None`` sneaking into any camera-side parameter
    brings this straight back, so this test exists to name that failure
    mode, not just to assert on the resulting params (see the tests below
    for that).
    """

    config_path = _write_new_schema_camera_config(tmp_path)

    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    assert len(_nodes_for_package(nodes, "aruco_locator_node")) == 1
    assert len(_nodes_for_package(nodes, "lidar_to_camera_solver")) == 1


def test_new_schema_camera_nodes_get_target_config_and_omit_legacy_keys(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """A new-schema marker routes target_config to both camera-side nodes
    (aruco_locator_node, lidar_to_camera_solver) and must not carry the
    legacy aruco_config_file key at all -- both nodes refuse to start if
    both sources are present, and a present-but-None value is still
    "present" to them.
    """

    config_path = _write_new_schema_camera_config(tmp_path)
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    locators = _nodes_for_package(nodes, "aruco_locator_node")
    solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")
    assert len(locators) == 1
    assert len(solvers) == 1

    locator_params = _parameters(locators[0])
    assert locator_params["target_config"].endswith("solid_600_aruco_1_v1.json5")
    assert "aruco_config_file" not in locator_params
    # Mandatory under both schemas.
    assert locator_params["aruco_detector_config_file"].endswith("aruco_detector.json5")
    assert None not in locator_params.values()

    solver_params = _parameters(solvers[0])
    assert solver_params["target_config"].endswith("solid_600_aruco_1_v1.json5")
    assert "aruco_config_file" not in solver_params
    assert None not in solver_params.values()


def test_one_locator_per_camera_shared_across_pairs(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """A camera observed by two different LiDARs against the same marker
    still yields exactly one aruco_locator_node.

    `config_parser.py` dedupes `cameras_needed` by camera name (~line 570)
    before building locator nodes, so the two calibration pairs below
    (top_lidar, front_center) and (front_lidar, front_center) must collapse
    into a single locator even though they generate two board detectors and
    two solvers. This asserts the generated graph, not the parser's
    internal dedup set.
    """

    config_path = _write_new_schema_shared_camera_config(tmp_path)
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    locators = _nodes_for_package(nodes, "aruco_locator_node")
    solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")

    # One detector per LiDAR that observes the shared marker.
    assert len(detectors) == 2
    # One locator per camera -- deduped even though two pairs name it.
    assert len(locators) == 1
    # One solver per (lidar, camera) pair.
    assert len(solvers) == 2

    assert _parameters(locators[0])["target_config"].endswith(
        "solid_600_aruco_1_v1.json5"
    )


def test_one_selected_target_per_sensor(calibrate_launch: ModuleType, tmp_path: Path):
    """Every node belonging to a given sensor names the same target.

    Calibration correlates what several sensors saw of ONE physical board at
    one instant, so every sensor in a session observes the same target by
    construction -- that shared target is what makes a correspondence exist
    at all. `config_parser._validate` (config_parser.py:483-497) enforces it
    per device.

    What is left for this test is therefore not cross-target isolation,
    which would be meaningless here: it is that routing hands every node the
    same target value, and does not silently diverge for one of the several
    nodes that share a sensor.

    Reuses the shared-camera config from
    `test_one_locator_per_camera_shared_across_pairs`: front_center pairs
    with two different LiDARs, producing TWO separate
    lidar_to_camera_solver nodes that both mention front_center. A routing
    bug that read the wrong marker's target for one of those solvers, or
    let the locator disagree with either solver, would show up as more than
    one target_config value in front_center's group below -- grouped by the
    namespace `generate_nodes` actually assigned each node, not by
    re-reading the YAML config.
    """

    config_path = _write_new_schema_shared_camera_config(tmp_path)
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    detectors = {
        _namespace(n): _parameters(n)
        for n in _nodes_for_package(nodes, "lidar_board_detector")
    }
    locators = {
        _namespace(n): _parameters(n)
        for n in _nodes_for_package(nodes, "aruco_locator_node")
    }
    solvers = {
        _namespace(n): _parameters(n)
        for n in _nodes_for_package(nodes, "lidar_to_camera_solver")
    }

    assert set(detectors) == {
        "calibration/top_lidar_calibration_target",
        "calibration/front_lidar_calibration_target",
    }
    assert set(locators) == {"calibration/front_center"}
    assert set(solvers) == {
        "calibration/top_lidar_front_center",
        "calibration/front_lidar_front_center",
    }

    top_lidar_group = {
        detectors["calibration/top_lidar_calibration_target"]["target_config"],
        solvers["calibration/top_lidar_front_center"]["target_config"],
    }
    front_lidar_group = {
        detectors["calibration/front_lidar_calibration_target"]["target_config"],
        solvers["calibration/front_lidar_front_center"]["target_config"],
    }
    # front_center is the shared sensor: it appears via the ONE locator and
    # via BOTH solvers, so this is the group where a divergence would most
    # plausibly slip in.
    front_center_group = {
        locators["calibration/front_center"]["target_config"],
        solvers["calibration/top_lidar_front_center"]["target_config"],
        solvers["calibration/front_lidar_front_center"]["target_config"],
    }

    assert len(top_lidar_group) == 1
    assert len(front_lidar_group) == 1
    assert len(front_center_group) == 1
    assert next(iter(front_center_group)).endswith("solid_600_aruco_1_v1.json5")


def test_new_schema_lidar_camera_graph_routes_each_identity(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """New-schema LiDAR-camera graph routes both identity remaps exactly,
    mirroring `test_legacy_lidar_camera_graph_routes_each_identity` for the
    new schema.

    The expected values are derived from the *actual* generated detector
    and locator nodes' namespaces, not recomputed independently, so this
    checks that the solver's remaps resolve to its real observers' siblings
    rather than merely matching `_identity_topic_for_detection`'s own logic
    against itself.
    """

    config_path = _write_new_schema_camera_config(tmp_path)
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    locators = _nodes_for_package(nodes, "aruco_locator_node")
    solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")
    assert len(detectors) == 1
    assert len(locators) == 1
    assert len(solvers) == 1

    detector_namespace = _namespace(detectors[0])
    locator_namespace = _namespace(locators[0])

    remappings = _remappings(solvers[0])
    assert (
        remappings["lidar_target_identity"] == f"/{detector_namespace}/target_identity"
    )
    assert (
        remappings["camera_target_identity"] == f"/{locator_namespace}/target_identity"
    )


_SESSION_MANIFESTS = sorted(SESSIONS_ROOT.glob(f"*/{MANIFEST_NAME}"))

# An empty parametrization is collected as no test at all, so a glob that stops
# matching would remove this coverage while the suite still reported green --
# the same vacuous-pass failure the Rust collection guard in the justfile exists
# to prevent. Fail at import instead.
assert _SESSION_MANIFESTS, (
    f"no maintained session manifests found under {SESSIONS_ROOT}"
)


def _missing_recording(manifest: Path) -> str | None:
    """Why this session cannot be parsed here, or None if it can.

    A ``kind: bag`` session is verified against its recording at parse time --
    that check is the whole point of M-26 -- so a session whose bag is
    gitignored simply cannot be graphed on a machine that does not have it.
    Skipping is the honest outcome; silently dropping the case from the
    parametrization would take the coverage away wherever the bag IS present.
    """
    import yaml
    from lctk_launch.session import resolve_config_path

    data = yaml.safe_load(manifest.read_text(encoding="utf-8")).get("data") or {}
    if data.get("kind") != "bag":
        return None
    bag = Path(resolve_config_path(str(data["path"]), manifest.parent))
    if bag.is_dir():
        return None
    return (
        f"{manifest.parent.name} needs its recording at {bag}, which is "
        "gitignored -- see ros/lctk_sample_data/bags/README.md to obtain it"
    )


_SESSION_PARAMS = [
    pytest.param(
        manifest,
        id=manifest.parent.name,
        marks=(
            [pytest.mark.skip(reason=reason)]
            if (reason := _missing_recording(manifest))
            else []
        ),
    )
    for manifest in _SESSION_MANIFESTS
]


@pytest.mark.parametrize("config_path", _SESSION_PARAMS)
def test_maintained_sessions_use_only_the_new_target_schema(
    calibrate_launch: ModuleType, config_path: Path
):
    """Every maintained session under sessions/ generates a graph
    whose nodes carry only new-schema configuration keys -- target_config
    and detector_config -- and never the legacy board_detector_file/
    aruco_pattern_file/aruco_config_file keys, because W5-D cut every
    maintained config over to the Target Definition schema. The legacy
    compatibility path itself is not removed until W5-E1; it keeps its own
    tmp_path-fixture coverage elsewhere in this file now that these examples
    no longer double as its fixtures.

    Parametrized over the manifests discovered on disk so a session added
    later is covered automatically. twolidar-vlp32-falcon has no camera
    (0 locators, 0 lidar-camera solvers); the assertions below don't fold
    those empty lists into an `all(...)` that would pass vacuously -- every
    branch first asserts there is at least one node of a type that must
    exist in every session (a board detector), and only then checks the
    optional node types when the session actually produced any.
    """

    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    locators = _nodes_for_package(nodes, "aruco_locator_node")
    solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")

    # Every maintained session pairs at least one lidar with a marker, so a
    # config that produced zero detectors would itself be a finding, not a
    # silently-passing empty parametrization.
    assert detectors, (
        f"{config_path.parent.name} produced no lidar_board_detector nodes"
    )

    for detector in detectors:
        params = _parameters(detector)
        # A present-but-None value would still be "present" to the node's
        # select_config_source, so assert absence, not falsiness.
        assert "board_detector_file" not in params
        assert "aruco_pattern_file" not in params
        assert params["target_config"]
        assert params["detector_config"]
        assert None not in params.values()

    # twolidar-vlp32-falcon legitimately has no camera, so it produces zero
    # locators and zero lidar-camera solvers -- the loops below then check
    # nothing for it, which would silently pass even if camera-side routing
    # were broken for every OTHER session. Name the one session allowed to
    # take that empty path so a future session with a camera that
    # unexpectedly produced no camera-side nodes fails loudly instead of
    # being read as "just another two-lidar session".
    if not locators and not solvers:
        assert config_path.parent.name == "twolidar-vlp32-falcon", (
            f"{config_path.parent.name} unexpectedly produced no camera-side "
            "nodes (zero aruco_locator_node and zero lidar_to_camera_solver)"
        )

    for locator in locators:
        params = _parameters(locator)
        assert "aruco_config_file" not in params
        assert params["target_config"]
        assert None not in params.values()

    for solver in solvers:
        params = _parameters(solver)
        assert "aruco_config_file" not in params
        assert params["target_config"]
        assert None not in params.values()


def test_solid_600_handheld_session_selects_solid_target(
    calibrate_launch: ModuleType,
):
    """solid600-handheld-zed is the property the W5-D cutover exists to
    demonstrate end to end: a maintained config selecting a target other
    than the hollow board, wired through a coherent LiDAR-camera graph, with
    its own (tighter) sync window intact.

    No other test in this module pins this: the inverted
    ``test_maintained_sessions_use_only_the_new_target_schema`` above checks
    every session's params carry *some* target_config, not that the solid
    session's graph is internally coherent (one of each node kind, all three
    naming the same target, the detector's tuning preset, and the identity
    remaps resolving to this graph's own generated namespaces rather than a
    hand-recomputed string).
    """

    nodes = calibrate_launch.generate_nodes(
        _LaunchContext(_session("solid600-handheld-zed"))
    )

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    locators = _nodes_for_package(nodes, "aruco_locator_node")
    solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")
    assert len(detectors) == 1
    assert len(locators) == 1
    assert len(solvers) == 1

    detector_params = _parameters(detectors[0])
    locator_params = _parameters(locators[0])
    solver_params = _parameters(solvers[0])

    assert detector_params["target_config"].endswith("solid_600_aruco_1_v1.json5")
    assert locator_params["target_config"] == detector_params["target_config"]
    assert solver_params["target_config"] == detector_params["target_config"]

    assert detector_params["detector_config"].endswith("solid_600/velodyne.json5")
    assert "bbox_file" not in detector_params

    detector_namespace = _namespace(detectors[0])
    locator_namespace = _namespace(locators[0])
    remappings = _remappings(solvers[0])
    assert (
        remappings["lidar_target_identity"] == f"/{detector_namespace}/target_identity"
    )
    assert (
        remappings["camera_target_identity"] == f"/{locator_namespace}/target_identity"
    )

    # The solid session's board is hand-held and moving, so its sync window
    # is deliberately tighter than every hollow session's 100ms -- a future
    # edit that quietly widened it back to the hollow default should fail
    # here.
    assert solver_params["sync_tolerance_ms"] == 50.0


def test_sync_settings_reach_both_solver_kinds(calibrate_launch: ModuleType):
    """The config's `sync:` section reaches both `lidar_to_camera_solver`
    and `lidar_to_lidar_solver` parameters, not just one of the two node
    kinds that read it.

    `vehicle-multisensor` is the one maintained session with both solver
    kinds (L1-C1/L1-C2/L2-C3/L2-C4 lidar-camera solvers, and one L1-L2
    lidar-lidar solver), and it carries the same
    tolerance_ms=100/queue_size=100/drop_policy=reject_new `sync:` block
    every other maintained session does.
    """

    nodes = calibrate_launch.generate_nodes(
        _LaunchContext(_session("vehicle-multisensor"))
    )

    camera_solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")
    lidar_solvers = _nodes_for_package(nodes, "lidar_to_lidar_solver")
    assert len(camera_solvers) == 4
    assert len(lidar_solvers) == 1

    for solver in camera_solvers + lidar_solvers:
        params = _parameters(solver)
        assert params["sync_tolerance_ms"] == 100.0
        assert params["sync_queue_size"] == 100
        assert params["sync_drop_policy"] == "reject_new"


def _write_both_solver_kinds_config(tmp_path: Path, sync_block: str) -> Path:
    """Write a new-schema config with one lidar-camera pair and one
    lidar-lidar pair, so a single graph exercises both solver kinds.

    Only the routing of `sync:` values is under test here, so the marker
    uses opaque tmp_path detector/target paths like the other new-schema
    fixtures in this file -- the target/detector files themselves are never
    opened by the parser or this launch file.
    """

    target_config = TARGETS_ROOT / "solid_600_aruco_1_v1.json5"
    detector_config = tmp_path / "not-a-real-file-detector-tuning.json5"
    config_path = tmp_path / "both_solver_kinds.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    L1:
      pointcloud_topic: /sensing/lidar/front/points
      frame_id: lidar_front
    L2:
      pointcloud_topic: /sensing/lidar/rear/points
      frame_id: lidar_rear
  cameras:
    C1:
      image_topic: /sensing/camera/front_left/image
      frame_id: camera_front_left

reference_frame: L1

markers:
  M1:
    target_config: {target_config}
    detector_config: {detector_config}
    pairs:
      - [L1, C1]
      - [L1, L2]

{sync_block}
"""
    )
    return config_path


def test_sync_settings_non_default_window_carried_through_unchanged(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """A `sync:` window that differs from every maintained session's
    100/100/reject_new value reaches both solver kinds' parameters exactly
    as configured, rather than being replaced by a mode-derived preset.

    Before this change, `calibrate.launch.py` derived these three values
    from the `mode` launch argument and ignored the config file entirely;
    this asserts the specific non-default numbers below -- which do not
    match either the old offline (100/100/reject_new) or realtime
    (50/2/drop_oldest) preset -- survive unchanged from the config to the
    generated node parameters.
    """

    config_path = _write_both_solver_kinds_config(
        tmp_path,
        "sync:\n  tolerance_ms: 50\n  queue_size: 7\n  drop_policy: drop_oldest\n",
    )
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    camera_solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")
    lidar_solvers = _nodes_for_package(nodes, "lidar_to_lidar_solver")
    assert len(camera_solvers) == 1
    assert len(lidar_solvers) == 1

    for solver in camera_solvers + lidar_solvers:
        params = _parameters(solver)
        assert params["sync_tolerance_ms"] == 50.0
        assert params["sync_queue_size"] == 7
        assert params["sync_drop_policy"] == "drop_oldest"


def test_assisted_is_an_accepted_solver_mode(calibrate_launch: ModuleType):
    """`assisted` generates the same graph as the other modes.

    The mode changes only what the solver node does with its pairs; the graph
    around it -- detectors, locators, remappings -- is mode-independent, and a
    regression that made `assisted` generate a different graph would be a
    regression in the two older modes too.
    """
    context = _LaunchContext(_session("seyond-left"))
    context.launch_configurations["solver_mode"] = "assisted"

    nodes = calibrate_launch.generate_nodes(context)

    solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")
    assert len(solvers) == 1
    assert _parameters(solvers[0])["solver_mode"] == "assisted"


def test_an_unknown_solver_mode_is_refused_and_names_all_three(
    calibrate_launch: ModuleType,
):
    """A typo must not silently ship a different solver policy.

    Same reasoning as the L-05 guard on `mode`: falling back to a default here
    would run a whole capture session under a policy the operator did not ask
    for. The message names every accepted value so the typo is self-correcting.
    """
    context = _LaunchContext(_session("seyond-left"))
    context.launch_configurations["solver_mode"] = "automatic"

    with pytest.raises(RuntimeError) as excinfo:
        calibrate_launch.generate_nodes(context)

    message = str(excinfo.value)
    assert "automatic" in message
    for mode in ("continuous", "manual", "assisted"):
        assert mode in message


def test_assisted_defaults_reach_the_solver_when_no_section_is_given(
    calibrate_launch: ModuleType,
):
    """`assisted:` is optional, unlike `sync:`.

    `continuous` and `manual` read none of these values, so refusing a config
    that omits the section would break both modes over a setting neither uses.
    The maintained sessions carry no `assisted:` block, so this also pins that
    they still parse.
    """
    nodes = calibrate_launch.generate_nodes(_LaunchContext(_session("seyond-left")))

    params = _parameters(_nodes_for_package(nodes, "lidar_to_camera_solver")[0])
    assert params["stability_window_frames"] == 10
    assert params["stability_max_translation_m"] == 0.005
    assert params["novelty_position_tol_m"] == 0.05
    assert params["review_bind_host"] == "127.0.0.1"
    assert params["review_port"] == 8080


def test_an_assisted_section_overrides_only_what_it_names(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """Named keys override; unnamed ones keep the node's defaults."""
    # A maintained session, so the graph really has a lidar-camera solver to
    # carry the parameters; the tmp_path fixtures in this file are lidar-only.
    # Copied rather than read in place: seyond-left is `kind: live` and names
    # no $(session-dir) path, so it stays valid outside its own directory.
    config_path = tmp_path / "assisted.yaml"
    config_path.write_text(
        _session("seyond-left").read_text(encoding="utf-8") + "\nassisted:\n"
        "  stability_window_frames: 4\n"
        '  review_bind_host: "0.0.0.0"\n',
        encoding="utf-8",
    )

    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    params = _parameters(_nodes_for_package(nodes, "lidar_to_camera_solver")[0])
    assert params["stability_window_frames"] == 4
    assert params["review_bind_host"] == "0.0.0.0"
    assert params["stability_max_translation_m"] == 0.005, (
        "unnamed key keeps its default"
    )


def test_a_misspelled_assisted_key_is_refused(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """Silently ignoring an unknown key would leave the operator tuning a value
    that never reaches the node -- the failure would look like the gate not
    working rather than like a typo."""
    config_path = tmp_path / "misspelled.yaml"
    config_path.write_text(
        _session("seyond-left").read_text(encoding="utf-8")
        + "\nassisted:\n  stability_window_frame: 4\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="stability_window_frame"):
        calibrate_launch.generate_nodes(_LaunchContext(config_path))


def test_each_solver_gets_its_own_review_port(calibrate_launch: ModuleType):
    """A multi-pair config must not hand every solver the same port.

    `ReviewServer` binds eagerly in its constructor, so two solvers sharing a
    port means the second dies with `OSError: Address already in use` -- and it
    dies at startup, after the graph has already been reported as launched.
    `vehicle-multisensor` is the maintained session with four lidar-camera
    pairs.
    """
    context = _LaunchContext(_session("vehicle-multisensor"))
    context.launch_configurations["solver_mode"] = "assisted"

    nodes = calibrate_launch.generate_nodes(context)

    solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")
    assert len(solvers) == 4
    ports = [_parameters(solver)["review_port"] for solver in solvers]
    assert len(set(ports)) == 4, f"ports collide: {ports}"
    assert min(ports) == 8080, "the first pair keeps the configured port"
