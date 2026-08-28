"""
Configuration parser for multi-sensor calibration pipeline.

Parses YAML configuration files describing devices, markers, and calibration
relationships, then derives the required nodes and their connections.

Calibration pairs are defined within marker definitions via `pairs` keys.
The planner computes a spanning tree for TF broadcasting and identifies
validation edges.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import yaml
from lctk_target import TargetIdentity, load_target

from lctk_launch.calibration_planner import CalibrationPlan, compute_plan, format_plan

# Detector tuning used when a marker does not name its own. The node itself has no hidden
# defaults; this is the launch layer supplying the shipped file so pre-existing configs, which
# predate the aruco_detector_config key, keep working.
DEFAULT_ARUCO_DETECTOR_CONFIG = (
    "$(find-pkg-share lctk_launch)/config/aruco/aruco_detector.json5"
)

# W5-A compatibility has exactly one meaning: the physical target used by all
# maintained pre-target-config examples.  Do not infer a target from legacy
# board/ArUco files: those files contain split, non-authoritative geometry.
LEGACY_HOLLOW_TARGET_CONFIG = (
    "$(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5"
)


def resolve_package_path(path: str) -> str:
    """
    Resolve ROS2 package path substitutions like $(find-pkg-share package_name).

    Args:
        path: Path string that may contain $(find-pkg-share ...) substitutions

    Returns:
        Resolved absolute path string
    """
    # Pattern to match $(find-pkg-share package_name)
    pattern = r"\$\(find-pkg-share\s+([^)]+)\)"

    def replace_func(match):
        package_name = match.group(1).strip()
        try:
            from ament_index_python.packages import (
                PackageNotFoundError,
                get_package_share_directory,
            )

            return get_package_share_directory(package_name)
        except (PackageNotFoundError, ValueError) as e:
            # PackageNotFoundError: not on AMENT_PREFIX_PATH. ValueError: the
            # package is registered but its share directory does not exist.
            raise ValueError(f"Failed to find package '{package_name}': {e}") from e

    return re.sub(pattern, replace_func, path)


class DeviceType(Enum):
    LIDAR = "lidar"
    CAMERA = "camera"


class MarkerType(Enum):
    HOLLOW_BOARD = "hollow_board"


@dataclass
class LidarDevice:
    """LiDAR sensor device configuration."""

    name: str
    pointcloud_topic: str
    frame_id: str
    board_config_override: str | None = (
        None  # Per-lidar board_config, overrides marker's
    )
    detector_config_override: str | None = None
    bbox_config_override: str | None = None  # Per-lidar bbox_config, overrides marker's


@dataclass
class CameraDevice:
    """Camera sensor device configuration."""

    name: str
    image_topic: str
    frame_id: str


@dataclass
class Marker:
    """Calibration marker configuration."""

    name: str
    marker_type: MarkerType | None
    # Target Definition and detector tuning are the new, separate contract.
    target_config: str
    target_identity: TargetIdentity
    detector_config: str | None
    # The remaining legacy fields exist only until W5-E1.  They let the old
    # launch graph run maintained hollow examples while W5-C is pending.
    board_config: str | None
    aruco_config: str | None = None  # Path to ArUco pattern config (optional)
    bbox_config: str | None = None  # Path to bounding box filter config (optional)
    # Path to ArUco *detector* tuning (corner refinement, adaptive threshold). Optional:
    # falls back to DEFAULT_ARUCO_DETECTOR_CONFIG, so existing configs keep working.
    aruco_detector_config: str | None = None


@dataclass
class CalibrationPair:
    """A pair of devices to be calibrated together."""

    device1: str
    device2: str
    marker: str


@dataclass
class LidarBoardDetectorNode:
    """Configuration for a lidar_board_detector node instance."""

    node_name: str
    namespace: str
    lidar_name: str
    marker_name: str
    pointcloud_topic: str
    target_config: str
    target_identity: TargetIdentity
    detector_config: str | None
    board_config: str | None
    aruco_config: str | None
    bbox_config: str | None
    output_topic: str  # Detection output topic


@dataclass
class ArucoLocatorNode:
    """Configuration for an aruco_locator_node instance."""

    node_name: str
    namespace: str
    camera_name: str
    image_topic: str
    frame_id: str
    target_config: str
    target_identity: TargetIdentity
    aruco_config: str | None
    aruco_detector_config: str
    output_topic: str  # Detection output topic


@dataclass
class LidarCameraSolverNode:
    """Configuration for an extrinsic_solver (lidar-camera) node instance."""

    node_name: str
    namespace: str
    lidar_name: str
    camera_name: str
    marker_name: str
    parent_frame: str  # LiDAR frame
    child_frame: str  # Camera frame
    board_detections_topic: str
    aruco_detections_topic: str
    camera_topic: str  # For camera_info derivation
    target_config: str
    target_identity: TargetIdentity
    aruco_config: str | None
    output_topic: str


@dataclass
class LidarLidarSolverNode:
    """Configuration for a lidar_to_lidar_solver node instance."""

    node_name: str
    namespace: str
    lidar1_name: str
    lidar2_name: str
    marker_name: str
    lidar1_frame: str
    lidar2_frame: str
    lidar1_detections_topic: str
    lidar2_detections_topic: str
    output_topic: str


@dataclass
class PipelineConfig:
    """Complete pipeline configuration derived from user config."""

    lidar_board_detectors: list[LidarBoardDetectorNode] = field(default_factory=list)
    aruco_locators: list[ArucoLocatorNode] = field(default_factory=list)
    lidar_camera_solvers: list[LidarCameraSolverNode] = field(default_factory=list)
    lidar_lidar_solvers: list[LidarLidarSolverNode] = field(default_factory=list)
    lidars: dict[str, LidarDevice] = field(default_factory=dict)
    cameras: dict[str, CameraDevice] = field(default_factory=dict)
    calibration_plan: CalibrationPlan | None = None  # Set by planner
    calibration_plan_text: str | None = None  # Formatted ASCII plan for display


class CalibrationConfigParser:
    """
    Parser for multi-sensor calibration configuration files.

    Reads a YAML configuration describing devices, markers, and calibration
    pairs, then derives the complete pipeline of nodes needed.
    """

    def __init__(self, config_path: str):
        self.config_path = Path(config_path)
        self.lidars: dict[str, LidarDevice] = {}
        self.cameras: dict[str, CameraDevice] = {}
        self.markers: dict[str, Marker] = {}
        self.calibration_pairs: list[CalibrationPair] = []
        self._reference_frame: str | None = None

    def parse(self) -> PipelineConfig:
        """Parse configuration file and derive pipeline configuration."""
        with open(self.config_path) as f:
            raw_config = yaml.safe_load(f)

        self._parse_devices(raw_config.get("devices", {}))

        markers_config = raw_config.get("markers", {})
        self._parse_markers(markers_config)
        self._parse_marker_pairs(markers_config)

        self._reference_frame = raw_config.get("reference_frame")
        if self._reference_frame is None:
            # Default to first lidar
            if self.lidars:
                self._reference_frame = next(iter(self.lidars))
            else:
                raise ValueError(
                    "reference_frame must be specified when no lidars are defined"
                )

        self._validate()

        pipeline = self._derive_pipeline()
        self._run_planner(pipeline)

        return pipeline

    def _parse_marker_pairs(self, markers_config: dict) -> None:
        """Extract calibration pairs from marker definitions (new format)."""
        # M-10: de-duplicate pairs. Board detectors and aruco locators are keyed
        # by a set, but solver nodes are generated per pair, so a repeated pair
        # would create two nodes with identical name and namespace (a ROS name
        # collision). Skip pairs already seen (order-independent, per marker).
        seen: set = set()
        for marker_name, config in markers_config.items():
            for pair in config.get("pairs", []):
                if len(pair) != 2:
                    raise ValueError(
                        f"Marker {marker_name}: each pair must have exactly "
                        f"2 devices, got {len(pair)}: {pair}"
                    )
                key = (frozenset(pair), marker_name)
                if key in seen:
                    continue
                seen.add(key)
                self.calibration_pairs.append(
                    CalibrationPair(
                        device1=pair[0],
                        device2=pair[1],
                        marker=marker_name,
                    )
                )

    def _run_planner(self, pipeline: PipelineConfig) -> None:
        """Run the calibration planner and attach results to pipeline."""
        pairs = [(p.device1, p.device2, p.marker) for p in self.calibration_pairs]
        assert self._reference_frame is not None  # validated in parse()
        plan = compute_plan(
            pairs=pairs,
            lidars=set(self.lidars.keys()),
            cameras=set(self.cameras.keys()),
            reference_frame=self._reference_frame,
        )

        # Build device → frame_id map for display
        device_frame_ids: dict[str, str] = {}
        for name, lidar in self.lidars.items():
            device_frame_ids[name] = lidar.frame_id
        for name, camera in self.cameras.items():
            device_frame_ids[name] = camera.frame_id

        pipeline.calibration_plan = plan
        pipeline.calibration_plan_text = format_plan(plan, device_frame_ids)

    def _parse_devices(self, devices_config: dict) -> None:
        """Parse device definitions."""
        # Parse LiDARs
        for name, config in devices_config.get("lidars", {}).items():
            board_config_override = config.get("board_config")
            if board_config_override:
                board_config_override = resolve_package_path(board_config_override)
            detector_config_override = config.get("detector_config")
            if detector_config_override:
                detector_config_override = resolve_package_path(
                    detector_config_override
                )
            if board_config_override and detector_config_override:
                raise ValueError(
                    f"LiDAR '{name}' supplies both legacy 'board_config' and "
                    "new 'detector_config'. Select one configuration schema."
                )
            bbox_config_override = config.get("bbox_config")
            if bbox_config_override:
                bbox_config_override = resolve_package_path(bbox_config_override)
            self.lidars[name] = LidarDevice(
                name=name,
                pointcloud_topic=config["pointcloud_topic"],
                frame_id=config["frame_id"],
                board_config_override=board_config_override,
                detector_config_override=detector_config_override,
                bbox_config_override=bbox_config_override,
            )

        # Parse cameras
        for name, config in devices_config.get("cameras", {}).items():
            self.cameras[name] = CameraDevice(
                name=name,
                image_topic=config["image_topic"],
                frame_id=config["frame_id"],
            )

    def _parse_markers(self, markers_config: dict) -> None:
        """Parse marker definitions."""
        for name, config in markers_config.items():
            has_new_schema = "target_config" in config or "detector_config" in config
            has_legacy_schema = any(
                key in config for key in ("type", "board_config", "aruco_config")
            )
            if has_new_schema and has_legacy_schema:
                raise ValueError(
                    f"Marker '{name}' mixes legacy (type/board_config/aruco_config) "
                    "and new (target_config/detector_config) parameters. Select one schema."
                )
            if has_new_schema:
                self._parse_new_marker(name, config)
                continue
            if has_legacy_schema:
                self._parse_legacy_marker(name, config)
                continue
            raise ValueError(
                f"Marker '{name}' must provide 'target_config' and 'detector_config', "
                "or use the temporary hollow-board compatibility schema."
            )

    def _parse_new_marker(self, name: str, config: dict) -> None:
        """Parse the target-definition schema without interpreting its geometry."""
        missing = [
            key for key in ("target_config", "detector_config") if not config.get(key)
        ]
        if missing:
            raise ValueError(
                f"Marker '{name}' is missing required new-schema parameter(s): "
                f"{', '.join(missing)}"
            )
        target_config = resolve_package_path(config["target_config"])
        target = load_target(target_config)
        detector_config = resolve_package_path(config["detector_config"])
        bbox_config = config.get("bbox_config")
        if bbox_config:
            bbox_config = resolve_package_path(bbox_config)
        aruco_detector_config = config.get("aruco_detector_config")
        if aruco_detector_config:
            aruco_detector_config = resolve_package_path(aruco_detector_config)

        self.markers[name] = Marker(
            name=name,
            marker_type=None,
            target_config=target_config,
            target_identity=target.identity,
            detector_config=detector_config,
            board_config=None,
            aruco_config=None,
            bbox_config=bbox_config,
            aruco_detector_config=aruco_detector_config,
        )

    def _parse_legacy_marker(self, name: str, config: dict) -> None:
        """Translate the retired split schema to the one explicit hollow target."""
        if "type" not in config or "board_config" not in config:
            raise ValueError(
                f"Legacy marker '{name}' requires both 'type: hollow_board' and "
                "'board_config'. Migrate to target_config/detector_config."
            )
        marker_type = MarkerType(config["type"])
        target_config = resolve_package_path(LEGACY_HOLLOW_TARGET_CONFIG)
        target = load_target(target_config)
        board_config = resolve_package_path(config["board_config"])
        aruco_config = config.get("aruco_config")
        if aruco_config:
            aruco_config = resolve_package_path(aruco_config)
        bbox_config = config.get("bbox_config")
        if bbox_config:
            bbox_config = resolve_package_path(bbox_config)
        aruco_detector_config = config.get("aruco_detector_config")
        if aruco_detector_config:
            aruco_detector_config = resolve_package_path(aruco_detector_config)

        self.markers[name] = Marker(
            name=name,
            marker_type=marker_type,
            target_config=target_config,
            target_identity=target.identity,
            detector_config=None,
            board_config=board_config,
            aruco_config=aruco_config,
            bbox_config=bbox_config,
            aruco_detector_config=aruco_detector_config,
        )

    def _validate(self) -> None:
        """Validate configuration consistency."""
        all_devices = set(self.lidars.keys()) | set(self.cameras.keys())
        identities_by_device: dict[str, TargetIdentity] = {}

        for pair in self.calibration_pairs:
            # Check devices exist
            if pair.device1 not in all_devices:
                raise ValueError(f"Unknown device in calibration pair: {pair.device1}")
            if pair.device2 not in all_devices:
                raise ValueError(f"Unknown device in calibration pair: {pair.device2}")

            # Check marker exists
            if pair.marker not in self.markers:
                raise ValueError(f"Unknown marker in calibration pair: {pair.marker}")

            # Check pair types are valid
            d1_is_lidar = pair.device1 in self.lidars
            d2_is_lidar = pair.device2 in self.lidars
            d1_is_camera = pair.device1 in self.cameras
            d2_is_camera = pair.device2 in self.cameras

            valid_pair = (
                (d1_is_lidar and d2_is_camera)
                or (d1_is_camera and d2_is_lidar)
                or (d1_is_lidar and d2_is_lidar)
            )

            if not valid_pair:
                raise ValueError(
                    f"Invalid device pair type: {pair.device1}, {pair.device2}. "
                    "Supported: lidar-camera, lidar-lidar"
                )

            marker = self.markers[pair.marker]
            for device_name in (pair.device1, pair.device2):
                lidar = self.lidars.get(device_name)
                if (
                    lidar is not None
                    and marker.marker_type is None
                    and lidar.board_config_override is not None
                ):
                    raise ValueError(
                        f"Sensor '{device_name}' uses legacy 'board_config' while marker "
                        f"'{pair.marker}' uses new 'target_config'/'detector_config'. "
                        "Conflicting configuration schemas cannot be combined."
                    )
                if (
                    lidar is not None
                    and marker.marker_type is not None
                    and lidar.detector_config_override is not None
                ):
                    raise ValueError(
                        f"Sensor '{device_name}' uses new 'detector_config' while marker "
                        f"'{pair.marker}' uses legacy 'type'/'board_config'/'aruco_config'. "
                        "Conflicting configuration schemas cannot be combined."
                    )
                previous = identities_by_device.setdefault(
                    device_name, marker.target_identity
                )
                if previous != marker.target_identity:
                    raise ValueError(
                        f"Sensor '{device_name}' is assigned different Calibration Target "
                        "Identities: "
                        f"{previous.target_id}@{previous.revision} "
                        f"({previous.semantic_sha256}) and "
                        f"{marker.target_identity.target_id}@{marker.target_identity.revision} "
                        f"({marker.target_identity.semantic_sha256}). One sensor must use "
                        "one semantic target per launch."
                    )

    def _get_device_type(self, device_name: str) -> DeviceType:
        """Get the type of a device by name."""
        if device_name in self.lidars:
            return DeviceType.LIDAR
        elif device_name in self.cameras:
            return DeviceType.CAMERA
        else:
            raise ValueError(f"Unknown device: {device_name}")

    def _derive_pipeline(self) -> PipelineConfig:
        """Derive the complete pipeline configuration from parsed config."""
        config = PipelineConfig(
            lidars=dict(self.lidars),
            cameras=dict(self.cameras),
        )

        # Collect unique (lidar, marker) pairs for board detectors
        lidar_marker_pairs: set[tuple[str, str]] = set()
        for pair in self.calibration_pairs:
            if pair.device1 in self.lidars:
                lidar_marker_pairs.add((pair.device1, pair.marker))
            if pair.device2 in self.lidars:
                lidar_marker_pairs.add((pair.device2, pair.marker))

        # Create lidar_board_detector nodes
        for lidar_name, marker_name in sorted(lidar_marker_pairs):
            lidar = self.lidars[lidar_name]
            marker = self.markers[marker_name]

            # The old graph still needs a standalone pattern config.  It only
            # applies to the compatibility schema and is removed with that
            # graph path in W5-E1; target-config markers provide geometry from
            # their Target Definition instead.
            if marker.marker_type is not None and not marker.aruco_config:
                raise ValueError(
                    f"Marker '{marker_name}' (used by lidar '{lidar_name}') is missing "
                    "'aruco_config', which the lidar_board_detector requires."
                )
            # bbox_config is NOT unconditionally required here: it is read
            # only when the detector tuning file selects detection_mode=bbox,
            # and that file is an opaque path to this parser. Both cases are
            # live -- sample_data.yaml's board_detector.json5 is bbox mode and
            # genuinely needs its crop box, while the hollow_1000, solid_600
            # and board_detector_{velodyne,seyond} presets are bbox_free and
            # never read it. Only the node parses detector tuning, so only the
            # node can tell the two apart; it owns the rule and reports it as
            # "bbox_file is required when detector_config selects
            # detection_mode=bbox" (ros/lidar_board_detector/src/main.rs).
            # Enforcement therefore moved from launch parse to node startup.

            node_name = f"board_detector_{lidar_name}_{marker_name}"
            namespace = f"calibration/{lidar_name}_{marker_name}"
            output_topic = f"/{namespace}/calibration_board_detections"

            # Per-lidar overrides take precedence over marker-level configs
            board_config = lidar.board_config_override or marker.board_config
            detector_config = lidar.detector_config_override or marker.detector_config
            bbox_config = lidar.bbox_config_override or marker.bbox_config

            config.lidar_board_detectors.append(
                LidarBoardDetectorNode(
                    node_name=node_name,
                    namespace=namespace,
                    lidar_name=lidar_name,
                    marker_name=marker_name,
                    pointcloud_topic=lidar.pointcloud_topic,
                    target_config=marker.target_config,
                    target_identity=marker.target_identity,
                    detector_config=detector_config,
                    board_config=board_config,
                    aruco_config=marker.aruco_config,
                    bbox_config=bbox_config,
                    output_topic=output_topic,
                )
            )

        # Collect unique cameras for aruco locators
        cameras_needed: set[str] = set()
        for pair in self.calibration_pairs:
            if pair.device1 in self.cameras:
                cameras_needed.add(pair.device1)
            if pair.device2 in self.cameras:
                cameras_needed.add(pair.device2)

        # Create aruco_locator_node nodes
        for camera_name in sorted(cameras_needed):
            camera = self.cameras[camera_name]

            # M-10: one aruco_locator is created per camera, so all markers this
            # camera observes must share a single ArUco pattern. Collect the
            # distinct configs and fail loudly on a conflict instead of silently
            # using whichever came first (which would make the detector and the
            # solver for another pair disagree on the pattern).
            aruco_configs = set()
            target_identities = set()
            for pair in self.calibration_pairs:
                if camera_name in (pair.device1, pair.device2):
                    marker = self.markers[pair.marker]
                    target_identities.add(marker.target_identity)
                    if marker.aruco_config:
                        aruco_configs.add(marker.aruco_config)

            if len(target_identities) != 1:
                # _validate reports the same conflict per sensor, but retain a
                # local guard because this construction owns one locator.
                raise ValueError(
                    f"Camera {camera_name} does not have one Calibration Target Identity"
                )
            if len(aruco_configs) > 1:
                raise ValueError(
                    f"Camera {camera_name} observes markers with different ArUco "
                    f"configs {sorted(aruco_configs)}; a single aruco_locator can only "
                    "use one pattern. Use the same ArUco pattern for all boards this "
                    "camera observes."
                )
            aruco_config = next(iter(aruco_configs)) if aruco_configs else None

            # Same one-locator-per-camera constraint applies to the detector tuning (H-08).
            # Markers that omit the key fall back to the shipped default.
            aruco_detector_configs = set()
            for pair in self.calibration_pairs:
                if camera_name in (pair.device1, pair.device2):
                    marker = self.markers[pair.marker]
                    aruco_detector_configs.add(
                        marker.aruco_detector_config
                        or resolve_package_path(DEFAULT_ARUCO_DETECTOR_CONFIG)
                    )

            if len(aruco_detector_configs) > 1:
                raise ValueError(
                    f"Camera {camera_name} observes markers with different ArUco detector "
                    f"configs {sorted(aruco_detector_configs)}; a single aruco_locator can only "
                    "use one. Use the same aruco_detector_config for all boards this camera "
                    "observes."
                )
            aruco_detector_config = next(iter(aruco_detector_configs))

            node_name = f"aruco_locator_{camera_name}"
            namespace = f"calibration/{camera_name}"
            output_topic = f"/{namespace}/aruco_detections"

            config.aruco_locators.append(
                ArucoLocatorNode(
                    node_name=node_name,
                    namespace=namespace,
                    camera_name=camera_name,
                    image_topic=camera.image_topic,
                    frame_id=camera.frame_id,
                    target_config=self.markers[
                        next(
                            pair.marker
                            for pair in self.calibration_pairs
                            if camera_name in (pair.device1, pair.device2)
                        )
                    ].target_config,
                    target_identity=next(iter(target_identities)),
                    aruco_config=aruco_config,
                    aruco_detector_config=aruco_detector_config,
                    output_topic=output_topic,
                )
            )

        # Create solver nodes for each calibration pair
        for pair in self.calibration_pairs:
            d1_type = self._get_device_type(pair.device1)
            d2_type = self._get_device_type(pair.device2)

            if d1_type == DeviceType.LIDAR and d2_type == DeviceType.CAMERA:
                self._add_lidar_camera_solver(
                    config, pair.device1, pair.device2, pair.marker
                )
            elif d1_type == DeviceType.CAMERA and d2_type == DeviceType.LIDAR:
                self._add_lidar_camera_solver(
                    config, pair.device2, pair.device1, pair.marker
                )
            elif d1_type == DeviceType.LIDAR and d2_type == DeviceType.LIDAR:
                self._add_lidar_lidar_solver(
                    config, pair.device1, pair.device2, pair.marker
                )

        return config

    def _add_lidar_camera_solver(
        self,
        config: PipelineConfig,
        lidar_name: str,
        camera_name: str,
        marker_name: str,
    ) -> None:
        """Add a lidar-camera solver node to the config."""
        lidar = self.lidars[lidar_name]
        camera = self.cameras[camera_name]
        marker = self.markers[marker_name]

        node_name = f"solver_{lidar_name}_{camera_name}"
        namespace = f"calibration/{lidar_name}_{camera_name}"

        # Find the board detector output topic (matches lidar_board_detector's publisher)
        board_topic = (
            f"/calibration/{lidar_name}_{marker_name}/calibration_board_detections"
        )
        aruco_topic = f"/calibration/{camera_name}/aruco_detections"
        output_topic = f"/{namespace}/extrinsic_transform"

        if marker.marker_type is not None and marker.aruco_config is None:
            raise ValueError(
                f"ArUco config required for lidar-camera solver with marker {marker_name}"
            )

        config.lidar_camera_solvers.append(
            LidarCameraSolverNode(
                node_name=node_name,
                namespace=namespace,
                lidar_name=lidar_name,
                camera_name=camera_name,
                marker_name=marker_name,
                parent_frame=lidar.frame_id,
                child_frame=camera.frame_id,
                board_detections_topic=board_topic,
                aruco_detections_topic=aruco_topic,
                camera_topic=camera.image_topic,
                target_config=marker.target_config,
                target_identity=marker.target_identity,
                aruco_config=marker.aruco_config,
                output_topic=output_topic,
            )
        )

    def _add_lidar_lidar_solver(
        self,
        config: PipelineConfig,
        lidar1_name: str,
        lidar2_name: str,
        marker_name: str,
    ) -> None:
        """Add a lidar-lidar solver node to the config."""
        lidar1 = self.lidars[lidar1_name]
        lidar2 = self.lidars[lidar2_name]

        node_name = f"solver_{lidar1_name}_{lidar2_name}"
        namespace = f"calibration/{lidar1_name}_{lidar2_name}"

        # Find the board detector output topics (matches lidar_board_detector's publisher)
        lidar1_topic = (
            f"/calibration/{lidar1_name}_{marker_name}/calibration_board_detections"
        )
        lidar2_topic = (
            f"/calibration/{lidar2_name}_{marker_name}/calibration_board_detections"
        )
        output_topic = f"/{namespace}/lidar_to_lidar_transform"

        config.lidar_lidar_solvers.append(
            LidarLidarSolverNode(
                node_name=node_name,
                namespace=namespace,
                lidar1_name=lidar1_name,
                lidar2_name=lidar2_name,
                marker_name=marker_name,
                lidar1_frame=lidar1.frame_id,
                lidar2_frame=lidar2.frame_id,
                lidar1_detections_topic=lidar1_topic,
                lidar2_detections_topic=lidar2_topic,
                output_topic=output_topic,
            )
        )


def parse_config(config_path: str) -> PipelineConfig:
    """Convenience function to parse a configuration file."""
    parser = CalibrationConfigParser(config_path)
    return parser.parse()
