"""
Configuration parser for multi-sensor calibration pipeline.

Parses YAML configuration files describing devices, markers, and calibration
relationships, then derives the required nodes and their connections.

Calibration pairs are defined within marker definitions via `pairs` keys.
The planner computes a spanning tree for TF broadcasting and identifies
validation edges.
"""

from __future__ import annotations

import math
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

# The only two drop policies Conflux's DetectionPairSource accepts.
VALID_SYNC_DROP_POLICIES = ("reject_new", "drop_oldest")


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


@dataclass
class LidarDevice:
    """LiDAR sensor device configuration."""

    name: str
    pointcloud_topic: str
    frame_id: str
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
    # Target Definition and detector tuning are the sole contract.
    target_config: str
    target_identity: TargetIdentity
    detector_config: str
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


@dataclass(frozen=True)
class SyncSettings:
    """Conflux synchronizer window/buffer settings.

    These are a physical judgement about the scene -- how far the
    calibration target can move between a camera frame and a LiDAR sweep --
    not something derivable from whether the data is live or recorded. They
    used to be silently derived from the launch `mode` argument; they are
    now required, explicit config, read from the `sync:` section.
    """

    tolerance_ms: float
    queue_size: int
    drop_policy: str


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
    detector_config: str
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
    sync: SyncSettings | None = None  # Set by _parse_sync; required, never left None


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
        self._sync: SyncSettings | None = None

    def parse(self) -> PipelineConfig:
        """Parse configuration file and derive pipeline configuration."""
        with open(self.config_path) as f:
            raw_config = yaml.safe_load(f)

        self._parse_devices(raw_config.get("devices", {}))

        markers_config = raw_config.get("markers", {})
        self._parse_markers(markers_config)
        self._parse_marker_pairs(markers_config)

        self._parse_sync(raw_config.get("sync"))

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

    def _parse_sync(self, sync_config: dict | None) -> None:
        """Parse and validate the required `sync:` section.

        The synchronizer window, buffer size and drop policy are a physical
        judgement about the scene -- how far the calibration target can move
        between a camera frame and a LiDAR sweep -- not something derivable
        from whether the data is live or recorded (that split is `mode`,
        which controls QoS only). This section is therefore required, with
        no mode-derived fallback: a config missing it is refused here rather
        than silently defaulting.
        """
        if sync_config is None:
            raise ValueError(
                "Missing required 'sync' section (tolerance_ms, queue_size, "
                "drop_policy). The synchronizer window is a physical "
                "judgement about the scene and must be stated explicitly in "
                "the config; it is not derived from 'mode'."
            )

        missing = [
            key
            for key in ("tolerance_ms", "queue_size", "drop_policy")
            if key not in sync_config
        ]
        if missing:
            raise ValueError(
                f"'sync' section is missing required key(s): {', '.join(missing)}"
            )

        self._sync = SyncSettings(
            tolerance_ms=self._parse_sync_tolerance_ms(sync_config["tolerance_ms"]),
            queue_size=self._parse_sync_queue_size(sync_config["queue_size"]),
            drop_policy=self._parse_sync_drop_policy(sync_config["drop_policy"]),
        )

    @staticmethod
    def _parse_sync_tolerance_ms(value: object) -> float:
        """Validate `sync.tolerance_ms`.

        Conflux only matches messages by time when a finite window is set:
        with an infinite window it skips the pruning step in
        `State::try_match` and pairs whatever is at the FRONT of each
        buffer -- i.e. by arrival order -- so two streams at different rates
        drift apart without bound. Measured on this repository's own
        conflux build: camera 10Hz + LiDAR 1Hz reaches a 53s gap INSIDE one
        "synchronized" group; 30Hz + 10Hz saturates at 10s; the seyond rig's
        5.4Hz + 4.4Hz passes 11s and keeps climbing. The same runs with a
        50ms window stay within 33ms.

        That failure is silent and ruinous: the solver pairs ArUco corners
        with a board pose on the assumption both saw the board at the same
        instant. Pair a camera frame with a LiDAR sweep 11s apart and the
        board has MOVED, so the solve is wrong while the reprojection error
        still looks fine. This is exactly why zero (infinite window) must be
        refused, and why the refusal has to be airtight: a bare
        `value <= 0` test does not catch `inf` (which is `> 0`) or `nan`
        (every comparison with `nan` is `False`). `float()` also turns the
        strings "inf", "Infinity" and any overflowing numeric literal such
        as "1e400" into `inf`, so the check below uses `math.isfinite`
        rather than a range comparison alone.
        """
        tolerance_ms: float | None
        try:
            tolerance_ms = None if isinstance(value, bool) else float(value)
        except (TypeError, ValueError):
            tolerance_ms = None
        if tolerance_ms is None or not math.isfinite(tolerance_ms) or tolerance_ms <= 0:
            raise ValueError(
                "sync.tolerance_ms must be a finite, strictly positive "
                f"number of milliseconds, got {value!r}"
            )
        return tolerance_ms

    @staticmethod
    def _parse_sync_queue_size(value: object) -> int:
        """Validate `sync.queue_size`.

        `bool` is a subclass of `int` in Python, so an explicit `bool` check
        is required -- otherwise `True` would silently pass as a queue size
        of 1.
        """
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(
                f"sync.queue_size must be a positive integer, got {value!r}"
            )
        return value

    @staticmethod
    def _parse_sync_drop_policy(value: object) -> str:
        """Validate `sync.drop_policy`."""
        if value not in VALID_SYNC_DROP_POLICIES:
            raise ValueError(
                f"sync.drop_policy must be one of {VALID_SYNC_DROP_POLICIES}, "
                f"got {value!r}"
            )
        return value

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
            if "board_config" in config:
                raise ValueError(
                    f"LiDAR '{name}' sets retired schema key 'board_config'. "
                    "This build reads only 'detector_config' -- see "
                    "config/examples/two_lidar.yaml, which overrides "
                    "detector_config per lidar. The retired board-tuning "
                    "file is not automatically translatable to detector "
                    "tuning: doing so would make the device's meaning "
                    "depend on the build that opened it, the same reason a "
                    "saved detection archive is not migrated automatically "
                    "(see lidar_to_camera_solver/detection_format.py). "
                    "There is no migration tool for launch YAML; replace "
                    "'board_config' with 'detector_config' by hand."
                )
            detector_config_override = config.get("detector_config")
            if detector_config_override:
                detector_config_override = resolve_package_path(
                    detector_config_override
                )
            bbox_config_override = config.get("bbox_config")
            if bbox_config_override:
                bbox_config_override = resolve_package_path(bbox_config_override)
            self.lidars[name] = LidarDevice(
                name=name,
                pointcloud_topic=config["pointcloud_topic"],
                frame_id=config["frame_id"],
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
            retired_keys = [
                key for key in ("type", "board_config", "aruco_config") if key in config
            ]
            if retired_keys:
                raise ValueError(
                    f"Marker '{name}' sets retired schema key(s) "
                    f"{', '.join(retired_keys)}. This build reads only "
                    "'target_config' and 'detector_config' -- see "
                    "config/examples/sample_data.yaml for a maintained "
                    "marker using them. The retired board/ArUco files "
                    "contain split, non-authoritative geometry, so there "
                    "is no automatic translation: doing so would make the "
                    "marker's meaning depend on the build that opened it, "
                    "the same reason a saved detection archive is not "
                    "migrated automatically (see "
                    "lidar_to_camera_solver/detection_format.py). There is "
                    "no migration tool for launch YAML; replace these keys "
                    "with 'target_config' and 'detector_config' by hand."
                )
            has_new_schema = "target_config" in config or "detector_config" in config
            if has_new_schema:
                self._parse_new_marker(name, config)
                continue
            raise ValueError(
                f"Marker '{name}' must provide 'target_config' and 'detector_config'."
            )

    def _parse_new_marker(self, name: str, config: dict) -> None:
        """Parse the target-definition schema without interpreting its geometry."""
        missing = [
            key for key in ("target_config", "detector_config") if not config.get(key)
        ]
        if missing:
            raise ValueError(
                f"Marker '{name}' is missing required parameter(s): "
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
            target_config=target_config,
            target_identity=target.identity,
            detector_config=detector_config,
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
        assert self._sync is not None  # _parse_sync always sets it or raises
        config = PipelineConfig(
            lidars=dict(self.lidars),
            cameras=dict(self.cameras),
            sync=self._sync,
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

            # bbox_config is NOT unconditionally required here: it is read
            # only when the detector tuning file selects detection_mode=bbox,
            # and that file is an opaque path to this parser. Both cases are
            # live -- sample_data.yaml's detector tuning is now
            # config/board/hollow_1000/velodyne_bbox.json5, which is bbox
            # mode and genuinely needs its crop box, and it is the only
            # maintained example that does. Only the node parses detector
            # tuning, so only the node can tell the two apart; it owns the
            # rule and reports it as "bbox_file is required when
            # detector_config selects detection_mode=bbox"
            # (ros/lidar_board_detector/src/main.rs). Enforcement therefore
            # moved from launch parse to node startup.

            node_name = f"board_detector_{lidar_name}_{marker_name}"
            namespace = f"calibration/{lidar_name}_{marker_name}"
            output_topic = f"/{namespace}/calibration_board_detections"

            # Per-lidar overrides take precedence over marker-level configs
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
            # camera observes must share a single Calibration Target Identity --
            # every target_config resolves to exactly one physical pattern, so
            # this check alone now carries the M-10 invariant (it used to be
            # backed by a second, separate conflict check over `aruco_config`
            # paths; that legacy field is gone as of W5-E1). Fail loudly on a
            # conflict instead of silently using whichever pair came first
            # (which would make the detector and the solver for another pair
            # disagree on the pattern).
            target_identities = set()
            for pair in self.calibration_pairs:
                if camera_name in (pair.device1, pair.device2):
                    marker = self.markers[pair.marker]
                    target_identities.add(marker.target_identity)

            if len(target_identities) != 1:
                # _validate reports the same conflict per sensor, but retain a
                # local guard because this construction owns one locator.
                raise ValueError(
                    f"Camera {camera_name} does not have one Calibration Target Identity"
                )

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
