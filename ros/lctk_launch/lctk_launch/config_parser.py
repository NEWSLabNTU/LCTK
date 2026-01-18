"""
Configuration parser for multi-sensor calibration pipeline.

Parses YAML configuration files describing devices, markers, and calibration
relationships, then derives the required nodes and their connections.
"""

import re
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import yaml


def resolve_package_path(path: str) -> str:
    """
    Resolve ROS2 package path substitutions like $(find-pkg-share package_name).

    Args:
        path: Path string that may contain $(find-pkg-share ...) substitutions

    Returns:
        Resolved absolute path string
    """
    # Pattern to match $(find-pkg-share package_name)
    pattern = r'\$\(find-pkg-share\s+([^)]+)\)'

    def replace_func(match):
        package_name = match.group(1).strip()
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory(package_name)
        except Exception as e:
            raise ValueError(f"Failed to find package '{package_name}': {e}")

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
    marker_type: MarkerType
    board_config: str  # Path to board configuration file
    aruco_config: Optional[str] = None  # Path to ArUco pattern config (optional)
    bbox_config: Optional[str] = None  # Path to bounding box filter config (optional)


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
    board_config: str
    aruco_config: Optional[str]
    bbox_config: Optional[str]
    output_topic: str  # Detection output topic


@dataclass
class ArucoLocatorNode:
    """Configuration for an aruco_locator_node instance."""

    node_name: str
    namespace: str
    camera_name: str
    image_topic: str
    frame_id: str
    aruco_config: str
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
    aruco_config: str  # Path to ArUco pattern config (required for solver)
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

    lidar_board_detectors: List[LidarBoardDetectorNode] = field(default_factory=list)
    aruco_locators: List[ArucoLocatorNode] = field(default_factory=list)
    lidar_camera_solvers: List[LidarCameraSolverNode] = field(default_factory=list)
    lidar_lidar_solvers: List[LidarLidarSolverNode] = field(default_factory=list)


class CalibrationConfigParser:
    """
    Parser for multi-sensor calibration configuration files.

    Reads a YAML configuration describing devices, markers, and calibration
    pairs, then derives the complete pipeline of nodes needed.
    """

    def __init__(self, config_path: str):
        self.config_path = Path(config_path)
        self.lidars: Dict[str, LidarDevice] = {}
        self.cameras: Dict[str, CameraDevice] = {}
        self.markers: Dict[str, Marker] = {}
        self.calibration_pairs: List[CalibrationPair] = []

    def parse(self) -> PipelineConfig:
        """Parse configuration file and derive pipeline configuration."""
        with open(self.config_path) as f:
            raw_config = yaml.safe_load(f)

        self._parse_devices(raw_config.get("devices", {}))
        self._parse_markers(raw_config.get("markers", {}))
        self._parse_calibration_pairs(raw_config.get("calibration_pairs", []))
        self._validate()

        return self._derive_pipeline()

    def _parse_devices(self, devices_config: dict) -> None:
        """Parse device definitions."""
        # Parse LiDARs
        for name, config in devices_config.get("lidars", {}).items():
            self.lidars[name] = LidarDevice(
                name=name,
                pointcloud_topic=config["pointcloud_topic"],
                frame_id=config["frame_id"],
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
            marker_type = MarkerType(config["type"])
            # Resolve package paths in config file paths
            board_config = resolve_package_path(config["board_config"])
            aruco_config = config.get("aruco_config")
            if aruco_config:
                aruco_config = resolve_package_path(aruco_config)
            bbox_config = config.get("bbox_config")
            if bbox_config:
                bbox_config = resolve_package_path(bbox_config)

            self.markers[name] = Marker(
                name=name,
                marker_type=marker_type,
                board_config=board_config,
                aruco_config=aruco_config,
                bbox_config=bbox_config,
            )

    def _parse_calibration_pairs(self, pairs_config: list) -> None:
        """Parse calibration pair definitions."""
        for pair in pairs_config:
            devices = pair["devices"]
            if len(devices) != 2:
                raise ValueError(f"Calibration pair must have exactly 2 devices: {devices}")

            self.calibration_pairs.append(
                CalibrationPair(
                    device1=devices[0],
                    device2=devices[1],
                    marker=pair["marker"],
                )
            )

    def _validate(self) -> None:
        """Validate configuration consistency."""
        all_devices = set(self.lidars.keys()) | set(self.cameras.keys())

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

            valid_pair = (d1_is_lidar and d2_is_camera) or (d1_is_camera and d2_is_lidar) or (d1_is_lidar and d2_is_lidar)

            if not valid_pair:
                raise ValueError(f"Invalid device pair type: {pair.device1}, {pair.device2}. " "Supported: lidar-camera, lidar-lidar")

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
        config = PipelineConfig()

        # Collect unique (lidar, marker) pairs for board detectors
        lidar_marker_pairs: Set[Tuple[str, str]] = set()
        for pair in self.calibration_pairs:
            if pair.device1 in self.lidars:
                lidar_marker_pairs.add((pair.device1, pair.marker))
            if pair.device2 in self.lidars:
                lidar_marker_pairs.add((pair.device2, pair.marker))

        # Create lidar_board_detector nodes
        for lidar_name, marker_name in sorted(lidar_marker_pairs):
            lidar = self.lidars[lidar_name]
            marker = self.markers[marker_name]

            node_name = f"board_detector_{lidar_name}_{marker_name}"
            namespace = f"calibration/{lidar_name}_{marker_name}"
            output_topic = f"/{namespace}/calibration_board_detections"

            config.lidar_board_detectors.append(
                LidarBoardDetectorNode(
                    node_name=node_name,
                    namespace=namespace,
                    lidar_name=lidar_name,
                    marker_name=marker_name,
                    pointcloud_topic=lidar.pointcloud_topic,
                    board_config=marker.board_config,
                    aruco_config=marker.aruco_config,
                    bbox_config=marker.bbox_config,
                    output_topic=output_topic,
                )
            )

        # Collect unique cameras for aruco locators
        cameras_needed: Set[str] = set()
        for pair in self.calibration_pairs:
            if pair.device1 in self.cameras:
                cameras_needed.add(pair.device1)
            if pair.device2 in self.cameras:
                cameras_needed.add(pair.device2)

        # Create aruco_locator_node nodes
        for camera_name in sorted(cameras_needed):
            camera = self.cameras[camera_name]

            # Find a marker config for this camera (use first available)
            aruco_config = None
            for pair in self.calibration_pairs:
                if camera_name in (pair.device1, pair.device2):
                    marker = self.markers[pair.marker]
                    if marker.aruco_config:
                        aruco_config = marker.aruco_config
                        break

            if aruco_config is None:
                raise ValueError(f"No ArUco config found for camera {camera_name}")

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
                    aruco_config=aruco_config,
                    output_topic=output_topic,
                )
            )

        # Create solver nodes for each calibration pair
        for pair in self.calibration_pairs:
            d1_type = self._get_device_type(pair.device1)
            d2_type = self._get_device_type(pair.device2)

            if d1_type == DeviceType.LIDAR and d2_type == DeviceType.CAMERA:
                self._add_lidar_camera_solver(config, pair.device1, pair.device2, pair.marker)
            elif d1_type == DeviceType.CAMERA and d2_type == DeviceType.LIDAR:
                self._add_lidar_camera_solver(config, pair.device2, pair.device1, pair.marker)
            elif d1_type == DeviceType.LIDAR and d2_type == DeviceType.LIDAR:
                self._add_lidar_lidar_solver(config, pair.device1, pair.device2, pair.marker)

        return config

    def _add_lidar_camera_solver(
        self, config: PipelineConfig, lidar_name: str, camera_name: str, marker_name: str
    ) -> None:
        """Add a lidar-camera solver node to the config."""
        lidar = self.lidars[lidar_name]
        camera = self.cameras[camera_name]
        marker = self.markers[marker_name]

        node_name = f"solver_{lidar_name}_{camera_name}"
        namespace = f"calibration/{lidar_name}_{camera_name}"

        # Find the board detector output topic (matches lidar_board_detector's publisher)
        board_topic = f"/calibration/{lidar_name}_{marker_name}/calibration_board_detections"
        aruco_topic = f"/calibration/{camera_name}/aruco_detections"
        output_topic = f"/{namespace}/extrinsic_transform"

        if marker.aruco_config is None:
            raise ValueError(f"ArUco config required for lidar-camera solver with marker {marker_name}")

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
                aruco_config=marker.aruco_config,
                output_topic=output_topic,
            )
        )

    def _add_lidar_lidar_solver(
        self, config: PipelineConfig, lidar1_name: str, lidar2_name: str, marker_name: str
    ) -> None:
        """Add a lidar-lidar solver node to the config."""
        lidar1 = self.lidars[lidar1_name]
        lidar2 = self.lidars[lidar2_name]

        node_name = f"solver_{lidar1_name}_{lidar2_name}"
        namespace = f"calibration/{lidar1_name}_{lidar2_name}"

        # Find the board detector output topics (matches lidar_board_detector's publisher)
        lidar1_topic = f"/calibration/{lidar1_name}_{marker_name}/calibration_board_detections"
        lidar2_topic = f"/calibration/{lidar2_name}_{marker_name}/calibration_board_detections"
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
