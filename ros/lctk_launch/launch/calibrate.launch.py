"""
Dynamic launch file for multi-sensor calibration pipeline.

This launch file reads a YAML configuration describing the sensor arrangement
and calibration pairs, then dynamically generates the required nodes.

Usage:
    ros2 launch lctk_launch calibrate.launch.py config_file:=/path/to/config.yaml

The config file describes:
- Devices (lidars and cameras with their topics and frame IDs)
- Markers (calibration boards with their configuration files)
- Calibration pairs (which devices to calibrate together using which marker)

Modes:
- offline: For processing recorded data (rosbags). Uses RELIABLE QoS to avoid
  message drops. Synchronizer attempts perfect timestamp matches.
- realtime: For live data processing. Uses BEST_EFFORT QoS with no buffering
  for lowest latency.

See config/examples/ for example configurations.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_nodes(context, *args, **kwargs) -> list:
    """Generate nodes based on the configuration file."""
    from lctk_launch.config_parser import parse_config

    # Get launch arguments
    config_file = LaunchConfiguration("config_file").perform(context)
    debug_mode = LaunchConfiguration("debug_mode").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    use_advanced_solver = LaunchConfiguration("use_advanced_solver").perform(context) == "true"

    # Derive settings from mode
    # - offline: RELIABLE QoS, exact sync matching, larger queues
    # - realtime: BEST_EFFORT QoS, approximate sync, minimal buffering
    is_realtime = mode == "realtime"
    use_best_effort_qos = is_realtime

    # Synchronization settings based on mode
    if is_realtime:
        sync_mode = "approximate"
        sync_tolerance_ms = 50.0  # 50ms tolerance for real-time
        sync_queue_size = 2       # Minimal buffering
    else:
        sync_mode = "exact"
        sync_tolerance_ms = 10.0  # Tight tolerance for offline (fallback if exact fails)
        sync_queue_size = 20      # Larger queue for rosbag playback

    # Parse configuration
    pipeline = parse_config(config_file)

    nodes = []

    # Log pipeline summary
    nodes.append(
        LogInfo(
            msg=f"Calibration Pipeline ({mode} mode): "
            f"{len(pipeline.lidar_board_detectors)} board detectors, "
            f"{len(pipeline.aruco_locators)} aruco locators, "
            f"{len(pipeline.lidar_camera_solvers)} lidar-camera solvers, "
            f"{len(pipeline.lidar_lidar_solvers)} lidar-lidar solvers"
        )
    )

    # Generate lidar_board_detector nodes
    for detector in pipeline.lidar_board_detectors:
        nodes.append(
            LogInfo(msg=f"  Board detector: {detector.node_name} ({detector.lidar_name} -> {detector.marker_name})")
        )

        node_args = ["--ros-args", "--log-level", log_level]

        params = {
            "board_detector_file": detector.board_config,
            "enable_debug": debug_mode == "true",
            "enable_icp_iteration_debug": debug_mode == "true",
            "use_best_effort_qos": use_best_effort_qos,
        }

        if detector.aruco_config:
            params["aruco_pattern_file"] = detector.aruco_config
        if detector.bbox_config:
            params["bbox_file"] = detector.bbox_config

        nodes.append(
            Node(
                package="lidar_board_detector",
                executable="lidar_board_detector",
                name="lidar_board_detector",
                namespace=detector.namespace,
                output="screen",
                arguments=node_args,
                parameters=[params],
                remappings=[
                    ("input_pointcloud", detector.pointcloud_topic),
                ],
            )
        )

    # Generate aruco_locator_node nodes
    for locator in pipeline.aruco_locators:
        nodes.append(LogInfo(msg=f"  ArUco locator: {locator.node_name} ({locator.camera_name})"))

        node_args = ["--ros-args", "--log-level", log_level]

        nodes.append(
            Node(
                package="aruco_locator_node",
                executable="aruco_locator_node",
                name="aruco_locator",
                namespace=locator.namespace,
                output="screen",
                arguments=node_args,
                parameters=[
                    {
                        "aruco_config_file": locator.aruco_config,
                        "debug_mode": debug_mode == "true",
                        "debug_overlay_enabled": debug_mode == "true",
                        "use_best_effort_qos": use_best_effort_qos,
                    }
                ],
                remappings=[
                    ("image", locator.image_topic),
                    ("aruco_detections", locator.output_topic),
                ],
            )
        )

    # Generate lidar-camera solver nodes
    for solver in pipeline.lidar_camera_solvers:
        solver_type = "advanced" if use_advanced_solver else "standard"
        nodes.append(
            LogInfo(msg=f"  LiDAR-Camera solver: {solver.node_name} ({solver.lidar_name} <-> {solver.camera_name}) [{solver_type}]")
        )

        node_args = ["--ros-args", "--log-level", log_level]

        if use_advanced_solver:
            nodes.append(
                Node(
                    package="advanced_extrinsic_solver",
                    executable="advanced_extrinsic_solver",
                    name="advanced_extrinsic_solver",
                    namespace=solver.namespace,
                    output="screen",
                    arguments=node_args,
                    parameters=[
                        {
                            "parent_frame": solver.parent_frame,
                            "child_frame": solver.child_frame,
                            "camera_topic": solver.camera_topic,
                            "aruco_config_file": solver.aruco_config,
                            "debug_mode": debug_mode == "true",
                            "publishing_rate": 10.0,
                            "use_best_effort_qos": use_best_effort_qos,
                        }
                    ],
                    remappings=[
                        ("aruco_detections", solver.aruco_detections_topic),
                        ("calibration_board_detections", solver.board_detections_topic),
                        ("extrinsic_transform", solver.output_topic),
                    ],
                )
            )
        else:
            # Standard solver - publishes transform on each detection pair
            nodes.append(
                Node(
                    package="extrinsic_solver_node",
                    executable="extrinsic_solver_node",
                    name="extrinsic_solver",
                    namespace=solver.namespace,
                    output="screen",
                    arguments=node_args,
                    parameters=[
                        {
                            "parent_frame": solver.parent_frame,
                            "child_frame": solver.child_frame,
                            "camera_topic": solver.camera_topic,
                            "aruco_config_file": solver.aruco_config,
                            "debug_mode": debug_mode == "true",
                            "use_best_effort_qos": use_best_effort_qos,
                        }
                    ],
                    remappings=[
                        ("aruco_detections", solver.aruco_detections_topic),
                        ("calibration_board_detections", solver.board_detections_topic),
                        ("extrinsic_transform", solver.output_topic),
                    ],
                )
            )

    # Generate lidar-lidar solver nodes
    for solver in pipeline.lidar_lidar_solvers:
        nodes.append(
            LogInfo(msg=f"  LiDAR-LiDAR solver: {solver.node_name} ({solver.lidar1_name} <-> {solver.lidar2_name})")
        )

        node_args = ["--ros-args", "--log-level", log_level]

        nodes.append(
            Node(
                package="lidar_to_lidar_solver",
                executable="lidar_to_lidar_solver",
                name="lidar_to_lidar_solver",
                namespace=solver.namespace,
                output="screen",
                arguments=node_args,
                parameters=[
                    {
                        "lidar1_detections_topic": solver.lidar1_detections_topic,
                        "lidar2_detections_topic": solver.lidar2_detections_topic,
                        "lidar1_frame": solver.lidar1_frame,
                        "lidar2_frame": solver.lidar2_frame,
                        "sync_mode": sync_mode,
                        "sync_tolerance_ms": sync_tolerance_ms,
                        "sync_queue_size": sync_queue_size,
                        "publish_tf": True,
                        "use_best_effort_qos": use_best_effort_qos,
                    }
                ],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description."""
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "config_file",
                description="Path to calibration configuration YAML file",
            ),
            DeclareLaunchArgument(
                "debug_mode",
                default_value="false",
                description="Enable debug logging and visualization",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level (debug, info, warn, error, fatal)",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="offline",
                description="Processing mode: 'offline' (RELIABLE QoS, perfect sync) or 'realtime' (BEST_EFFORT QoS, no buffering)",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Launch RViz for calibration visualization",
            ),
            DeclareLaunchArgument(
                "use_advanced_solver",
                default_value="false",
                description="Use advanced multi-pose solver (requires manual detection buffering) vs standard solver (auto-publishes)",
            ),
            # Dynamic node generation
            OpaqueFunction(function=generate_nodes),
            # RViz visualization (optional)
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("lctk_launch"), "launch", "rviz.launch.xml"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
            ),
        ]
    )
