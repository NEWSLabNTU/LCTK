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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _identity_topic_for_detection(detection_topic: str) -> str:
    """Return the observer identity sibling for a detection topic.

    Observer nodes publish ``target_identity`` relative to the namespace that
    already owns their detection output.  Deriving the sibling from the
    parser-provided detection topic keeps launch routing coupled to the
    existing graph contract instead of duplicating namespace construction.
    """

    prefix, separator, _leaf = detection_topic.rpartition("/")
    if not separator:
        return "target_identity"
    return f"{prefix}/target_identity"


def _uses_target_definition(node) -> bool:
    """Report whether a generated node's marker used the new Target Definition
    schema (target_config/detector_config), rather than the legacy
    board_config/aruco_config schema.

    Works for any of the three node dataclasses that carry an `aruco_config`
    field -- `LidarBoardDetectorNode`, `ArucoLocatorNode`,
    `LidarCameraSolverNode` (W5-C pieces C1/C2/C3 respectively) -- because
    `node.aruco_config is None` is a sound signal for all three, though not
    an obvious one:

    - `_parse_new_marker` in config_parser.py sets `aruco_config=None` on the
      `Marker` (config_parser.py:392), which propagates unchanged into every
      node dataclass built from that marker.
    - `aruco_config` is *optional in the YAML* even for a legacy marker
      (`config.get("aruco_config")`, config_parser.py:408), so on its own a
      legacy marker's `aruco_config` could also be `None` -- which would make
      this signal ambiguous.
    - What removes the ambiguity is that a legacy marker (`marker_type is not
      None`) missing `aruco_config` is refused before it ever reaches a node
      dataclass: `_validate` raises at config_parser.py:534 (build validation
      for `lidar_board_detector`) and again at config_parser.py:699 (the
      camera/solver validation path). Between them, every legacy marker that
      survives parsing has a non-None `aruco_config`.

    If either of those two guards is ever relaxed, this predicate silently
    breaks -- a legacy marker could then reach here with `aruco_config is
    None` and be misread as "new schema".

    Do NOT build this signal from `target_config` instead. `target_config`
    is populated under BOTH schemas: `_parse_legacy_marker` in
    config_parser.py always sets it to `LEGACY_HOLLOW_TARGET_CONFIG`, the
    explicit hollow manifest, as a migration fallback for the legacy graph.
    Keying on `target_config is not None` would therefore report "new
    schema" for legacy markers too, and a generated lidar_board_detector
    node would then receive BOTH the new target_config/detector_config pair
    AND the legacy board_detector_file/aruco_pattern_file pair -- which
    `select_config_source` in `ros/lidar_board_detector/src/main.rs` refuses
    outright at startup: "target_config/detector_config and legacy
    board_detector_file/aruco_pattern_file cannot be mixed; select one
    source."
    """

    return node.aruco_config is None


def generate_nodes(context, *args, **kwargs) -> list:
    """Generate nodes based on the configuration file."""
    from lctk_launch.config_parser import parse_config

    # Get launch arguments
    config_file = LaunchConfiguration("config_file").perform(context)
    debug_mode = LaunchConfiguration("debug_mode").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    # L-05: fail on an unknown mode instead of silently falling back to offline
    # (a typo like "realtim" would otherwise ship offline QoS to a live sensor).
    if mode not in ("offline", "realtime"):
        raise RuntimeError(f"Invalid mode '{mode}'; expected 'offline' or 'realtime'.")
    solver_mode = LaunchConfiguration("solver_mode").perform(context)
    if solver_mode not in ("continuous", "manual"):
        raise RuntimeError(
            f"Invalid solver_mode '{solver_mode}'; expected 'continuous' or 'manual'."
        )
    enable_overlay = LaunchConfiguration("enable_overlay").perform(context) == "true"
    enable_judge = LaunchConfiguration("enable_judge").perform(context) == "true"

    # Derive settings from mode
    # - offline: RELIABLE QoS, exact sync matching, larger queues
    # - realtime: BEST_EFFORT QoS, approximate sync, minimal buffering
    is_realtime = mode == "realtime"
    use_best_effort_qos = is_realtime

    # Synchronization settings based on mode (used by Conflux synchronizer)
    # - sync_tolerance_ms: Time window for grouping messages (0 = infinite window)
    # - sync_queue_size: Buffer size per stream
    # - sync_drop_policy: "reject_new" (preserve data) or "drop_oldest" (prefer latest)
    if is_realtime:
        sync_tolerance_ms = 50.0  # 50ms tolerance for real-time
        sync_queue_size = 2  # Minimal buffering
        sync_drop_policy = "drop_oldest"  # Always process latest data
    else:
        # 100ms window, NOT the infinite window this used to use.
        #
        # Conflux only matches by time when a finite window is set: with an infinite
        # window it skips the pruning step in `State::try_match` and pairs whatever is
        # at the FRONT of each buffer -- i.e. by arrival order. Two streams at
        # different rates then drift apart without bound. Measured on this repository's
        # own conflux build: camera 10Hz + LiDAR 1Hz reaches a 53s gap INSIDE one
        # "synchronized" group; 30Hz + 10Hz saturates at 10s; the seyond rig's 5.4Hz +
        # 4.4Hz passes 11s and keeps climbing. The same runs with a 50ms window stay
        # within 33ms.
        #
        # That failure is silent and ruinous here: the solver pairs ArUco corners with
        # a board pose on the assumption both saw the board at the same instant. Pair a
        # camera frame with a LiDAR sweep 11s apart and the board has MOVED, so the
        # solve is wrong while the reprojection error still looks fine.
        #
        # 100ms is a little over one frame interval at these rates -- wide enough to
        # absorb the offset between a camera frame and the LiDAR sweep that overlaps
        # it, narrow enough that a moving board cannot travel far within it.
        sync_tolerance_ms = 100.0
        sync_queue_size = 100  # Large queue for rosbag playback
        sync_drop_policy = "reject_new"  # Preserve all data

    # Parse configuration
    pipeline = parse_config(config_file)

    nodes = []

    # Log calibration plan (always present — planner runs on every config)
    assert pipeline.calibration_plan_text is not None
    assert pipeline.calibration_plan is not None
    for line in pipeline.calibration_plan_text.split("\n"):
        nodes.append(LogInfo(msg=line))
    nodes.append(LogInfo(msg=""))

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
            LogInfo(
                msg=f"  Board detector: {detector.node_name} ({detector.lidar_name} -> {detector.marker_name})"
            )
        )

        node_args = ["--ros-args", "--log-level", log_level]

        # bbox_config is guaranteed present by config_parser validation for
        # both schemas (mandatory for the lidar_board_detector either way).
        params = {
            "enable_debug": debug_mode == "true",
            "enable_icp_iteration_debug": debug_mode == "true",
            "use_best_effort_qos": use_best_effort_qos,
            "bbox_file": detector.bbox_config,
        }

        uses_target_definition = _uses_target_definition(detector)
        if uses_target_definition != (detector.detector_config is not None):
            # config_parser guarantees these two signals agree (see
            # `_uses_target_definition`'s docstring). If they ever disagree,
            # a parser regression is about to hand `detector_config=None`
            # straight into `Node(...)`, which fails deep inside
            # launch_ros's `normalize_parameters` with a bare
            # "TypeError: Unexpected type for parameter value None" that
            # names neither the node nor the cause. Fail loudly here instead.
            raise RuntimeError(
                f"lidar_board_detector node '{detector.node_name}' has "
                f"inconsistent schema signals: aruco_config="
                f"{detector.aruco_config!r} (uses_target_definition="
                f"{uses_target_definition}) but detector_config="
                f"{detector.detector_config!r}. This indicates a config_parser "
                "regression; the two must always agree."
            )

        if uses_target_definition:
            # New Target Definition schema: hand the node the manifest +
            # tuning pair and omit the legacy keys entirely. lidar_board_detector
            # declares them `.optional()` and reads a missing key as "not
            # supplied" -- a `None` parameter value is not the same thing and
            # would still trip select_config_source's mixed-source refusal.
            params["target_config"] = detector.target_config
            params["detector_config"] = detector.detector_config
        else:
            params["board_detector_file"] = detector.board_config
            params["aruco_pattern_file"] = detector.aruco_config

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
        nodes.append(
            LogInfo(msg=f"  ArUco locator: {locator.node_name} ({locator.camera_name})")
        )

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
                        "aruco_detector_config_file": locator.aruco_detector_config,
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
        nodes.append(
            LogInfo(
                msg=f"  LiDAR-Camera solver: {solver.node_name} ({solver.lidar_name} <-> {solver.camera_name}) [{solver_mode}]"
            )
        )

        node_args = ["--ros-args", "--log-level", log_level]
        nodes.append(
            Node(
                package="lidar_to_camera_solver",
                executable="lidar_to_camera_solver",
                name="lidar_to_camera_solver",
                namespace=solver.namespace,
                output="screen",
                arguments=node_args,
                parameters=[
                    {
                        "solver_mode": solver_mode,
                        "parent_frame": solver.parent_frame,
                        "child_frame": solver.child_frame,
                        "camera_topic": solver.camera_topic,
                        "aruco_config_file": solver.aruco_config,
                        # Keep the solver-side identity endpoints relative and
                        # remap each one to its corresponding observer below.
                        "lidar_target_identity_topic": "lidar_target_identity",
                        "camera_target_identity_topic": "camera_target_identity",
                        "debug_mode": debug_mode == "true",
                        "publishing_rate": 10.0,
                        "use_best_effort_qos": use_best_effort_qos,
                        "sync_tolerance_ms": sync_tolerance_ms,
                        "sync_queue_size": sync_queue_size,
                        "sync_drop_policy": sync_drop_policy,
                    }
                ],
                remappings=[
                    ("aruco_detections", solver.aruco_detections_topic),
                    ("calibration_board_detections", solver.board_detections_topic),
                    (
                        "lidar_target_identity",
                        _identity_topic_for_detection(solver.board_detections_topic),
                    ),
                    (
                        "camera_target_identity",
                        _identity_topic_for_detection(solver.aruco_detections_topic),
                    ),
                    ("extrinsic_transform", solver.output_topic),
                ],
            )
        )

    # Generate lidar-lidar solver nodes
    for solver in pipeline.lidar_lidar_solvers:
        nodes.append(
            LogInfo(
                msg=f"  LiDAR-LiDAR solver: {solver.node_name} ({solver.lidar1_name} <-> {solver.lidar2_name})"
            )
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
                        # Same mode-derived sync preset as the LiDAR-camera solvers.
                        # This used to hardcode a 0.0 window, which conflux reads as
                        # INFINITE: it then pairs by arrival order instead of by time
                        # and two streams at different rates drift apart without bound.
                        "sync_tolerance_ms": sync_tolerance_ms,
                        "sync_queue_size": sync_queue_size,
                        "sync_drop_policy": sync_drop_policy,
                        "publish_tf": True,
                        "use_best_effort_qos": use_best_effort_qos,
                        "max_message_age_ms": 0.0,
                    }
                ],
                remappings=[
                    (
                        "lidar1_target_identity",
                        _identity_topic_for_detection(solver.lidar1_detections_topic),
                    ),
                    (
                        "lidar2_target_identity",
                        _identity_topic_for_detection(solver.lidar2_detections_topic),
                    ),
                ],
            )
        )

    # Spawn TF tree broadcaster — subscribes to tree-edge solver topics
    plan = pipeline.calibration_plan

    # Collect output topics for tree edges only
    tree_edge_set = {(e.parent, e.child) for e in plan.tree_edges}

    tf_topics = []
    for solver in pipeline.lidar_camera_solvers:
        key1 = (solver.lidar_name, solver.camera_name)
        key2 = (solver.camera_name, solver.lidar_name)
        if key1 in tree_edge_set or key2 in tree_edge_set:
            tf_topics.append(solver.output_topic)
    for solver in pipeline.lidar_lidar_solvers:
        key1 = (solver.lidar1_name, solver.lidar2_name)
        key2 = (solver.lidar2_name, solver.lidar1_name)
        if key1 in tree_edge_set or key2 in tree_edge_set:
            tf_topics.append(solver.output_topic)

    if tf_topics:
        nodes.append(
            LogInfo(msg=f"  TF tree broadcaster: {len(tf_topics)} tree edge(s)")
        )
        nodes.append(
            Node(
                package="lctk_launch",
                executable="tf_tree_broadcaster",
                name="tf_tree_broadcaster",
                namespace="calibration",
                output="screen",
                parameters=[{"topics": tf_topics}],
            )
        )

    # Generate overlay nodes (one per lidar-camera solver)
    if enable_overlay and pipeline.lidar_camera_solvers:
        nodes.append(
            LogInfo(msg=f"  Overlay nodes: {len(pipeline.lidar_camera_solvers)}")
        )
        for solver in pipeline.lidar_camera_solvers:
            # Look up the lidar's pointcloud topic
            lidar = pipeline.lidars[solver.lidar_name]
            node_args = ["--ros-args", "--log-level", log_level]
            nodes.append(
                Node(
                    package="pointcloud_image_overlay",
                    executable="overlay_node",
                    name="pointcloud_image_overlay",
                    namespace=solver.namespace,
                    output="screen",
                    arguments=node_args,
                    parameters=[{"use_best_effort_qos": use_best_effort_qos}],
                    remappings=[
                        ("image", solver.camera_topic),
                        ("pointcloud", lidar.pointcloud_topic),
                        (
                            "plane_inliers",
                            f"/calibration/{solver.lidar_name}_{solver.marker_name}/debug/plane_inliers",
                        ),
                        ("extrinsic_transform", solver.output_topic),
                    ],
                )
            )

    # Generate judge nodes (one per lidar-camera solver)
    if enable_judge and pipeline.lidar_camera_solvers:
        nodes.append(
            LogInfo(msg=f"  Judge nodes: {len(pipeline.lidar_camera_solvers)}")
        )
        for solver in pipeline.lidar_camera_solvers:
            nodes.append(
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("lctk_launch"),
                                "launch",
                                "calibration_judge.launch.xml",
                            ]
                        )
                    ),
                    launch_arguments={
                        "transform_topic": solver.output_topic,
                        "namespace": solver.namespace,
                        "log_level": log_level,
                    }.items(),
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
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("lctk_launch"),
                        "config",
                        "rviz",
                        "calibration.rviz",
                    ]
                ),
                description="Path to RViz config file. Override for different setups, e.g. two_lidar_calibration.rviz",
            ),
            DeclareLaunchArgument(
                "solver_mode",
                default_value="continuous",
                description="LiDAR-camera solver behaviour: 'continuous' (auto-publishes latest pair) or 'manual' (service-driven multi-pose buffer)",
            ),
            DeclareLaunchArgument(
                "enable_overlay",
                default_value="false",
                description="Enable pointcloud-image overlay visualization (one per lidar-camera pair)",
            ),
            DeclareLaunchArgument(
                "enable_judge",
                default_value="false",
                description="Enable calibration quality judge (ground truth comparison, one per lidar-camera pair)",
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
                launch_arguments={
                    "rviz_config": LaunchConfiguration("rviz_config")
                }.items(),
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
            ),
        ]
    )
