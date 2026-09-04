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
- A required `sync:` section (tolerance_ms, queue_size, drop_policy) for the
  Conflux synchronizer -- see config_parser.py's `_parse_sync_tolerance_ms`

Modes:
- offline: For processing recorded data (rosbags). Uses RELIABLE QoS to avoid
  message drops.
- realtime: For live data processing. Uses BEST_EFFORT QoS for lowest
  latency.

`mode` controls transport QoS only. The synchronizer window/buffer/drop
policy are a physical judgement about the scene, not about live-vs-recorded
data, so they come from the config file's `sync:` section instead.

See sessions/ (installed at share/lctk_launch/sessions/) for maintained
configurations; a session manifest is a calibration config plus a `data:`
section, which this launch file does not read.
"""

from dataclasses import asdict

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
from lctk_launch.session import DEFAULT_RVIZ_CONFIG_PARTS


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


def generate_nodes(context, *args, **kwargs) -> list:
    """Generate nodes based on the configuration file."""
    from lctk_launch.config_parser import parse_config

    # Get launch arguments
    config_file = LaunchConfiguration("config_file").perform(context)
    debug_mode = LaunchConfiguration("debug_mode").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    solver_mode = LaunchConfiguration("solver_mode").perform(context)
    if solver_mode not in ("continuous", "manual", "assisted"):
        raise RuntimeError(
            f"Invalid solver_mode '{solver_mode}'; "
            "expected 'continuous', 'manual' or 'assisted'."
        )
    enable_overlay = LaunchConfiguration("enable_overlay").perform(context) == "true"
    enable_judge = LaunchConfiguration("enable_judge").perform(context) == "true"

    # Transport reliability is NOT decided here. It is a property of what
    # publishes each sensor topic -- a recording knows its own answer, and a
    # live rig's is a fact about the driver -- so it is resolved per device by
    # `lctk_launch.transport` and arrives on each node's config below. The
    # `mode` argument that used to decide it graph-wide is gone: it guessed one
    # answer for every topic, and `twolidar-vlp32-falcon` proved that wrong by
    # recording a RELIABLE Falcon and a BEST_EFFORT VLP-32 in the same bag.
    #
    # Only sensor subscriptions take an answer from the session. LCTK's own
    # detection and transform topics are ours on both ends and are pinned
    # RELIABLE inside the nodes.

    # Parse configuration
    pipeline = parse_config(config_file)

    # Synchronizer window/buffer/drop-policy come from the config's required
    # `sync:` section, NOT from `mode`. The window is a physical judgement
    # about the scene -- how far the calibration target can move between a
    # camera frame and a LiDAR sweep -- and is not derivable from whether the
    # data happens to be live or recorded. See
    # `CalibrationConfigParser._parse_sync_tolerance_ms` in config_parser.py
    # for the validation and the measured infinite-window failure mode this
    # guards against (a config with `sync:` omitted is refused at parse time,
    # before reaching this point).
    assert pipeline.sync is not None
    sync_tolerance_ms = pipeline.sync.tolerance_ms
    sync_queue_size = pipeline.sync.queue_size
    sync_drop_policy = pipeline.sync.drop_policy

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
            msg="Calibration Pipeline: "
            f"{len(pipeline.lidar_board_detectors)} board detectors, "
            f"{len(pipeline.aruco_locators)} aruco locators, "
            f"{len(pipeline.lidar_camera_solvers)} lidar-camera solvers, "
            f"{len(pipeline.lidar_lidar_solvers)} lidar-lidar solvers"
        )
    )

    # Log effective sync settings (from the config's required `sync:`
    # section) so a replay's log records what was actually used.
    nodes.append(
        LogInfo(
            msg=f"Sync settings: tolerance_ms={sync_tolerance_ms}, "
            f"queue_size={sync_queue_size}, drop_policy={sync_drop_policy}"
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

        params = {
            "enable_debug": debug_mode == "true",
            "enable_icp_iteration_debug": debug_mode == "true",
            "use_best_effort_qos": detector.use_best_effort_qos,
            "target_config": detector.target_config,
            "detector_config": detector.detector_config,
        }

        # bbox_config is optional (config_parser no longer requires it: it is
        # only read when detector tuning selects detection_mode=bbox). Omit
        # the key entirely when absent -- launch_ros's Node() normalizes
        # parameters eagerly at construction time and raises on a `None`
        # value, same failure mode as commit eb58770 fixed for the
        # camera-side nodes.
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
        nodes.append(
            LogInfo(msg=f"  ArUco locator: {locator.node_name} ({locator.camera_name})")
        )

        node_args = ["--ros-args", "--log-level", log_level]

        # aruco_detector_config_file tunes the detector (corner refinement,
        # adaptive threshold) independently of target_config, which supplies
        # the physical marker layout below.
        params = {
            "aruco_detector_config_file": locator.aruco_detector_config,
            "debug_mode": debug_mode == "true",
            "debug_overlay_enabled": debug_mode == "true",
            "use_best_effort_qos": locator.use_best_effort_qos,
            "target_config": locator.target_config,
        }

        nodes.append(
            Node(
                package="aruco_locator_node",
                executable="aruco_locator_node",
                name="aruco_locator",
                namespace=locator.namespace,
                output="screen",
                arguments=node_args,
                parameters=[params],
                remappings=[
                    ("image", locator.image_topic),
                    ("aruco_detections", locator.output_topic),
                ],
            )
        )

    # Generate lidar-camera solver nodes
    for solver_index, solver in enumerate(pipeline.lidar_camera_solvers):
        nodes.append(
            LogInfo(
                msg=f"  LiDAR-Camera solver: {solver.node_name} ({solver.lidar_name} <-> {solver.camera_name}) [{solver_mode}]"
            )
        )

        node_args = ["--ros-args", "--log-level", log_level]

        # One review server per solver, so a multi-pair config must not hand
        # every one the same port -- ReviewServer binds eagerly, and the second
        # node would die with "Address already in use". Offsetting by the
        # solver's index keeps the first pair on the configured port, which is
        # what a single-pair config (every maintained example) still gets.
        assisted_params = asdict(pipeline.assisted)
        assisted_params["review_port"] = pipeline.assisted.review_port + solver_index

        params = {
            "solver_mode": solver_mode,
            "parent_frame": solver.parent_frame,
            "child_frame": solver.child_frame,
            "camera_topic": solver.camera_topic,
            # Keep the solver-side identity endpoints relative and
            # remap each one to its corresponding observer below.
            "lidar_target_identity_topic": "lidar_target_identity",
            "camera_target_identity_topic": "camera_target_identity",
            "debug_mode": debug_mode == "true",
            "publishing_rate": 10.0,
            # The camera's answer. This node's detection subscriptions are
            # LCTK's own topics and are pinned RELIABLE inside the node; this
            # governs the camera_info it reads and, in assisted mode, the
            # preview frame.
            "use_best_effort_qos": solver.use_best_effort_qos,
            "sync_tolerance_ms": sync_tolerance_ms,
            "sync_queue_size": sync_queue_size,
            "sync_drop_policy": sync_drop_policy,
            "target_config": solver.target_config,
            # assisted-mode tuning; harmless for the other two, which never read it.
            **assisted_params,
        }

        nodes.append(
            Node(
                package="lidar_to_camera_solver",
                executable="lidar_to_camera_solver",
                name="lidar_to_camera_solver",
                namespace=solver.namespace,
                output="screen",
                arguments=node_args,
                parameters=[params],
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
                        # This solver has no sensor subscription at all -- both
                        # inputs are detection topics LCTK publishes -- so it
                        # always asks for the reliability those are pinned to.
                        "use_best_effort_qos": False,
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
                    # A viewer, not a pipeline stage: dropping a frame under
                    # load costs nothing, and BEST_EFFORT can receive from a
                    # publisher of either kind, so it needs no session answer.
                    parameters=[{"use_best_effort_qos": True}],
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
                "enable_rviz",
                default_value="true",
                description="Launch RViz for calibration visualization",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                # Shared with session.launch.py via lctk_launch.session so the
                # two cannot drift; see DEFAULT_RVIZ_CONFIG_PARTS for why the
                # default has to be named in both places rather than only here.
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lctk_launch"), *DEFAULT_RVIZ_CONFIG_PARTS]
                ),
                description="Path to RViz config file. Override for different setups, e.g. two_lidar_calibration.rviz",
            ),
            DeclareLaunchArgument(
                "solver_mode",
                default_value="continuous",
                description=(
                    "LiDAR-camera solver behaviour: 'continuous' (auto-publishes the "
                    "latest pair), 'manual' (service-driven multi-pose buffer), or "
                    "'assisted' (auto-captures still, novel poses and serves a review "
                    "page)"
                ),
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
