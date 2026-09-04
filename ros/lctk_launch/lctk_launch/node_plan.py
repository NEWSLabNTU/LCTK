"""Every node the calibration graph runs, as data.

`calibrate.launch.py` used to decide all of this inside one 356-line
`generate_nodes(context)`: which nodes exist, what each is called and namespaced,
every parameter, every remapping, and the order the log lines appear in. Its only
interface was a launch callback, so a test asking "does a two-lidar config
produce two board detectors" had to stage a launch context and read
`launch_ros.Node` internals. `test_calibrate_launch_graph.py` grew to a thousand
lines doing exactly that.

The decisions live here instead, as plain values with no launch types anywhere in
the module. The launch file keeps only the mapping from these values to launch
actions, which has no decisions left in it.

A plan is an *ordered* sequence, because the order is part of what it describes:
each node is preceded by the log line announcing it, and a replay's log is read
top to bottom.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

from lctk_launch.config_parser import PipelineConfig

SOLVER_MODES = ("continuous", "manual", "assisted")


@dataclass(frozen=True)
class Message:
    """A line the launch file logs before the node it announces."""

    text: str


@dataclass(frozen=True)
class NodeSpec:
    """One node, described the way `launch_ros.Node` wants to receive it."""

    package: str
    executable: str
    name: str
    namespace: str
    parameters: dict = field(default_factory=dict)
    remappings: tuple[tuple[str, str], ...] = ()
    arguments: tuple[str, ...] = ()


@dataclass(frozen=True)
class JudgeInclude:
    """The quality judge, which ships as a launch file rather than a node."""

    transform_topic: str
    namespace: str
    log_level: str


@dataclass(frozen=True)
class RunSettings:
    """What the operator chose on the command line, already validated.

    Everything else about the graph comes from the manifest. These are the four
    things that genuinely belong to a run rather than to a rig: how loud it is,
    which solver policy to use, and whether to bring up the two optional
    viewers.
    """

    solver_mode: str = "continuous"
    debug_mode: bool = False
    log_level: str = "info"
    enable_overlay: bool = False
    enable_judge: bool = False

    def __post_init__(self) -> None:
        if self.solver_mode not in SOLVER_MODES:
            # RuntimeError rather than ValueError to match the guard this
            # replaced, which callers and tests already expect. A typo must not
            # silently ship a different solver policy: falling back to a default
            # would run a whole capture session under a policy nobody asked for.
            raise RuntimeError(
                f"Invalid solver_mode '{self.solver_mode}'; "
                f"expected {', '.join(repr(mode) for mode in SOLVER_MODES[:-1])} "
                f"or {SOLVER_MODES[-1]!r}."
            )


PlanEntry = Message | NodeSpec | JudgeInclude


def _identity_topic_for_detection(detection_topic: str) -> str:
    """Return the observer identity sibling for a detection topic.

    Observer nodes publish ``target_identity`` relative to the namespace that
    already owns their detection output. Deriving the sibling from the
    parser-provided detection topic keeps routing coupled to the existing graph
    contract instead of duplicating namespace construction.
    """
    prefix, separator, _leaf = detection_topic.rpartition("/")
    if not separator:
        return "target_identity"
    return f"{prefix}/target_identity"


def build_node_plan(pipeline: PipelineConfig, settings: RunSettings) -> list[PlanEntry]:
    """Every node this configuration runs, in the order it should appear."""
    log_arguments = ("--ros-args", "--log-level", settings.log_level)

    # `sync:` is required and validated at parse time, so it is present here.
    # The window is a physical judgement about the scene -- how far the target
    # can move between a camera frame and a LiDAR sweep -- not something
    # derivable from the data source; see `_parse_sync_tolerance_ms` for the
    # infinite-window failure this guards against.
    assert pipeline.sync is not None
    sync = {
        "sync_tolerance_ms": pipeline.sync.tolerance_ms,
        "sync_queue_size": pipeline.sync.queue_size,
        "sync_drop_policy": pipeline.sync.drop_policy,
    }

    assert pipeline.calibration_plan_text is not None
    assert pipeline.calibration_plan is not None

    entries: list[PlanEntry] = [
        Message(line) for line in pipeline.calibration_plan_text.split("\n")
    ]
    entries.append(Message(""))
    entries.append(
        Message(
            "Calibration Pipeline: "
            f"{len(pipeline.lidar_board_detectors)} board detectors, "
            f"{len(pipeline.aruco_locators)} aruco locators, "
            f"{len(pipeline.lidar_camera_solvers)} lidar-camera solvers, "
            f"{len(pipeline.lidar_lidar_solvers)} lidar-lidar solvers"
        )
    )
    # Record the effective sync settings, so a replay's log says what was used.
    entries.append(
        Message(
            f"Sync settings: tolerance_ms={pipeline.sync.tolerance_ms}, "
            f"queue_size={pipeline.sync.queue_size}, "
            f"drop_policy={pipeline.sync.drop_policy}"
        )
    )

    entries += _board_detectors(pipeline, settings, log_arguments)
    entries += _aruco_locators(pipeline, settings, log_arguments)
    entries += _lidar_camera_solvers(pipeline, settings, sync, log_arguments)
    entries += _lidar_lidar_solvers(pipeline, sync, log_arguments)
    entries += _tf_tree_broadcaster(pipeline)
    entries += _overlays(pipeline, settings, log_arguments)
    entries += _judges(pipeline, settings)
    return entries


def _board_detectors(pipeline, settings, log_arguments) -> list[PlanEntry]:
    entries: list[PlanEntry] = []
    for detector in pipeline.lidar_board_detectors:
        entries.append(
            Message(
                f"  Board detector: {detector.node_name} "
                f"({detector.lidar_name} -> {detector.marker_name})"
            )
        )
        parameters = {
            "enable_debug": settings.debug_mode,
            "enable_icp_iteration_debug": settings.debug_mode,
            "use_best_effort_qos": detector.use_best_effort_qos,
            "target_config": detector.target_config,
            "detector_config": detector.detector_config,
        }
        # bbox_config is optional: the detector reads it only when its tuning
        # selects detection_mode=bbox. Omit the key entirely when absent --
        # launch_ros's Node() normalizes parameters eagerly and raises on a
        # `None` value, the failure eb58770 fixed for the camera-side nodes.
        if detector.bbox_config:
            parameters["bbox_file"] = detector.bbox_config

        entries.append(
            NodeSpec(
                package="lidar_board_detector",
                executable="lidar_board_detector",
                name="lidar_board_detector",
                namespace=detector.namespace,
                parameters=parameters,
                remappings=(("input_pointcloud", detector.pointcloud_topic),),
                arguments=log_arguments,
            )
        )
    return entries


def _aruco_locators(pipeline, settings, log_arguments) -> list[PlanEntry]:
    entries: list[PlanEntry] = []
    for locator in pipeline.aruco_locators:
        entries.append(
            Message(f"  ArUco locator: {locator.node_name} ({locator.camera_name})")
        )
        entries.append(
            NodeSpec(
                package="aruco_locator_node",
                executable="aruco_locator_node",
                name="aruco_locator",
                namespace=locator.namespace,
                parameters={
                    # Tunes the detector (corner refinement, adaptive
                    # threshold) independently of target_config, which supplies
                    # the physical marker layout.
                    "aruco_detector_config_file": locator.aruco_detector_config,
                    "debug_mode": settings.debug_mode,
                    "debug_overlay_enabled": settings.debug_mode,
                    "use_best_effort_qos": locator.use_best_effort_qos,
                    "target_config": locator.target_config,
                },
                remappings=(
                    ("image", locator.image_topic),
                    ("aruco_detections", locator.output_topic),
                ),
                arguments=log_arguments,
            )
        )
    return entries


def _lidar_camera_solvers(pipeline, settings, sync, log_arguments) -> list[PlanEntry]:
    entries: list[PlanEntry] = []
    for index, solver in enumerate(pipeline.lidar_camera_solvers):
        entries.append(
            Message(
                f"  LiDAR-Camera solver: {solver.node_name} "
                f"({solver.lidar_name} <-> {solver.camera_name}) "
                f"[{settings.solver_mode}]"
            )
        )
        # One review server per solver, so a multi-pair config must not hand
        # every one the same port -- ReviewServer binds eagerly, and the second
        # node would die with "Address already in use". Offsetting by the
        # solver's index keeps the first pair on the configured port, which is
        # what a single-pair config (every maintained example) still gets.
        assisted = asdict(pipeline.assisted)
        assisted["review_port"] = pipeline.assisted.review_port + index

        entries.append(
            NodeSpec(
                package="lidar_to_camera_solver",
                executable="lidar_to_camera_solver",
                name="lidar_to_camera_solver",
                namespace=solver.namespace,
                parameters={
                    "solver_mode": settings.solver_mode,
                    "parent_frame": solver.parent_frame,
                    "child_frame": solver.child_frame,
                    "camera_topic": solver.camera_topic,
                    # Keep the solver-side identity endpoints relative and
                    # remap each one to its corresponding observer below.
                    "lidar_target_identity_topic": "lidar_target_identity",
                    "camera_target_identity_topic": "camera_target_identity",
                    "debug_mode": settings.debug_mode,
                    "publishing_rate": 10.0,
                    # The camera's answer. This node's detection subscriptions
                    # are LCTK's own topics, pinned RELIABLE inside the node;
                    # this governs the camera_info it reads and, in assisted
                    # mode, the preview frame.
                    "use_best_effort_qos": solver.use_best_effort_qos,
                    **sync,
                    "target_config": solver.target_config,
                    # assisted-mode tuning; harmless for the other two, which
                    # never read it.
                    **assisted,
                },
                remappings=(
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
                ),
                arguments=log_arguments,
            )
        )
    return entries


def _lidar_lidar_solvers(pipeline, sync, log_arguments) -> list[PlanEntry]:
    entries: list[PlanEntry] = []
    for solver in pipeline.lidar_lidar_solvers:
        entries.append(
            Message(
                f"  LiDAR-LiDAR solver: {solver.node_name} "
                f"({solver.lidar1_name} <-> {solver.lidar2_name})"
            )
        )
        entries.append(
            NodeSpec(
                package="lidar_to_lidar_solver",
                executable="lidar_to_lidar_solver",
                name="lidar_to_lidar_solver",
                namespace=solver.namespace,
                parameters={
                    "lidar1_detections_topic": solver.lidar1_detections_topic,
                    "lidar2_detections_topic": solver.lidar2_detections_topic,
                    "lidar1_frame": solver.lidar1_frame,
                    "lidar2_frame": solver.lidar2_frame,
                    # The same `sync:` section the LiDAR-camera solvers read.
                    # This used to hardcode a 0.0 window, which conflux reads as
                    # INFINITE: it then pairs by arrival order instead of by
                    # time, and two streams at different rates drift apart
                    # without bound.
                    **sync,
                    "publish_tf": True,
                    # This solver has no sensor subscription at all -- both
                    # inputs are detection topics LCTK publishes -- so it always
                    # asks for the reliability those are pinned to.
                    "use_best_effort_qos": False,
                    "max_message_age_ms": 0.0,
                },
                remappings=(
                    (
                        "lidar1_target_identity",
                        _identity_topic_for_detection(solver.lidar1_detections_topic),
                    ),
                    (
                        "lidar2_target_identity",
                        _identity_topic_for_detection(solver.lidar2_detections_topic),
                    ),
                ),
                arguments=log_arguments,
            )
        )
    return entries


def _tf_tree_broadcaster(pipeline) -> list[PlanEntry]:
    """One broadcaster, fed only by the solvers on the plan's spanning tree.

    A validation edge's solver publishes a transform too, but broadcasting it
    would put a second parent on a frame that already has one.
    """
    tree_edges = {
        (edge.parent, edge.child) for edge in pipeline.calibration_plan.tree_edges
    }

    topics = []
    for solver in pipeline.lidar_camera_solvers:
        pair = (solver.lidar_name, solver.camera_name)
        if pair in tree_edges or pair[::-1] in tree_edges:
            topics.append(solver.output_topic)
    for solver in pipeline.lidar_lidar_solvers:
        pair = (solver.lidar1_name, solver.lidar2_name)
        if pair in tree_edges or pair[::-1] in tree_edges:
            topics.append(solver.output_topic)

    if not topics:
        return []
    return [
        Message(f"  TF tree broadcaster: {len(topics)} tree edge(s)"),
        NodeSpec(
            package="lctk_launch",
            executable="tf_tree_broadcaster",
            name="tf_tree_broadcaster",
            namespace="calibration",
            parameters={"topics": topics},
        ),
    ]


def _overlays(pipeline, settings, log_arguments) -> list[PlanEntry]:
    if not (settings.enable_overlay and pipeline.lidar_camera_solvers):
        return []
    entries: list[PlanEntry] = [
        Message(f"  Overlay nodes: {len(pipeline.lidar_camera_solvers)}")
    ]
    for solver in pipeline.lidar_camera_solvers:
        lidar = pipeline.lidars[solver.lidar_name]
        inliers_topic = (
            f"/calibration/{solver.lidar_name}_{solver.marker_name}/debug/plane_inliers"
        )
        entries.append(
            NodeSpec(
                package="pointcloud_image_overlay",
                executable="overlay_node",
                name="pointcloud_image_overlay",
                namespace=solver.namespace,
                # A viewer, not a pipeline stage: dropping a frame under load
                # costs nothing, and BEST_EFFORT can receive from a publisher of
                # either kind, so it needs no session answer.
                parameters={"use_best_effort_qos": True},
                remappings=(
                    ("image", solver.camera_topic),
                    ("pointcloud", lidar.pointcloud_topic),
                    ("plane_inliers", inliers_topic),
                    ("extrinsic_transform", solver.output_topic),
                ),
                arguments=log_arguments,
            )
        )
    return entries


def _judges(pipeline, settings) -> list[PlanEntry]:
    if not (settings.enable_judge and pipeline.lidar_camera_solvers):
        return []
    entries: list[PlanEntry] = [
        Message(f"  Judge nodes: {len(pipeline.lidar_camera_solvers)}")
    ]
    entries += [
        JudgeInclude(
            transform_topic=solver.output_topic,
            namespace=solver.namespace,
            log_level=settings.log_level,
        )
        for solver in pipeline.lidar_camera_solvers
    ]
    return entries
