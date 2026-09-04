"""ROS adapter for continuous, manual and assisted LiDAR-to-camera calibration."""

import json
import os
import sys
import tempfile
import threading
from collections.abc import Sequence
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from lctk_autoware_export.export import ExportError, patch_calibration
from lctk_interfaces.msg import CalibrationTargetIdentity
from lctk_interfaces.srv import (
    AddDetectionToBuffer,
    AdjustTransform,
    ClearDetectionBuffer,
    DumpDetections,
    GetBufferStatus,
    GetPoseInfo,
    ListDetectionBuffer,
    LoadDetections,
    RemoveDetectionFromBuffer,
    ResetTransform,
)
from lctk_quality import compute_diversity
from lctk_quality.placements import (
    DEFAULT_ORIENTATION_TOL_DEG,
    DEFAULT_POSITION_TOL_M,
    Placement,
    board_normal,
)
from lctk_sync import DetectionPairSource, PairSourceConfig
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from lidar_to_camera_solver.board_geometry import (
    CAMERA_TARGET_IDENTITY_TOPIC,
    LIDAR_TARGET_IDENTITY_TOPIC,
    TargetIdentityGate,
    ValidatedTarget,
    load_target_definition,
    marker_geometry_summary,
    rotation_matrix_to_quaternion,
)
from lidar_to_camera_solver.detection_buffer import (
    BufferSnapshot,
    BufferUpdate,
    DetectionBuffer,
    DetectionPair,
    Empty,
    Failed,
    NotReady,
    Refused,
    RejectionCode,
    Solved,
    pose_reprojection_rms,
    reject_outlier_poses,
)
from lidar_to_camera_solver.detection_format import (
    decode_detection_archive,
    encode_detection_archive,
    select_loaded_adjustment,
)
from lidar_to_camera_solver.preview import PreviewStore, decode_image
from lidar_to_camera_solver.review_server import ReviewServer
from lidar_to_camera_solver.stability import StillnessTracker

SOLVER_MODES = ("continuous", "manual", "assisted")


def target_identity_qos_profile() -> QoSProfile:
    """QoS contract for late-joining Target Identity consumers."""

    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def create_target_identity_subscriptions(
    node,
    callback,
    *,
    lidar_topic: str = LIDAR_TARGET_IDENTITY_TOPIC,
    camera_topic: str = CAMERA_TARGET_IDENTITY_TOPIC,
) -> tuple[object, object]:
    """Subscribe to both relative, latched observer identity endpoints."""

    qos = target_identity_qos_profile()
    lidar_subscription = node.create_subscription(
        CalibrationTargetIdentity,
        lidar_topic,
        lambda message: callback("lidar", message),
        qos,
    )
    camera_subscription = node.create_subscription(
        CalibrationTargetIdentity,
        camera_topic,
        lambda message: callback("camera", message),
        qos,
    )
    return lidar_subscription, camera_subscription


def parse_solver_mode(value: str) -> str:
    """Validate the operator-facing solver policy."""
    if value not in SOLVER_MODES:
        choices = "', '".join(SOLVER_MODES)
        raise ValueError(f"Invalid solver_mode '{value}'; expected '{choices}'.")
    return value


def board_pose_from_detections(
    message: object,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
    """The board's position and unit quaternion, or ``None`` if unreadable.

    This is deliberately a *lenient* reader, unlike ``DetectionBuffer._read_board``
    which raises an admission error naming the exact defect. The stillness tracker
    runs on every synchronized pair, including the ones the buffer would refuse, and
    a detector hiccup must not be able to raise out of a subscription callback.
    The buffer still does the strict reading when a capture is actually attempted.
    """

    detections = getattr(message, "detections", ())
    if not detections:
        return None
    results = getattr(detections[0], "results", ())
    if not results:
        return None
    pose = results[0].pose.pose
    position = np.asarray(
        (pose.position.x, pose.position.y, pose.position.z), dtype=np.float64
    )
    quaternion = np.asarray(
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quaternion))
    if (
        not np.all(np.isfinite(position))
        or not np.all(np.isfinite(quaternion))
        or norm < 1e-12
    ):
        return None
    quaternion = quaternion / norm
    return (
        tuple(float(value) for value in position),
        tuple(float(value) for value in quaternion),
    )


def aruco_corner_quads(message: object) -> list[np.ndarray]:
    """One 4x2 array of corner pixels per detected marker.

    The same extraction ``DetectionBuffer._prepare_pair`` performs, minus the
    board-geometry lookup: the preview draws whatever the detector saw, including
    markers that are not part of the selected target, because a stray marker in the
    frame is exactly the kind of thing a reviewer needs to see.
    """

    quads: list[np.ndarray] = []
    for detection in getattr(message, "detections", ()):
        results = getattr(detection, "results", ())
        if len(results) < 4:
            continue
        pixels = np.asarray(
            [
                (result.pose.pose.position.x, result.pose.pose.position.y)
                for result in results[:4]
            ],
            dtype=np.float64,
        )
        if pixels.shape != (4, 2) or not np.all(np.isfinite(pixels)):
            continue
        quads.append(pixels)
    return quads


def placement_is_new(
    position: Sequence[float],
    quaternion: Sequence[float],
    placements: Sequence[Placement],
    *,
    position_tol_m: float = DEFAULT_POSITION_TOL_M,
    orientation_tol_deg: float = DEFAULT_ORIENTATION_TOL_DEG,
) -> bool:
    """Would this pose form a placement the buffer does not already hold?

    The same test `lctk_quality.distinct_placements` applies when it groups frames,
    asked of one candidate pose against the placements already captured. It has to
    agree with that grouping: N is the number of *distinct placements*, never the
    frame count, so a capture the metric would fold into an existing placement adds
    nothing and makes every frame-counting number more confident about a worse
    calibration.

    The tolerances are configurable because how far apart two placements must be is
    a judgement about the scene, not a property of the code; they default to
    lctk_quality's own.
    """

    candidate_position = np.asarray(position, dtype=np.float64)
    candidate_normal = board_normal(quaternion)
    for placement in placements:
        distance = float(
            np.linalg.norm(
                candidate_position - np.asarray(placement.position, dtype=np.float64)
            )
        )
        if distance > position_tol_m:
            continue
        # abs(): the board is a plane, so a normal and its negation are the same
        # orientation.
        cosine = abs(
            float(np.dot(candidate_normal, np.asarray(placement.normal, np.float64)))
        )
        angle = float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))
        if angle <= orientation_tol_deg:
            return False
    return True


def rotation_vector_to_euler(rvec: np.ndarray, *, degrees: bool = False) -> np.ndarray:
    """Render one solved rotation vector for ROS response fields."""
    owned_rvec = np.array(rvec, dtype=np.float64, copy=True).reshape(3)
    return Rotation.from_rotvec(owned_rvec).as_euler("xyz", degrees=degrees)


class LidarToCameraSolver(Node):
    """ROS services and publication around :class:`DetectionBuffer`."""

    # M-12 lives in detection_buffer next to the solve it guards; it is surfaced here
    # because the node class is the package's public seam -- one implementation, two names.
    _pose_reprojection_rms = staticmethod(pose_reprojection_rms)
    _reject_outlier_poses = staticmethod(reject_outlier_poses)

    def __init__(self):
        super().__init__("lidar_to_camera_solver")
        self._declare_parameters()

        self.solver_mode = parse_solver_mode(self._string_parameter("solver_mode"))
        self.parent_frame = self._string_parameter("parent_frame")
        self.child_frame = self._string_parameter("child_frame")
        camera_topic = self._string_parameter("camera_topic")
        target_config_file = self._string_parameter("target_config")
        publishing_rate = self._double_parameter("publishing_rate")
        self.min_frames_required = self._integer_parameter("min_frames_required")
        self.solve_min_frames = (
            1 if self.solver_mode == "continuous" else self.min_frames_required
        )
        self.min_normal_spread_deg = self._double_parameter("min_normal_spread_deg")
        self.min_depth_range_m = self._double_parameter("min_depth_range_m")
        self.enforce_pose_diversity = self._bool_parameter("enforce_pose_diversity")
        self.axis_length = self._double_parameter("axis_length")
        self.axis_diameter = self._double_parameter("axis_diameter")
        use_best_effort_qos = self._bool_parameter("use_best_effort_qos")

        pair_source_config = PairSourceConfig(
            window_ms=self._double_parameter("sync_tolerance_ms"),
            queue_size=self._integer_parameter("sync_queue_size"),
            drop_policy=self._string_parameter("sync_drop_policy"),
            require_non_empty=True,
            max_pair_age_s=self._double_parameter("max_pair_age_s"),
            stats_interval_s=self._double_parameter("sync_stats_interval_s"),
            epoch_check_interval_s=self._double_parameter("epoch_check_interval_s"),
        )

        self.target = self._load_target_definition(target_config_file)
        self.marker_corners_by_id = self.target.marker_corners_by_id
        self.identity_gate = TargetIdentityGate(self.target.identity)
        self.get_logger().debug(
            f"Board geometry: {marker_geometry_summary(self.target)}"
        )

        # Buffer owns captures, solve state, quality, and its own lock. This lock owns
        # only node-level adjustment/publication state.
        self.detection_buffer: DetectionBuffer | None = None
        self.camera_info: CameraInfo | None = None
        self._camera_matrix: np.ndarray | None = None
        self.current_rvec: np.ndarray | None = None
        self.current_tvec: np.ndarray | None = None
        self.last_transform: TransformStamped | None = None
        self.publishing_enabled = False
        self._continuous_solve_count = 0
        # Assisted-mode state.  All three stay None in continuous and manual, which
        # is what keeps those two paths byte-for-byte the behaviour they had.
        self._stillness: StillnessTracker | None = None
        self._preview_store: PreviewStore | None = None
        self._review_server: ReviewServer | None = None
        self._last_stillness = None
        self._last_epoch_resets = 0
        self._novelty_position_tol_m = DEFAULT_POSITION_TOL_M
        self._novelty_orientation_tol_deg = DEFAULT_ORIENTATION_TOL_DEG
        self.state_lock = threading.RLock()
        # Bump whenever a solver-state reset invalidates an in-flight mutation.
        # Continuous solves capture this token before doing work and must match it
        # again before rebasing or publishing their result.
        self._identity_generation = 0

        # Observer identities are relative and latched by both detectors.  Their
        # QoS is independent of detection QoS so a late-starting solver receives
        # the selected target even in a realtime (best-effort) graph.
        (
            self.lidar_identity_subscription,
            self.camera_identity_subscription,
        ) = create_target_identity_subscriptions(
            self,
            self._target_identity_callback,
            lidar_topic=self._string_parameter("lidar_target_identity_topic"),
            camera_topic=self._string_parameter("camera_target_identity_topic"),
        )

        # Two profiles, because this node has two kinds of endpoint and only one
        # of them is anybody else's decision.
        #
        # The detection topics and the transform this node publishes are LCTK's
        # own, ours on both ends, so they are pinned RELIABLE. The camera_info
        # and the assisted-mode preview frame come from a camera we do not own,
        # so they take the reliability the session resolved for that device (see
        # lctk_launch/transport.py). Asking for RELIABLE against a BEST_EFFORT
        # publisher receives nothing at all.
        internal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if use_best_effort_qos
                else ReliabilityPolicy.RELIABLE
            ),
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.get_logger().info(
            "QoS: detections and transforms RELIABLE; camera "
            f"{'BEST_EFFORT' if use_best_effort_qos else 'RELIABLE'}"
        )

        self.transform_publisher = self.create_publisher(
            TransformStamped, "extrinsic_transform", internal_qos
        )
        self.axis_marker_publisher = self.create_publisher(
            MarkerArray, "axis_markers", internal_qos
        )
        self.publishing_timer = self.create_timer(
            1.0 / publishing_rate, self._publishing_timer_callback
        )
        self.pair_source = DetectionPairSource(
            self,
            topics=["aruco_detections", "calibration_board_detections"],
            msg_types=[Detection2DArray, Detection3DArray],
            config=pair_source_config,
            qos=internal_qos,
            on_pair=(
                self._continuous_pair_callback
                if self.solver_mode == "continuous"
                else self._assisted_pair_callback
                if self.solver_mode == "assisted"
                else None
            ),
            admit_pair=self._admit_detection_pair,
            admission_lock=self.state_lock,
        )

        if camera_topic and "/" in camera_topic:
            camera_info_topic = f"{camera_topic.rsplit('/', 1)[0]}/camera_info"
        else:
            camera_info_topic = "camera_info"
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, sensor_qos
        )
        self.image_subscription = None
        if self.solver_mode == "assisted":
            self._start_assisted(camera_topic, sensor_qos)
        # Assisted is a multi-pose buffer too, so it gets the manual services: the
        # interactive controller still attaches, and dump/load stays reachable.
        if self.solver_mode in ("manual", "assisted"):
            self._create_services()
        else:
            self._services = []
        self.get_logger().info(
            "LiDAR-to-camera solver initialized\n"
            f"Solver mode: {self.solver_mode}\n"
            f"Minimum frames before solving: {self.solve_min_frames}\n"
            f"Camera info: {camera_info_topic}\n"
            f"Target: {self.target.target_id}@{self.target.revision}\n"
            f"Transform: {self.parent_frame} -> {self.child_frame}"
        )

    def _declare_parameters(self) -> None:
        parameters = (
            ("solver_mode", "continuous"),
            ("parent_frame", "lidar"),
            ("child_frame", "camera"),
            ("camera_topic", ""),
            ("target_config", ""),
            ("debug_mode", True),
            ("publishing_rate", 10.0),
            ("min_frames_required", 2),
            ("min_normal_spread_deg", 20.0),
            ("min_depth_range_m", 1.0),
            ("enforce_pose_diversity", False),
            ("axis_length", 0.3),
            ("axis_diameter", 0.02),
            ("use_best_effort_qos", True),
            ("sync_tolerance_ms", 50.0),
            ("sync_queue_size", 10),
            ("sync_drop_policy", "reject_new"),
            ("max_pair_age_s", 2.0),
            ("sync_stats_interval_s", 10.0),
            ("epoch_check_interval_s", 1.0),
            ("lidar_target_identity_topic", LIDAR_TARGET_IDENTITY_TOPIC),
            ("camera_target_identity_topic", CAMERA_TARGET_IDENTITY_TOPIC),
            # Assisted mode only.  Read once in _start_assisted; ignored by the
            # other two modes, which never construct the subsystems that use them.
            ("stability_window_s", 1.0),
            ("stability_max_translation_m", 0.005),
            ("stability_max_rotation_deg", 0.5),
            ("stability_cooldown_s", 1.0),
            ("novelty_position_tol_m", DEFAULT_POSITION_TOL_M),
            ("novelty_orientation_tol_deg", DEFAULT_ORIENTATION_TOL_DEG),
            ("review_bind_host", "127.0.0.1"),
            ("review_port", 8080),
            ("review_jpeg_quality", 80),
            ("review_max_previews", 64),
            ("review_archive_path", ""),
            ("export_autoware_target", ""),
            ("export_camera_frame", ""),
            ("export_lidar_frame", ""),
        )
        for name, default in parameters:
            self.declare_parameter(name, default)

    def _start_assisted(self, camera_topic: str, sensor_qos: QoSProfile) -> None:
        """Build the stillness gate, the preview store and the review server.

        Called only for ``solver_mode=assisted``.  Nothing here is reachable from
        the other two modes, so neither can change behaviour because of it.
        """

        self._stillness = StillnessTracker(
            window_s=self._double_parameter("stability_window_s"),
            max_translation_m=self._double_parameter("stability_max_translation_m"),
            max_rotation_deg=self._double_parameter("stability_max_rotation_deg"),
            cooldown_s=self._double_parameter("stability_cooldown_s"),
        )
        self._novelty_position_tol_m = self._double_parameter("novelty_position_tol_m")
        self._novelty_orientation_tol_deg = self._double_parameter(
            "novelty_orientation_tol_deg"
        )
        self._preview_store = PreviewStore(
            max_previews=self._integer_parameter("review_max_previews"),
            jpeg_quality=self._integer_parameter("review_jpeg_quality"),
        )
        if camera_topic:
            # The image is for the reviewer, never for the solve, so it takes the
            # same QoS as the detections and keeps only the newest frame.
            self.image_subscription = self.create_subscription(
                Image, camera_topic, self._image_callback, sensor_qos
            )
        else:
            self.get_logger().warn(
                "camera_topic is unset; the review page will show no previews"
            )

        host = self._string_parameter("review_bind_host")
        self._review_server = ReviewServer(
            self, host=host, port=self._integer_parameter("review_port")
        )
        self._review_server.start()
        if host not in ("127.0.0.1", "localhost"):
            self.get_logger().warn(
                f"review server bound to {host}:{self._review_server.port} -- "
                "the queue, the camera previews and the solved extrinsic are "
                "readable by anyone who can reach that port, and there is no "
                "authentication"
            )
        else:
            self.get_logger().info(
                f"review server on http://{host}:{self._review_server.port}"
            )

    def _image_callback(self, message) -> None:
        """Store the newest frame and return.

        Per the ArcSwap guidance in CLAUDE.md this stays cheap: the annotate and
        JPEG encode happen at capture time, on the pair callback, not here.
        """

        if self._preview_store is None:
            return
        try:
            frame = decode_image(
                height=message.height,
                width=message.width,
                encoding=message.encoding,
                step=message.step,
                data=bytes(message.data),
            )
        except ValueError as error:
            self.get_logger().warn(
                f"preview disabled for this frame: {error}",
                throttle_duration_sec=10.0,
            )
            return
        self._preview_store.set_latest(frame)

    def _create_services(self) -> None:
        services = (
            (AddDetectionToBuffer, "~/add_detection", self.add_detection_callback),
            (ClearDetectionBuffer, "~/clear_buffer", self.clear_buffer_callback),
            (GetBufferStatus, "~/get_status", self.get_status_callback),
            (ListDetectionBuffer, "~/list_buffer", self.list_buffer_callback),
            (
                RemoveDetectionFromBuffer,
                "~/remove_detection",
                self.remove_detection_callback,
            ),
            (DumpDetections, "~/dump_detections", self.dump_detections_callback),
            (LoadDetections, "~/load_detections", self.load_detections_callback),
            (AdjustTransform, "~/adjust_transform", self.adjust_transform_callback),
            (ResetTransform, "~/reset_transform", self.reset_transform_callback),
            (GetPoseInfo, "~/get_pose_info", self.get_pose_info_callback),
        )
        self._services = [
            self.create_service(service_type, name, callback)
            for service_type, name, callback in services
        ]

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _integer_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _new_buffer(self, camera_matrix: np.ndarray) -> DetectionBuffer:
        return DetectionBuffer(
            camera_matrix=camera_matrix,
            marker_corners_by_id=self.marker_corners_by_id,
            min_frames_required=self.solve_min_frames,
            min_normal_spread_deg=self.min_normal_spread_deg,
            min_depth_range_m=self.min_depth_range_m,
            enforce_pose_diversity=self.enforce_pose_diversity,
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Start a session lazily; changed intrinsics start a clean session."""
        camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        replacement = self._new_buffer(camera_matrix)
        with self.state_lock:
            if self._camera_matrix is not None and np.array_equal(
                camera_matrix, self._camera_matrix
            ):
                self.camera_info = msg
                return
            changed = self._camera_matrix is not None
            self.camera_info = msg
            self._camera_matrix = camera_matrix.copy()
            self.detection_buffer = replacement
            self._clear_adjustment_locked()
            if changed:
                self._identity_generation += 1
                self.pair_source.discard_cached_pair()
        if changed:
            self.get_logger().warn(
                "Camera intrinsic matrix changed; started a new calibration session"
            )
        else:
            self.get_logger().debug(f"Camera info received: {msg.width}x{msg.height}")

    def _publishing_timer_callback(self, expected_generation: int | None = None):
        with self.state_lock:
            if expected_generation is not None and (
                expected_generation != self._identity_generation
            ):
                return False
            if self.identity_gate.error is not None:
                return False
            if not self.publishing_enabled or self.last_transform is None:
                return False
            self.last_transform.header.stamp = self.get_clock().now().to_msg()
            self.transform_publisher.publish(self.last_transform)
            self._publish_axis_markers()
            return True

    def _continuous_pair_callback(self, messages: tuple[object, ...]) -> None:
        """Replace the latest capture, solve it, and publish without operator action."""
        aruco, board = messages
        # Keep the identity check and the buffer mutation under one node lock.  An
        # identity callback may arrive on another executor thread; it must not be
        # able to change the accepted target between these two operations.
        with self.state_lock:
            if not self.pair_source.is_cached_pair(messages):
                self.get_logger().warn(
                    "Ignoring synchronized detection pair: cache entry was "
                    "discarded or superseded before continuous restore",
                    throttle_duration_sec=5.0,
                )
                return
            generation = self._identity_generation
            error = self.identity_gate.error
            buffer = self.detection_buffer
            if error is not None:
                self.get_logger().warn(
                    f"Ignoring synchronized detection pair: {error}",
                    throttle_duration_sec=5.0,
                )
                return
            if buffer is None:
                self.get_logger().warn(
                    "Ignoring synchronized detection pair: no camera info available",
                    throttle_duration_sec=5.0,
                )
                return
            update = buffer.restore(
                (DetectionPair(aruco=aruco, board=board),), append=False
            )
        if not update.accepted:
            self.get_logger().error(
                f"Continuous solve rejected detection pair: {self._rejection_text(update)}",
                throttle_duration_sec=5.0,
            )
            return

        self._continuous_solve_count += 1
        applied = self._apply_update(
            update,
            log_quality_warnings=self._continuous_solve_count % 30 == 1,
            expected_generation=generation,
        )
        if not applied:
            self.get_logger().warn(
                "Continuous solve result invalidated by a target or session reset",
                throttle_duration_sec=5.0,
            )
            return
        if not isinstance(update.snapshot.outcome, Solved):
            self.get_logger().warn(
                f"Continuous solve unavailable: {self._status_text(update.snapshot)}",
                throttle_duration_sec=5.0,
            )
            return

        # Match the superseded continuous solver's observable behaviour: each
        # synchronized pair produces a publication immediately. The normal timer
        # continues refreshing the latest transform for late subscribers.
        self._publishing_timer_callback(expected_generation=generation)

    def _assisted_pair_callback(self, messages: tuple[object, ...]) -> None:
        """Auto-capture a pair when the board is held still in a new placement.

        Two gates, and both are load-bearing.  Stillness stops motion blur.
        Novelty stops the degenerate capture -- one placement filmed forty times --
        that `lctk_quality` exists to detect and that every residual-based number
        rates as excellent.

        Novelty is asked twice, and the two questions differ.  `placement_is_new`
        asks it against the operator's configured tolerances *before* capturing, so
        a rig that wants placements further apart than lctk_quality's 5 cm / 5 deg
        gets what it asked for.  `added_new_placement` then reports what the buffer's
        own grouping concluded, which is what the diversity metric will count; a
        capture the metric would fold into an existing placement is undone rather
        than left to pad the buffer.
        """

        aruco, board = messages
        epoch_resets = self.pair_source.epoch_resets
        if epoch_resets != self._last_epoch_resets:
            # The recording changed under the synchronizer.  The window this
            # tracker filled belongs to the previous epoch, and a "still" verdict
            # must not carry across the seam.
            self._last_epoch_resets = epoch_resets
            self._stillness.reset()

        pose = board_pose_from_detections(board)
        if pose is None:
            self.get_logger().warn(
                "Ignoring synchronized detection pair: no readable board pose",
                throttle_duration_sec=5.0,
            )
            return
        position, orientation = pose
        stamp_s = self.get_clock().now().nanoseconds * 1e-9
        state = self._stillness.push(position, orientation, stamp_s)
        self._last_stillness = state
        # Every verdict, at debug level, because tuning the thresholds without it
        # means guessing. The obvious way to measure them -- record the board
        # detection topic and replay it through StillnessTracker offline -- feeds
        # the tracker a stream it never sees: this gate runs on *synchronized
        # pairs*, so a board detection with no ArUco partner inside the sync
        # window never reaches it. That mistake overestimated the usable captures
        # on this rig by more than a factor of two.
        self.get_logger().debug(
            f"stillness: {state.reason} "
            f"[{state.translation_span_m * 1000:.0f} mm / "
            f"{state.rotation_span_deg:.1f} deg over {state.frames} pairs]"
        )
        if not state.should_capture:
            return

        # One node lock over the identity check and the mutation, exactly as the
        # manual capture path does: an identity update must not be able to change
        # the accepted target between the two.
        with self.state_lock:
            generation = self._identity_generation
            buffer = self.detection_buffer
            identity_error = self.identity_gate.error
            if identity_error is not None:
                self.get_logger().warn(
                    "Skipping assisted capture before Target Identity agreement: "
                    f"{identity_error}",
                    throttle_duration_sec=5.0,
                )
                return
            if buffer is None:
                self.get_logger().warn(
                    "Skipping assisted capture: no camera info available",
                    throttle_duration_sec=5.0,
                )
                return
            if not placement_is_new(
                position,
                orientation,
                buffer.snapshot().placements,
                position_tol_m=self._novelty_position_tol_m,
                orientation_tol_deg=self._novelty_orientation_tol_deg,
            ):
                self.get_logger().info(
                    "Held still, but this placement is already captured; "
                    "move or tilt the board",
                    throttle_duration_sec=5.0,
                )
                return
            update = buffer.capture(DetectionPair(aruco=aruco, board=board))
            if not update.accepted:
                self.get_logger().warn(
                    f"Assisted capture rejected: {self._rejection_text(update)}",
                    throttle_duration_sec=5.0,
                )
                return
            pair_id = update.snapshot.frame_count - 1
            if update.added_new_placement is False:
                # Still, but not a new placement.  Undo rather than pad the buffer
                # with a view that adds no geometry and inflates every metric that
                # counts frames.
                buffer.remove(pair_id)
                self.get_logger().info(
                    "Held still, but this is not a new board placement; "
                    "move or tilt the board",
                    throttle_duration_sec=5.0,
                )
                return

        # Outside the lock: annotating and JPEG-encoding a frame is real work, and
        # state_lock is also DetectionPairSource's admission lock.
        self._preview_store.capture(
            pair_id, corners=aruco_corner_quads(aruco), reprojected=None
        )
        if not self._apply_update(update, expected_generation=generation):
            self.get_logger().warn(
                "Assisted capture invalidated by a target or session reset",
                throttle_duration_sec=5.0,
            )
            return
        self.get_logger().info(
            f"Captured new board placement #{pair_id}. "
            f"{self._status_text(update.snapshot)}"
        )

    def _admit_detection_pair(self, _messages: tuple[object, ...]) -> str | None:
        """Reject a pair before :class:`DetectionPairSource` mutates its cache.

        ``DetectionPairSource`` calls this while holding ``admission_lock``.  The
        camera solver supplies ``state_lock`` as that lock, so acquiring it here
        again would couple this callback to a reentrant-lock implementation.
        """

        return self.identity_gate.error

    def _publish_axis_markers(self):
        if self.last_transform is None:
            return
        transform = self.last_transform.transform
        origin = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z]
        )
        quaternion = np.array(
            [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ]
        )
        rotation = Rotation.from_quat(quaternion).as_matrix()
        colors = (
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
        )
        markers = MarkerArray()
        for index, (axis, color) in enumerate(zip(rotation.T, colors)):
            endpoint = origin + axis * self.axis_length
            marker = Marker()
            marker.header = self.last_transform.header
            marker.ns = "calibration_axes"
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.points = [
                Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2])),
                Point(
                    x=float(endpoint[0]),
                    y=float(endpoint[1]),
                    z=float(endpoint[2]),
                ),
            ]
            marker.scale.x = self.axis_diameter
            marker.scale.y = self.axis_diameter * 1.5
            marker.scale.z = self.axis_length * 0.15
            marker.color = color
            marker.lifetime.nanosec = 200_000_000
            markers.markers.append(marker)
        self.axis_marker_publisher.publish(markers)

    def _snapshot(self) -> BufferSnapshot | None:
        buffer = self.detection_buffer
        return buffer.snapshot() if buffer is not None else None

    @staticmethod
    def _status_text(snapshot: BufferSnapshot) -> str:
        outcome = snapshot.outcome
        if isinstance(outcome, Empty):
            return "Empty buffer"
        if isinstance(outcome, NotReady):
            return f"Insufficient frames: {outcome.frame_count}/{outcome.required} required"
        if isinstance(outcome, Refused):
            return (
                "Refused: insufficient pose diversity "
                f"(normal spread {outcome.normal_spread_deg:.1f} deg, "
                f"depth range {outcome.depth_range_m:.2f} m)"
            )
        if isinstance(outcome, Failed):
            detail = f": {outcome.detail}" if outcome.detail else ""
            return f"Solve failed ({outcome.code.name.lower()}){detail}"
        return outcome.estimate.quality.status_line()

    @staticmethod
    def _rejection_text(update: BufferUpdate) -> str:
        rejection = update.rejection
        if rejection is None:
            return "Mutation rejected"
        messages = {
            RejectionCode.INVALID_INDEX: "Invalid buffer index",
            RejectionCode.NO_BOARD_DETECTION: "No board detection available",
            RejectionCode.NO_BOARD_POSE: "No board pose result available",
            RejectionCode.INVALID_BOARD_POSE: "Board pose is invalid",
            RejectionCode.INVALID_COVARIANCE: "Board pose covariance is invalid",
            RejectionCode.NO_REAL_ARUCO_CORNERS: "No real ArUco corners available",
            RejectionCode.NO_CONFIGURED_MARKER: (
                "No detected marker exists in configured geometry"
            ),
            RejectionCode.INSUFFICIENT_CORRESPONDENCES: (
                "Fewer than four usable correspondences"
            ),
        }
        detail = f" ({rejection.detail})" if rejection.detail else ""
        return f"{messages[rejection.code]}{detail}"

    def _apply_update(
        self,
        update: BufferUpdate,
        *,
        log_quality_warnings: bool = True,
        expected_generation: int | None = None,
    ) -> bool:
        """Rebase or clear adjustment after one accepted buffer revision."""
        if not update.accepted:
            return False
        outcome = update.snapshot.outcome
        with self.state_lock:
            if expected_generation is not None and (
                expected_generation != self._identity_generation
            ):
                return False
            # A solved outcome is calibration-target-bound.  Never restore one
            # after the sticky identity gate has closed, even if the caller did
            # not have a generation token (for example, a legacy service path).
            if isinstance(outcome, Solved) and self.identity_gate.error is not None:
                return False
            if isinstance(outcome, Solved):
                estimate = outcome.estimate
                self.current_rvec = np.array(estimate.rvec, copy=True)
                self.current_tvec = np.array(estimate.tvec, copy=True)
                self.last_transform = self._create_transform_message(
                    self.current_rvec, self.current_tvec
                )
                self.publishing_enabled = True
            else:
                self._clear_adjustment_locked()
        if isinstance(outcome, Solved) and log_quality_warnings:
            warnings = outcome.estimate.quality.warnings()
            if warnings:
                self.get_logger().warn("\n".join(warnings))
        return True

    def _clear_adjustment_locked(self) -> None:
        self.current_rvec = None
        self.current_tvec = None
        self.last_transform = None
        self.publishing_enabled = False

    def add_detection_callback(self, request, response):
        with self.state_lock:
            buffer = self.detection_buffer
            identity_error = self.identity_gate.error
            generation = self._identity_generation
        if identity_error is not None:
            response.success = False
            response.message = (
                f"Cannot capture before Target Identity agreement: {identity_error}"
            )
            response.buffer_size = buffer.snapshot().frame_count if buffer else 0
            return response
        if buffer is None:
            response.success = False
            response.message = "No camera info available"
            response.buffer_size = 0
            return response
        pair_outcome = self.pair_source.take_fresh_pair()
        if not pair_outcome.ok:
            response.success = False
            response.message = pair_outcome.reason
            response.buffer_size = buffer.snapshot().frame_count
            return response
        aruco, board = pair_outcome.messages
        with self.state_lock:
            # Re-check after waiting for the pair.  A clear, camera reset, or
            # identity update while waiting invalidates the cached pair.
            if generation != self._identity_generation:
                response.success = False
                response.message = (
                    "Cannot capture: calibration session changed while waiting; retry"
                )
                response.buffer_size = buffer.snapshot().frame_count
                return response
            identity_error = self.identity_gate.error
            if identity_error is not None:
                response.success = False
                response.message = (
                    f"Cannot capture before Target Identity agreement: {identity_error}"
                )
                response.buffer_size = buffer.snapshot().frame_count
                return response
            update = buffer.capture(DetectionPair(aruco=aruco, board=board))
        response.buffer_size = update.snapshot.frame_count
        if not update.accepted:
            response.success = False
            response.message = self._rejection_text(update)
            self.get_logger().error(response.message)
            return response
        if not self._apply_update(update, expected_generation=generation):
            response.success = False
            response.message = "Capture invalidated by a target or session reset; retry"
            response.buffer_size = (
                self._snapshot().frame_count if self._snapshot() else 0
            )
            return response
        response.success = True
        placement = "new" if update.added_new_placement else "duplicate"
        response.message = (
            f"Captured {placement} board placement. "
            f"{self._status_text(update.snapshot)}"
        )
        if update.added_new_placement:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message + "; move or tilt the board")
        return response

    def clear_buffer_callback(self, request, response):
        snapshot = self._snapshot()
        old_size = snapshot.frame_count if snapshot is not None else 0
        with self.state_lock:
            self._identity_generation += 1
            if self.detection_buffer is not None:
                self.detection_buffer.clear()
                self._clear_adjustment_locked()
            else:
                self._clear_adjustment_locked()
            self.pair_source.discard_cached_pair()
        response.success = True
        response.message = f"Cleared {old_size} detection pairs from buffer"
        return response

    def get_status_callback(self, request, response):
        snapshot = self._snapshot()
        with self.state_lock:
            response.is_publishing = self.publishing_enabled
        if snapshot is None:
            response.buffer_size = 0
            response.total_correspondences = 0
            response.last_solve_status = "No camera info available"
        else:
            response.buffer_size = snapshot.frame_count
            response.total_correspondences = snapshot.correspondence_count
            response.last_solve_status = self._status_text(snapshot)
        return response

    def list_buffer_callback(self, request, response):
        snapshot = self._snapshot()
        pairs = snapshot.pairs if snapshot is not None else ()
        response.success = True
        response.buffer_size = len(pairs)
        response.message = f"Buffer contains {len(pairs)} detection pairs"
        response.aruco_counts = [len(pair.aruco.detections) for pair in pairs]
        response.board_counts = [len(pair.board.detections) for pair in pairs]
        response.timestamps_sec = [pair.aruco.header.stamp.sec for pair in pairs]
        response.timestamps_nanosec = [
            pair.aruco.header.stamp.nanosec for pair in pairs
        ]
        return response

    def remove_detection_callback(self, request, response):
        with self.state_lock:
            buffer = self.detection_buffer
            generation = self._identity_generation
            if buffer is None:
                response.success = False
                response.message = "No camera info available"
                response.buffer_size = 0
                return response
            update = buffer.remove(request.index)
        response.buffer_size = update.snapshot.frame_count
        if not update.accepted:
            response.success = False
            response.message = (
                f"Invalid index {request.index}. Buffer size is {response.buffer_size}"
            )
            return response
        if not self._apply_update(update, expected_generation=generation):
            response.success = False
            response.message = "Removal invalidated by a target or session reset; retry"
            return response
        response.success = True
        response.message = (
            f"Removed detection at index {request.index}. "
            f"{self._status_text(update.snapshot)}"
        )
        return response

    def dump_detections_callback(self, request, response):
        # Read the snapshot, adjusted transform, generation and gate error as one
        # mutually consistent view. Taking them under separate lock acquisitions
        # (as the pre-fix code did) let a target change or camera-intrinsics reset
        # land in between and write an archive that mixed a previous session's
        # captures with the current identity.
        with self.state_lock:
            snapshot = self._snapshot()
            adjusted_rvec = (
                None if self.current_rvec is None else self.current_rvec.copy()
            )
            adjusted_tvec = (
                None if self.current_tvec is None else self.current_tvec.copy()
            )
            generation = self._identity_generation
            identity_error = self.identity_gate.error
            local_identity = self.target.identity
        if snapshot is None or (snapshot.frame_count == 0 and adjusted_rvec is None):
            response.success = False
            response.message = (
                "Buffer is empty and no transform available, nothing to save"
            )
            response.num_detections = 0
            return response
        if identity_error is not None:
            response.success = False
            response.message = (
                f"Cannot save before Target Identity agreement: {identity_error}"
            )
            response.num_detections = 0
            return response
        try:
            archive = encode_detection_archive(
                snapshot,
                local_identity=local_identity,
                adjusted_rvec=adjusted_rvec,
                adjusted_tvec=adjusted_tvec,
            )
        except (TypeError, ValueError) as error:
            response.success = False
            response.message = f"Failed to save detections: {error!s}"
            response.num_detections = 0
            return response

        # Serialize to a sibling temp file with NO lock held: this is the
        # potentially slow part (disk I/O over a possibly multi-hundred-KB
        # archive), and state_lock is also DetectionPairSource's admission lock
        # and the publishing timer's lock, so holding it here would block
        # detection admission and publication for the duration of the write.
        # Mirrors load_detections_callback, which does its file I/O outside the
        # lock and only re-checks generation under the lock afterward.
        destination = Path(request.file_path)
        temp_path: Path | None = None
        try:
            descriptor, temp_name = tempfile.mkstemp(
                dir=str(destination.parent),
                prefix=f".{destination.name}.",
                suffix=".tmp",
            )
            temp_path = Path(temp_name)
            with os.fdopen(descriptor, "w") as file:
                json.dump(archive, file, indent=2)
            with self.state_lock:
                # Re-check immediately before the atomic rename: a target change
                # or camera-intrinsics reset between the snapshot above and here
                # must not let this archive land as if it were still current.
                if generation != self._identity_generation:
                    response.success = False
                    response.message = (
                        "Cannot save: calibration session changed while writing; retry"
                    )
                    response.num_detections = 0
                    return response
                os.replace(temp_path, destination)
                temp_path = None
        except (OSError, TypeError, ValueError) as error:
            response.success = False
            response.message = f"Failed to save detections: {error!s}"
            response.num_detections = 0
            return response
        finally:
            if temp_path is not None:
                try:
                    temp_path.unlink()
                except OSError as cleanup_error:
                    self.get_logger().warn(
                        "Failed to remove temporary archive file "
                        f"{temp_path}: {cleanup_error}"
                    )
        response.success = True
        response.num_detections = snapshot.frame_count
        response.message = (
            f"Saved {snapshot.frame_count} detection pairs to {request.file_path}"
        )
        return response

    def load_detections_callback(self, request, response):
        with self.state_lock:
            buffer = self.detection_buffer
            generation = self._identity_generation
            identity_error = self.identity_gate.error
        if buffer is None:
            response.success = False
            response.message = "No camera info available"
            response.num_detections = 0
            response.buffer_size = 0
            return response
        if identity_error is not None:
            response.success = False
            response.message = (
                f"Cannot load before Target Identity agreement: {identity_error}"
            )
            response.num_detections = 0
            response.buffer_size = buffer.snapshot().frame_count
            return response
        try:
            with open(request.file_path) as file:
                archive = decode_detection_archive(
                    json.load(file), local_identity=self.target.identity
                )
        except FileNotFoundError:
            response.success = False
            response.message = f"File not found: {request.file_path}"
            response.num_detections = 0
            response.buffer_size = buffer.snapshot().frame_count
            return response
        except (
            OSError,
            json.JSONDecodeError,
            KeyError,
            TypeError,
            ValueError,
        ) as error:
            response.success = False
            response.message = f"Failed to load detections: {error!s}"
            response.num_detections = 0
            response.buffer_size = buffer.snapshot().frame_count
            return response
        # Keep the generation check and restore under the same node lock.  If
        # identity/session invalidation wins first, the archive must not refill
        # the buffer that the invalidation just cleared.
        with self.state_lock:
            if generation != self._identity_generation:
                response.success = False
                response.message = (
                    "Cannot load: calibration session changed while reading; retry"
                )
                response.num_detections = 0
                response.buffer_size = buffer.snapshot().frame_count
                return response
            identity_error = self.identity_gate.error
            if identity_error is not None:
                response.success = False
                response.message = (
                    f"Cannot load before Target Identity agreement: {identity_error}"
                )
                response.num_detections = 0
                response.buffer_size = buffer.snapshot().frame_count
                return response
            update = buffer.restore(archive.pairs, append=request.append)
        response.num_detections = len(archive.pairs)
        response.buffer_size = update.snapshot.frame_count
        if not update.accepted:
            response.success = False
            response.message = (
                f"Failed to load detections: {self._rejection_text(update)}"
            )
            return response
        if not self._apply_update(update, expected_generation=generation):
            response.success = False
            response.message = "Load invalidated by a target or session reset; retry"
            response.buffer_size = (
                self._snapshot().frame_count if self._snapshot() else 0
            )
            return response
        selected_adjustment = select_loaded_adjustment(
            archive, update.snapshot, append=request.append
        )
        if selected_adjustment is not None:
            with self.state_lock:
                if (
                    generation != self._identity_generation
                    or self.identity_gate.error is not None
                ):
                    response.success = False
                    response.message = (
                        "Load invalidated by a target or session reset; retry"
                    )
                    response.buffer_size = buffer.snapshot().frame_count
                    return response
                self.current_rvec = np.array(selected_adjustment.rvec, copy=True)
                self.current_tvec = np.array(selected_adjustment.tvec, copy=True)
                self.last_transform = self._create_transform_message(
                    self.current_rvec, self.current_tvec
                )
                self.publishing_enabled = True
        restored_adjustment = (
            not request.append
            and update.snapshot.estimate is not None
            and archive.adjusted_transform is not None
        )
        response.success = True
        suffix = " with adjusted transform" if restored_adjustment else ""
        response.message = (
            f"Loaded {len(archive.pairs)} detection pairs{suffix}. "
            f"{self._status_text(update.snapshot)}"
        )
        return response

    def adjust_transform_callback(self, request, response):
        with self.state_lock:
            if self.current_rvec is None or self.current_tvec is None:
                response.success = False
                response.message = (
                    "No transform available to adjust. Solve calibration first."
                )
                return response
            self.current_tvec += np.array(
                [[request.delta_x], [request.delta_y], [request.delta_z]]
            )
            delta = (request.delta_roll, request.delta_pitch, request.delta_yaw)
            if any(value != 0.0 for value in delta):
                current_matrix, _ = cv2.Rodrigues(self.current_rvec)
                adjusted = Rotation.from_euler("xyz", delta) * Rotation.from_matrix(
                    current_matrix
                )
                self.current_rvec, _ = cv2.Rodrigues(adjusted.as_matrix())
            self.last_transform = self._create_transform_message(
                self.current_rvec, self.current_tvec
            )
            euler = rotation_vector_to_euler(self.current_rvec, degrees=True)
            response.success = True
            response.message = (
                "Transform adjusted: "
                f"t=({self.current_tvec[0, 0]:.4f}, "
                f"{self.current_tvec[1, 0]:.4f}, {self.current_tvec[2, 0]:.4f}), "
                f"rpy=({euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}) deg"
            )
        return response

    def reset_transform_callback(self, request, response):
        with self.state_lock:
            generation = self._identity_generation
            buffer = self.detection_buffer
            snapshot = buffer.snapshot() if buffer is not None else None
            if snapshot is None or snapshot.estimate is None:
                response.success = False
                response.message = "Cannot reset: current buffer has no solved estimate"
                return response
            if self.identity_gate.error is not None:
                response.success = False
                response.message = (
                    "Cannot reset before Target Identity agreement: "
                    f"{self.identity_gate.error}"
                )
                return response
            if generation != self._identity_generation:
                response.success = False
                response.message = (
                    "Reset invalidated by a target or session reset; retry"
                )
                return response
            self.current_rvec = np.array(snapshot.estimate.rvec, copy=True)
            self.current_tvec = np.array(snapshot.estimate.tvec, copy=True)
            self.last_transform = self._create_transform_message(
                self.current_rvec, self.current_tvec
            )
            self.publishing_enabled = True
        response.success = True
        response.message = "Reset transform to current solved estimate"
        return response

    def get_pose_info_callback(self, request, response):
        snapshot = self._snapshot()
        if snapshot is None or snapshot.estimate is None:
            response.has_pose = False
            return response
        with self.state_lock:
            if self.current_rvec is None or self.current_tvec is None:
                response.has_pose = False
                return response
            solved_rvec = np.asarray(snapshot.estimate.rvec)
            solved_tvec = np.asarray(snapshot.estimate.tvec)
            current_rvec = self.current_rvec.copy()
            current_tvec = self.current_tvec.copy()
        solved_euler = rotation_vector_to_euler(solved_rvec)
        current_euler = rotation_vector_to_euler(current_rvec)
        response.has_pose = True
        response.solved_x, response.solved_y, response.solved_z = map(
            float, solved_tvec.ravel()
        )
        response.solved_roll, response.solved_pitch, response.solved_yaw = map(
            float, solved_euler
        )
        response.current_x, response.current_y, response.current_z = map(
            float, current_tvec.ravel()
        )
        response.current_roll, response.current_pitch, response.current_yaw = map(
            float, current_euler
        )
        response.adjust_x = response.current_x - response.solved_x
        response.adjust_y = response.current_y - response.solved_y
        response.adjust_z = response.current_z - response.solved_z
        response.adjust_roll = response.current_roll - response.solved_roll
        response.adjust_pitch = response.current_pitch - response.solved_pitch
        response.adjust_yaw = response.current_yaw - response.solved_yaw
        return response

    # --- NodeFacade -----------------------------------------------------------
    # The review server calls these from its own thread.  DetectionBuffer is
    # internally locked, so a snapshot needs no node lock; anything reading node
    # state takes state_lock, and nothing holds it across HTTP or disk I/O --
    # state_lock is also DetectionPairSource's admission lock and the publishing
    # timer's lock, so holding it over a file write would stall the graph.

    def state(self) -> dict:
        """Everything the review page renders, as plain JSON-able data."""

        snapshot = self._snapshot()
        stillness = self._last_stillness
        diversity = (
            compute_diversity(snapshot.placements) if snapshot is not None else None
        )
        estimate = snapshot.estimate if snapshot is not None else None
        per_pose_rms = (
            list(estimate.quality.residuals.per_pose_rms_px)
            if estimate is not None
            else []
        )
        with self.state_lock:
            identity_error = self.identity_gate.error
        return {
            "mode": self.solver_mode,
            "sync": self.pair_source.status_line(),
            "identity_error": identity_error,
            "stillness": {
                "is_still": bool(stillness is not None and stillness.is_still),
                "reason": (
                    stillness.reason
                    if stillness is not None
                    else "waiting for detections"
                ),
                "frames": stillness.frames if stillness is not None else 0,
            },
            "diversity": (
                {
                    "n_placements": diversity.n_placements,
                    "normal_span_deg": diversity.normal_span_deg,
                    "depth_range_m": diversity.depth_range_m,
                    "lateral_span_m": diversity.lateral_span_m,
                    "is_degenerate": diversity.is_degenerate,
                    "shortfalls": diversity.shortfalls(),
                }
                if diversity is not None
                else {"n_placements": 0, "shortfalls": ["no placements yet"]}
            ),
            "solve": {
                "status": (
                    self._status_text(snapshot)
                    if snapshot is not None
                    else "No camera info available"
                ),
                "rms_px": (
                    estimate.quality.residuals.rms_px if estimate is not None else None
                ),
            },
            "pairs": [
                {
                    "id": index,
                    "rms_px": (
                        per_pose_rms[index] if index < len(per_pose_rms) else None
                    ),
                    "has_preview": self.preview(index) is not None,
                }
                for index in range(snapshot.frame_count if snapshot is not None else 0)
            ],
            "export": {
                "archive_path": self._string_parameter("review_archive_path"),
                "autoware_ready": bool(
                    self._string_parameter("export_autoware_target")
                    and self._string_parameter("export_camera_frame")
                    and self._string_parameter("export_lidar_frame")
                ),
            },
        }

    def preview(self, pair_id: int) -> bytes | None:
        if self._preview_store is None:
            return None
        return self._preview_store.get(pair_id)

    def drop(self, pair_id: int) -> tuple[bool, str]:
        with self.state_lock:
            buffer = self.detection_buffer
            if buffer is None:
                return False, "No camera info available"
            generation = self._identity_generation
            update = buffer.remove(pair_id)
        if not update.accepted:
            return False, self._rejection_text(update)
        if self._preview_store is not None:
            self._preview_store.drop(pair_id)
        if not self._apply_update(update, expected_generation=generation):
            return False, "Removal invalidated by a target or session reset; retry"
        return True, f"Dropped pair {pair_id}. {self._status_text(update.snapshot)}"

    def export_archive(self, path: str) -> tuple[bool, str]:
        """Write the detection archive, reusing the dump service's whole body.

        Duplicating the encode/temp-file/rename here would give the archive two
        implementations and one of them would drift; the service already does its
        file I/O outside state_lock.
        """

        request = DumpDetections.Request()
        request.file_path = path
        response = self.dump_detections_callback(request, DumpDetections.Response())
        return bool(response.success), response.message

    def export_autoware(self, dry_run: bool) -> tuple[bool, str, dict | None]:
        target = self._string_parameter("export_autoware_target")
        camera_frame = self._string_parameter("export_camera_frame")
        lidar_frame = self._string_parameter("export_lidar_frame")
        missing = [
            name
            for name, value in (
                ("export_autoware_target", target),
                ("export_camera_frame", camera_frame),
                ("export_lidar_frame", lidar_frame),
            )
            if not value
        ]
        if missing:
            return False, f"unset parameter(s): {', '.join(missing)}", None
        snapshot = self._snapshot()
        estimate = snapshot.estimate if snapshot is not None else None
        if estimate is None:
            return False, "no solved estimate to export", None
        try:
            # The raw solver rvec/tvec (T_optical<-lidar) is what the exporter
            # consumes -- never the published transform, whose frame labels are
            # inverted (M-01).
            entry = patch_calibration(
                target,
                rvec=np.asarray(estimate.rvec, dtype=np.float64).reshape(3),
                tvec=np.asarray(estimate.tvec, dtype=np.float64).reshape(3),
                camera_frame=camera_frame,
                lidar_frame=lidar_frame,
                dry_run=dry_run,
            )
        except (ExportError, OSError, KeyError, TypeError, ValueError) as error:
            return False, f"{error!s}", None
        verb = "Would write" if dry_run else "Wrote"
        return True, f"{verb} {camera_frame} under {target}", dict(entry)

    def _target_identity_callback(
        self, source: str, message: CalibrationTargetIdentity
    ) -> None:
        """Record one latched observer identity and update the admission gate."""

        with self.state_lock:
            was_ready = self.identity_gate.ready
            error = self.identity_gate.update(source, message)
            if error is not None and was_ready:
                # A source restart must not leave a previously solved transform
                # publishing under a different target binding.
                self._identity_generation += 1
                buffer = self.detection_buffer
                if buffer is not None:
                    buffer.clear()
                self.pair_source.discard_cached_pair()
                self._clear_adjustment_locked()

        if error is None and not was_ready:
            self.get_logger().info(
                "LiDAR, camera, and local Target Identities agree; "
                "Detection Pair admission enabled"
            )
        elif error is not None:
            self.get_logger().error(
                f"Target Identity gate closed after {source} update: {error}",
                throttle_duration_sec=5.0,
            )

    def _load_target_definition(self, target_config_file: str) -> ValidatedTarget:
        """Load the selected target."""

        target_config_file = target_config_file.strip()
        if not target_config_file:
            raise ValueError("target_config is required")
        target_path = Path(target_config_file)
        self.get_logger().info(f"Loading Target Definition from: {target_path}")
        target = load_target_definition(target_path)
        self.get_logger().info(
            f"Loaded Target Definition: {target.target_id}@{target.revision} "
            f"({target.identity.semantic_sha256})"
        )
        return target

    def _create_transform_message(
        self, rvec: np.ndarray, tvec: np.ndarray
    ) -> TransformStamped:
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # M-01: publish with ROS TF semantics.
        #
        # solvePnP returns (R, t) with p_cam = R @ p_lidar + t -- that is T_camera<-lidar.
        # A transform labelled `frame_id=lidar, child_frame_id=camera` means the *opposite* in
        # TF: the camera's pose expressed in lidar coordinates. Publishing the raw solve under
        # those labels pointed every tf2 consumer the wrong way, and `pointcloud_image_overlay`
        # inverts this message back before `projectPoints` -- the two must move together.
        #
        # The dumped JSON keeps the raw rvec/tvec -- that is what `lctk_autoware_export`
        # consumes, and it is deliberately not touched here.
        rotation_matrix = rotation_matrix.T
        tvec = -rotation_matrix @ tvec.reshape(3, 1)

        quaternion = rotation_matrix_to_quaternion(rotation_matrix)
        message = TransformStamped()
        message.header = Header()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.parent_frame
        message.child_frame_id = self.child_frame
        translation = tvec.ravel()
        message.transform.translation = Vector3(
            x=float(translation[0]),
            y=float(translation[1]),
            z=float(translation[2]),
        )
        message.transform.rotation = Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        )
        return message


def main(args=None):
    rclpy.init(args=args)
    node = LidarToCameraSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._review_server is not None:
            node._review_server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
