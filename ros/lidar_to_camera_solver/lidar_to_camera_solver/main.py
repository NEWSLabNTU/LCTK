"""ROS adapter for continuous and manual LiDAR-to-camera calibration."""

import json
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
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
from lctk_sync import DetectionPairSource, PairSourceConfig
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
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
)
from lidar_to_camera_solver.detection_format import (
    decode_detection_archive,
    encode_detection_archive,
    select_loaded_adjustment,
)

SOLVER_MODES = ("continuous", "manual")


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


def rotation_vector_to_euler(rvec: np.ndarray, *, degrees: bool = False) -> np.ndarray:
    """Render one solved rotation vector for ROS response fields."""
    owned_rvec = np.array(rvec, dtype=np.float64, copy=True).reshape(3)
    return Rotation.from_rotvec(owned_rvec).as_euler("xyz", degrees=degrees)


class LidarToCameraSolver(Node):
    """ROS services and publication around :class:`DetectionBuffer`."""

    def __init__(self):
        super().__init__("lidar_to_camera_solver")
        self._declare_parameters()

        self.solver_mode = parse_solver_mode(self._string_parameter("solver_mode"))
        self.parent_frame = self._string_parameter("parent_frame")
        self.child_frame = self._string_parameter("child_frame")
        camera_topic = self._string_parameter("camera_topic")
        target_config_file = self._string_parameter("target_config")
        aruco_config_file = self._string_parameter("aruco_config_file")
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

        self.target = self._load_target_definition(
            target_config_file, aruco_config_file
        )
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

        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if use_best_effort_qos
            else ReliabilityPolicy.RELIABLE
        )
        qos_profile = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.get_logger().info(
            f"Using {'BEST_EFFORT' if use_best_effort_qos else 'RELIABLE'} QoS"
        )

        self.transform_publisher = self.create_publisher(
            TransformStamped, "extrinsic_transform", qos_profile
        )
        self.axis_marker_publisher = self.create_publisher(
            MarkerArray, "axis_markers", qos_profile
        )
        self.publishing_timer = self.create_timer(
            1.0 / publishing_rate, self._publishing_timer_callback
        )
        self.pair_source = DetectionPairSource(
            self,
            topics=["aruco_detections", "calibration_board_detections"],
            msg_types=[Detection2DArray, Detection3DArray],
            config=pair_source_config,
            qos=qos_profile,
            on_pair=(
                self._continuous_pair_callback
                if self.solver_mode == "continuous"
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
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile
        )
        if self.solver_mode == "manual":
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
            ("aruco_config_file", ""),
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
        )
        for name, default in parameters:
            self.declare_parameter(name, default)

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
        snapshot = self._snapshot()
        with self.state_lock:
            adjusted_rvec = (
                None if self.current_rvec is None else self.current_rvec.copy()
            )
            adjusted_tvec = (
                None if self.current_tvec is None else self.current_tvec.copy()
            )
        if snapshot is None or (snapshot.frame_count == 0 and adjusted_rvec is None):
            response.success = False
            response.message = (
                "Buffer is empty and no transform available, nothing to save"
            )
            response.num_detections = 0
            return response
        try:
            archive = encode_detection_archive(
                snapshot,
                local_identity=self.target.identity,
                adjusted_rvec=adjusted_rvec,
                adjusted_tvec=adjusted_tvec,
            )
            with open(request.file_path, "w") as file:
                json.dump(archive, file, indent=2)
        except (OSError, TypeError, ValueError) as error:
            response.success = False
            response.message = f"Failed to save detections: {error!s}"
            response.num_detections = 0
            return response
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

    @staticmethod
    def _legacy_hollow_target_path() -> Path:
        """Locate the explicit hollow manifest for the temporary old parameter."""

        try:
            from ament_index_python.packages import get_package_share_directory

            return (
                Path(get_package_share_directory("lctk_launch"))
                / "config"
                / "targets"
                / "hollow_1000_aruco_4_v1.json5"
            )
        except (ImportError, LookupError):
            # Source-tree tests can run before ament has indexed the package.
            return (
                Path(__file__).resolve().parents[2]
                / "lctk_launch"
                / "config"
                / "targets"
                / "hollow_1000_aruco_4_v1.json5"
            )

    def _load_target_definition(
        self, target_config_file: str, legacy_aruco_config_file: str
    ) -> ValidatedTarget:
        """Load the selected target, with the temporary explicit-hollow bridge."""

        target_config_file = target_config_file.strip()
        legacy_aruco_config_file = legacy_aruco_config_file.strip()
        if target_config_file and legacy_aruco_config_file:
            raise ValueError(
                "target_config and legacy aruco_config_file cannot both be set; "
                "select one"
            )
        if not target_config_file:
            if not legacy_aruco_config_file:
                raise ValueError(
                    "target_config is required (or temporary legacy "
                    "aruco_config_file during migration)"
                )
            target_path = self._legacy_hollow_target_path()
            self.get_logger().warn(
                "legacy aruco_config_file selects the explicit hollow_1000_aruco_4 "
                "Target Definition; migrate to target_config before W5-E1"
            )
        else:
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
