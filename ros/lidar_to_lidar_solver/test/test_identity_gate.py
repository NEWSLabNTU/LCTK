"""Target Identity gate contract for the LiDAR-to-LiDAR solver."""

from __future__ import annotations

import threading
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import Transform
from lctk_interfaces.msg import CalibrationTargetIdentity
from lidar_to_lidar_solver.identity_gate import (
    IdentityComparison,
    IdentityStatus,
    TargetIdentityGate,
    TargetIdentitySubscriptions,
    compare_target_identities,
    identity_qos_profile,
)
from lidar_to_lidar_solver.main import SyncStatistics
from rclpy.clock import ClockType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rclpy.time import Time
from vision_msgs.msg import Detection3D, Detection3DArray

VALID_VALUES = {
    "schema_version": 1,
    "target_id": "solid_600_aruco_1",
    "revision": 1,
    "semantic_sha256": "a" * 64,
    "board_frame_convention": "corner_aligned_plate_center_v1",
}


def identity(**overrides) -> CalibrationTargetIdentity:
    values = {**VALID_VALUES, **overrides}
    return CalibrationTargetIdentity(**values)


def test_comparator_reports_missing_input_before_comparing_fields():
    result = compare_target_identities(None, identity())

    assert result.status is IdentityStatus.MISSING
    assert not result.accepted
    assert "lidar1" in result.reason


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("schema_version", 0),
        ("target_id", ""),
        ("revision", 0),
        ("semantic_sha256", "A" * 64),
        ("semantic_sha256", "a" * 63),
        ("board_frame_convention", ""),
    ],
)
def test_comparator_rejects_malformed_identity(field, value):
    result = compare_target_identities(identity(**{field: value}), identity())

    assert result.status is IdentityStatus.MALFORMED
    assert not result.accepted
    assert field in result.reason


def test_comparator_requires_exact_equality_of_all_five_fields():
    result = compare_target_identities(identity(), identity(revision=2))

    assert result.status is IdentityStatus.MISMATCH
    assert not result.accepted


def test_comparator_accepts_exact_identity_match():
    result = compare_target_identities(identity(), identity())

    assert result.status is IdentityStatus.MATCH
    assert result.accepted


def test_gate_keeps_malformed_input_out_of_the_accepted_state():
    gate = TargetIdentityGate()

    malformed = gate.update(0, identity(revision=0))
    waiting = gate.compare()

    assert malformed.status is IdentityStatus.MALFORMED
    assert waiting.status is IdentityStatus.MALFORMED
    assert not waiting.accepted
    assert gate.identities == (None, None)


def test_gate_rejects_identity_changes_until_solver_restart():
    gate = TargetIdentityGate()
    gate.update(0, identity())
    gate.update(1, identity())

    changed = gate.update(0, identity(revision=2))

    assert changed.status is IdentityStatus.MISMATCH
    assert not gate.compare().accepted
    assert "restart" in gate.compare().reason


class FakeNode:
    """Capture ROS-facing subscriptions without requiring a running graph."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((msg_type, topic, callback, qos))
        return object()


def test_identity_subscriptions_are_relative_latched_and_callback_driven():
    node = FakeNode()
    gate = TargetIdentityGate()

    TargetIdentitySubscriptions(node, gate)

    assert [subscription[1] for subscription in node.subscriptions] == [
        "lidar1_target_identity",
        "lidar2_target_identity",
    ]
    assert all(
        subscription[0] is CalibrationTargetIdentity
        for subscription in node.subscriptions
    )
    assert all(
        not subscription[1].startswith("/") for subscription in node.subscriptions
    )
    for _, _, callback, qos in node.subscriptions:
        assert qos.reliability is ReliabilityPolicy.RELIABLE
        assert qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
        assert qos.history is HistoryPolicy.KEEP_LAST
        assert qos.depth == 1

    node.subscriptions[0][2](identity())
    assert gate.compare().status is IdentityStatus.MISSING
    node.subscriptions[1][2](identity())
    assert gate.compare().status is IdentityStatus.MATCH


def test_identity_subscription_reports_protocol_failures_to_the_owner():
    node = FakeNode()
    gate = TargetIdentityGate()
    updates = []
    TargetIdentitySubscriptions(
        node,
        gate,
        on_update=lambda index, result: updates.append((index, result)),
    )

    node.subscriptions[0][2](identity())
    node.subscriptions[1][2](identity())
    node.subscriptions[0][2](identity(revision=2))

    assert updates[-1][0] == 0
    assert updates[-1][1].status is IdentityStatus.MISMATCH


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)

    def info(self, _message):
        pass

    def debug(self, _message):
        pass

    def error(self, _message):
        pass


class _PairSource:
    def __init__(self):
        self.discarded = 0

    def discard_cached_pair(self):
        self.discarded += 1


class _Broadcaster:
    def __init__(self):
        self.sent = []

    def sendTransform(self, transform):
        self.sent.append(transform)


class _Publisher:
    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(message)


class _Clock:
    def now(self):
        return Time(nanoseconds=20_000_000_000, clock_type=ClockType.ROS_TIME)


def _detection(stamp, position):
    message = Detection3DArray()
    message.header.stamp.sec = stamp
    detection = Detection3D()
    detection.bbox.center.position.x = position[0]
    detection.bbox.center.position.y = position[1]
    detection.bbox.center.position.z = position[2]
    detection.bbox.center.orientation.w = 1.0
    message.detections.append(detection)
    return message


def test_stable_identity_preserves_latest_pair_transform_policy_and_direction():
    """Stable identities leave H-13's latest-pair composition unchanged."""
    from lidar_to_lidar_solver.main import LidarToLidarSolver

    solver = object.__new__(LidarToLidarSolver)
    solver.target_identity_gate = TargetIdentityGate()
    solver.target_identity_gate.update(0, identity())
    solver.target_identity_gate.update(1, identity())
    solver.stats = SyncStatistics()
    solver.current_transform = None
    solver.pair_source = _PairSource()
    solver.state_lock = threading.RLock()
    solver._identity_generation = 0
    solver.same_face_mode = True
    solver.max_message_age_ms = 0.0
    solver.lidar1_frame = "lidar1"
    solver.lidar2_frame = "lidar2"
    solver.transform_pub = _Publisher()
    solver.publish_tf = False
    solver._clock = _Clock()
    solver.get_clock = lambda: solver._clock
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger

    first = (
        _detection(10, (1.0, 2.0, 3.0)),
        _detection(10, (0.0, 0.0, 3.0)),
    )
    second = (
        _detection(11, (4.0, 5.0, 3.0)),
        _detection(11, (0.0, 0.0, 3.0)),
    )
    LidarToLidarSolver._handle_sync_group(solver, first)
    LidarToLidarSolver._handle_sync_group(solver, second)

    assert solver.stats.synced_pairs == 2
    assert len(solver.transform_pub.published) == 2
    assert solver.current_transform.header.frame_id == "lidar1"
    assert solver.current_transform.child_frame_id == "lidar2"
    assert solver.current_transform.transform.translation.x == pytest.approx(4.0)
    assert solver.current_transform.transform.translation.y == pytest.approx(5.0)


def test_identity_change_clears_cached_pair_and_old_tf_output():
    from lidar_to_lidar_solver.main import LidarToLidarSolver

    solver = object.__new__(LidarToLidarSolver)
    solver.current_transform = object()
    solver.pair_source = _PairSource()
    solver.state_lock = threading.RLock()
    solver._identity_generation = 0
    solver.tf_broadcaster = _Broadcaster()
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger

    LidarToLidarSolver._handle_target_identity_update(
        solver,
        0,
        IdentityComparison(IdentityStatus.MISMATCH, "identity changed"),
    )

    assert solver.current_transform is None
    assert solver.pair_source.discarded == 1
    solver.publish_timer_callback()
    assert solver.tf_broadcaster.sent == []


def test_solver_rejects_pair_before_mutating_solver_state():
    """The callback's identity gate is before transform/stats state changes."""
    from lidar_to_lidar_solver.main import LidarToLidarSolver

    solver = object.__new__(LidarToLidarSolver)
    solver.target_identity_gate = TargetIdentityGate()
    solver.target_identity_gate.update(0, identity())
    solver.target_identity_gate.update(1, identity(target_id="hollow_1000_aruco_4"))
    solver.stats = SyncStatistics()
    solver.current_transform = None
    solver.pair_source = _PairSource()
    solver.state_lock = threading.RLock()
    solver._identity_generation = 0
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger

    LidarToLidarSolver._handle_sync_group(solver, (object(), object()))

    assert solver.current_transform is None
    assert solver.stats.synced_pairs == 0
    assert solver.stats.identity_rejections == 1
    assert solver.pair_source.discarded == 1
    assert solver._logger.warnings


def test_identity_update_during_compute_cannot_resurrect_transform_or_tf():
    """The generation recheck rejects a pair invalidated during computation."""
    from lidar_to_lidar_solver.main import LidarToLidarSolver

    solver = object.__new__(LidarToLidarSolver)
    solver.target_identity_gate = TargetIdentityGate()
    solver.target_identity_gate.update(0, identity())
    solver.target_identity_gate.update(1, identity())
    solver.stats = SyncStatistics()
    solver.current_transform = None
    solver.pair_source = _PairSource()
    solver.state_lock = threading.RLock()
    solver._identity_generation = 0
    solver.same_face_mode = True
    solver.max_message_age_ms = 0.0
    solver.lidar1_frame = "lidar1"
    solver.lidar2_frame = "lidar2"
    solver.transform_pub = _Publisher()
    solver.publish_tf = True
    solver.tf_broadcaster = _Broadcaster()
    solver._clock = _Clock()
    solver.get_clock = lambda: solver._clock
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger

    compute_started = threading.Event()
    release_compute = threading.Event()

    def blocked_compute(_pose1, _pose2):
        compute_started.set()
        assert release_compute.wait(timeout=2.0)
        transform = Transform()
        transform.translation.x = 4.0
        transform.rotation.w = 1.0
        return transform

    solver.compute_transform = blocked_compute
    pair = (
        _detection(10, (1.0, 2.0, 3.0)),
        _detection(10, (0.0, 0.0, 3.0)),
    )
    pair_thread = threading.Thread(
        target=LidarToLidarSolver._handle_sync_group,
        args=(solver, pair),
    )
    pair_thread.start()
    assert compute_started.wait(timeout=2.0)

    node = FakeNode()
    TargetIdentitySubscriptions(
        node,
        solver.target_identity_gate,
        on_update=solver._handle_target_identity_update,
        update_lock=solver.state_lock,
    )
    identity_thread = threading.Thread(
        target=node.subscriptions[0][2], args=(identity(revision=2),)
    )
    identity_thread.start()
    identity_thread.join(timeout=2.0)
    assert not identity_thread.is_alive()

    release_compute.set()
    pair_thread.join(timeout=2.0)
    assert not pair_thread.is_alive()

    assert solver.current_transform is None
    assert solver.stats.synced_pairs == 0
    assert solver.transform_pub.published == []
    assert solver.tf_broadcaster.sent == []
    assert solver.pair_source.discarded >= 1

    solver.publish_timer_callback()
    assert solver.tf_broadcaster.sent == []


@pytest.fixture(scope="module")
def ros_context():
    already_initialized = rclpy.ok()
    if not already_initialized:
        rclpy.init()
    yield
    if not already_initialized and rclpy.ok():
        rclpy.shutdown()


def _spin_until(node, predicate, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return predicate()


def test_latched_identity_survives_late_solver_join_and_restart(ros_context):
    """A restarted/late solver receives both detector identities from DDS history."""
    namespace = f"/identity_gate_{uuid.uuid4().hex}"
    publisher_node = rclpy.create_node("identity_publishers", namespace=namespace)
    qos = identity_qos_profile()
    publishers = [
        publisher_node.create_publisher(CalibrationTargetIdentity, topic, qos)
        for topic in TargetIdentitySubscriptions.TOPICS
    ]
    first_node = None
    second_node = None
    try:
        for publisher in publishers:
            publisher.publish(identity())

        first_node = rclpy.create_node("late_solver", namespace=namespace)
        first_gate = TargetIdentityGate()
        first_node._identity_subscriptions = TargetIdentitySubscriptions(
            first_node, first_gate
        )
        assert _spin_until(first_node, lambda: first_gate.compare().accepted)

        first_node.destroy_node()
        first_node = None

        second_node = rclpy.create_node("restarted_solver", namespace=namespace)
        second_gate = TargetIdentityGate()
        second_node._identity_subscriptions = TargetIdentitySubscriptions(
            second_node, second_gate
        )
        assert _spin_until(second_node, lambda: second_gate.compare().accepted)
    finally:
        if first_node is not None:
            first_node.destroy_node()
        if second_node is not None:
            second_node.destroy_node()
        publisher_node.destroy_node()
