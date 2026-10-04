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
        self.cached_pair = None

    def discard_cached_pair(self):
        self.discarded += 1
        self.cached_pair = None

    def is_cached_pair(self, messages):
        return self.cached_pair is messages


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


def _detection(stamp, position, frame_id=""):
    message = Detection3DArray()
    message.header.stamp.sec = stamp
    message.header.frame_id = frame_id
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
    solver.lidar1_frame = None
    solver.lidar2_frame = None
    solver._frame_generation = 0
    solver.same_face_mode = True
    solver.max_message_age_ms = 0.0
    solver.transform_pub = _Publisher()
    solver.publish_tf = False
    solver._clock = _Clock()
    solver.get_clock = lambda: solver._clock
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger

    first = (
        _detection(10, (1.0, 2.0, 3.0), "publisher_lidar_1"),
        _detection(10, (0.0, 0.0, 3.0), "publisher_lidar_2"),
    )
    second = (
        _detection(11, (4.0, 5.0, 3.0), "publisher_lidar_1"),
        _detection(11, (0.0, 0.0, 3.0), "publisher_lidar_2"),
    )
    solver.pair_source.cached_pair = first
    LidarToLidarSolver._handle_sync_group(solver, first)
    solver.pair_source.cached_pair = second
    LidarToLidarSolver._handle_sync_group(solver, second)

    assert solver.stats.synced_pairs == 2
    assert len(solver.transform_pub.published) == 2
    assert solver.current_transform.header.frame_id == "publisher_lidar_1"
    assert solver.current_transform.child_frame_id == "publisher_lidar_2"
    assert solver.current_transform.transform.translation.x == pytest.approx(4.0)
    assert solver.current_transform.transform.translation.y == pytest.approx(5.0)


@pytest.mark.parametrize(
    "frames",
    [("", "lidar2"), ("lidar", "lidar")],
)
def test_solver_rejects_empty_or_repeated_header_frames(frames):
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
    solver.lidar1_frame = None
    solver.lidar2_frame = None
    solver._frame_generation = 0
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger

    pair = (
        _detection(10, (1.0, 2.0, 3.0), frames[0]),
        _detection(10, (0.0, 0.0, 3.0), frames[1]),
    )
    solver.pair_source.cached_pair = pair
    LidarToLidarSolver._handle_sync_group(solver, pair)

    assert solver.current_transform is None
    assert solver.lidar1_frame is None
    assert solver.lidar2_frame is None
    assert solver.stats.synced_pairs == 0
    assert solver.stats.frame_rejections == 1
    assert solver.pair_source.discarded == 1


def test_frame_epoch_change_clears_old_output_and_suppresses_stale_work():
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
    solver.lidar1_frame = None
    solver.lidar2_frame = None
    solver._frame_generation = 0
    solver.same_face_mode = True
    solver.max_message_age_ms = 0.0
    solver.transform_pub = _Publisher()
    solver.publish_tf = False
    solver._clock = _Clock()
    solver.get_clock = lambda: solver._clock
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger

    replacement_pair = (
        _detection(11, (2.0, 0.0, 3.0), "new_lidar_1"),
        _detection(11, (0.0, 0.0, 3.0), "new_lidar_2"),
    )
    nested = False

    def compute_with_frame_change(pose1, _pose2):
        nonlocal nested
        if not nested:
            nested = True
            assert (
                LidarToLidarSolver._admit_sync_group(solver, replacement_pair) is None
            )
            solver.pair_source.cached_pair = replacement_pair
            LidarToLidarSolver._handle_sync_group(solver, replacement_pair)
        transform = Transform()
        transform.translation.x = pose1.position.x
        transform.rotation.w = 1.0
        return transform

    solver.compute_transform = compute_with_frame_change
    first_pair = (
        _detection(10, (1.0, 0.0, 3.0), "old_lidar_1"),
        _detection(10, (0.0, 0.0, 3.0), "old_lidar_2"),
    )
    assert LidarToLidarSolver._admit_sync_group(solver, first_pair) is None
    solver.pair_source.cached_pair = first_pair
    LidarToLidarSolver._handle_sync_group(solver, first_pair)

    assert solver.stats.synced_pairs == 1
    assert solver.stats.frame_rejections == 1
    assert solver.pair_source.discarded == 1
    assert len(solver.transform_pub.published) == 1
    assert solver.current_transform.header.frame_id == "new_lidar_1"
    assert solver.current_transform.child_frame_id == "new_lidar_2"
    assert solver.current_transform.transform.translation.x == pytest.approx(2.0)


def test_pair_source_admission_invalidates_estimate_without_numerical_callback(
    monkeypatch,
):
    """The source caches changed frames only after admission clears old output."""
    import lctk_sync.pair_source as pair_source_module
    from lctk_sync import DetectionPairSource, PairSourceConfig
    from lidar_to_lidar_solver.main import LidarToLidarSolver

    class FakeSyncGroup:
        def __init__(self, topics, messages):
            self._topics = topics
            self._messages = messages

        def get(self, topic):
            return self._messages[topic]

        def topics(self):
            return self._topics

    synchronizers = []

    class FakeSynchronizer:
        def __init__(self, *_args, **_kwargs):
            self.callback = None
            synchronizers.append(self)

        def add_subscription(self, *_args):
            pass

        def on_synchronized(self, callback):
            self.callback = callback
            return callback

        def emit(self, topics, messages):
            self.callback(FakeSyncGroup(topics, messages))

    class FakeNode:
        def __init__(self):
            self.logger = _Logger()

        def get_logger(self):
            return self.logger

    monkeypatch.setattr(pair_source_module, "ROS2Synchronizer", FakeSynchronizer)

    solver = object.__new__(LidarToLidarSolver)
    solver.target_identity_gate = TargetIdentityGate()
    solver.target_identity_gate.update(0, identity())
    solver.target_identity_gate.update(1, identity())
    solver.stats = SyncStatistics()
    solver.current_transform = object()
    solver.state_lock = threading.RLock()
    solver.lidar1_frame = "lidar_1_v1"
    solver.lidar2_frame = "lidar_2_v1"
    solver._frame_generation = 0
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger
    topics = ("lidar1", "lidar2")
    source = DetectionPairSource(
        FakeNode(),
        topics=topics,
        msg_types=(Detection3DArray, Detection3DArray),
        config=PairSourceConfig(stats_interval_s=0.0, epoch_check_interval_s=0.0),
        admit_pair=solver._admit_sync_group,
        admission_lock=solver.state_lock,
    )
    solver.pair_source = source

    changed_pair = (
        _detection(12, (1.0, 0.0, 3.0), "lidar_1_v2"),
        _detection(12, (0.0, 0.0, 3.0), "lidar_2_v2"),
    )
    synchronizers[0].emit(topics, dict(zip(topics, changed_pair)))

    outcome = source.take_fresh_pair()
    assert outcome.ok
    assert outcome.messages == changed_pair
    assert source.is_cached_pair(outcome.messages)
    assert solver.current_transform is None
    assert (solver.lidar1_frame, solver.lidar2_frame) == (
        "lidar_1_v2",
        "lidar_2_v2",
    )
    assert solver._frame_generation == 1
    assert solver.stats.synced_pairs == 0


def test_delayed_callback_cannot_rebind_frames_after_new_pair_is_admitted():
    """A queued old pair cannot undo the frame binding made at admission."""
    from lidar_to_lidar_solver.main import LidarToLidarSolver

    solver = object.__new__(LidarToLidarSolver)
    solver.target_identity_gate = TargetIdentityGate()
    solver.target_identity_gate.update(0, identity())
    solver.target_identity_gate.update(1, identity())
    solver.stats = SyncStatistics()
    solver.current_transform = object()
    solver.pair_source = _PairSource()
    solver.state_lock = threading.RLock()
    solver._identity_generation = 0
    solver.lidar1_frame = "lidar_1_v1"
    solver.lidar2_frame = "lidar_2_v1"
    solver._frame_generation = 0
    solver.same_face_mode = True
    solver.max_message_age_ms = 0.0
    solver.transform_pub = _Publisher()
    solver.publish_tf = False
    solver._clock = _Clock()
    solver.get_clock = lambda: solver._clock
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger

    def valid_transform(*_poses):
        transform = Transform()
        transform.rotation.w = 1.0
        return transform

    solver.compute_transform = valid_transform

    old_pair = (
        _detection(10, (1.0, 0.0, 3.0), "lidar_1_v1"),
        _detection(10, (0.0, 0.0, 3.0), "lidar_2_v1"),
    )
    new_pair = (
        _detection(11, (2.0, 0.0, 3.0), "lidar_1_v2"),
        _detection(11, (0.0, 0.0, 3.0), "lidar_2_v2"),
    )

    # Pair A was queued for the push callback when pair B passed admission.
    solver.pair_source.cached_pair = old_pair
    assert LidarToLidarSolver._admit_sync_group(solver, new_pair) is None
    solver.pair_source.cached_pair = new_pair

    LidarToLidarSolver._handle_sync_group(solver, old_pair)

    assert (solver.lidar1_frame, solver.lidar2_frame) == (
        "lidar_1_v2",
        "lidar_2_v2",
    )
    assert solver._frame_generation == 1
    assert solver.current_transform is None
    assert solver.stats.synced_pairs == 0
    assert solver.transform_pub.published == []
    assert solver.pair_source.is_cached_pair(new_pair)


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

    rejected_pair = (object(), object())
    solver.pair_source.cached_pair = rejected_pair
    LidarToLidarSolver._handle_sync_group(solver, rejected_pair)

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
    solver.lidar1_frame = None
    solver.lidar2_frame = None
    solver._frame_generation = 0
    solver.same_face_mode = True
    solver.max_message_age_ms = 0.0
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
        _detection(10, (1.0, 2.0, 3.0), "lidar1"),
        _detection(10, (0.0, 0.0, 3.0), "lidar2"),
    )
    solver.pair_source.cached_pair = pair
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
