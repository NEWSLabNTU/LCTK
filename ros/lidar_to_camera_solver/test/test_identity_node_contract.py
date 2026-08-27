"""ROS-adapter contracts for Target Identity admission in the camera solver."""

from __future__ import annotations

import threading
import time
import uuid
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from lctk_interfaces.msg import CalibrationTargetIdentity
from lctk_sync import DetectionPairSource, PairSourceConfig
from lctk_target import load_target
from lidar_to_camera_solver.board_geometry import (
    CAMERA_TARGET_IDENTITY_TOPIC,
    LIDAR_TARGET_IDENTITY_TOPIC,
    TargetIdentityGate,
)
from lidar_to_camera_solver.detection_buffer import BufferSnapshot, BufferUpdate, Solved
from lidar_to_camera_solver.main import (
    LidarToCameraSolver,
    create_target_identity_subscriptions,
    target_identity_qos_profile,
)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray

SOLID = (
    Path(__file__).resolve().parents[2]
    / "lctk_launch"
    / "config"
    / "targets"
    / "solid_600_aruco_1_v1.json5"
)


def identity(**overrides) -> CalibrationTargetIdentity:
    target = load_target(SOLID)
    values = {
        "schema_version": target.identity.schema_version,
        "target_id": target.identity.target_id,
        "revision": target.identity.revision,
        "semantic_sha256": target.identity.semantic_sha256,
        "board_frame_convention": target.identity.board_frame_convention,
    }
    values.update(overrides)
    return CalibrationTargetIdentity(**values)


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message, **_kwargs):
        self.messages.append(message)

    def warn(self, message, **_kwargs):
        self.messages.append(message)

    def error(self, message, **_kwargs):
        self.messages.append(message)


class _Buffer:
    def __init__(self):
        self.cleared = 0
        self.restored = 0

    def clear(self):
        self.cleared += 1

    def restore(self, *_args, **_kwargs):
        self.restored += 1
        raise AssertionError("identity gate must run before DetectionBuffer.restore")


class _PairSource:
    def __init__(self):
        self.discarded = 0
        self.cached_pair = None

    def discard_cached_pair(self):
        self.discarded += 1
        self.cached_pair = None

    def is_cached_pair(self, messages):
        return self.cached_pair is messages


class _SubscriptionNode:
    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((msg_type, topic, callback, qos))
        return object()


def solver_harness(*, ready: bool) -> LidarToCameraSolver:
    solver = object.__new__(LidarToCameraSolver)
    target = load_target(SOLID)
    solver.state_lock = threading.RLock()
    solver.identity_gate = TargetIdentityGate(target.identity)
    solver._identity_generation = 0
    if ready:
        solver.identity_gate.update("lidar", identity())
        solver.identity_gate.update("camera", identity())
    solver.detection_buffer = _Buffer()
    solver.pair_source = _PairSource()
    solver.current_rvec = np.ones(3)
    solver.current_tvec = np.ones(3)
    solver.last_transform = object()
    solver.publishing_enabled = True
    solver._continuous_solve_count = 0
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger
    return solver


def _solved_update() -> BufferUpdate:
    estimate = SimpleNamespace(
        rvec=np.zeros((3, 1)),
        tvec=np.zeros((3, 1)),
        quality=SimpleNamespace(warnings=list),
    )
    snapshot = BufferSnapshot(
        revision=1,
        pairs=(),
        placements=(),
        correspondence_count=4,
        outcome=Solved(estimate),
    )
    return BufferUpdate(accepted=True, changed=True, snapshot=snapshot)


def test_identity_subscriptions_use_relative_latched_contract():
    node = _SubscriptionNode()
    updates = []
    create_target_identity_subscriptions(
        node, lambda source, message: updates.append((source, message))
    )

    assert [item[1] for item in node.subscriptions] == [
        LIDAR_TARGET_IDENTITY_TOPIC,
        CAMERA_TARGET_IDENTITY_TOPIC,
    ]
    for msg_type, topic, _callback, qos in node.subscriptions:
        assert msg_type is CalibrationTargetIdentity
        assert not topic.startswith("/")
        assert qos.reliability is ReliabilityPolicy.RELIABLE
        assert qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
        assert qos.history is HistoryPolicy.KEEP_LAST
        assert qos.depth == 1

    node.subscriptions[0][2](identity())
    node.subscriptions[1][2](identity())
    assert [source for source, _message in updates] == ["lidar", "camera"]

    qos = target_identity_qos_profile()
    assert qos.depth == 1


def test_continuous_callback_gates_before_detection_buffer_mutation():
    solver = solver_harness(ready=False)

    LidarToCameraSolver._continuous_pair_callback(solver, (object(), object()))

    assert solver.detection_buffer.restored == 0


def test_target_change_clears_capture_transform_and_publication_state():
    solver = solver_harness(ready=True)

    LidarToCameraSolver._target_identity_callback(
        solver, "camera", identity(revision=2)
    )

    assert solver.detection_buffer.cleared == 1
    assert solver.pair_source.discarded == 1
    assert solver.current_rvec is None
    assert solver.current_tvec is None
    assert solver.last_transform is None
    assert not solver.publishing_enabled


def test_source_tree_legacy_hollow_target_path_uses_ros_workspace(monkeypatch):
    """The temporary legacy bridge resolves the manifest from a source checkout."""

    try:
        from ament_index_python import packages
    except ImportError:
        packages = None
    if packages is not None:

        def unavailable(_name):
            raise LookupError("package is not indexed")

        monkeypatch.setattr(packages, "get_package_share_directory", unavailable)

    expected = (
        Path(__file__).resolve().parents[2]
        / "lctk_launch"
        / "config"
        / "targets"
        / "hollow_1000_aruco_4_v1.json5"
    )
    assert LidarToCameraSolver._legacy_hollow_target_path() == expected


def test_apply_update_rejects_stale_generation_before_repopulating_output():
    solver = solver_harness(ready=True)
    solver._identity_generation = 1
    solver._create_transform_message = lambda *_args: object()
    update = _solved_update()

    applied = LidarToCameraSolver._apply_update(solver, update, expected_generation=0)

    assert not applied
    assert np.array_equal(solver.current_rvec, np.ones(3))
    assert np.array_equal(solver.current_tvec, np.ones(3))
    assert solver.last_transform is not None
    assert solver.publishing_enabled


def test_continuous_result_cannot_resurrect_after_identity_invalidation():
    """Identity invalidation wins between solve completion and state application."""

    solver = solver_harness(ready=True)
    update = _solved_update()
    messages = (object(), object())
    solver.pair_source.cached_pair = messages

    class ContinuousBuffer(_Buffer):
        def restore(self, *_args, **_kwargs):
            self.restored += 1
            return update

    solver.detection_buffer = ContinuousBuffer()
    solver._create_transform_message = lambda *_args: object()
    solver._publishing_timer_callback = lambda **_kwargs: pytest.fail(
        "stale continuous result was published"
    )
    apply_entered = threading.Event()
    original_apply = LidarToCameraSolver._apply_update

    def wait_for_invalidation(update, **kwargs):
        apply_entered.set()
        assert invalidated.wait(timeout=2.0)
        return original_apply(solver, update, **kwargs)

    invalidated = threading.Event()
    solver._apply_update = wait_for_invalidation
    worker = threading.Thread(
        target=LidarToCameraSolver._continuous_pair_callback,
        args=(solver, messages),
    )
    worker.start()
    assert apply_entered.wait(timeout=2.0)

    LidarToCameraSolver._target_identity_callback(
        solver, "camera", identity(revision=2)
    )
    invalidated.set()
    worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert solver.detection_buffer.cleared == 1
    assert solver.last_transform is None
    assert not solver.publishing_enabled


def test_delayed_continuous_pair_is_rejected_after_target_session_reset():
    """A cache write followed by reset cannot refill the new session."""

    solver = solver_harness(ready=True)
    callback_started = threading.Event()
    release_callback = threading.Event()

    class Group:
        def __init__(self, messages):
            self.messages = messages

        def get(self, topic):
            return self.messages[topic]

        def topics(self):
            return tuple(self.messages)

    def message(stamp):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=stamp, nanosec=0),
            ),
            detections=[object()],
        )

    def delayed_callback(messages):
        callback_started.set()
        assert release_callback.wait(timeout=2.0)
        LidarToCameraSolver._continuous_pair_callback(solver, messages)

    source = object.__new__(DetectionPairSource)
    source._node = SimpleNamespace(get_logger=solver.get_logger)
    source._topics = ["aruco", "board"]
    source._config = SimpleNamespace(require_non_empty=True)
    source._admit_pair = solver._admit_detection_pair
    source._on_pair = delayed_callback
    source._admission_lock = solver.state_lock
    source._latest = None
    source._latest_at = None
    source._last_group = None
    source._last_group_at = None
    source._last_skew_ms = None
    source._max_skew_ms = 0.0
    solver.pair_source = source

    group = Group({"aruco": message(10), "board": message(10)})
    worker = threading.Thread(target=source._handle_group, args=(group,))
    worker.start()
    assert callback_started.wait(timeout=2.0)
    cached_pair = source._latest
    assert cached_pair is not None
    assert source.is_cached_pair(cached_pair)

    # This is the production reset path: it increments the session generation,
    # clears the buffer/output state, and discards the source cache while holding
    # the same reentrant admission lock used by the callback precondition.
    LidarToCameraSolver._target_identity_callback(
        solver, "camera", identity(revision=2)
    )
    assert not source.is_cached_pair(cached_pair)
    release_callback.set()
    worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert solver.detection_buffer.restored == 0
    assert solver.last_transform is None
    assert not solver.publishing_enabled


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


def _publish_pair(aruco_publisher, board_publisher, stamp: int) -> None:
    aruco = Detection2DArray()
    aruco.header.stamp.sec = stamp
    aruco.detections = [Detection2D()]
    board = Detection3DArray()
    board.header.stamp.sec = stamp
    board.detections = [Detection3D()]
    aruco_publisher.publish(aruco)
    board_publisher.publish(board)


def test_preagreement_pair_cannot_be_captured_after_late_identity_match(ros_context):
    """Manual mode cannot resurrect a pair rejected before cache admission."""

    suffix = uuid.uuid4().hex
    topics = [f"/camera_gate_{suffix}/aruco", f"/camera_gate_{suffix}/board"]
    source_node = rclpy.create_node(f"camera_gate_source_{suffix}")
    publisher_node = rclpy.create_node(f"camera_gate_publisher_{suffix}")
    solver = solver_harness(ready=False)
    admission_checks = []

    def admit_pair(messages):
        admission_checks.append(messages)
        return solver._admit_detection_pair(messages)

    source = DetectionPairSource(
        source_node,
        topics=topics,
        msg_types=[Detection2DArray, Detection3DArray],
        config=PairSourceConfig(
            window_ms=50.0,
            stats_interval_s=0.0,
            epoch_check_interval_s=0.0,
            max_pair_age_s=2.0,
        ),
        admit_pair=admit_pair,
    )
    aruco_publisher = publisher_node.create_publisher(Detection2DArray, topics[0], 10)
    board_publisher = publisher_node.create_publisher(Detection3DArray, topics[1], 10)
    try:
        assert _spin_until(
            source_node,
            lambda: (
                aruco_publisher.get_subscription_count() == 1
                and board_publisher.get_subscription_count() == 1
            ),
        )
        _publish_pair(aruco_publisher, board_publisher, 10)
        assert _spin_until(source_node, lambda: len(admission_checks) == 1)
        assert not source.take_fresh_pair().ok

        with solver.state_lock:
            solver.identity_gate.update("lidar", identity())
            solver.identity_gate.update("camera", identity())

        # Agreement changes admission for future pairs; it cannot make the old
        # pre-gate pair appear in the manual capture cache.
        assert not source.take_fresh_pair().ok

        _publish_pair(aruco_publisher, board_publisher, 20)
        assert _spin_until(source_node, lambda: source.take_fresh_pair().ok)
    finally:
        publisher_node.destroy_node()
        source_node.destroy_node()
