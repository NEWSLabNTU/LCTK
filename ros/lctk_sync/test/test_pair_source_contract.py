"""`DetectionPairSource` contract tests over a real ROS graph."""

import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass
from types import SimpleNamespace

import pytest
import rclpy
from lctk_sync import DetectionPairSource, PairSourceConfig
from rclpy.node import Node
from rclpy.publisher import Publisher
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray


@dataclass
class PairSourceHarness:
    source_node: Node
    publisher_node: Node
    source: DetectionPairSource
    aruco_publisher: Publisher
    board_publisher: Publisher
    aruco_topic: str
    board_topic: str
    pairs: list[tuple]

    def publish(
        self, *, aruco_stamp: float, board_stamp: float, aruco_count=1, board_count=1
    ) -> None:
        aruco = Detection2DArray()
        aruco.header.stamp.sec = int(aruco_stamp)
        aruco.header.stamp.nanosec = int((aruco_stamp % 1) * 1_000_000_000)
        aruco.detections = [Detection2D() for _ in range(aruco_count)]

        board = Detection3DArray()
        board.header.stamp.sec = int(board_stamp)
        board.header.stamp.nanosec = int((board_stamp % 1) * 1_000_000_000)
        board.detections = [Detection3D() for _ in range(board_count)]

        self.aruco_publisher.publish(aruco)
        self.board_publisher.publish(board)

    def spin_until(self, predicate: Callable[[], bool], timeout_s=1.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.source_node, timeout_sec=0.01)
            if predicate():
                return True
        return predicate()

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.source_node, timeout_sec=0.01)

    def received_both(self) -> bool:
        line = self.source.status_line()
        return all(
            f"{topic}: received=1 " in line
            for topic in (self.aruco_topic, self.board_topic)
        )

    def destroy(self) -> None:
        self.publisher_node.destroy_node()
        self.source_node.destroy_node()


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness_factory(ros, request):
    def create_harness(*, admit_pair=None, **config_overrides) -> PairSourceHarness:
        suffix = uuid.uuid4().hex
        aruco_topic = f"/pair_source_contract_{suffix}/aruco"
        board_topic = f"/pair_source_contract_{suffix}/board"
        source_node = rclpy.create_node(f"pair_source_contract_source_{suffix}")
        publisher_node = rclpy.create_node(f"pair_source_contract_publisher_{suffix}")
        pairs = []
        config_values = {
            "window_ms": 50.0,
            "stats_interval_s": 0.0,
            "epoch_check_interval_s": 0.0,
            "max_pair_age_s": 2.0,
        }
        config_values.update(config_overrides)
        source = DetectionPairSource(
            source_node,
            topics=[aruco_topic, board_topic],
            msg_types=[Detection2DArray, Detection3DArray],
            config=PairSourceConfig(**config_values),
            on_pair=pairs.append,
            admit_pair=admit_pair,
        )
        result = PairSourceHarness(
            source_node=source_node,
            publisher_node=publisher_node,
            source=source,
            aruco_publisher=publisher_node.create_publisher(
                Detection2DArray, aruco_topic, 10
            ),
            board_publisher=publisher_node.create_publisher(
                Detection3DArray, board_topic, 10
            ),
            aruco_topic=aruco_topic,
            board_topic=board_topic,
            pairs=pairs,
        )
        request.addfinalizer(result.destroy)
        assert result.spin_until(
            lambda: (
                result.aruco_publisher.get_subscription_count() == 1
                and result.board_publisher.get_subscription_count() == 1
            )
        )
        return result

    return create_harness


def test_within_window_pair_reaches_callback_and_take_fresh_pair(harness_factory):
    harness = harness_factory()
    harness.publish(aruco_stamp=10.000, board_stamp=10.030)

    assert harness.spin_until(lambda: len(harness.pairs) == 1)
    outcome = harness.source.take_fresh_pair()
    assert outcome.ok
    assert outcome.messages == harness.pairs[0]
    assert harness.source.is_cached_pair(harness.pairs[0])
    assert not harness.source.is_cached_pair((harness.pairs[0][0], harness.pairs[0][1]))


def test_discarded_pair_is_not_handed_out_again(harness_factory):
    harness = harness_factory()
    harness.publish(aruco_stamp=10.000, board_stamp=10.030)
    assert harness.spin_until(lambda: len(harness.pairs) == 1)

    harness.source.discard_cached_pair()

    assert not harness.source.is_cached_pair(harness.pairs[0])
    assert not harness.source.take_fresh_pair().ok


def test_rejected_pair_is_not_cached_or_handed_out(harness_factory):
    rejected = []

    def reject_pair(messages):
        rejected.append(messages)
        return "Target Identity values do not match"

    harness = harness_factory(admit_pair=reject_pair)
    harness.publish(aruco_stamp=10.000, board_stamp=10.030)

    assert harness.spin_until(harness.received_both)
    harness.spin_for(0.05)
    assert rejected
    assert not harness.pairs
    assert not harness.source.take_fresh_pair().ok


def test_admission_lock_serializes_identity_clear_with_cache_write():
    """An invalidation cannot be overtaken by an already-admitted pair."""

    lock = threading.RLock()
    admission_started = threading.Event()
    release_admission = threading.Event()
    invalidation_finished = threading.Event()
    callbacks = []

    class Logger:
        def warn(self, *_args, **_kwargs):
            pass

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

    source = object.__new__(DetectionPairSource)
    source._node = SimpleNamespace(get_logger=lambda: Logger())
    source._topics = ["lidar1", "lidar2"]
    source._config = SimpleNamespace(require_non_empty=True)
    source._admit_pair = lambda _messages: (
        admission_started.set(),
        release_admission.wait(timeout=2.0),
        None,
    )[-1]
    source._on_pair = callbacks.append
    source._admission_lock = lock
    source._latest = None
    source._latest_at = None
    source._last_group = None
    source._last_group_at = None
    source._last_skew_ms = None
    source._max_skew_ms = 0.0

    pair = Group({"lidar1": message(10), "lidar2": message(10)})
    producer = threading.Thread(target=source._handle_group, args=(pair,))
    producer.start()
    assert admission_started.wait(timeout=2.0)

    def invalidate():
        with lock:
            source.discard_cached_pair()
            invalidation_finished.set()

    invalidator = threading.Thread(target=invalidate)
    invalidator.start()
    # The source owns the same lock while running admission, so invalidation
    # cannot clear state and then be overtaken by the cache write.
    assert not invalidation_finished.wait(timeout=0.05)

    release_admission.set()
    producer.join(timeout=2.0)
    invalidator.join(timeout=2.0)

    assert not producer.is_alive()
    assert not invalidator.is_alive()
    assert invalidation_finished.is_set()
    assert callbacks
    assert not source.take_fresh_pair().ok


def test_outside_window_pair_is_not_delivered(harness_factory):
    harness = harness_factory()
    harness.publish(aruco_stamp=10.000, board_stamp=10.051)

    assert harness.spin_until(harness.received_both)
    harness.spin_for(0.05)
    assert not harness.pairs
    assert not harness.source.take_fresh_pair().ok


def test_empty_group_is_not_delivered_and_names_empty_side(harness_factory):
    harness = harness_factory()
    harness.publish(aruco_stamp=10.000, board_stamp=10.030, aruco_count=0)

    assert harness.spin_until(
        lambda: (
            "camera side is empty" in harness.source.take_fresh_pair().reason.lower()
        )
    )
    assert not harness.pairs


def test_stale_pair_is_refused_after_real_delivery(harness_factory):
    harness = harness_factory(max_pair_age_s=0.02)
    harness.publish(aruco_stamp=10.000, board_stamp=10.030)
    assert harness.spin_until(lambda: len(harness.pairs) == 1)

    harness.spin_for(0.03)
    outcome = harness.source.take_fresh_pair()

    assert not outcome.ok
    assert "old" in outcome.reason.lower()


def test_replayed_timestamp_epoch_resets_and_resumes_pairing(harness_factory):
    harness = harness_factory(epoch_check_interval_s=0.05)
    harness.publish(aruco_stamp=100.000, board_stamp=100.030)
    assert harness.spin_until(lambda: len(harness.pairs) == 1)

    replay_stamp = 1.0
    deadline = time.monotonic() + 3.0
    while harness.source.epoch_resets == 0 and time.monotonic() < deadline:
        harness.publish(aruco_stamp=replay_stamp, board_stamp=replay_stamp + 0.030)
        replay_stamp += 0.1
        harness.spin_for(0.03)

    assert harness.source.epoch_resets == 1

    pairs_after_reset = len(harness.pairs)
    harness.publish(aruco_stamp=50.000, board_stamp=50.030)
    assert harness.spin_until(lambda: len(harness.pairs) > pairs_after_reset)
