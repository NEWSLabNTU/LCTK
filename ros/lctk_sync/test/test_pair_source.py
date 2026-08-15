"""`DetectionPairSource` against a real (unconnected) node.

These exercise the interface, not the internals: what a caller gets before anything has
arrived, and what discarding does. Nothing publishes here, so the source is permanently
in its "nothing has arrived" state -- which is exactly the state whose answer used to be
an unhelpful "No synchronized detection pair available".
"""

import pytest
import rclpy
from vision_msgs.msg import Detection2DArray, Detection3DArray

from lctk_sync import DetectionPairSource, PairSourceConfig


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def source(ros):
    node = rclpy.create_node("pair_source_test")
    yield DetectionPairSource(
        node,
        topics=["aruco_detections", "board_detections"],
        msg_types=[Detection2DArray, Detection3DArray],
        config=PairSourceConfig(stats_interval_s=0.0, epoch_check_interval_s=0.0),
    )
    node.destroy_node()


def test_before_anything_arrives_the_refusal_says_so(source):
    outcome = source.take_fresh_pair()

    assert not outcome.ok
    assert outcome.messages is None
    assert "no synchronized" in outcome.reason.lower()


def test_the_status_line_is_available_immediately(source):
    line = source.status_line()

    assert "groups=0" in line
    assert "aruco_detections" in line and "board_detections" in line


def test_discarding_when_there_is_nothing_cached_is_harmless(source):
    """`clear_buffer` calls this without knowing whether a pair is cached."""
    source.discard_cached_pair()

    assert not source.take_fresh_pair().ok


def test_a_discarded_pair_is_not_handed_out_again(source):
    """Clearing the solver's buffer means "start over". A pair captured BEFORE the
    clear must not be addable after it, even while it is still inside the freshness
    window."""
    import time

    source._latest = ("aruco", "board")  # what a group would have cached
    source._latest_at = time.monotonic()
    assert source.take_fresh_pair().ok

    source.discard_cached_pair()

    assert not source.take_fresh_pair().ok


def test_the_source_starts_with_no_epoch_resets(source):
    assert source.epoch_resets == 0
