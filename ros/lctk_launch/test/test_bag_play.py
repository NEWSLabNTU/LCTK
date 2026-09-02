"""`lctk_bag_play`: do not start the recording before anything is listening.

`ros2 bag play` is ready in about a second; the Rust detectors have to load, read
a Target Definition and build a background model. The bag's topics replay
BEST_EFFORT and VOLATILE, so every message published into that gap is lost to a
subscriber that has not appeared yet -- and with RViz, the overlay and the judge
enabled the gap can swallow a short recording whole. The failure is silent: the
graph is healthy, the bag played, and no detection ever happened.

A `sleep` cannot fix this honestly. The readiness condition is observable, so
these tests pin the decision that reads it.
"""

from lctk_launch import bag_play
from lctk_launch.bag_play import missing_subscribers


class FakeOs:
    """Captures an execv call instead of replacing the test process."""

    def __init__(self):
        self.command = None

    def execv(self, path, command):
        self.command = command


class FakeNode:
    """Stands in for an rclpy Node's discovery view."""

    def __init__(self, subscribed):
        self._subscribed = set(subscribed)

    def get_subscriptions_info_by_topic(self, topic):
        # rclpy returns a list of endpoint descriptions; only emptiness matters.
        return ["endpoint"] if topic in self._subscribed else []


def test_nothing_is_missing_when_every_topic_has_a_subscriber():
    node = FakeNode({"/points", "/image/compressed"})
    assert missing_subscribers(node, ["/points", "/image/compressed"]) == []


def test_a_topic_with_no_subscriber_is_reported():
    node = FakeNode({"/points"})
    assert missing_subscribers(node, ["/points", "/image/compressed"]) == [
        "/image/compressed"
    ]


def test_a_partly_assembled_graph_is_not_treated_as_ready():
    """The whole point: one detector up is not the graph being up."""
    node = FakeNode({"/points"})
    assert missing_subscribers(node, ["/points", "/other_points"])


def test_no_topics_to_wait_for_is_not_a_wait():
    assert missing_subscribers(FakeNode(set()), []) == []


def test_launch_appended_ros_args_are_dropped_not_parsed():
    """This runs as a launch `Node`, so launch_ros appends `--ros-args ...`.

    It has to be a Node: `play_launch` records and replays Node actions only, so
    an ExecuteProcess data source runs during the recording pass -- before any
    node exists -- and never appears in the replay at all. Paying for that with
    an argparse failure on the ROS arguments would trade one silent failure for
    a loud but equally fatal one.
    """
    fake = FakeOs()
    real_os = bag_play.os
    bag_play.os = fake
    try:
        bag_play.main(
            [
                "/some/bag",
                "--play-arg=--clock",
                "--ros-args",
                "-r",
                "__node:=bag_player",
                "--params-file",
                "/tmp/whatever.yaml",
            ]
        )
    finally:
        bag_play.os = real_os

    assert fake.command is not None, "the player was never launched"
    assert fake.command[1:] == ["bag", "play", "/some/bag", "--clock"]
