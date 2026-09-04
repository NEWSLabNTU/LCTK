"""Which reliability each sensor topic is received with.

The failure this guards is silent by construction: a RELIABLE subscriber meeting
a BEST_EFFORT publisher receives nothing, with no error on either side. It
shipped once -- `twolidar-vlp32-falcon` ran with the graph-wide `mode:=offline`
against a recording whose VLP-32 offers BEST_EFFORT, and only the Falcon
detector ever warmed up.
"""

from __future__ import annotations

import json
import textwrap
from pathlib import Path

import pytest
from lctk_launch.transport import (
    BEST_EFFORT,
    RELIABLE,
    TransportError,
    bag_offered_reliability,
    parse_reliability,
    resolve_reliability,
)

# rmw reliability: 1 = RELIABLE, 2 = BEST_EFFORT.
_PROFILE = """\
- history: 1
  depth: 10
  reliability: {reliability}
  durability: 2
  deadline:
    sec: 9223372036
    nsec: 854775807
  lifespan:
    sec: 9223372036
    nsec: 854775807
  liveliness: 0
  liveliness_lease_duration:
    sec: 9223372036
    nsec: 854775807
  avoid_ros_namespace_conventions: false
"""


def write_bag(directory: Path, topics: dict[str, int | None]) -> Path:
    """A rosbag2 directory carrying only what this module reads.

    `None` stands for a recording that does not record the profile at all, which
    older rosbag2 versions produce.
    """
    directory.mkdir(parents=True, exist_ok=True)
    entries = []
    for name, reliability in topics.items():
        # A YAML double-quoted scalar, so the embedded newlines survive the
        # round trip -- a single-quoted one would keep the backslash-n literal.
        offered = json.dumps(
            "" if reliability is None else _PROFILE.format(reliability=reliability)
        )
        entries.append(
            f"  - topic_metadata:\n"
            f"      name: {name}\n"
            f"      type: sensor_msgs/msg/PointCloud2\n"
            f"      serialization_format: cdr\n"
            f"      offered_qos_profiles: {offered}\n"
            f"    message_count: 100\n"
        )
    (directory / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  topics_with_message_count:\n"
        + "".join(entries),
        encoding="utf-8",
    )
    return directory


def test_a_recording_states_a_different_answer_per_topic(tmp_path):
    """The TWO_LIDAR_1 shape: two lidars, one bag, opposite reliability.

    This is why the answer cannot be one graph-wide flag, and why it cannot be
    derived from `data.kind` either -- both topics are in the same recording.
    """
    bag = write_bag(
        tmp_path / "two_lidar",
        {"/lidar/falcon/iv_points": 1, "/lidar/vlp32/velodyne_points": 2},
    )
    offered = bag_offered_reliability(bag)
    assert offered == {
        "/lidar/falcon/iv_points": RELIABLE,
        "/lidar/vlp32/velodyne_points": BEST_EFFORT,
    }


def test_a_topic_with_no_recorded_profile_is_omitted(tmp_path):
    """An older recording simply does not carry the answer.

    Omitting it rather than guessing lets the caller fall back to BEST_EFFORT,
    which can receive from a publisher of either kind.
    """
    bag = write_bag(tmp_path / "old", {"/points": None})
    assert bag_offered_reliability(bag) == {}
    assert resolve_reliability("/points", None, {}) == BEST_EFFORT


def test_a_system_default_profile_is_treated_as_unknown(tmp_path):
    """Reliability 0 is SYSTEM_DEFAULT, which says nothing about the publisher."""
    bag = write_bag(tmp_path / "sysdefault", {"/points": 0})
    assert bag_offered_reliability(bag) == {}


def test_offered_qos_profiles_may_be_a_real_list(tmp_path):
    """rosbag2 has written this field both as an embedded string and as a list."""
    directory = tmp_path / "listform"
    directory.mkdir()
    (directory / "metadata.yaml").write_text(
        textwrap.dedent("""\
            rosbag2_bagfile_information:
              topics_with_message_count:
              - topic_metadata:
                  name: /points
                  offered_qos_profiles:
                  - history: 1
                    depth: 10
                    reliability: 2
                message_count: 5
            """),
        encoding="utf-8",
    )
    assert bag_offered_reliability(directory) == {"/points": BEST_EFFORT}


def test_one_best_effort_publisher_decides_a_multi_publisher_topic(tmp_path):
    """A RELIABLE subscriber would miss that publisher entirely."""
    directory = tmp_path / "multi"
    directory.mkdir()
    (directory / "metadata.yaml").write_text(
        textwrap.dedent("""\
            rosbag2_bagfile_information:
              topics_with_message_count:
              - topic_metadata:
                  name: /points
                  offered_qos_profiles:
                  - {history: 1, depth: 10, reliability: 1}
                  - {history: 1, depth: 10, reliability: 2}
                message_count: 5
            """),
        encoding="utf-8",
    )
    assert bag_offered_reliability(directory) == {"/points": BEST_EFFORT}


def test_no_recording_means_best_effort():
    """A live rig, the pcap/avi playback, or a plain calibration config.

    BEST_EFFORT is the only answer that is compatible with every publisher, so
    it is what an unstated topic gets.
    """
    assert resolve_reliability("/points", None, None) == BEST_EFFORT


def test_the_recording_decides_when_the_manifest_says_nothing():
    offered = {"/points": RELIABLE}
    assert resolve_reliability("/points", None, offered) == RELIABLE


def test_a_stated_value_wins_over_the_recording():
    """BEST_EFFORT against a RELIABLE publisher is compatible, just lossier.

    An operator who wants that -- to keep a slow consumer from blocking the
    graph, say -- is allowed to ask for it.
    """
    offered = {"/points": RELIABLE}
    assert resolve_reliability("/points", BEST_EFFORT, offered) == BEST_EFFORT


def test_stating_reliable_against_a_best_effort_recording_is_refused():
    """The one combination that receives nothing at all.

    Refusing at parse time is the whole point: the alternative is a graph that
    launches cleanly, logs no error, and never produces a detection (M-30).
    """
    offered = {"/lidar/vlp32/velodyne_points": BEST_EFFORT}
    with pytest.raises(TransportError, match="receives nothing"):
        resolve_reliability("/lidar/vlp32/velodyne_points", RELIABLE, offered)


def test_stating_reliable_is_allowed_when_no_recording_contradicts_it():
    """A live rig cannot be checked, so the operator's claim stands."""
    assert resolve_reliability("/points", RELIABLE, None) == RELIABLE


@pytest.mark.parametrize("value", ["RELIABLE", "sensor_data", "", 1, None, True])
def test_an_unknown_qos_value_is_refused(value):
    with pytest.raises(ValueError, match="expected one of"):
        parse_reliability(value, "camera 'zed'")


@pytest.mark.parametrize("value", [RELIABLE, BEST_EFFORT])
def test_both_reliabilities_parse(value):
    assert parse_reliability(value, "camera 'zed'") == value
