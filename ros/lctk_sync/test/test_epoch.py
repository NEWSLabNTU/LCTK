"""Replaying a second rosbag is a new epoch, and conflux cannot see it.

Conflux is strictly time-ordered by design: `State::push` rejects any message whose
stamp is at or before `commit_ts` (the newest group's time), and `commit_ts` only ever
moves forward. That is the right contract for a live sensor.

It is the wrong shape for this workflow. The operator replays many recorded bags to
collect board placements, and both detectors copy the stamp of the message they
consumed -- `aruco_locator_node` from the image, `lidar_board_detector` from the point
cloud -- so every new bag (or every `--loop` wrap) sends stamps BACKWARD by the length
of the previous one. Conflux then rejects every message as late, forever, and the
solver silently stops producing pairs. Measured on a looping 19.8s bag: groups froze
at 32 while `dropped` climbed 1:1 with `received` on both streams for the next four
minutes.

The fix belongs here rather than in conflux: the core rule stays, and this node
recognises when the source restarted and starts a fresh synchronizer.

The trigger is deliberately expressed in terms conflux already reports -- groups have
stopped, and EVERY stream is having its messages thrown away -- because that is the
exact signature of a rewound source, and it needs no access to the raw stamps.
"""

import pytest

from lctk_sync import should_reset_for_new_epoch


def test_a_rewound_source_is_detected():
    """Both streams dropping everything, no groups: the source went backward."""
    assert should_reset_for_new_epoch(
        previous_received={"aruco": 10, "board": 8},
        current_received={"aruco": 40, "board": 30},
        last_group_age_s=30.0,
    )


def test_a_healthy_pipeline_is_left_alone():
    """Groups are flowing; resetting would throw away good buffered data."""
    assert not should_reset_for_new_epoch(
        previous_received={"aruco": 10, "board": 8},
        current_received={"aruco": 40, "board": 30},
        last_group_age_s=0.3,
    )


def test_one_quiet_detector_is_not_an_epoch_change():
    """If only ONE stream is being dropped, the other has simply gone silent -- a
    detector problem. Resetting the synchronizer would hide it."""
    assert not should_reset_for_new_epoch(
        previous_received={"aruco": 10, "board": 8},
        current_received={"aruco": 40, "board": 8},
        last_group_age_s=30.0,
    )


def test_a_stall_with_no_drops_is_not_an_epoch_change():
    """Nothing is being rejected, so nothing is arriving. That is a dead stream, not a
    rewind, and it needs the operator's attention rather than a silent reset."""
    assert not should_reset_for_new_epoch(
        previous_received={"aruco": 10, "board": 8},
        current_received={"aruco": 10, "board": 8},
        last_group_age_s=30.0,
    )


def test_before_the_first_group_there_is_nothing_to_reset():
    """conflux cannot reject anything as late until it has committed a group, so this
    state is 'waiting for playback', not a stall."""
    assert not should_reset_for_new_epoch(
        previous_received={},
        current_received={"aruco": 0, "board": 0},
        last_group_age_s=None,
    )


@pytest.mark.parametrize("age", [4.9, 5.0, 5.1])
def test_the_quiet_threshold_is_honoured(age):
    triggered = should_reset_for_new_epoch(
        previous_received={"a": 0, "b": 0},
        current_received={"a": 5, "b": 5},
        last_group_age_s=age,
        quiet_after_s=5.0,
    )
    assert triggered == (age >= 5.0)


def test_the_default_threshold_is_short_enough_for_a_looping_bag():
    """A bag loops every bag-length -- 19.8s for the seyond recording -- and every
    second spent unpaired after a wrap is a second the operator cannot add. The first
    implementation used a 5s threshold on a 10s timer and lost 15s of every 20s cycle."""
    assert should_reset_for_new_epoch(
        previous_received={"aruco": 0, "board": 0},
        current_received={"aruco": 5, "board": 5},
        last_group_age_s=3.0,
    )


def test_a_deadlock_before_the_first_group_is_still_an_epoch_change():
    """The operator's real sequence: play a background bag, then a calibration bag.

    The two recordings have different time ranges, so once the board buffer holds the
    background bag's stamps, the calibration bag's stamps fall outside them. Conflux's
    readiness check (`inf_ts + window > sup_ts`) then returns BEFORE the pruning step
    that would drop the stale messages, and with reject_new the buffer never drains:
    groups=0, forever, measured.

    No group has ever been emitted here, so an implementation that measures "time since
    the last group" finds nothing to measure and never resets. That was this module's
    blind spot.
    """
    assert should_reset_for_new_epoch(
        previous_received={"aruco": 100, "board": 40},
        current_received={"aruco": 130, "board": 70},
        last_group_age_s=None,
        age_since_start_s=30.0,
    )


def test_messages_arriving_on_only_one_stream_is_not_an_epoch_change():
    """During the background bag only the LiDAR side publishes. There is nothing to
    pair, which is not a fault and must not trigger a reset."""
    assert not should_reset_for_new_epoch(
        previous_received={"aruco": 0, "board": 40},
        current_received={"aruco": 0, "board": 70},
        last_group_age_s=None,
        age_since_start_s=30.0,
    )


def test_a_young_source_is_given_time_to_start():
    """Right after startup nothing has paired yet simply because nothing has arrived."""
    assert not should_reset_for_new_epoch(
        previous_received={"aruco": 1, "board": 1},
        current_received={"aruco": 5, "board": 5},
        last_group_age_s=None,
        age_since_start_s=1.0,
    )
