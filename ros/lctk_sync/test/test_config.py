"""The sync window is a correctness setting, so the module refuses a wrong one.

Conflux only matches by time when a finite window is set: with an infinite window it
skips the pruning step in `State::try_match` and pairs whatever sits at the front of
each buffer -- by arrival order. Two streams at different rates then drift apart
without bound. Measured against this repository's conflux build: camera 10Hz + LiDAR
1Hz reached a 53s gap INSIDE one "synchronized" group, and the seyond rig (5.4Hz /
4.4Hz) passed 11s and was still climbing.

That is worse than a stall because it succeeds. A solver pairs detections on the
assumption both sensors saw the board at one instant; pair frames 11s apart and the
board has moved, while the reprojection error still looks fine.

`calibrate.launch.py` shipped `0.0` for both the LiDAR-camera and the LiDAR-LiDAR path,
so this is not a hypothetical caller mistake -- it is the mistake that was actually
made. Refusing it here means no launch file, node, or future caller can make it again.
"""

import pytest

from lctk_sync import PairSourceConfig


def test_a_finite_window_is_accepted():
    assert PairSourceConfig(window_ms=100.0).window_ms == 100.0


@pytest.mark.parametrize("window_ms", [0.0, -1.0])
def test_an_infinite_window_is_refused(window_ms):
    with pytest.raises(ValueError) as excinfo:
        PairSourceConfig(window_ms=window_ms)

    message = str(excinfo.value)
    assert "window" in message.lower()
    # The message must say WHY, or the next person just picks a different number.
    assert "arrival order" in message or "drift" in message


def test_the_default_window_matches_the_shipped_offline_preset():
    """100ms is a little over one frame interval at the rig's rates -- wide enough to
    absorb the offset between a camera frame and the LiDAR sweep overlapping it, narrow
    enough that a moving board cannot travel far within it."""
    assert PairSourceConfig().window_ms == 100.0


def test_the_defaults_are_the_offline_playback_settings():
    config = PairSourceConfig()
    assert config.queue_size == 100
    assert config.drop_policy == "reject_new"
    assert config.require_non_empty is True


def test_an_unknown_drop_policy_is_refused():
    with pytest.raises(ValueError, match="drop_policy"):
        PairSourceConfig(drop_policy="drop_newest")


def test_a_negative_staleness_limit_is_refused():
    with pytest.raises(ValueError, match="max_pair_age_s"):
        PairSourceConfig(max_pair_age_s=-1.0)


def test_staleness_checking_can_be_switched_off():
    """Zero disables the gate, for a workflow that deliberately pauses playback."""
    assert PairSourceConfig(max_pair_age_s=0.0).max_pair_age_s == 0.0
