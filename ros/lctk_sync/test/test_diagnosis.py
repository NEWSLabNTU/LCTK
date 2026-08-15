"""Adding a detection must use data the operator is actually looking at.

Two failures motivated these tests, both seen on a real seyond_left bag:

1. `Add` refused with "No synchronized detection pair available" while RViz showed
   detections. The reason was invisible: the two detectors succeed in DIFFERENT time
   windows (the LiDAR detector failed while ArUco saw markers, then ArUco went blank
   just before the LiDAR detector started working), and a sync group with an empty
   ArUco array was ignored at *debug* level while the empty-board case warned. The
   operator could not tell which side was missing.

2. `Add` SUCCEEDED long after the bag stopped, because the cached pair never expired.
   That silently buffers a pose from an unknown moment — the worst outcome of the
   three, because it looks like it worked.
"""

import pytest
from lctk_sync import (
    SyncGroupSummary,
    format_sync_stats,
    sync_pair_staleness_error,
    sync_wait_diagnosis,
)


def test_a_fresh_pair_is_accepted():
    assert sync_pair_staleness_error(age_s=0.3, max_age_s=2.0) is None


def test_a_stale_pair_is_refused_and_states_its_age():
    """The bag stopped minutes ago; the cached pair is from whenever it stopped."""
    message = sync_pair_staleness_error(age_s=137.0, max_age_s=2.0)

    assert message is not None
    assert "137" in message
    assert "2.0" in message


def test_the_gate_can_be_disabled():
    """Zero disables it, for a workflow that deliberately pauses playback."""
    assert sync_pair_staleness_error(age_s=999.0, max_age_s=0.0) is None


def test_no_group_at_all_says_so():
    """Neither detector is producing anything the synchronizer can pair."""
    message = sync_wait_diagnosis(None)

    assert "no synchronized" in message.lower()


def test_a_group_missing_only_the_board_names_the_lidar_side():
    message = sync_wait_diagnosis(
        SyncGroupSummary(aruco_count=4, board_count=0, age_s=0.4)
    )

    assert "4 ArUco" in message
    assert "0 board" in message
    assert "lidar_board_detector" in message


def test_a_group_missing_only_the_markers_names_the_camera_side():
    message = sync_wait_diagnosis(
        SyncGroupSummary(aruco_count=0, board_count=1, age_s=0.2)
    )

    assert "0 ArUco" in message
    assert "1 board" in message
    assert "aruco_locator" in message


def test_a_group_missing_both_names_both():
    message = sync_wait_diagnosis(
        SyncGroupSummary(aruco_count=0, board_count=0, age_s=0.1)
    )

    assert "aruco_locator" in message and "lidar_board_detector" in message


def test_the_diagnosis_reports_how_old_the_last_group_is():
    """A stale summary means the pipeline has gone quiet — a different problem from
    'the detectors disagree', and the operator must be able to tell them apart."""
    message = sync_wait_diagnosis(
        SyncGroupSummary(aruco_count=4, board_count=0, age_s=64.5)
    )

    assert "64.5" in message


@pytest.mark.parametrize("age_s", [0.0, 1.999])
def test_boundary_ages_are_fresh(age_s):
    assert sync_pair_staleness_error(age_s=age_s, max_age_s=2.0) is None


def test_the_stats_line_names_each_stream_and_what_happened_to_it():
    """When groups stop arriving while both detectors are clearly publishing, the
    question is which stream stopped reaching THIS node, and whether the
    synchronizer dropped it. Received / dropped / rejected per topic answers it;
    without them the operator is guessing."""
    line = format_sync_stats(
        received={"aruco_detections": 4210, "calibration_board_detections": 0},
        dropped={"aruco_detections": 0, "calibration_board_detections": 0},
        rejected={"aruco_detections": 3900, "calibration_board_detections": 0},
        groups=118,
    )

    assert "aruco_detections" in line and "calibration_board_detections" in line
    assert "4210" in line and "3900" in line
    assert "118" in line


def test_the_stats_line_survives_missing_topics():
    assert format_sync_stats(received={}, dropped={}, rejected={}, groups=0)


def test_a_silent_stream_is_named():
    """Groups stopped while both detectors were visibly publishing. The question is
    which stream stopped reaching THIS node — so say which one went quiet."""
    from lctk_sync import sync_health_warning

    warning = sync_health_warning(
        previous={"aruco_detections": 100, "calibration_board_detections": 40},
        current={"aruco_detections": 400, "calibration_board_detections": 40},
        last_group_age_s=120.0,
    )

    assert warning is not None
    assert "calibration_board_detections" in warning
    assert "aruco_detections" not in warning.split("no new messages")[1]


def test_both_streams_alive_but_no_groups_is_its_own_warning():
    """This is the case that points at the synchronizer rather than a detector."""
    from lctk_sync import sync_health_warning

    warning = sync_health_warning(
        previous={"aruco_detections": 100, "calibration_board_detections": 40},
        current={"aruco_detections": 400, "calibration_board_detections": 80},
        last_group_age_s=120.0,
    )

    assert warning is not None
    assert "120" in warning
    assert "both" in warning.lower() or "no group" in warning.lower()


def test_a_healthy_pipeline_warns_about_nothing():
    from lctk_sync import sync_health_warning

    assert (
        sync_health_warning(
            previous={"a": 1, "b": 1},
            current={"a": 30, "b": 10},
            last_group_age_s=0.2,
        )
        is None
    )


def test_a_pipeline_that_has_not_started_is_not_a_stall():
    """Before playback begins nothing is arriving and no group ever has; that is
    waiting, not a fault, and must not cry wolf."""
    from lctk_sync import sync_health_warning

    assert (
        sync_health_warning(
            previous={}, current={"a": 0, "b": 0}, last_group_age_s=None
        )
        is None
    )


def test_the_stats_line_reports_the_pair_skew():
    """The skew INSIDE a group is the number that says whether "synchronized" means
    anything. With conflux's infinite window it grew without bound (11s on this rig)
    while every other counter looked healthy, so it must be visible at a glance."""
    line = format_sync_stats(
        received={"a": 10},
        dropped={"a": 0},
        rejected={"a": 0},
        groups=10,
        skew_ms=(12.5, 47.0),
    )

    assert "12.5" in line and "47.0" in line
    assert "skew" in line.lower()


def test_the_stats_line_is_fine_before_any_group():
    line = format_sync_stats(
        received={"a": 0}, dropped={"a": 0}, rejected={"a": 0}, groups=0, skew_ms=None
    )
    assert "skew" not in line.lower()
