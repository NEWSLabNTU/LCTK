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

from lidar_to_camera_solver.main import (
    SyncGroupSummary,
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
