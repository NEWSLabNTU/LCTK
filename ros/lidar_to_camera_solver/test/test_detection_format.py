"""The saved-detection file format, version 4.

A saved calibration is a stored *pose*, and Phase 1 changed what a board pose means.
Version 3 files record no convention at all — the board-local corners are recomputed at
load time from `aruco_pattern.json5` — so a file written before the change and one
written after are indistinguishable, and either reloads under whatever convention the
loading build believes in. Version 4 records the convention that produced it.

Version 3 is therefore **rejected** rather than migrated on load: automatic migration
would make a file's meaning depend on which build opened it, which is the same class of
silent difference this whole phase exists to remove.

Version 4 also stores the board pose's 6x6 covariance, which version 3 dropped. Without
it a reloaded buffer solves with uniform weight 1.0 and quietly differs from the live
buffer it was saved from.
"""

import numpy as np
import pytest
from geometry_msgs.msg import Pose, PoseWithCovariance
from lidar_to_camera_solver.board_geometry import BOARD_FRAME_CONVENTION
from lidar_to_camera_solver.detection_format import (
    FORMAT_VERSION,
    deserialize_detection3d_array,
    format_version_error,
    migrate_v3_to_v4,
    serialize_detection3d_array,
)
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


def board_msg(covariance):
    msg = Detection3DArray()
    msg.header.frame_id = "velodyne_top"
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 34

    detection = Detection3D()
    result = ObjectHypothesisWithPose()
    result.pose = PoseWithCovariance()
    result.pose.pose = Pose()
    result.pose.pose.position.x = 1.5
    result.pose.pose.position.y = -0.25
    result.pose.pose.position.z = 0.75
    result.pose.pose.orientation.w = 1.0
    result.pose.covariance = [float(v) for v in np.asarray(covariance).flatten()]
    detection.results.append(result)
    msg.detections.append(detection)
    return msg


def test_the_format_version_is_four():
    assert FORMAT_VERSION == 4


def test_a_version_4_file_is_accepted():
    data = {"version": 4, "board_frame_convention": BOARD_FRAME_CONVENTION}
    assert format_version_error(data) is None


def test_a_version_3_file_is_rejected_and_points_at_the_conversion_command():
    """Rejected, not silently reinterpreted: a stale calibration must not quietly
    become a wrong one."""
    message = format_version_error({"version": 3})

    assert message is not None
    assert "3" in message and "4" in message
    assert "migrate_detections" in message, "the message must name the way out"


@pytest.mark.parametrize("version", [0, 1, 2, 5])
def test_other_versions_are_rejected(version):
    assert format_version_error({"version": version}) is not None


def test_a_version_4_file_carrying_the_wrong_convention_is_rejected():
    """The version says how the file is laid out; the tag says what the poses mean.
    Both have to agree with this build."""
    message = format_version_error(
        {"version": 4, "board_frame_convention": "edge_aligned_corner_origin_v0"}
    )

    assert message is not None
    assert "edge_aligned_corner_origin_v0" in message
    assert BOARD_FRAME_CONVENTION in message


def test_a_version_4_file_without_a_convention_tag_is_rejected():
    assert format_version_error({"version": 4}) is not None


def test_the_board_pose_covariance_survives_a_round_trip():
    """Version 3 dropped it, so a reloaded buffer always solved with uniform weight 1.0
    and quietly differed from the live buffer it was saved from (M-13)."""
    covariance = np.diag([1e-4, 2e-4, 3e-4, 4e-6, 5e-6, 6e-6])
    covariance[0, 1] = covariance[1, 0] = 7e-5

    restored = deserialize_detection3d_array(
        serialize_detection3d_array(board_msg(covariance))
    )

    round_tripped = np.asarray(
        restored.detections[0].results[0].pose.covariance
    ).reshape(6, 6)
    assert np.allclose(round_tripped, covariance, atol=0.0, rtol=0.0)


def test_the_pose_survives_a_round_trip():
    restored = deserialize_detection3d_array(
        serialize_detection3d_array(board_msg(np.zeros((6, 6))))
    )
    pose = restored.detections[0].results[0].pose.pose

    assert (pose.position.x, pose.position.y, pose.position.z) == (1.5, -0.25, 0.75)
    assert pose.orientation.w == 1.0
    assert restored.header.frame_id == "velodyne_top"


def test_migration_stamps_the_convention_the_operator_asserted():
    """The conversion is an operator's claim about a file's provenance, so the
    convention is named explicitly rather than assumed."""
    migrated = migrate_v3_to_v4(
        {"version": 3, "num_detections": 0, "detections": []},
        convention=BOARD_FRAME_CONVENTION,
    )

    assert migrated["version"] == 4
    assert migrated["board_frame_convention"] == BOARD_FRAME_CONVENTION
    assert format_version_error(migrated) is None


def test_migration_refuses_anything_but_a_version_3_file():
    with pytest.raises(ValueError, match="version 3"):
        migrate_v3_to_v4({"version": 4}, convention=BOARD_FRAME_CONVENTION)
