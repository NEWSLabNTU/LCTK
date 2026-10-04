"""The saved-detection file format, version 6.

A saved calibration is a stored pose. Version 4 records the board-frame convention
and retains pose covariance; version 5 adds the exact Target Identity. Version 6
also binds observations to camera projection metadata and sensor frames, and is the
only format the solver restores.

Archives from versions before 6 are unsupported. The solver does not migrate them;
their camera projection and frame provenance cannot be recovered reliably.
"""

from pathlib import Path

import numpy as np
import pytest
from geometry_msgs.msg import Pose, PoseWithCovariance
from lctk_target import load_target
from lidar_to_camera_solver.board_geometry import BOARD_FRAME_CONVENTION
from lidar_to_camera_solver.detection_format import (
    FORMAT_VERSION,
    deserialize_detection3d_array,
    format_version_error,
    serialize_detection3d_array,
)
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

SOLID = load_target(
    Path(__file__).resolve().parents[2]
    / "lctk_launch"
    / "config"
    / "targets"
    / "solid_600_aruco_1_v1.json5"
)
IDENTITY = {
    field: getattr(SOLID.identity, field)
    for field in (
        "schema_version",
        "target_id",
        "revision",
        "semantic_sha256",
        "board_frame_convention",
    )
}


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


def test_the_format_version_is_six():
    assert FORMAT_VERSION == 6


def test_version_six_requires_consistent_camera_projection_and_detection_frames():
    archive = {
        "version": 6,
        "board_frame_convention": BOARD_FRAME_CONVENTION,
        "target_identity": IDENTITY,
        "camera_projection": {
            "model": "undistorted_pixels_using_k",
            "frame_id": "camera_optical",
            "width": 640,
            "height": 480,
            "k": [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        },
        "num_detections": 1,
        "detections": [
            {
                "aruco": {"header": {"frame_id": "camera_optical"}},
                "board": {"header": {"frame_id": "lidar_top"}},
            }
        ],
    }

    assert format_version_error(archive) is None
    archive["detections"][0]["board"]["header"]["frame_id"] = ""
    assert "frame" in format_version_error(archive)


def test_version_six_is_the_only_supported_archive_version():
    archive = {
        "version": 6,
        "board_frame_convention": BOARD_FRAME_CONVENTION,
        "target_identity": IDENTITY,
        "camera_projection": {
            "model": "undistorted_pixels_using_k",
            "frame_id": "camera_optical",
            "width": 640,
            "height": 480,
            "k": [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        },
        "num_detections": 0,
        "detections": [],
    }

    assert format_version_error(archive) is None
    for version in (4, 5):
        assert "version 6" in format_version_error({**archive, "version": version})


def test_a_version_3_file_is_rejected_without_a_migration_path():
    message = format_version_error({"version": 3})

    assert message is not None
    assert "version 6" in message
    assert "migrate_detections" not in message


@pytest.mark.parametrize("version", [0, 1, 2, 3, 4, 5, 7])
def test_other_versions_are_rejected(version):
    assert format_version_error({"version": version}) is not None


def test_a_version_6_file_carrying_the_wrong_convention_is_rejected():
    """The version says how the file is laid out; the tag says what the poses mean.
    Both have to agree with this build."""
    message = format_version_error(
        {
            "version": 6,
            "board_frame_convention": "edge_aligned_corner_origin_v0",
            "target_identity": IDENTITY,
        }
    )

    assert message is not None
    assert "edge_aligned_corner_origin_v0" in message
    assert BOARD_FRAME_CONVENTION in message


def test_a_version_6_file_without_a_convention_tag_is_rejected():
    assert format_version_error({"version": 6, "target_identity": IDENTITY}) is not None


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
