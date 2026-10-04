import pytest
from lctk_quality.projection_metadata import archive_frames, normalize_camera_projection


def test_projection_metadata_is_validated_and_detached():
    value = {
        "model": "undistorted_pixels_using_k",
        "frame_id": "optical",
        "width": 640,
        "height": 480,
        "k": [100, 0, 320, 0, 100, 240, 0, 0, 1],
    }
    normalized = normalize_camera_projection(value)
    value["k"][0] = -1
    assert normalized["k"][0] == 100.0
    with pytest.raises(ValueError, match="focal"):
        normalize_camera_projection(value)


def test_archive_frames_bind_headers_without_inspecting_geometry():
    projection = {"frame_id": "optical"}
    pair = {
        "aruco": {"header": {"frame_id": "optical"}},
        "board": {"header": {"frame_id": "lidar"}},
    }
    archive = {"num_detections": 1, "detections": [pair]}
    assert archive_frames(archive, projection) == ("lidar", "optical")
    pair["board"]["header"]["frame_id"] = "optical"
    with pytest.raises(ValueError, match="distinct"):
        archive_frames(archive, projection)


def test_empty_archive_has_no_lidar_frame():
    assert archive_frames(
        {"num_detections": 0, "detections": []}, {"frame_id": "optical"}
    ) == (None, "optical")


def test_unrepresentable_camera_matrix_reports_a_validation_error():
    value = {
        "model": "undistorted_pixels_using_k",
        "frame_id": "optical",
        "width": 640,
        "height": 480,
        "k": [10**1000, 0, 320, 0, 100, 240, 0, 0, 1],
    }
    with pytest.raises(ValueError, match="finite"):
        normalize_camera_projection(value)
