"""Projection provenance shared by Detection Archive consumers; no ROS types."""

from collections.abc import Mapping

import numpy as np

PROJECTION_MODEL = "undistorted_pixels_using_k"


def archive_frames(data: Mapping, projection: Mapping) -> tuple[str | None, str]:
    """Validate the archive envelope and bind consistent detection header frames.

    Geometry belongs to each consumer's scoring or admission rules. Empty archives
    have no observed LiDAR frame and return ``None`` for that label.
    """
    detections = data.get("detections")
    count = data.get("num_detections")
    if not isinstance(detections, list):
        raise ValueError("Detection archive detections must be a list")  # noqa: TRY004
    if (
        not isinstance(count, int)
        or isinstance(count, bool)
        or count != len(detections)
    ):
        raise ValueError("Detection archive count mismatch")
    camera_frame = projection["frame_id"]
    lidar_frame = None
    for pair in detections:
        try:
            camera = pair["aruco"]["header"]["frame_id"]
            lidar = pair["board"]["header"]["frame_id"]
        except (KeyError, TypeError) as error:
            raise ValueError(
                "Detection archive requires sensor header frame_id"
            ) from error
        if not all(
            isinstance(frame, str) and frame.strip() for frame in (lidar, camera)
        ):
            raise ValueError("Detection archive sensor frames must be nonempty")
        if camera != camera_frame:
            raise ValueError("Camera detection frame does not match camera_projection")
        if lidar == camera:
            raise ValueError("LiDAR and camera frames must be distinct")
        if lidar_frame is not None and lidar_frame != lidar:
            raise ValueError("Detection archive mixes LiDAR frames")
        lidar_frame = lidar
    return lidar_frame, camera_frame


def normalize_camera_projection(value: object) -> dict:
    """Validate projection provenance and return a detached normalized value."""
    if not isinstance(value, Mapping):
        raise ValueError("camera_projection must be an object")  # noqa: TRY004
    if value.get("model") != PROJECTION_MODEL:
        raise ValueError(f"camera_projection.model must be {PROJECTION_MODEL}")
    frame = value.get("frame_id")
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError("camera_projection.frame_id must be nonempty")
    for field in ("width", "height"):
        number = value.get(field)
        if not isinstance(number, int) or isinstance(number, bool) or number <= 0:
            raise ValueError(f"camera_projection.{field} must be a positive integer")
    raw = value.get("k")
    if (
        not isinstance(raw, (list, tuple))
        or len(raw) != 9
        or any(
            isinstance(item, bool) or not isinstance(item, (int, float)) for item in raw
        )
    ):
        raise ValueError("camera_projection.k must contain nine numbers")
    try:
        k = np.asarray(raw, dtype=np.float64).reshape(3, 3)
    except (OverflowError, ValueError) as error:
        raise ValueError("camera_projection.k must contain finite numbers") from error
    if not np.all(np.isfinite(k)):
        raise ValueError("camera_projection.k must be finite")
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("camera_projection.k focal lengths must be positive")
    if not np.array_equal(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0:
        raise ValueError("camera_projection.k must be a standard pinhole camera matrix")
    return {
        "model": PROJECTION_MODEL,
        "frame_id": frame,
        "width": value["width"],
        "height": value["height"],
        "k": k.reshape(9).tolist(),
    }
