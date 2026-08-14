"""The calibration board's marker geometry, and the plain-value helpers around it.

This module is the Python half of a **cross-language contract**: the Rust
`hollow-board-config` crate computes the same marker corner positions for the
detector, and this module computes them for the camera solver. The published board
pose is ``T_sensor<-board`` and the solver supplies board-local marker coordinates to
it, so the frame convention appears on *both* sides of one product. If the two sides
disagree, the error is partly *silent*: the 2x2 marker grid is symmetric, so an
in-plane 45-degree disagreement still solves cleanly with a low reprojection error.

Hence two rules for this module:

1. It imports nothing from ``rclpy``, so the arithmetic is testable without a ROS
   graph. Logging is the caller's business.
2. Its output is asserted against ``fixtures/marker_corners_world.golden.json``, the
   same fixture the Rust ``marker_layout_golden`` test uses.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

#: The board-frame convention this module's coordinates are expressed in, and the one
#: `lidar_board_detector` publishes (see its `BOARD_FRAME_CONVENTION`). It lives beside
#: the geometry deliberately: the tag and the coordinates it describes must not be
#: changeable independently of each other.
BOARD_FRAME_CONVENTION = "corner_aligned_plate_center_v1"

#: Absolute topic carrying the tag. Absolute rather than node-relative because the
#: launch system generates one detector node per sensor-marker pair, so a relative name
#: would couple every consumer to a generated node name.
BOARD_FRAME_CONVENTION_TOPIC = "/lctk/board_frame_convention"


def frame_convention_error(received: Optional[str]) -> Optional[str]:
    """Decide whether a received frame-convention tag is acceptable.

    Returns ``None`` when it is, or an operator-facing failure message when it is not.
    Pure over the received string, so the whole decision table — match, mismatch,
    absent — is testable without a ROS graph.

    **Absence is failure, not consent.** The tag is published latched
    (transient-local), but a latched sample only reaches a late joiner while a
    publisher is alive: a solver started before any detector, or after the bag ended
    and the detector exited, receives nothing at all.
    """
    if received is None:
        return (
            f"No board-frame convention announced on {BOARD_FRAME_CONVENTION_TOPIC}. "
            f"Expected '{BOARD_FRAME_CONVENTION}'. Nothing published there means no "
            f"detector agreed with this solver about what a board pose means -- which "
            f"is a failure, not consent. Start lidar_board_detector first, and check "
            f"with: ros2 topic echo {BOARD_FRAME_CONVENTION_TOPIC} --once"
        )

    tag = received.strip()
    if tag == BOARD_FRAME_CONVENTION:
        return None

    return (
        f"Board-frame convention mismatch on {BOARD_FRAME_CONVENTION_TOPIC}: "
        f"received '{tag}', expected '{BOARD_FRAME_CONVENTION}'. The detector and this "
        f"solver disagree about what a published board pose means. Half of that "
        f"disagreement is silent -- a 45-degree in-plane rotation still solves with a "
        f"low reprojection error, because the 2x2 marker grid is symmetric. Rebuild "
        f"both sides from the same checkout."
    )


@dataclass
class ArUcoMarker:
    """Represents an ArUco marker detection in image coordinates."""

    id: int
    corners: List[Tuple[float, float]]  # 4 corners in pixel coordinates
    center: Tuple[float, float]  # Center point in pixels


def parse_dimension(dim_str: str) -> float:
    """Parse dimension string like '500mm' or '10mm' to meters."""
    if dim_str.endswith("mm"):
        return float(dim_str[:-2]) / 1000.0
    elif dim_str.endswith("m"):
        return float(dim_str[:-1])
    else:
        return float(dim_str)


def load_aruco_pattern_config(config_file_path: str) -> dict:
    """Load ArUco pattern configuration from a JSON5 file."""
    if not config_file_path:
        raise ValueError("aruco_config_file parameter is required")

    import json5

    with open(config_file_path, "r") as f:
        return json5.load(f)


def compute_multi_marker_corners(
    config: dict,
) -> Dict[int, List[Tuple[float, float, float]]]:
    """Compute 3D corner positions for all ArUco markers in the board frame.

    Returns a mapping from ArUco marker id to its four corners in the order
    ``[right, top, left, bottom]``, matching the Rust
    ``BoardModel::multi_marker_corners`` contract.
    """
    board_size = parse_dimension(config["board_size"])
    board_border_size = parse_dimension(config["board_border_size"])
    marker_square_size_ratio = config["marker_square_size_ratio"]
    num_squares = config["num_squares_per_side"]
    marker_ids = config["marker_ids"]
    # M-09: the 2x2 board layout indexes marker_ids[0..3]; fail with a clear
    # message instead of an IndexError deep inside the solve service.
    if len(marker_ids) < 4:
        raise ValueError(
            f"ArUco config must define at least 4 marker_ids for the 2x2 board, "
            f"got {len(marker_ids)}: {marker_ids}"
        )

    square_size = (board_size - 2.0 * board_border_size) / num_squares
    marker_size = square_size * marker_square_size_ratio
    marker_border = (square_size - marker_size) / 2.0

    def make_corners(base_x: float, base_y: float) -> List[Tuple[float, float, float]]:
        """Create 4 corner points for a marker in board-local coordinates."""
        bottom = (base_x, base_y, 0.0)
        left = (base_x + marker_size, base_y, 0.0)
        right = (base_x, base_y + marker_size, 0.0)
        top = (base_x + marker_size, base_y + marker_size, 0.0)
        return [right, top, left, bottom]

    origin_x = board_border_size + marker_border
    origin_y = board_border_size + marker_border

    marker_corners = {}
    marker_corners[marker_ids[0]] = make_corners(origin_x, origin_y)
    marker_corners[marker_ids[1]] = make_corners(origin_x + square_size, origin_y)
    marker_corners[marker_ids[2]] = make_corners(origin_x, origin_y + square_size)
    marker_corners[marker_ids[3]] = make_corners(
        origin_x + square_size, origin_y + square_size
    )

    return marker_corners


def detection2d_to_aruco_markers(detection_msg) -> List[ArUcoMarker]:
    """Convert a ROS ``Detection2DArray`` to ``ArUcoMarker`` objects.

    The real detected marker corners are carried in ``detection.results``
    (one entry per corner, in detector order TL, TR, BR, BL) by the ArUco
    locator node; the axis-aligned bounding box is only a fallback.
    """
    markers = []

    for detection in detection_msg.detections:
        bbox = detection.bbox
        center = (bbox.center.position.x, bbox.center.position.y)

        # C-01: prefer the real per-corner pixel coordinates. Reconstructing
        # corners from `center +/- size/2` discards rotation and perspective,
        # biasing the PnP correspondences for any angled view of the board.
        if len(detection.results) >= 4:
            corners = [
                (r.pose.pose.position.x, r.pose.pose.position.y)
                for r in detection.results[:4]
            ]
        else:
            size_x = bbox.size_x
            size_y = bbox.size_y
            cx, cy = center
            corners = [
                (cx - size_x / 2.0, cy - size_y / 2.0),  # Top-left
                (cx + size_x / 2.0, cy - size_y / 2.0),  # Top-right
                (cx + size_x / 2.0, cy + size_y / 2.0),  # Bottom-right
                (cx - size_x / 2.0, cy + size_y / 2.0),  # Bottom-left
            ]

        marker_id = detection.id if hasattr(detection, "id") else 0

        markers.append(ArUcoMarker(id=marker_id, corners=corners, center=center))

    return markers


def rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a quaternion ``[x, y, z, w]``."""
    rvec, _ = cv2.Rodrigues(rotation_matrix)
    angle = np.linalg.norm(rvec)

    if angle < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])

    axis = rvec.flatten() / angle
    half_angle = angle / 2.0

    qx = axis[0] * np.sin(half_angle)
    qy = axis[1] * np.sin(half_angle)
    qz = axis[2] * np.sin(half_angle)
    qw = np.cos(half_angle)

    return np.array([qx, qy, qz, qw])


def marker_geometry_summary(config: dict) -> str:
    """One-line description of the derived marker sizes, for a caller to log."""
    board_size = parse_dimension(config["board_size"])
    board_border_size = parse_dimension(config["board_border_size"])
    num_squares = config["num_squares_per_side"]
    square_size = (board_size - 2.0 * board_border_size) / num_squares
    marker_size = square_size * config["marker_square_size_ratio"]
    marker_border = (square_size - marker_size) / 2.0
    return (
        f"square_size={square_size * 1000:.1f}mm, "
        f"marker_size={marker_size * 1000:.1f}mm, "
        f"marker_border={marker_border * 1000:.1f}mm"
    )


__all__ = [
    "ArUcoMarker",
    "BOARD_FRAME_CONVENTION",
    "BOARD_FRAME_CONVENTION_TOPIC",
    "compute_multi_marker_corners",
    "detection2d_to_aruco_markers",
    "frame_convention_error",
    "load_aruco_pattern_config",
    "marker_geometry_summary",
    "parse_dimension",
    "rotation_matrix_to_quaternion",
]
