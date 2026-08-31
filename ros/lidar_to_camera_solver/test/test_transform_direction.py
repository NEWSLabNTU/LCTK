"""M-01: the published TransformStamped must mean what its frame labels say.

`cv2.solvePnP` returns `(R, t)` with `p_cam = R @ p_lidar + t` -- that is `T_camera<-lidar`.
Publishing that raw, labelled `frame_id=lidar, child_frame_id=camera`, states the opposite of
what ROS TF means by those labels: a transform labelled lidar->camera must give the camera's
pose *expressed in lidar coordinates*, i.e. the inverse.

The error was invisible inside LCTK because the overlay consumes the topic directly as
rvec/tvec for `projectPoints`, so it wanted the un-inverted form. Both sides therefore have to
move together, which is why this sat deferred: the only stated correctness signal was "does the
overlay still look right".

These tests replace that visual check with an arithmetic one. If the solver inverts on publish
and the overlay inverts back, the projection is bit-for-bit what it always was, and a tf2
consumer finally gets the direction it asks for.
"""

import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
from lidar_to_camera_solver.main import LidarToCameraSolver as S
from pointcloud_image_overlay.overlay_node import transform_to_rvec_tvec

# A representative solve: camera looking along the LiDAR's +x, REP-103 style.
R_CAM_LIDAR = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
RVEC_SOLVE, _ = cv2.Rodrigues(R_CAM_LIDAR)
TVEC_SOLVE = np.array([[0.07], [-0.02], [-0.89]])


class _FakeClock:
    def now(self):
        class _T:
            def to_msg(self):
                from builtin_interfaces.msg import Time

                return Time()

        return _T()


def make_solver():
    solver = S.__new__(S)
    solver.parent_frame = "lidar"
    solver.child_frame = "camera"
    solver.get_clock = lambda: _FakeClock()
    return solver


def msg_to_matrix(msg: TransformStamped) -> np.ndarray:
    """The 4x4 the message denotes, read literally."""
    q = msg.transform.rotation
    rot = cv2.Rodrigues(
        np.array(
            cv2.Rodrigues(
                np.array(
                    [
                        [
                            1 - 2 * (q.y**2 + q.z**2),
                            2 * (q.x * q.y - q.z * q.w),
                            2 * (q.x * q.z + q.y * q.w),
                        ],
                        [
                            2 * (q.x * q.y + q.z * q.w),
                            1 - 2 * (q.x**2 + q.z**2),
                            2 * (q.y * q.z - q.x * q.w),
                        ],
                        [
                            2 * (q.x * q.z - q.y * q.w),
                            2 * (q.y * q.z + q.x * q.w),
                            1 - 2 * (q.x**2 + q.y**2),
                        ],
                    ]
                )
            )[0]
        )
    )[0]
    t = msg.transform.translation
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = [t.x, t.y, t.z]
    return out


def solve_matrix() -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = R_CAM_LIDAR
    out[:3, 3] = TVEC_SOLVE.ravel()
    return out


def test_published_transform_follows_tf_semantics():
    """lidar -> camera must be the camera's pose in lidar coordinates: inv(T_camera<-lidar)."""
    msg = make_solver()._create_transform_message(RVEC_SOLVE, TVEC_SOLVE)

    assert msg.header.frame_id == "lidar"
    assert msg.child_frame_id == "camera"

    published = msg_to_matrix(msg)
    expected = np.linalg.inv(solve_matrix())

    np.testing.assert_allclose(published, expected, atol=1e-9)


def test_published_transform_is_not_the_raw_solve():
    """Guard against a future 'simplification' that drops the inversion again."""
    msg = make_solver()._create_transform_message(RVEC_SOLVE, TVEC_SOLVE)
    assert not np.allclose(msg_to_matrix(msg), solve_matrix(), atol=1e-6)


def test_overlay_recovers_the_projection_inputs():
    """The overlay must get back exactly the rvec/tvec projectPoints needs.

    This is the arithmetic replacement for the visual check: if this holds, the point cloud
    projects onto the image exactly as it did before the direction was corrected.
    """
    msg = make_solver()._create_transform_message(RVEC_SOLVE, TVEC_SOLVE)

    rvec, tvec = transform_to_rvec_tvec(msg)

    np.testing.assert_allclose(rvec.reshape(3, 1), RVEC_SOLVE, atol=1e-9)
    np.testing.assert_allclose(tvec.reshape(3, 1), TVEC_SOLVE, atol=1e-9)


def test_projection_is_unchanged_end_to_end():
    """Project real points through both paths and compare pixels."""
    K = np.array([[1164.6, 0, 950.1], [0, 1161.1, 538.6], [0, 0, 1]], dtype=np.float64)
    pts = np.array(
        [[2.5, 0.3, 0.1], [3.1, -0.2, 0.4], [2.0, 0.0, -0.3], [4.2, 0.6, 0.2]],
        dtype=np.float64,
    )
    dist = np.zeros(5)

    before, _ = cv2.projectPoints(pts, RVEC_SOLVE, TVEC_SOLVE, K, dist)

    msg = make_solver()._create_transform_message(RVEC_SOLVE, TVEC_SOLVE)
    rvec, tvec = transform_to_rvec_tvec(msg)
    after, _ = cv2.projectPoints(pts, rvec, tvec, K, dist)

    np.testing.assert_allclose(after, before, atol=1e-6)
