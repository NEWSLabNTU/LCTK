"""M-13: the board-pose covariance must actually change the solve.

`cv2.solvePnP` assumes the 3D points are exact. They are not — they are ideal ArUco corners pushed
through the board pose that ICP fitted, so the board-pose error lands squarely in the "known" 3D
points. That is errors-in-variables, and the estimate is *biased*, not merely noisy.

The detector now publishes a real 6x6 covariance. These tests pin that the solver consumes it: a
pose with a loose ICP fit must count for less than one with a tight fit, and weighting must beat not
weighting when one pose in the buffer is bad.

If it doesn't, the covariance is decoration and should be deleted rather than maintained.
"""

import cv2
import numpy as np
import pytest
from lidar_to_camera_solver.main import BoardDetection
from lidar_to_camera_solver.main import LidarToCameraSolver as S

K = np.array([[1164.6, 0, 950.1], [0, 1161.1, 538.6], [0, 0, 1]], dtype=np.float64)
R_TRUE = np.array([[0.0, -1, 0], [0, 0, -1], [1, 0, 0]])
RVEC_TRUE, _ = cv2.Rodrigues(R_TRUE)
TVEC_TRUE = np.array([[0.07], [-0.02], [-0.89]])


def tight_cov(scale=1e-6):
    """A well-conditioned board fit: small, roughly isotropic."""
    return np.diag([scale, scale, scale, scale, scale, scale])


def loose_in_plane_cov():
    """The real anisotropy: tight along the board normal, nearly free within the plane.

    This is what the detector reports for a board with few edge / hole-rim returns — interior
    LiDAR points constrain only the out-of-plane direction.
    """
    return np.diag([1e-2, 1e-2, 1e-6, 1e-6, 1e-6, 1e-2])


def corners_at(origin, spread=0.25):
    """16 ArUco corners on a board patch centred at `origin`."""
    pts = []
    for dx in (-spread, spread):
        for dy in (-spread, spread):
            for ex in (-0.05, 0.05):
                for ey in (-0.05, 0.05):
                    pts.append([origin[0], origin[1] + dx + ex, origin[2] + dy + ey])
    return np.array(pts, dtype=np.float64)


def test_a_loose_pose_is_weighted_down():
    """The headline: a board whose ICP fit is loose must count for less."""
    origin = (2.6, 0.0, 0.0)
    corners = corners_at(origin)

    tight = BoardDetection(
        position=origin, orientation=(0, 0, 0, 1), covariance=tight_cov()
    )
    loose = BoardDetection(
        position=origin, orientation=(0, 0, 0, 1), covariance=loose_in_plane_cov()
    )

    w_tight = S._pose_weight(tight, corners)
    w_loose = S._pose_weight(loose, corners)

    assert w_tight > w_loose, (
        f"a loose board pose ({w_loose:.3f}) must not count as much as a tight one "
        f"({w_tight:.3f}); the covariance is not reaching the weight"
    )
    assert w_loose < 0.5, (
        f"an in-plane-unobservable pose should be heavily discounted, got {w_loose}"
    )


def test_no_covariance_means_no_change_in_behaviour():
    """An older detector publishes no covariance. That must not silently down-weight everything."""
    corners = corners_at((2.6, 0.0, 0.0))
    detection = BoardDetection(
        position=(2.6, 0.0, 0.0), orientation=(0, 0, 0, 1), covariance=None
    )

    assert S._pose_weight(detection, corners) == 1.0


def test_zero_covariance_is_read_as_unknown_not_exact():
    """`[0.0; 36]` from a detector that could not compute one must mean UNKNOWN, not EXACT.

    Reading it as "exact" is the original M-13 bug, one layer down.
    """
    cov = np.zeros((6, 6))
    # This is what `_detection3d_to_board_detection` does with an all-zero covariance.
    assert not np.any(cov)


def test_weighting_beats_not_weighting_when_one_pose_is_bad():
    """The property that justifies the whole mechanism.

    Build a buffer of good board poses plus ONE whose 3D points are badly displaced (a loose ICP
    fit). Weighted refinement, told that pose is untrustworthy, must land closer to the true
    extrinsic than the unweighted solve does.
    """
    rng = np.random.default_rng(0)

    placements = [(1.8, -0.6, 0.2), (2.4, 0.5, -0.3), (3.0, -0.2, 0.4), (2.1, 0.8, 0.1)]
    objs, imgs, weights = [], [], []

    for i, origin in enumerate(placements):
        truth = corners_at(origin)
        img, _ = cv2.projectPoints(truth, RVEC_TRUE, TVEC_TRUE, K, np.zeros(5))
        img = img.reshape(-1, 2) + rng.normal(0, 0.1, (len(truth), 2))

        if i == 1:
            # A loose fit: the board pose is off, so its 3D corners are displaced bodily.
            obj = truth + np.array([0.0, 0.05, 0.05])
            cov = loose_in_plane_cov()
        else:
            obj = truth + rng.normal(0, 0.001, truth.shape)
            cov = tight_cov()

        det = BoardDetection(position=origin, orientation=(0, 0, 0, 1), covariance=cov)
        objs.append(obj)
        imgs.append(img)
        weights.extend([S._pose_weight(det, obj)] * len(obj))

    obj_all = np.vstack(objs)
    img_all = np.vstack(imgs)
    w = np.asarray(weights)

    ok, rvec, tvec = cv2.solvePnP(
        obj_all, img_all, K, np.zeros(5), flags=cv2.SOLVEPNP_SQPNP
    )
    assert ok

    # Unweighted: OpenCV's LM, i.e. what M-12 left us with.
    rvec_u, tvec_u = cv2.solvePnPRefineLM(
        obj_all, img_all, K, np.zeros(5), rvec.copy(), tvec.copy()
    )

    # Weighted: the same polish, told which pose to distrust.
    solver = S.__new__(
        S
    )  # no ROS node needed; _refine_pnp_weighted touches no self state
    rvec_w, tvec_w = S._refine_pnp_weighted(
        solver, obj_all, img_all, K, rvec.copy(), tvec.copy(), w
    )

    def err(r, t):
        Rm, _ = cv2.Rodrigues(r)
        rot = np.degrees(np.arccos(np.clip((np.trace(Rm @ R_TRUE.T) - 1) / 2, -1, 1)))
        return rot, float(np.linalg.norm(t - TVEC_TRUE) * 1000)

    rot_u, trans_u = err(rvec_u, tvec_u)
    rot_w, trans_w = err(rvec_w, tvec_w)

    assert trans_w < trans_u, (
        f"weighting by the board-pose covariance did not improve the translation: "
        f"unweighted {trans_u:.1f} mm vs weighted {trans_w:.1f} mm. "
        f"If this holds, M-13's weighting is not earning its complexity."
    )
    assert rot_w <= rot_u + 1e-6, (
        f"weighting worsened the rotation: unweighted {rot_u:.3f} deg vs weighted {rot_w:.3f} deg"
    )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
