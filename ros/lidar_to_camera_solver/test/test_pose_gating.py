"""M-12 (items 2-3): one bad pose in the buffer must not corrupt the whole calibration.

A pose's 16 corners share a single rigid `T_board`, so when that pose is wrong -- a partially
occluded board, a grazing-incidence ICP local minimum, or the quarter-turn origin-corner slip of
M-14 -- all 16 corners are outliers *together*. That correlated regime is the one least-squares
handles worst, and until now nothing looked at per-pose error, so nothing could exclude it.

Rejection is deliberately at **pose** granularity. Rejecting individual corners would be
statistically wrong: the errors inside a pose are not independent draws, they are one rigid
misplacement seen 16 times.
"""

import cv2
import numpy as np

from lidar_to_camera_solver.main import LidarToCameraSolver as S

K = np.array([[1164.6, 0, 950.1], [0, 1161.1, 538.6], [0, 0, 1]], dtype=np.float64)
R_TRUE = np.array([[0.0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64)
RVEC_TRUE, _ = cv2.Rodrigues(R_TRUE)
TVEC_TRUE = np.array([[0.07], [-0.02], [-0.89]], dtype=np.float64)
DIST = np.zeros(5, dtype=np.float64)


def board_corners(centre, yaw=0.0, spread=0.25):
    """16 ArUco corners on a board patch, optionally rotated in its own plane."""
    pts = []
    for dx in (-spread, spread):
        for dy in (-spread, spread):
            for ex in (-0.05, 0.05):
                for ey in (-0.05, 0.05):
                    u, v = dx + ex, dy + ey
                    # rotate within the board plane (the y-z plane for this rig)
                    ru = u * np.cos(yaw) - v * np.sin(yaw)
                    rv = u * np.sin(yaw) + v * np.cos(yaw)
                    pts.append([centre[0], centre[1] + ru, centre[2] + rv])
    return np.array(pts, dtype=np.float64)


def project(object_points, rng=None, noise_px=0.0):
    projected, _ = cv2.projectPoints(object_points, RVEC_TRUE, TVEC_TRUE, K, DIST)
    img = projected.reshape(-1, 2)
    if noise_px and rng is not None:
        img = img + rng.normal(0.0, noise_px, img.shape)
    return img


def buffer(n_good=5, corrupt_yaw=None, noise_px=0.2, seed=0):
    """`n_good` clean poses, plus one quarter-turned pose when `corrupt_yaw` is set."""
    rng = np.random.default_rng(seed)
    centres = [(2.2 + 0.3 * i, -0.4 + 0.2 * i, 0.1 * i) for i in range(n_good)]
    objs, imgs = [], []
    for c in centres:
        obj = board_corners(c)
        objs.append(obj)
        imgs.append(project(obj, rng, noise_px))

    if corrupt_yaw is not None:
        c = (2.4, 0.1, -0.05)
        true_obj = board_corners(c)
        # The image sees the board as it really is...
        img = project(true_obj, rng, noise_px)
        # ...but the LiDAR side hands over a quarter-turned corner set (M-14).
        objs.append(board_corners(c, yaw=corrupt_yaw))
        imgs.append(img)
    return objs, imgs


def solve(objs, imgs):
    obj = np.vstack(objs)
    img = np.vstack(imgs)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, DIST, flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, DIST, rvec, tvec)
    return rvec, tvec


def pose_error_deg_m(rvec, tvec):
    rot_err = np.linalg.norm(cv2.Rodrigues(cv2.Rodrigues(rvec)[0] @ R_TRUE.T)[0])
    return np.degrees(rot_err), float(np.linalg.norm(tvec - TVEC_TRUE))


def test_a_quarter_turned_pose_is_rejected():
    """The M-14 failure mode: one pose's corners rotated 90 deg about the board normal."""
    objs, imgs = buffer(n_good=5, corrupt_yaw=np.pi / 2)
    rvec, tvec = solve(objs, imgs)

    rejected, rms = S._reject_outlier_poses(objs, imgs, K, rvec, tvec)

    assert 5 in rejected, f"the corrupted pose was not rejected (per-pose RMS: {rms})"
    assert set(rejected) == {5}, f"a clean pose was rejected too: {rejected}"


def test_a_clean_buffer_rejects_nothing():
    """Noise alone must not trigger rejection, or good data bleeds away."""
    objs, imgs = buffer(n_good=6, corrupt_yaw=None)
    rvec, tvec = solve(objs, imgs)

    rejected, _ = S._reject_outlier_poses(objs, imgs, K, rvec, tvec)

    assert rejected == [], f"rejected poses from a clean buffer: {rejected}"


def test_rejecting_the_bad_pose_improves_the_extrinsic():
    """The point of the exercise: the re-solve must actually be closer to truth."""
    objs, imgs = buffer(n_good=5, corrupt_yaw=np.pi / 2)

    rvec_all, tvec_all = solve(objs, imgs)
    rejected, _ = S._reject_outlier_poses(objs, imgs, K, rvec_all, tvec_all)
    keep = [i for i in range(len(objs)) if i not in rejected]
    rvec_kept, tvec_kept = solve([objs[i] for i in keep], [imgs[i] for i in keep])

    rot_all, trans_all = pose_error_deg_m(rvec_all, tvec_all)
    rot_kept, trans_kept = pose_error_deg_m(rvec_kept, tvec_kept)

    assert rot_kept < rot_all, f"rotation error not improved: {rot_kept} vs {rot_all}"
    assert trans_kept < trans_all, (
        f"translation not improved: {trans_kept} vs {trans_all}"
    )


def test_never_rejects_below_the_minimum_kept():
    """With too few poses, refuse to reject -- an under-constrained solve is worse."""
    objs, imgs = buffer(n_good=2, corrupt_yaw=np.pi / 2)
    rvec, tvec = solve(objs, imgs)

    rejected, _ = S._reject_outlier_poses(objs, imgs, K, rvec, tvec, min_keep=3)

    assert rejected == [], "rejected despite leaving too few poses to solve with"


def test_reports_per_pose_rms_for_every_pose():
    """The residuals are the diagnostic, so they must come back even when nothing is rejected."""
    objs, imgs = buffer(n_good=4, corrupt_yaw=None)
    rvec, tvec = solve(objs, imgs)

    _, rms = S._reject_outlier_poses(objs, imgs, K, rvec, tvec)

    assert len(rms) == 4
    assert all(r >= 0.0 for r in rms)
