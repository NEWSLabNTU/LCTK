"""Tests for lctk_quality.

The important ones are not the happy paths. They are the two traps this package exists to close,
both of which were live in earlier drafts of the design and both of which were caught only by
running against real data:

  * reprojection error INVERTS -- a degenerate capture scores a better RMSE than a good one;
  * resampled uncertainty INVERTS if you feed it frames instead of distinct placements -- a static
    board filmed nine times reports the most confident number in the suite.

If someone later "simplifies" this package down to reprojection error, or removes the placement
dedup, these tests fail. That is their job.
"""

import cv2
import numpy as np
import pytest
from lctk_quality import build_report, compute_spread, distinct_placements
from lctk_quality.resampling import MIN_PLACEMENTS_FOR_SPREAD
from scipy.spatial.transform import Rotation

K = np.array([[1164.6, 0, 950.1], [0, 1161.1, 538.6], [0, 0, 1]], dtype=np.float64)

# LiDAR (x fwd, y left, z up) -> camera (x right, y down, z fwd)
R_TRUE = np.array([[0.0, -1, 0], [0, 0, -1], [1, 0, 0]])
RVEC_TRUE, _ = cv2.Rodrigues(R_TRUE)
TVEC_TRUE = np.array([[0.07], [-0.02], [-0.89]])

#: board-local -> LiDAR axes, board facing the sensor
BASE = np.array([[0.0, 0, -1], [-1, 0, 0], [0, 1, 0]])


def board_corners():
    """16 ArUco corners on a 0.5 m patch, board-local, z = 0."""
    pts = []
    for cx, cy in [(-0.125, -0.125), (0.125, -0.125), (-0.125, 0.125), (0.125, 0.125)]:
        for dx, dy in [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)]:
            pts.append([cx + dx, cy + dy, 0.0])
    return np.array(pts)


def place(fwd, yaw=0.0, pitch=0.0, lat=0.0, height=0.0):
    """A board placement in the LiDAR frame. Returns (points, position, quaternion)."""
    Ry, _ = cv2.Rodrigues(np.array([0.0, 0.0, yaw]))
    Rp, _ = cv2.Rodrigues(np.array([0.0, pitch, 0.0]))
    R = Ry @ Rp @ BASE
    origin = np.array([fwd, lat, height])
    pts = (R @ board_corners().T).T + origin
    return pts, tuple(origin), tuple(Rotation.from_matrix(R).as_quat())


def observe(points, rng, icp_t=0.0, icp_r=0.0, px=0.1):
    """One frame: the image sees the TRUE board; the 3D points carry the ICP pose error."""
    img, _ = cv2.projectPoints(
        points.astype(np.float64), RVEC_TRUE, TVEC_TRUE, K, np.zeros(5)
    )
    img = img.reshape(-1, 2) + rng.normal(0, px, (len(points), 2))

    obj = points
    if icp_t or icp_r:
        dR, _ = cv2.Rodrigues(rng.normal(0, icp_r, 3))
        c = points.mean(0)
        obj = (dR @ (points - c).T).T + c + rng.normal(0, icp_t, 3)
    return obj.astype(np.float64), img.astype(np.float64)


def capture(placements, frames_each, rng, correlated=False, **noise):
    """Build a capture. `correlated=True` reuses ONE ICP error for every frame of a placement --
    which is what a real static board does, and what breaks naive resampling."""
    objs, imgs, poses = [], [], []
    for pts, pos, quat in placements:
        fixed = None
        for _ in range(frames_each):
            if correlated:
                if fixed is None:
                    fixed = observe(pts, rng, **noise)
                # Same 3D error every frame; only the pixel noise is fresh.
                o = fixed[0]
                i, _ = cv2.projectPoints(
                    pts.astype(np.float64), RVEC_TRUE, TVEC_TRUE, K, np.zeros(5)
                )
                i = (i.reshape(-1, 2) + rng.normal(0, 0.1, (len(pts), 2))).astype(
                    np.float64
                )
            else:
                o, i = observe(pts, rng, **noise)
            objs.append(o)
            imgs.append(i)
            poses.append((pos, quat))
    return objs, imgs, poses


def report_for(objs, imgs, poses):
    obj, img = np.vstack(objs), np.vstack(imgs)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, np.zeros(5), flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    return build_report(objs, imgs, poses, K, rvec, tvec)


def spread_poses():
    return [
        place(1.6 + 0.25 * i, yaw=y, pitch=p, lat=lt, height=h)
        for i, (y, p, lt, h) in enumerate(
            [
                (-0.5, 0.3, -1.1, 0.2),
                (0.4, -0.4, 0.9, -0.3),
                (-0.3, 0.45, 0.5, 0.4),
                (0.5, -0.2, -0.7, -0.2),
                (-0.45, -0.35, 1.0, 0.3),
                (0.35, 0.4, -0.9, -0.1),
                (-0.4, 0.25, 0.3, 0.5),
                (0.45, -0.45, -0.4, -0.4),
                (0.2, 0.35, 0.8, 0.1),
                (-0.25, -0.3, -1.0, 0.35),
            ]
        )
    ]


# ------------------------------------------------------------------ placement dedup (the fix)


def test_static_board_collapses_to_one_placement():
    """Nine frames of a board that never moves are ONE placement, not nine poses."""
    rng = np.random.default_rng(0)
    objs, imgs, poses = capture(
        [place(2.6)], frames_each=9, rng=rng, icp_t=0.01, icp_r=0.01
    )

    placements = distinct_placements(poses)

    assert len(placements) == 1
    assert placements[0].n_frames == 9


def test_distinct_placements_are_kept_apart():
    rng = np.random.default_rng(0)
    ps = [place(2.0, yaw=-0.5), place(3.2, yaw=0.5, pitch=0.4)]
    _, _, poses = capture(ps, frames_each=5, rng=rng)

    assert len(distinct_placements(poses)) == 2


# ------------------------------------------------- THE TRAP: resampling lies if fed raw frames


def test_resampling_refuses_when_placements_are_too_few():
    """The core guard. A static board filmed many times must NOT yield an uncertainty.

    Measured on real data, feeding those frames in as poses reports +/-0.22 deg / +/-9 mm -- the
    most confident number the suite can produce -- for a capture that cannot constrain the
    extrinsic at all.
    """
    rng = np.random.default_rng(1)
    objs, imgs, poses = capture(
        [place(2.6)], frames_each=9, rng=rng, correlated=True, icp_t=0.01, icp_r=0.01
    )

    report = report_for(objs, imgs, poses)

    assert report.n_frames == 9
    assert report.n_placements == 1
    assert report.spread is None, (
        "resampling produced an uncertainty from a single board placement; it is measuring "
        "duplication, not information"
    )
    assert report.is_degenerate


def test_resampling_over_raw_frames_would_be_falsely_confident():
    """Guard against the guard passing vacuously.

    Prove that the naive thing -- resampling over frames rather than placements -- really does
    manufacture confidence. If it did not, the test above would be meaningless.
    """
    rng = np.random.default_rng(2)
    objs, imgs, _ = capture(
        [place(2.6)], frames_each=9, rng=rng, correlated=True, icp_t=0.01, icp_r=0.01
    )

    naive = compute_spread(objs, imgs, K)  # frames, NOT placements -- the bug

    assert naive is not None
    assert naive.rot_deg < 1.0, (
        "expected the naive frame-based resampling to look falsely confident; if it does not, "
        "this test can no longer prove the dedup matters"
    )


# ---------------------------------------------------- THE OTHER TRAP: reprojection error inverts


def test_reprojection_error_inverts_and_must_not_be_ranked_on():
    """A degenerate capture scores a BETTER reprojection error than a well-conditioned one."""
    rng = np.random.default_rng(3)

    d_objs, d_imgs, d_poses = capture(
        [place(2.6)], frames_each=6, rng=rng, correlated=True, icp_t=0.01, icp_r=0.01
    )
    g_objs, g_imgs, g_poses = capture(
        spread_poses(), frames_each=1, rng=rng, icp_t=0.01, icp_r=0.01
    )

    degenerate = report_for(d_objs, d_imgs, d_poses)
    good = report_for(g_objs, g_imgs, g_poses)

    assert degenerate.residuals.rms_px < good.residuals.rms_px, (
        "the degenerate capture no longer scores a better RMSE; if reprojection error has stopped "
        "inverting, revisit the design -- but do NOT start ranking on it without new evidence"
    )
    # And yet the verdicts are right, because the verdict does not come from RMSE.
    assert degenerate.is_degenerate
    assert not good.is_degenerate


# ------------------------------------------------------------------------- diversity is the gate


def test_diversity_separates_where_everything_else_fails():
    rng = np.random.default_rng(4)

    d_objs, d_imgs, d_poses = capture(
        [place(2.6)], frames_each=6, rng=rng, correlated=True
    )
    g_objs, g_imgs, g_poses = capture(spread_poses(), frames_each=1, rng=rng)

    degenerate = report_for(d_objs, d_imgs, d_poses)
    good = report_for(g_objs, g_imgs, g_poses)

    assert degenerate.diversity.normal_span_deg < 10.0
    assert good.diversity.normal_span_deg > 30.0
    assert degenerate.diversity.is_degenerate
    assert not good.diversity.is_degenerate


def test_warnings_are_actionable():
    """A number nobody can act on is not a metric. The warning must say what to DO."""
    rng = np.random.default_rng(5)
    objs, imgs, poses = capture([place(2.6)], frames_each=5, rng=rng, correlated=True)

    report = report_for(objs, imgs, poses)
    text = " ".join(report.warnings()).lower()

    assert report.warnings()
    assert "placement" in text
    assert "reprojection error" in text and "not evidence" in text


def test_status_line_is_one_line_and_carries_the_verdict():
    rng = np.random.default_rng(6)
    objs, imgs, poses = capture(spread_poses(), frames_each=1, rng=rng)

    line = report_for(objs, imgs, poses).status_line()

    assert "\n" not in line
    assert line.startswith("OK") or line.startswith("DEGENERATE")
    assert "placements" in line


# ------------------------------------------------------------------------------ the Jacobian claim


def test_projectpoints_jacobian_first_six_columns_are_the_extrinsic_derivatives():
    """The whole conditioning module rests on this. Pin it against finite differences."""
    rng = np.random.default_rng(7)
    obj = rng.uniform(-0.25, 0.25, (16, 3))
    obj[:, 2] = 0.0

    img, jac = cv2.projectPoints(obj, RVEC_TRUE, TVEC_TRUE, K, np.zeros(5))

    eps = 1e-7
    for col, (vec, idx) in enumerate(
        [
            (RVEC_TRUE, 0),
            (RVEC_TRUE, 1),
            (RVEC_TRUE, 2),
            (TVEC_TRUE, 0),
            (TVEC_TRUE, 1),
            (TVEC_TRUE, 2),
        ]
    ):
        bumped = vec.copy()
        bumped[idx] += eps
        args = (obj, bumped, TVEC_TRUE) if col < 3 else (obj, RVEC_TRUE, bumped)
        img2, _ = cv2.projectPoints(*args, K, np.zeros(5))
        numeric = ((img2 - img) / eps).reshape(-1)
        assert np.abs(numeric - jac[:, col]).max() < 1e-4


def test_spread_needs_enough_placements():
    rng = np.random.default_rng(8)
    ps = spread_poses()[: MIN_PLACEMENTS_FOR_SPREAD - 1]
    objs, imgs, _ = capture(ps, frames_each=1, rng=rng)

    assert compute_spread(objs, imgs, K) is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
