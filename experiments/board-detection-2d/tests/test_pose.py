import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.geometry import fit_plane, project_to_plane
from boarddet.pose import board_pose
from boarddet.scorer import score_candidate
from boarddet.synth import make_board


def test_pose_recovers_truth():
    rng = np.random.default_rng(6)
    pts, truth = make_board(
        side=1.0, center=np.array([4.0, 0.5, 0.3]),
        normal=np.array([-1.0, 0.15, 0.05]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=0.02, noise=0.004, rng=rng,
    )
    plane = fit_plane(pts)
    res = score_candidate(project_to_plane(pts, plane), BoardConfig())
    det = board_pose(plane, res)
    assert np.linalg.norm(det.center - truth.center) < 0.02
    assert abs(det.rotation[:, 2] @ truth.normal) > 0.999
    # rotation is orthonormal, right-handed
    r = det.rotation
    np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-9)
    assert np.linalg.det(r) > 0.99
    # corners_3d match truth corners as a set
    for c in det.corners_3d:
        assert np.linalg.norm(truth.corners - c, axis=1).min() < 0.03


def _square_scoreresult(corners_2d):
    # Minimal ScoreResult carrying only what board_pose reads.
    from boarddet.scorer import ScoreResult
    return ScoreResult(
        score=1.0, corners_2d=np.asarray(corners_2d, dtype=float),
        side_lengths=np.ones(4), fill_ratio=1.0, angle_err_deg=0.0,
        raster=np.zeros((1, 1), dtype=np.uint8), origin=np.zeros(2),
        cell_m=0.02,
    )


def test_board_pose_x_axis_follows_up_axis():
    from boarddet.geometry import PlaneModel
    from boarddet.pose import board_pose
    # Plane in the x=const plane: u=+y, v=+z, normal=+x (points away from
    # sensor at origin -> board_pose must flip it to -x).
    plane = PlaneModel(center=np.array([4.0, 0.0, 0.0]),
                       normal=np.array([1.0, 0.0, 0.0]),
                       u=np.array([0.0, 1.0, 0.0]),
                       v=np.array([0.0, 0.0, 1.0]))
    # A diamond: corners up/down/left/right in (u,v). Up corner is +v.
    corners_2d = np.array([[0.0, 0.7], [0.7, 0.0], [0.0, -0.7], [-0.7, 0.0]])
    det = board_pose(plane, _square_scoreresult(corners_2d),
                     up=np.array([0.0, 0.0, 1.0]))
    # X axis (col 0) must point from center toward the highest (max-z) corner.
    top = det.corners_3d[np.argmax(det.corners_3d @ np.array([0., 0., 1.]))]
    x_expected = top - det.center
    x_expected = x_expected / np.linalg.norm(x_expected)
    assert det.rotation[:, 0] @ x_expected > 0.999
    # Normal (col 2) must face the sensor at the origin: normal . (-center) > 0
    assert det.rotation[:, 2] @ (-det.center) > 0.0


def test_board_pose_uses_given_up_not_world_z():
    from boarddet.geometry import PlaneModel
    from boarddet.pose import board_pose
    # z-forward rig: gravity along +y. Board faces the sensor along +z-ish.
    plane = PlaneModel(center=np.array([0.0, 0.0, 4.0]),
                       normal=np.array([0.0, 0.0, 1.0]),
                       u=np.array([1.0, 0.0, 0.0]),
                       v=np.array([0.0, 1.0, 0.0]))
    corners_2d = np.array([[0.0, 0.7], [0.7, 0.0], [0.0, -0.7], [-0.7, 0.0]])
    det = board_pose(plane, _square_scoreresult(corners_2d),
                     up=np.array([0.0, 1.0, 0.0]))
    up = np.array([0.0, 1.0, 0.0])
    top = det.corners_3d[np.argmax(det.corners_3d @ up)]
    x_expected = top - det.center
    x_expected = x_expected / np.linalg.norm(x_expected)
    assert det.rotation[:, 0] @ x_expected > 0.999


def test_board_pose_winding_is_canonical_ccw():
    from boarddet.geometry import PlaneModel
    from boarddet.pose import board_pose
    plane = PlaneModel(center=np.array([4.0, 0.0, 0.0]),
                       normal=np.array([1.0, 0.0, 0.0]),
                       u=np.array([0.0, 1.0, 0.0]),
                       v=np.array([0.0, 0.0, 1.0]))
    corners_2d = np.array([[0.0, 0.7], [0.7, 0.0], [0.0, -0.7], [-0.7, 0.0]])
    # Same geometry, corners handed in scrambled order:
    scrambled = corners_2d[[2, 0, 3, 1]]
    det_a = board_pose(plane, _square_scoreresult(corners_2d))
    det_b = board_pose(plane, _square_scoreresult(scrambled))
    # Canonical ordering => identical corner sequence regardless of input order.
    np.testing.assert_allclose(det_a.corners_3d, det_b.corners_3d, atol=1e-9)
    # And it is CCW about the (sensor-facing) normal: signed area > 0 in the
    # (board_x, board_y) basis.
    r = det_a.rotation
    xy = (det_a.corners_3d - det_a.center) @ r[:, :2]
    area = 0.5 * np.sum(xy[:, 0] * np.roll(xy[:, 1], -1)
                        - np.roll(xy[:, 0], -1) * xy[:, 1])
    assert area > 0.0
