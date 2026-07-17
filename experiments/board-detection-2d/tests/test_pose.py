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
