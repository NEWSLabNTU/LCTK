import numpy as np
import pytest

from boarddet.detector import detect
from boarddet.presets import production_config
from boarddet.sim import Rect, Vlp32cSensor, make_diamond_board, render


def _scene_with_board(center, normal, side=1.0):
    ground = Rect(center=np.array([0.0, 0.0, -1.2]),
                  normal=np.array([0.0, 0.0, 1.0]),
                  u_axis=np.array([1.0, 0.0, 0.0]),
                  half_u=20.0, half_v=20.0)
    rect, corners = make_diamond_board(center, normal, up_hint=[0.0, 0.0, 1.0],
                                       side=side)
    return [ground, rect], corners


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_sim_pose_accuracy(seed):
    rng = np.random.default_rng(seed)
    sensor = Vlp32cSensor()
    center = np.array([4.0, 0.0, 0.2])
    normal = np.array([-1.0, 0.0, 0.0])  # faces sensor at origin
    scene, truth_corners = _scene_with_board(center, normal, side=1.0)
    frame = render(scene, sensor, range_noise_std=0.01,
                   dropout_grazing=0.1, dropout_random=0.01, rng=rng)

    out = detect(frame.points, production_config(), generator="b")
    assert out.detection is not None, "board not detected in clean sim scene"
    det = out.detection
    # Center within a few cm.
    assert np.linalg.norm(det.center - center) < 0.08
    # Normal aligned (sign-invariant, since sensor-facing vs truth may differ
    # in sign convention).
    truth_n = normal / np.linalg.norm(normal)
    assert abs(det.rotation[:, 2] @ truth_n) > 0.98
    # Every detected corner matches some truth corner within ~10 cm.
    for c in det.corners_3d:
        assert np.linalg.norm(truth_corners - c, axis=1).min() < 0.10
