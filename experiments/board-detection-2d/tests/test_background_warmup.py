import numpy as np

from boarddet.background import BackgroundModel
from boarddet.detector import detect
from boarddet.presets import production_config
from boarddet.sim import Rect, Vlp32cSensor, make_diamond_board, render


def _empty_room():
    ground = Rect(center=np.array([0.0, 0.0, -1.2]),
                  normal=np.array([0.0, 0.0, 1.0]),
                  u_axis=np.array([1.0, 0.0, 0.0]),
                  half_u=20.0, half_v=20.0)
    wall = Rect(center=np.array([10.0, 0.0, 0.5]),
                normal=np.array([-1.0, 0.0, 0.0]),
                u_axis=np.array([0.0, 1.0, 0.0]),
                half_u=8.0, half_v=3.0)
    clutter = Rect(center=np.array([3.0, 3.0, 0.0]),
                   normal=np.array([-1.0, -1.0, 0.0]) / np.sqrt(2),
                   u_axis=np.array([1.0, -1.0, 0.0]) / np.sqrt(2),
                   half_u=0.5, half_v=0.5)
    return [ground, wall, clutter]


def test_single_session_warmup_then_detect():
    sensor = Vlp32cSensor()
    room = _empty_room()

    # Warm-up: observe several board-FREE frames as one live source.
    bg = BackgroundModel(voxel=0.06, dilation_radius=1, min_sources=1)
    for seed in range(5):
        rng = np.random.default_rng(seed)
        frame = render(room, sensor, range_noise_std=0.01,
                       dropout_random=0.01, rng=rng)
        bg.observe(frame.points, source="live")
    bg.finalize()
    assert bg.n_voxels > 0

    # Reveal the board: same room + a diamond board walked in.
    center = np.array([4.0, 0.0, 0.2])
    rect, _ = make_diamond_board(center, np.array([-1.0, 0.0, 0.0]),
                                 up_hint=[0.0, 0.0, 1.0], side=1.0)
    rng = np.random.default_rng(99)
    frame = render(room + [rect], sensor, range_noise_std=0.01,
                   dropout_random=0.01, rng=rng)

    out = detect(frame.points, production_config(), generator="e", background=bg)
    assert out.detection is not None, "board not found via warmup background"
    assert np.linalg.norm(out.detection.center - center) < 0.15
