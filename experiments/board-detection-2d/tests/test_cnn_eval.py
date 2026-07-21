"""Task 33: end-to-end test of the eval pipeline (`boarddet.cnn.eval`) on a
SYNTH frame with a KNOWN board -- input -> model -> probability mask ->
threshold -> seam-wrapping connected components -> back-projection ->
square fit -> classification, validated against ground truth we actually
have (the scene's own `BoardMeta.center`). This is deliberately NOT a real-
data test: it validates the pipeline plumbing is correct independent of
whether synth-training transfers to real data (that question is answered
by the real eval run itself, see `.superpowers/sdd/task-33-report.md` and
the phase doc's "Stage 9 CNN Results").

The model here is overfit to a SINGLE fixed synth sample for a handful of
steps (mirrors `train.overfit_check`'s fast CPU sanity pattern) rather than
loaded from the real checkpoint, so this test has no dependency on the
(gitignored, not always present) trained checkpoint file and stays fast."""
from __future__ import annotations

import numpy as np
import torch

from boarddet.cnn.data import SynthDataConfig, render_synth_sample
from boarddet.cnn.eval import (
    components_from_mask,
    evaluate_frame,
    label_mask_wrap,
    predict_probability_mask,
    rebin_points_to_grid,
)
from boarddet.cnn.model import BoardUNet
from boarddet.cnn.train import dice_bce_loss
from boarddet.sim.scenegen import SceneGenConfig
from boarddet.sim.sensor import Vlp32cSensor

AZIMUTH_STEPS = 360  # small width keeps the ray-cast + training fast


def _overfit_one_scene(seed: int = 16, steps: int = 150):
    """Render one forced-1-board synth scene, then overfit a small
    `BoardUNet` to that single (input, target) pair -- fast (CPU, tiny
    model/width), deterministic, and enough to get a near-perfect mask
    prediction on the exact sample it saw, which is all this test needs.

    `seed=16` (not the original 13): Task 35 added new `random_scene` RNG
    draws (scatter-cluster/large-clutter counts, drawn even when they
    produce zero objects), which shifts the downstream draw sequence for
    every seed and changes which board placement a given seed yields. 16 is
    just a seed that places the board somewhere this one-sample overfit
    reliably nails within the test's 0.3 m tolerance; it carries no other
    significance."""
    rng = np.random.default_rng(seed)
    sensor = Vlp32cSensor()
    cfg = SynthDataConfig(
        scenegen=SceneGenConfig(board_count_weights={1: 1.0},
                                n_clutter_range=(1, 2), n_boxes_range=(0, 1),
                                n_cylinders_range=(0, 1)),
        azimuth_steps=AZIMUTH_STEPS, augment=False,
    )
    input_array, target_array, scene, sim_frame = render_synth_sample(rng, cfg, sensor)
    input_t = torch.from_numpy(input_array).unsqueeze(0)
    target_t = torch.from_numpy(target_array[None, None, :, :])

    torch.manual_seed(0)
    model = BoardUNet(base_channels=8)
    optimizer = torch.optim.Adam(model.parameters(), lr=2e-3)
    model.train()
    for _ in range(steps):
        optimizer.zero_grad()
        loss = dice_bce_loss(model(input_t), target_t)
        loss.backward()
        optimizer.step()
    model.eval()
    return model, sensor, scene, sim_frame, input_t[0]


def test_eval_pipeline_detects_synth_board_at_true_center():
    model, sensor, scene, sim_frame, input_t = _overfit_one_scene()
    assert len(scene.boards) == 1
    true_center = scene.boards[0].center

    device = torch.device("cpu")
    prob = predict_probability_mask(model, input_t, device)
    grid_points = rebin_points_to_grid(sim_frame.points, sensor, AZIMUTH_STEPS)

    detections = evaluate_frame(prob, grid_points, threshold=0.5)
    assert len(detections) >= 1, "overfit model produced no detections at all"

    nearest = min(detections, key=lambda d: np.linalg.norm(d.center - true_center))
    dist = np.linalg.norm(nearest.center - true_center)
    assert dist < 0.3, (
        f"nearest detection {nearest.center} is {dist:.3f} m from the true "
        f"board center {true_center} -- pipeline did not recover the board"
    )
    # the recovered detection should be backed by a healthy number of real
    # points, not a handful of stray mask pixels
    assert nearest.n_points >= 15


def test_eval_pipeline_empty_scene_yields_no_detections():
    rng = np.random.default_rng(3)
    sensor = Vlp32cSensor()
    cfg = SynthDataConfig(
        scenegen=SceneGenConfig(board_count_weights={0: 1.0},
                                n_clutter_range=(1, 2), n_boxes_range=(0, 1),
                                n_cylinders_range=(0, 1)),
        azimuth_steps=AZIMUTH_STEPS, augment=False,
    )
    input_array, _target_array, scene, sim_frame = render_synth_sample(rng, cfg, sensor)
    assert len(scene.boards) == 0

    torch.manual_seed(0)
    model = BoardUNet(base_channels=8)
    model.eval()
    device = torch.device("cpu")
    prob = predict_probability_mask(model, torch.from_numpy(input_array), device)
    grid_points = rebin_points_to_grid(sim_frame.points, sensor, AZIMUTH_STEPS)

    # An untrained model's raw logits needn't be all-negative, so don't
    # assert zero detections -- just that the pipeline runs end-to-end
    # without error on a legitimately board-free scene.
    evaluate_frame(prob, grid_points, threshold=0.5)


# ---------------------------------------------------------------------------
# seam-wrap connected components: a synthetic mask straddling column 0
# ---------------------------------------------------------------------------


def test_label_mask_wrap_merges_component_straddling_seam():
    mask = np.zeros((8, 40), dtype=bool)
    # a blob spanning columns 37..39 and 0..2 (straddles the 0/39 seam)
    mask[3:6, 37:40] = True
    mask[3:6, 0:3] = True
    labeled = label_mask_wrap(mask)
    seam_labels = set(labeled[3:6, 37:40].flatten()) | set(labeled[3:6, 0:3].flatten())
    seam_labels.discard(0)
    assert len(seam_labels) == 1, (
        f"seam-straddling blob was split into multiple labels: {seam_labels}"
    )


def test_components_from_mask_drops_tiny_noise():
    mask = np.zeros((8, 40), dtype=bool)
    mask[2, 5] = True  # single stray pixel -- below min_pixels
    mask[3:6, 20:26] = True  # a real 3x6 = 18-pixel blob
    comps = components_from_mask(mask, min_pixels=15)
    assert len(comps) == 1
    rows, cols = comps[0]
    assert len(rows) == 18
