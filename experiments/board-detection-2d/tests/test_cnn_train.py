"""Tests for the Task 32 training loop (`boarddet.cnn.train`): the Dice+BCE
loss's edge cases (perfect prediction, inverted prediction, all-zero/empty-
scene target), the overfit-one-batch sanity check (loss must decrease), and
a full end-to-end smoke run of `run_training` with a tiny config. The real,
full training run is a script (`uv run python -m boarddet.cnn.train`), not a
unit test -- see `results/stage9-cnn/` and `.superpowers/sdd/task-32-
report.md` for that run's outcome."""
from __future__ import annotations

import torch

from boarddet.cnn.train import (
    MaskCounts,
    TrainConfig,
    dice_bce_loss,
    overfit_check,
    run_training,
)

# ---------------------------------------------------------------------------
# Dice + BCE loss
# ---------------------------------------------------------------------------


def test_loss_near_zero_for_perfect_prediction():
    torch.manual_seed(0)
    target = (torch.rand(2, 1, 8, 16) > 0.7).float()
    # huge-magnitude logits matching the target sign -> sigmoid saturates to
    # ~0/~1, i.e. a near-perfect prediction.
    logits = (target * 2.0 - 1.0) * 20.0
    loss = dice_bce_loss(logits, target)
    assert torch.isfinite(loss)
    assert loss.item() < 0.01


def test_loss_high_for_inverted_prediction():
    torch.manual_seed(1)
    target = (torch.rand(2, 1, 8, 16) > 0.7).float()
    logits = (target * 2.0 - 1.0) * -20.0  # confidently WRONG everywhere
    loss = dice_bce_loss(logits, target)
    perfect_loss = dice_bce_loss((target * 2.0 - 1.0) * 20.0, target)
    assert loss.item() > perfect_loss.item()
    assert loss.item() > 1.0


def test_loss_handles_all_zero_target_without_nan():
    target = torch.zeros(3, 1, 8, 16)
    logits = torch.randn(3, 1, 8, 16)
    loss = dice_bce_loss(logits, target)
    assert torch.isfinite(loss)
    assert loss.item() >= 0.0

    # even a confidently-wrong (all-positive) prediction on an empty scene
    # must stay finite, not blow up to inf/NaN.
    confident_wrong = torch.full((3, 1, 8, 16), 20.0)
    loss_wrong = dice_bce_loss(confident_wrong, target)
    assert torch.isfinite(loss_wrong)
    assert loss_wrong.item() > loss.item()


def test_mask_counts_iou_precision_recall():
    counts = MaskCounts()
    target = torch.tensor([[[[1.0, 0.0, 1.0, 0.0]]]])
    logits = torch.tensor([[[[10.0, 10.0, -10.0, -10.0]]]])  # pred: [1,1,0,0]
    counts.update(logits, target, threshold=0.5)
    # tp=1 (col0), fp=1 (col1), fn=1 (col2), tn=1 (col3)
    assert counts.tp == 1.0
    assert counts.fp == 1.0
    assert counts.fn == 1.0
    assert counts.tn == 1.0
    assert abs(counts.iou() - (1.0 / 3.0)) < 1e-6
    assert abs(counts.precision() - 0.5) < 1e-6
    assert abs(counts.recall() - 0.5) < 1e-6


# ---------------------------------------------------------------------------
# overfit-one-batch sanity
# ---------------------------------------------------------------------------


def test_overfit_one_batch_loss_decreases():
    losses = overfit_check(steps=25, batch_size=4, azimuth_steps=64, device="cpu")
    assert len(losses) == 25
    assert all(l == l for l in losses), "NaN in overfit loss curve"  # noqa: E741
    # allow noisy individual steps but require clear net progress: the mean
    # of the last few steps must be well below the mean of the first few.
    early = sum(losses[:5]) / 5
    late = sum(losses[-5:]) / 5
    assert late < early, f"loss did not decrease: early={early:.4f} late={late:.4f}"


# ---------------------------------------------------------------------------
# end-to-end smoke run
# ---------------------------------------------------------------------------


def test_run_training_smoke(tmp_path):
    cfg = TrainConfig(
        batch_size=4, num_workers=0, train_epoch_size=8, val_size=8,
        max_epochs=2, patience=2, azimuth_steps=64, device="cpu",
        checkpoint_path=tmp_path / "board_unet.pt",
        curve_path=tmp_path / "train_curve.png",
        val_pred_path=tmp_path / "val_pred.png",
    )
    result = run_training(cfg)

    assert result.n_params < 2_000_000
    assert len(result.history) == 2
    assert 0.0 <= result.best_iou <= 1.0
    assert result.checkpoint_path == cfg.checkpoint_path
    assert cfg.checkpoint_path.exists()
    assert cfg.curve_path.exists()
    assert cfg.val_pred_path.exists()

    checkpoint = torch.load(cfg.checkpoint_path, weights_only=False)
    assert "model_state_dict" in checkpoint
    assert checkpoint["board_hollow_prob"] == cfg.board_hollow_prob
