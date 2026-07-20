"""Task 32: Dice+BCE loss, training loop, and validation for `BoardUNet`.

CRITICAL training-data decision (bake-in, do not revert to the sim's
plain-only default): the real eval data (Task 33) is the HOLLOW calibration
board, but the deployment target is a PLAIN diamond. `TrainConfig.
board_hollow_prob` therefore defaults to 0.5 -- a 50/50 mix of plain and
hollow boards -- NOT `SceneGenConfig`'s own default of 0.0 (plain-only).
Training on the mix makes the real-hollow eval a valid test of the model
AND keeps it correct for the plain-board deployment target. Every dataset
built by this module (train AND the fixed val set) goes through this same
`board_hollow_prob`, so validation is drawn from the identical mix the
model is trained on.

Two ways to run:
  - `uv run python -m boarddet.cnn.train` -- full training run, writes a
    checkpoint + curve + val-prediction PNG under `results/stage9-cnn/`
    (gitignored).
  - `uv run python -m boarddet.cnn.train --smoke` -- a few tiny steps
    (small batch, small W, 1-2 epochs, no plotting) so `just test`/CI can
    exercise the whole loop in well under a second; this is also what
    `tests/test_cnn_train.py`'s smoke test drives directly via
    `run_training`, not the CLI.
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import numpy as np
import torch
import torch.nn.functional as F
from torch import nn, optim
from torch.utils.data import DataLoader

from .data import SynthBoardDataset, SynthDataConfig
from .model import BoardUNet, count_parameters
from ..sim.range_image import DEFAULT_AZIMUTH_STEPS
from ..sim.scenegen import SceneGenConfig

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (backend must be set first)

_RESULTS_DIR = Path(__file__).resolve().parents[3] / "results" / "stage9-cnn"


# ---------------------------------------------------------------------------
# Dice + BCE loss
# ---------------------------------------------------------------------------


def dice_loss(logits: torch.Tensor, target: torch.Tensor, eps: float = 1.0) -> torch.Tensor:
    """Soft Dice loss over `sigmoid(logits)` vs. `target` (both `(B,1,H,W)`),
    per-sample then batch-averaged. `eps=1.0` (a full-pixel Laplace smooth,
    not a numerical-stability epsilon) is deliberate: it's what keeps an
    empty (all-zero-target) scene well-defined and low-loss when the model
    correctly predicts nothing -- with the tiny `eps=1e-6` conventional in
    some Dice implementations, an empty scene's `intersection=union=0`
    dice ratio is `eps/eps == 1.0` in the limit but is numerically fragile
    (the ratio of two ~1e-6 floats); `eps=1.0` makes that scene's dice loss
    smoothly -> 0 as the prediction -> all-zero, with no NaN/Inf risk.
    """
    probs = torch.sigmoid(logits)
    dims = (1, 2, 3)
    intersection = (probs * target).sum(dim=dims)
    union = probs.sum(dim=dims) + target.sum(dim=dims)
    dice = (2.0 * intersection + eps) / (union + eps)
    return 1.0 - dice.mean()


def dice_bce_loss(logits: torch.Tensor, target: torch.Tensor,
                  bce_weight: float = 0.5, dice_weight: float = 0.5) -> torch.Tensor:
    """`bce_weight * BCE(logits, target) + dice_weight * dice_loss(...)`.
    BCE supervises every pixel (handles the empty-scene all-negative case
    directly); Dice counters the board-is-a-small-minority-of-pixels class
    imbalance BCE alone underweights."""
    bce = F.binary_cross_entropy_with_logits(logits, target)
    dice = dice_loss(logits, target)
    return bce_weight * bce + dice_weight * dice


# ---------------------------------------------------------------------------
# metrics: accumulated globally over a validation pass (per-image IoU is
# undefined -- 0/0 -- for the ~15% of scenes with zero boards, so summing
# tp/fp/fn across the whole validation set and dividing once at the end
# avoids that, rather than averaging a per-image ratio).
# ---------------------------------------------------------------------------


@dataclass
class MaskCounts:
    tp: float = 0.0
    fp: float = 0.0
    fn: float = 0.0
    tn: float = 0.0

    def update(self, logits: torch.Tensor, target: torch.Tensor, threshold: float) -> None:
        pred = (torch.sigmoid(logits) >= threshold).float()
        t = target
        self.tp += float((pred * t).sum().item())
        self.fp += float((pred * (1.0 - t)).sum().item())
        self.fn += float(((1.0 - pred) * t).sum().item())
        self.tn += float(((1.0 - pred) * (1.0 - t)).sum().item())

    def iou(self) -> float:
        denom = self.tp + self.fp + self.fn
        return self.tp / denom if denom > 0 else 1.0

    def precision(self) -> float:
        denom = self.tp + self.fp
        return self.tp / denom if denom > 0 else 1.0

    def recall(self) -> float:
        denom = self.tp + self.fn
        return self.tp / denom if denom > 0 else 1.0


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------


@dataclass
class TrainConfig:
    # THE bake-in (see module docstring): mix plain + hollow boards so the
    # model transfers to both the hollow real-eval board and the plain
    # deployment board. NOT SceneGenConfig's own 0.0 (plain-only) default.
    board_hollow_prob: float = 0.5
    azimuth_steps: int = DEFAULT_AZIMUTH_STEPS

    batch_size: int = 64
    lr: float = 1e-3
    num_workers: int = 12

    train_epoch_size: int = 4096   # synth samples drawn per "epoch"
    val_size: int = 512            # fixed held-out synth samples
    train_seed: int = 0
    val_seed: int = 999_000_000    # far from train_seed's range -- no overlap

    max_epochs: int = 60
    patience: int = 10             # early stop: epochs without val-IoU improvement
    lr_patience: int = 3           # ReduceLROnPlateau patience (val IoU, maximize)
    lr_factor: float = 0.5
    threshold: float = 0.5

    device: str = "cuda" if torch.cuda.is_available() else "cpu"
    base_channels: int = 16

    checkpoint_path: Path = field(default_factory=lambda: _RESULTS_DIR / "board_unet.pt")
    curve_path: Path = field(default_factory=lambda: _RESULTS_DIR / "train_curve.png")
    val_pred_path: Path = field(default_factory=lambda: _RESULTS_DIR / "val_pred.png")
    save_artifacts: bool = True


@dataclass
class TrainResult:
    best_epoch: int
    best_iou: float
    best_precision: float
    best_recall: float
    history: list[dict]
    checkpoint_path: Path | None
    n_params: int


# ---------------------------------------------------------------------------
# dataset / dataloader construction
# ---------------------------------------------------------------------------


def _scenegen(cfg: TrainConfig) -> SceneGenConfig:
    return SceneGenConfig(board_hollow_prob=cfg.board_hollow_prob)


def build_datasets(cfg: TrainConfig) -> tuple[SynthBoardDataset, SynthBoardDataset]:
    train_ds = SynthBoardDataset(SynthDataConfig(
        scenegen=_scenegen(cfg), azimuth_steps=cfg.azimuth_steps,
        augment=True, base_seed=cfg.train_seed, virtual_size=cfg.train_epoch_size,
    ))
    val_ds = SynthBoardDataset(SynthDataConfig(
        scenegen=_scenegen(cfg), azimuth_steps=cfg.azimuth_steps,
        augment=False, base_seed=cfg.val_seed, virtual_size=cfg.val_size,
    ))
    return train_ds, val_ds


def build_dataloaders(cfg: TrainConfig, train_ds: SynthBoardDataset,
                      val_ds: SynthBoardDataset) -> tuple[DataLoader, DataLoader]:
    train_loader = DataLoader(
        train_ds, batch_size=cfg.batch_size, shuffle=True,
        num_workers=cfg.num_workers, drop_last=True,
        persistent_workers=False,  # workers respawn each epoch -> pick up set_epoch()
    )
    val_loader = DataLoader(
        val_ds, batch_size=cfg.batch_size, shuffle=False,
        num_workers=min(cfg.num_workers, 4),
    )
    return train_loader, val_loader


# ---------------------------------------------------------------------------
# train / eval steps
# ---------------------------------------------------------------------------


def train_epoch(model: nn.Module, loader: DataLoader, optimizer: optim.Optimizer,
                device: torch.device) -> float:
    model.train()
    total_loss = 0.0
    n = 0
    for inputs, targets in loader:
        inputs = inputs.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)
        optimizer.zero_grad()
        logits = model(inputs)
        loss = dice_bce_loss(logits, targets)
        loss.backward()
        optimizer.step()
        total_loss += float(loss.item()) * inputs.shape[0]
        n += inputs.shape[0]
    return total_loss / n if n else 0.0


@torch.no_grad()
def evaluate(model: nn.Module, loader: DataLoader, device: torch.device,
            threshold: float = 0.5) -> dict:
    model.eval()
    counts = MaskCounts()
    total_loss = 0.0
    n = 0
    for inputs, targets in loader:
        inputs = inputs.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)
        logits = model(inputs)
        loss = dice_bce_loss(logits, targets)
        total_loss += float(loss.item()) * inputs.shape[0]
        n += inputs.shape[0]
        counts.update(logits, targets, threshold)
    return {
        "loss": total_loss / n if n else 0.0,
        "iou": counts.iou(),
        "precision": counts.precision(),
        "recall": counts.recall(),
    }


def overfit_check(steps: int = 30, batch_size: int = 4, azimuth_steps: int = 64,
                  device: str = "cpu", lr: float = 1e-3) -> list[float]:
    """Overfit-ONE-fixed-batch sanity check: draw a single batch once, run
    `steps` optimizer updates on that SAME batch, return the per-step loss.
    Fast on CPU (small `azimuth_steps`) -- this is what
    `tests/test_cnn_train.py`'s smoke test asserts is monotonically-ish
    decreasing, independent of the full data-generation/DataLoader plumbing."""
    torch.manual_seed(0)
    device_t = torch.device(device)
    model = BoardUNet().to(device_t)
    optimizer = optim.Adam(model.parameters(), lr=lr)

    ds = SynthBoardDataset(SynthDataConfig(
        scenegen=SceneGenConfig(board_hollow_prob=0.5, board_count_weights={1: 1.0}),
        azimuth_steps=azimuth_steps, augment=False, virtual_size=batch_size,
    ))
    inputs = torch.stack([ds[i][0] for i in range(batch_size)]).to(device_t)
    targets = torch.stack([ds[i][1] for i in range(batch_size)]).to(device_t)

    losses = []
    model.train()
    for _ in range(steps):
        optimizer.zero_grad()
        logits = model(inputs)
        loss = dice_bce_loss(logits, targets)
        loss.backward()
        optimizer.step()
        losses.append(float(loss.item()))
    return losses


# ---------------------------------------------------------------------------
# plotting
# ---------------------------------------------------------------------------


def plot_curve(history: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    epochs = [h["epoch"] for h in history]
    fig, (ax_loss, ax_iou) = plt.subplots(1, 2, figsize=(10, 4))
    ax_loss.plot(epochs, [h["train_loss"] for h in history], label="train")
    ax_loss.plot(epochs, [h["val_loss"] for h in history], label="val")
    ax_loss.set_xlabel("epoch")
    ax_loss.set_ylabel("dice+bce loss")
    ax_loss.legend()
    ax_loss.set_title("loss")

    ax_iou.plot(epochs, [h["val_iou"] for h in history], label="IoU")
    ax_iou.plot(epochs, [h["val_precision"] for h in history], label="precision")
    ax_iou.plot(epochs, [h["val_recall"] for h in history], label="recall")
    ax_iou.set_xlabel("epoch")
    ax_iou.set_ylabel("score")
    ax_iou.legend()
    ax_iou.set_title("validation (fixed held-out synth)")

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


@torch.no_grad()
def plot_val_predictions(model: nn.Module, val_ds: SynthBoardDataset,
                         device: torch.device, path: Path,
                         n_samples: int = 4, threshold: float = 0.5) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    model.eval()
    fig, axes = plt.subplots(n_samples, 3, figsize=(14, 2.2 * n_samples))
    if n_samples == 1:
        axes = axes[None, :]
    for i in range(n_samples):
        input_t, target_t = val_ds[i]
        logits = model(input_t.unsqueeze(0).to(device))
        pred = (torch.sigmoid(logits) >= threshold).float().cpu()[0, 0]

        axes[i, 0].imshow(input_t[0].numpy(), aspect="auto", cmap="viridis")
        axes[i, 0].set_title(f"sample {i}: normalized range")
        axes[i, 1].imshow(target_t[0].numpy(), aspect="auto", cmap="gray", vmin=0, vmax=1)
        axes[i, 1].set_title("target mask")
        axes[i, 2].imshow(pred.numpy(), aspect="auto", cmap="gray", vmin=0, vmax=1)
        axes[i, 2].set_title("predicted mask")
        for ax in axes[i]:
            ax.set_xticks([])
            ax.set_yticks([])

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# full run
# ---------------------------------------------------------------------------


def run_training(cfg: TrainConfig) -> TrainResult:
    device = torch.device(cfg.device)
    model = BoardUNet(base_channels=cfg.base_channels).to(device)
    n_params = count_parameters(model)

    optimizer = optim.Adam(model.parameters(), lr=cfg.lr)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="max", factor=cfg.lr_factor, patience=cfg.lr_patience,
    )

    train_ds, val_ds = build_datasets(cfg)
    train_loader, val_loader = build_dataloaders(cfg, train_ds, val_ds)

    history: list[dict] = []
    best_iou = -1.0
    best_epoch = -1
    best_precision = 0.0
    best_recall = 0.0
    best_state: dict | None = None
    epochs_since_improvement = 0

    for epoch in range(cfg.max_epochs):
        train_ds.set_epoch(epoch)
        t0 = time.time()
        train_loss = train_epoch(model, train_loader, optimizer, device)
        val_metrics = evaluate(model, val_loader, device, cfg.threshold)
        scheduler.step(val_metrics["iou"])
        dt = time.time() - t0

        history.append({
            "epoch": epoch, "train_loss": train_loss,
            "val_loss": val_metrics["loss"], "val_iou": val_metrics["iou"],
            "val_precision": val_metrics["precision"], "val_recall": val_metrics["recall"],
            "seconds": dt,
        })
        print(
            f"epoch {epoch:3d}  train_loss={train_loss:.4f}  "
            f"val_loss={val_metrics['loss']:.4f}  val_iou={val_metrics['iou']:.4f}  "
            f"val_precision={val_metrics['precision']:.4f}  "
            f"val_recall={val_metrics['recall']:.4f}  ({dt:.1f}s)"
        )

        if val_metrics["iou"] > best_iou:
            best_iou = val_metrics["iou"]
            best_precision = val_metrics["precision"]
            best_recall = val_metrics["recall"]
            best_epoch = epoch
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
            epochs_since_improvement = 0
        else:
            epochs_since_improvement += 1
            if epochs_since_improvement >= cfg.patience:
                print(f"early stopping at epoch {epoch} (no val_iou improvement for "
                     f"{cfg.patience} epochs; best={best_iou:.4f} @ epoch {best_epoch})")
                break

    checkpoint_path = None
    if best_state is not None:
        model.load_state_dict(best_state)
        if cfg.save_artifacts:
            cfg.checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
            torch.save({
                "model_state_dict": best_state,
                "base_channels": cfg.base_channels,
                "azimuth_steps": cfg.azimuth_steps,
                "board_hollow_prob": cfg.board_hollow_prob,
                "best_epoch": best_epoch,
                "best_iou": best_iou,
                "best_precision": best_precision,
                "best_recall": best_recall,
            }, cfg.checkpoint_path)
            checkpoint_path = cfg.checkpoint_path

    if cfg.save_artifacts and history:
        plot_curve(history, cfg.curve_path)
        plot_val_predictions(model, val_ds, device, cfg.val_pred_path)

    return TrainResult(
        best_epoch=best_epoch, best_iou=best_iou, best_precision=best_precision,
        best_recall=best_recall, history=history, checkpoint_path=checkpoint_path,
        n_params=n_params,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch-size", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--num-workers", type=int, default=12)
    ap.add_argument("--train-epoch-size", type=int, default=4096)
    ap.add_argument("--val-size", type=int, default=512)
    ap.add_argument("--max-epochs", type=int, default=60)
    ap.add_argument("--patience", type=int, default=10)
    ap.add_argument("--board-hollow-prob", type=float, default=0.5,
                    help="mix fraction of hollow (vs. plain) diamond boards "
                         "in synth training/val data (default 0.5: THE "
                         "bake-in -- see module docstring)")
    ap.add_argument("--azimuth-steps", type=int, default=DEFAULT_AZIMUTH_STEPS)
    ap.add_argument("--device", type=str, default=None)
    ap.add_argument("--smoke", action="store_true",
                    help="tiny run (small batch/W, 1 epoch, no workers) for "
                         "a fast end-to-end sanity check, not real training")
    args = ap.parse_args()

    kwargs: dict = dict(
        batch_size=args.batch_size, lr=args.lr, num_workers=args.num_workers,
        train_epoch_size=args.train_epoch_size, val_size=args.val_size,
        max_epochs=args.max_epochs, patience=args.patience,
        board_hollow_prob=args.board_hollow_prob, azimuth_steps=args.azimuth_steps,
    )
    if args.device is not None:
        kwargs["device"] = args.device
    if args.smoke:
        kwargs.update(
            batch_size=4, num_workers=0, train_epoch_size=8, val_size=8,
            max_epochs=1, patience=1, azimuth_steps=64,
        )
    cfg = TrainConfig(**kwargs)
    result = run_training(cfg)
    print(
        f"best epoch {result.best_epoch}: IoU={result.best_iou:.4f} "
        f"precision={result.best_precision:.4f} recall={result.best_recall:.4f} "
        f"({result.n_params} params)"
    )


if __name__ == "__main__":
    main()
