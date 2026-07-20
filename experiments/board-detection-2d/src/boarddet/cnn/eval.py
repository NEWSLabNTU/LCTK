"""Task 33: evaluate the synth-trained `BoardUNet` checkpoint on REAL frames
-- the decisive synth -> real transfer test for the whole phase.

Pipeline per frame (real `boarddet.ingest.Frame` OR a synth `SimFrame`'s own
points, so the SAME code path backs both the real eval and the synth
end-to-end pipeline test):

1. `cnn.data.real_frame_to_input` -- THE shared input builder (identical
   normalization to training).
2. `BoardUNet.eval()` forward -> sigmoid -> per-pixel probability mask.
3. Threshold -> binary mask -> connected components, WRAPPING the azimuth
   seam (a board can straddle column 0/W-1 just as any other column) --
   see `label_mask_wrap`. Components smaller than `min_pixels` are dropped
   as noise, not detections.
4. Each surviving component's (row, col) pixels are looked up in a once-
   per-frame (row, col) -> 3D-point rebin of the frame's own points
   (`rebin_points_to_grid`, the SAME row/col binning `to_range_image` uses)
   -- the simplest robust back-projection: a masked cell either already
   has the frame's own real return, or it doesn't (no return -> no 3D point
   to contribute, the cell is skipped).
5. `geometry.fit_plane` + `square_fit.fit_fixed_square` (KNOWN board side,
   `BoardConfig().side_m`) on the component's 3D points -> refined pose
   center. If the fit fails (too few points), the component still counts
   as a raw detection at its points' centroid (brief's explicit fallback).
6. Classify the detection center against the real board's bounding box
   (`ros/lctk_launch/config/board/bbox.json5`, axis-aligned since its
   rotation is identity) -- in-bbox = true board, out-of-bbox = clutter
   false positive. Same rule stages 4-8's geometry pipeline used
   (`.superpowers/sdd/task-21-report.md`), so recall/precision are directly
   comparable.
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import numpy as np
import torch
from scipy import ndimage

from ..geometry import fit_plane, project_to_plane, unproject
from ..ingest import Frame, load_frames
from ..sim.range_image import points_to_rows_cols_ranges
from ..sim.sensor import Vlp32cSensor
from ..square_fit import fit_fixed_square
from .data import R_MAX, real_frame_to_input
from .model import BoardUNet

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (backend must be set first)

_RESULTS_DIR = Path(__file__).resolve().parents[3] / "results" / "stage9-cnn"
_CHECKPOINT_PATH = _RESULTS_DIR / "board_unet.pt"

# The real calibration board's bounding box (`bbox.json5`): axis-aligned
# (rotation is identity), so in/out is a plain per-axis half-extent check.
# Same rule stages 4-8's geometry pipeline used against this file --
# recall/precision below are directly comparable to those baselines.
BBOX_TRANSLATION = np.array([2.6, 0.0, 0.35])
BBOX_HALF_SIZE = np.array([3.1, 3.94, 2.2]) / 2.0

DEFAULT_BOARD_SIDE_M = 1.0  # BoardConfig().side_m -- the known diamond side
DEFAULT_MIN_PIXELS = 15
DEFAULT_THRESHOLDS = (0.3, 0.5, 0.7)


def in_bbox(center: np.ndarray) -> bool:
    """True if `center` (3,) falls inside the real board's bbox."""
    return bool(np.all(np.abs(center - BBOX_TRANSLATION) <= BBOX_HALF_SIZE))


# ---------------------------------------------------------------------------
# checkpoint loading -- reuses train.py's save format exactly
# ---------------------------------------------------------------------------


def load_checkpoint(path: Path, device: torch.device) -> tuple[BoardUNet, int]:
    """Load a `train.py`-saved checkpoint. Returns `(model.eval(), azimuth_steps)`
    -- `model.eval()` matters here (BatchNorm must use running stats, not the
    current batch's, at inference time)."""
    checkpoint = torch.load(path, map_location=device, weights_only=False)
    model = BoardUNet(base_channels=checkpoint["base_channels"]).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    return model, int(checkpoint["azimuth_steps"])


@torch.no_grad()
def predict_probability_mask(model: BoardUNet, input_t: torch.Tensor,
                             device: torch.device) -> np.ndarray:
    """`input_t`: `(3, H, W)` (no batch dim). Returns `(H, W)` float32
    sigmoid probability, on CPU."""
    logits = model(input_t.unsqueeze(0).to(device))
    prob = torch.sigmoid(logits)
    return prob[0, 0].cpu().numpy()


# ---------------------------------------------------------------------------
# (row, col) -> 3D point rebin -- the back-projection lookup table
# ---------------------------------------------------------------------------


def rebin_points_to_grid(points: np.ndarray, sensor: Vlp32cSensor,
                         azimuth_steps: int) -> np.ndarray:
    """Bin `points` (N, 3) onto the sensor's (n_rows, azimuth_steps) grid,
    the SAME row/col assignment `to_range_image` uses (nearest-range point
    wins a cell with multiple returns). Returns `(n_rows, azimuth_steps, 3)`
    float32, NaN where no point landed in a cell -- this is the per-frame
    lookup table `evaluate_frame` back-projects masked pixels through."""
    n_rows = len(sensor.elevations)
    n_cols = int(azimuth_steps)
    grid = np.full((n_rows * n_cols, 3), np.nan, dtype=np.float32)
    if len(points) == 0:
        return grid.reshape(n_rows, n_cols, 3)

    rows, cols, ranges = points_to_rows_cols_ranges(points, sensor, n_cols)
    flat = rows * n_cols + cols
    # sort by (flat cell, range ascending) so each cell's FIRST entry after
    # sorting is its nearest-range point -- matches to_range_image's
    # np.minimum.at nearest-return semantics.
    order = np.lexsort((ranges, flat))
    sorted_flat = flat[order]
    _, first_pos = np.unique(sorted_flat, return_index=True)
    winners = order[first_pos]
    grid[flat[winners]] = points[winners].astype(np.float32)
    return grid.reshape(n_rows, n_cols, 3)


# ---------------------------------------------------------------------------
# seam-wrapping connected components
# ---------------------------------------------------------------------------


def label_mask_wrap(mask: np.ndarray) -> np.ndarray:
    """8-connected component labeling of a binary `(H, W)` mask, WRAPPING
    the width (azimuth) axis only (the row/elevation axis does NOT wrap --
    there is no laser above the top channel or below the bottom one, same
    physical reasoning as `model.CircularWidthConv2d`): a board straddling
    column 0/W-1 must label as ONE component, not two.

    Implemented as ordinary (non-wrapping) `scipy.ndimage.label`, followed
    by a union-find merge of any two labels that touch across the column-0
    / column-(W-1) seam (8-connectivity: column 0 row r unions with column
    W-1 rows r-1, r, r+1). This is simpler and exactly correct, unlike
    padding-then-slicing: padding both edges to bridge the seam creates TWO
    duplicate copies of any seam-straddling blob (one per padding side),
    and slicing back down to the visible window keeps only one half of
    each duplicate -- the two halves end up in different, still-unmerged
    labels. Explicit post-hoc union avoids that duplication entirely.
    Returns an `(H, W)` int label array, 0 = background."""
    labeled, num_labels = ndimage.label(mask, structure=np.ones((3, 3)))
    if num_labels == 0:
        return labeled

    parent = np.arange(num_labels + 1)

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return int(x)

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    n_rows = mask.shape[0]
    left_col = labeled[:, 0]
    right_col = labeled[:, -1]
    for row in range(n_rows):
        left_label = left_col[row]
        if left_label == 0:
            continue
        for dr in (-1, 0, 1):
            neighbor_row = row + dr
            if 0 <= neighbor_row < n_rows and right_col[neighbor_row] != 0:
                union(left_label, right_col[neighbor_row])

    merged = np.array([find(lbl) for lbl in range(num_labels + 1)])
    return merged[labeled]


def components_from_mask(mask: np.ndarray, min_pixels: int = DEFAULT_MIN_PIXELS,
                         ) -> list[tuple[np.ndarray, np.ndarray]]:
    """`mask`: `(H, W)` bool. Returns a list of `(rows, cols)` index arrays,
    one per connected component with at least `min_pixels` pixels (drops
    isolated-noise-pixel "components")."""
    labeled = label_mask_wrap(mask)
    components = []
    for label in np.unique(labeled):
        if label == 0:
            continue
        rows, cols = np.nonzero(labeled == label)
        if len(rows) < min_pixels:
            continue
        components.append((rows, cols))
    return components


# ---------------------------------------------------------------------------
# per-component pose + classification
# ---------------------------------------------------------------------------


@dataclass
class ComponentDetection:
    n_pixels: int
    n_points: int             # real 3D points backing this component
    center: np.ndarray         # (3,) -- fitted pose center, or raw centroid
    fit_residual: float | None  # None when the square fit fell back to centroid
    in_bbox: bool


def evaluate_frame(prob: np.ndarray, grid_points: np.ndarray, threshold: float,
                   min_pixels: int = DEFAULT_MIN_PIXELS,
                   side_m: float = DEFAULT_BOARD_SIDE_M,
                   ) -> list[ComponentDetection]:
    """`prob`: `(H, W)` sigmoid probability from `predict_probability_mask`.
    `grid_points`: `(H, W, 3)` from `rebin_points_to_grid`, SAME (H, W) as
    `prob`. Returns one `ComponentDetection` per surviving component that
    has at least one real backed 3D point (a component with zero backing
    points -- predicted mask fired on empty cells -- has nothing to report
    and is dropped, not counted as a zero-point detection)."""
    mask = prob >= threshold
    detections: list[ComponentDetection] = []
    for rows, cols in components_from_mask(mask, min_pixels):
        pts = grid_points[rows, cols]
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid].astype(np.float64)
        if len(pts) == 0:
            continue

        fit_residual = None
        if len(pts) >= 3:
            plane = fit_plane(pts)
            coords_2d = project_to_plane(pts, plane)
            fit = fit_fixed_square(coords_2d, side_m)
        else:
            fit = None

        if fit is not None:
            center = unproject(fit.center[None, :], plane)[0]
            fit_residual = fit.residual
        else:
            center = pts.mean(axis=0)

        detections.append(ComponentDetection(
            n_pixels=len(rows), n_points=len(pts), center=center,
            fit_residual=fit_residual, in_bbox=in_bbox(center),
        ))
    return detections


# ---------------------------------------------------------------------------
# full real-data eval run
# ---------------------------------------------------------------------------


@dataclass
class ThresholdMetrics:
    threshold: float
    n_frames: int = 0
    hit_frames: int = 0        # frames with >= 1 in-bbox detection
    n_detections: int = 0
    n_in_bbox_detections: int = 0

    @property
    def recall(self) -> float:
        return self.hit_frames / self.n_frames if self.n_frames else 0.0

    @property
    def precision(self) -> float:
        return (self.n_in_bbox_detections / self.n_detections
                if self.n_detections else 0.0)

    def to_dict(self) -> dict:
        return {
            "threshold": self.threshold, "n_frames": self.n_frames,
            "hit_frames": self.hit_frames, "n_detections": self.n_detections,
            "n_in_bbox_detections": self.n_in_bbox_detections,
            "recall": self.recall, "precision": self.precision,
        }


@dataclass
class EvalResult:
    per_threshold: dict[float, ThresholdMetrics] = field(default_factory=dict)
    fwd_ms: list[float] = field(default_factory=list)
    post_ms: list[float] = field(default_factory=list)  # at reference_threshold only

    def to_dict(self) -> dict:
        fwd = np.asarray(self.fwd_ms)
        post = np.asarray(self.post_ms)
        return {
            "thresholds": {t: m.to_dict() for t, m in self.per_threshold.items()},
            "timing_ms": {
                "fwd_median": float(np.median(fwd)) if len(fwd) else 0.0,
                "fwd_p95": float(np.percentile(fwd, 95)) if len(fwd) else 0.0,
                "post_median": float(np.median(post)) if len(post) else 0.0,
                "post_p95": float(np.percentile(post, 95)) if len(post) else 0.0,
                "total_median": float(np.median(fwd + post)) if len(fwd) else 0.0,
                "total_p95": float(np.percentile(fwd + post, 95)) if len(fwd) else 0.0,
            },
        }


def run_eval(datasets: list[int], checkpoint_path: Path = _CHECKPOINT_PATH,
            thresholds: tuple[float, ...] = DEFAULT_THRESHOLDS,
            reference_threshold: float = 0.5,
            min_pixels: int = DEFAULT_MIN_PIXELS,
            side_m: float = DEFAULT_BOARD_SIDE_M,
            r_max: float = R_MAX,
            max_frames: int | None = None,
            device: torch.device | None = None) -> EvalResult:
    """Evaluate a checkpoint over every frame of `datasets` (real
    `boarddet.ingest.load_frames`). `reference_threshold` picks which
    threshold's post-processing time is timed (post-proc cost is threshold-
    dependent -- more components/points to fit at a looser threshold -- so
    timing at one representative threshold, not summed across the sweep, is
    the honest per-deployment-frame number)."""
    device = device if device is not None else torch.device(
        "cuda" if torch.cuda.is_available() else "cpu")
    model, azimuth_steps = load_checkpoint(checkpoint_path, device)
    sensor = Vlp32cSensor()

    result = EvalResult(per_threshold={
        t: ThresholdMetrics(threshold=t) for t in thresholds
    })

    for ds in datasets:
        frames = load_frames(ds, max_frames=max_frames)
        for frame in frames:
            input_t = real_frame_to_input(frame, sensor, azimuth_steps, r_max)
            if device.type == "cuda":
                torch.cuda.synchronize()
            t0 = time.perf_counter()
            prob = predict_probability_mask(model, input_t, device)
            if device.type == "cuda":
                torch.cuda.synchronize()
            t1 = time.perf_counter()
            result.fwd_ms.append((t1 - t0) * 1e3)

            grid_points = rebin_points_to_grid(frame.xyz, sensor, azimuth_steps)
            for threshold in thresholds:
                tp0 = time.perf_counter()
                dets = evaluate_frame(prob, grid_points, threshold,
                                     min_pixels, side_m)
                tp1 = time.perf_counter()
                if threshold == reference_threshold:
                    result.post_ms.append((tp1 - tp0) * 1e3)

                m = result.per_threshold[threshold]
                m.n_frames += 1
                m.n_detections += len(dets)
                in_bbox_dets = [d for d in dets if d.in_bbox]
                m.n_in_bbox_detections += len(in_bbox_dets)
                if in_bbox_dets:
                    m.hit_frames += 1

    return result


# ---------------------------------------------------------------------------
# visual evidence -- THE essential artifact for judging synth -> real transfer
# ---------------------------------------------------------------------------


def _bbox_reference_col_row(sensor: Vlp32cSensor, azimuth_steps: int,
                            ) -> tuple[int, int]:
    """(row, col) the bbox center itself would land on in the range image --
    the reference marker drawn on `real_pred.png`'s panels."""
    rows, cols, _ranges = points_to_rows_cols_ranges(
        BBOX_TRANSLATION[None, :], sensor, azimuth_steps)
    return int(rows[0]), int(cols[0])


def save_real_predictions(model: BoardUNet, azimuth_steps: int,
                          device: torch.device,
                          frame_specs: list[tuple[int, int]], path: Path,
                          threshold: float = 0.5, r_max: float = R_MAX,
                          ) -> None:
    """`frame_specs`: list of `(dataset, frame_index)`. For each, renders 3
    panels: the real input range image, the predicted probability mask
    (with the bbox reference row/col marked), and the thresholded binary
    mask -- the visual evidence Task 33 needs to judge synth -> real
    transfer directly, not just from aggregate numbers."""
    sensor = Vlp32cSensor()
    ref_row, ref_col = _bbox_reference_col_row(sensor, azimuth_steps)

    n = len(frame_specs)
    fig, axes = plt.subplots(n, 3, figsize=(16, 2.4 * n), squeeze=False)
    for i, (ds, frame_idx) in enumerate(frame_specs):
        frames = load_frames(ds, max_frames=frame_idx + 1)
        frame = frames[frame_idx]
        input_t = real_frame_to_input(frame, sensor, azimuth_steps, r_max)
        prob = predict_probability_mask(model, input_t, device)

        ax = axes[i, 0]
        ax.imshow(input_t[0].numpy(), aspect="auto", cmap="viridis")
        ax.axvline(ref_col, color="r", lw=0.8, ls="--")
        ax.axhline(ref_row, color="r", lw=0.8, ls="--")
        ax.set_title(f"ds{ds} f{frame_idx}: normalized range (red=bbox ref)")
        ax.set_yticks([])

        ax = axes[i, 1]
        ax.imshow(prob, aspect="auto", cmap="magma", vmin=0, vmax=1)
        ax.axvline(ref_col, color="c", lw=0.8, ls="--")
        ax.axhline(ref_row, color="c", lw=0.8, ls="--")
        ax.set_title("predicted probability (cyan=bbox ref)")
        ax.set_yticks([])

        ax = axes[i, 2]
        ax.imshow(prob >= threshold, aspect="auto", cmap="gray", vmin=0, vmax=1)
        ax.axvline(ref_col, color="r", lw=0.8, ls="--")
        ax.axhline(ref_row, color="r", lw=0.8, ls="--")
        ax.set_title(f"thresholded @ {threshold} (red=bbox ref)")
        ax.set_yticks([])

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=Path, default=_CHECKPOINT_PATH)
    ap.add_argument("--datasets", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--thresholds", type=float, nargs="+",
                    default=list(DEFAULT_THRESHOLDS))
    ap.add_argument("--reference-threshold", type=float, default=0.5)
    ap.add_argument("--min-pixels", type=int, default=DEFAULT_MIN_PIXELS)
    ap.add_argument("--side-m", type=float, default=DEFAULT_BOARD_SIDE_M)
    ap.add_argument("--max-frames", type=int, default=None,
                    help="cap frames per dataset (debugging only)")
    ap.add_argument("--out", type=Path, default=_RESULTS_DIR / "real_eval.json")
    ap.add_argument("--png", type=Path, default=_RESULTS_DIR / "real_pred.png")
    args = ap.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    result = run_eval(
        args.datasets, args.checkpoint, tuple(args.thresholds),
        args.reference_threshold, args.min_pixels, args.side_m,
        max_frames=args.max_frames, device=device,
    )
    summary = result.to_dict()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2))

    for t in args.thresholds:
        m = result.per_threshold[t]
        print(f"threshold={t:.2f}  recall={m.recall:.1%} "
             f"({m.hit_frames}/{m.n_frames})  precision={m.precision:.1%} "
             f"({m.n_in_bbox_detections}/{m.n_detections})")
    timing = summary["timing_ms"]
    print(f"fwd median={timing['fwd_median']:.2f}ms p95={timing['fwd_p95']:.2f}ms  "
         f"post median={timing['post_median']:.2f}ms p95={timing['post_p95']:.2f}ms  "
         f"total median={timing['total_median']:.2f}ms")

    model, azimuth_steps = load_checkpoint(args.checkpoint, device)
    # Mix of datasets, incl. ds3 (the well-known primary lidar-camera set),
    # a low/high index within each to catch both early and late poses.
    frame_specs = [(1, 0), (2, 0), (3, 0), (3, len(load_frames(3)) // 2),
                   (4, 0), (5, 0)]
    save_real_predictions(model, azimuth_steps, device, frame_specs, args.png,
                          threshold=args.reference_threshold)
    print(f"wrote {args.png}")


if __name__ == "__main__":
    main()
