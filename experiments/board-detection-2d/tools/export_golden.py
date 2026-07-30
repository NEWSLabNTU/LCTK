"""Export golden-vector parity fixtures for the Rust board-projection-detector port.

Runs the validated `boarddet` pipeline (both candidate generators, all five
sample datasets) and dumps per-(dataset, frame, generator) fixtures under
`rust/board-projection-detector/tests/fixtures/`:

- `<name>.input.f32`: raw little-endian f32 [x0,y0,z0, x1,y1,z1, ...] -- the
  RAW cloud (pre-downsample), so the Rust harness exercises
  `finite_only`+`downsample` too.
- `<name>.golden.json`: expected per-stage outputs (see task-1 brief for the
  schema).
- `<name>.bgkeys.i64` (background_subtraction fixtures only): raw little-
  endian i64 = the finalized cross-dataset LOO background's sorted voxel
  keys.

Regenerate with:
    cd experiments/board-detection-2d && uv run python tools/export_golden.py
"""
import json
import pathlib

import numpy as np

from boarddet.ingest import load_frames
from boarddet.benchmark_e_loo import build_background, DEFAULT_BBOX_PATH
from boarddet.bbox_ref import load_bbox
from boarddet.geometry import finite_only, downsample
from boarddet.candidates.cluster_after_ground import big_plane_residual
from boarddet.detector import detect
from boarddet.board_config import BoardConfig

# repo root = parents[3] of experiments/board-detection-2d/tools/export_golden.py
OUT = (pathlib.Path(__file__).resolve().parents[3]
       / "rust/board-projection-detector/tests/fixtures")
OUT.mkdir(parents=True, exist_ok=True)
VOXEL = 0.03
BOX = load_bbox(DEFAULT_BBOX_PATH)


def board():
    return BoardConfig(side_m=1.0, up_axis=(0.0, 0.0, 1.0),
                        cluster_min_points=30, square_icp=True,
                        stance_floor=0.9, isolation=True,
                        flatness_rms_max=0.045)


def dump(name, raw, generator, background=None, ds=None):
    raw = np.ascontiguousarray(raw[:, :3], dtype=np.float32)
    (OUT / f"{name}.input.f32").write_bytes(raw.tobytes())
    b = board()
    dn = downsample(finite_only(raw.astype(np.float64)), VOXEL)
    if generator == "background_subtraction":
        fg = background.foreground_points(dn)
        gen = "e"
    else:
        fg = big_plane_residual(dn, b, b.vertical_gap_deg)
        gen = "b"
    out = detect(raw.astype(np.float64), b, generator=gen, background=background)
    det = out.detection
    g = {"generator": generator, "dataset": ds, "voxel": VOXEL,
         "up_axis": list(b.up_axis), "cluster_min_points": b.cluster_min_points,
         "foreground_xyz": np.asarray(fg, float).tolist(),
         "n_candidates": out.n_candidates, "detected": det is not None}
    if det is not None:
        g["selected_centroid"] = det.center.astype(float).tolist()
        g["selected_corners_3d"] = det.corners_3d.astype(float).tolist()
        g["true_board"] = bool(BOX.contains(det.center))
    if background is not None:
        keys = np.asarray(background.keys() if hasattr(background, "keys") else background._keys, dtype="<i8")
        kf = f"{name}.bgkeys.i64"
        (OUT / kf).write_bytes(keys.tobytes())
        g["background_keys_file"] = kf
        g["background_params"] = {"voxel": background.voxel,
                                   "dilation_radius": background.dilation_radius,
                                   "min_sources": background.min_sources}
    (OUT / f"{name}.golden.json").write_text(json.dumps(g, indent=1))


def pick(frames, gen, background=None, ds=None):
    """Curate ~4 frames: first true-board hit, first non-detection, + a spread."""
    b = board()
    outs = [detect(f.xyz.astype(np.float64), b,
                    generator=("e" if gen == "background_subtraction" else "b"),
                    background=background) for f in frames]
    hits = [i for i, o in enumerate(outs) if o.detection is not None]
    miss = [i for i, o in enumerate(outs) if o.detection is None]
    idxs = ([hits[0]] if hits else []) + ([miss[0]] if miss else [])
    idxs += list(range(0, len(frames), max(1, len(frames) // 3)))
    seen = []
    for i in idxs:
        if i not in seen:
            seen.append(i)
        if len(seen) >= 4:
            break
    for i in seen:
        dump(f"ds{ds}_f{i:04d}_{gen[:2]}", frames[i].xyz, gen, background, ds)


if __name__ == "__main__":
    DATASETS = [1, 2, 3, 4, 5]
    sources = {str(d): load_frames(d) for d in DATASETS}
    for d in DATASETS:
        pick(sources[str(d)], "plane_strip", None, d)
        bg = build_background(sources, held_out=str(d), voxel=0.06,
                               dilation_radius=1, min_sources=3)
        pick(sources[str(d)], "background_subtraction", bg, d)
    print(f"wrote fixtures to {OUT}")
