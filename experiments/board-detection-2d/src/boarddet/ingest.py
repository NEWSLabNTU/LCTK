"""Decode sample pcaps to per-frame numpy arrays, cached as npz.

ring and intensity are cached for diagnostics only — algorithm code must
never read them (solid-state compatibility / geometry-only prior).

The sample captures are pcapng files, which velodyne_decoder's read_pcap
cannot read (its dpkt.pcap.Reader only handles classic pcap, and its
HDL-64E autodetect path trips first because Config.calibration is None).
We therefore iterate packets ourselves and feed vd.StreamDecoder directly.
"""
from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import dpkt
import numpy as np
import velodyne_decoder as vd

# Repo-relative anchors; ingest.py sits at src/boarddet/ingest.py.
_PKG_ROOT = Path(__file__).resolve().parents[2]
SESSIONS_DIR = _PKG_ROOT.parents[1] / "sessions"
CACHE_DIR = _PKG_ROOT / "cache"

# The five sample recordings used to sit together at
# ros/lctk_sample_data/data/<N>. Each now lives inside the calibration session
# that describes it, so a session directory is self-contained -- which is why
# this is a name map rather than an f-string: dataset 3 is the one with a
# descriptive session name.
_SAMPLE_SESSIONS = {
    1: "sample1",
    2: "sample2",
    3: "sample3-hollow-velodyne",
    4: "sample4",
    5: "sample5",
}


def sample_pcap(dataset: int | str) -> Path:
    """Path to sample dataset N's lidar.pcap, inside its session directory."""
    key = int(dataset)
    if key not in _SAMPLE_SESSIONS:
        raise KeyError(
            f"no sample dataset {dataset!r}; known: {sorted(_SAMPLE_SESSIONS)}"
        )
    return SESSIONS_DIR / _SAMPLE_SESSIONS[key] / "data" / "lidar.pcap"

# Field names as produced by velodyne_decoder as_pcl_structs=True.
# Single place to adjust if the installed decoder version differs.
_F_X, _F_Y, _F_Z = "x", "y", "z"
_F_INTENSITY = "intensity"
_F_RING = "ring"

_PCAPNG_MAGIC = b"\x0a\x0d\x0d\x0a"


@dataclass
class Frame:
    stamp: float
    xyz: np.ndarray        # (N, 3) float32
    intensity: np.ndarray  # (N,) float32 — diagnostics only
    ring: np.ndarray       # (N,) uint8   — diagnostics only


def _iter_udp_payloads(pcap: Path) -> Iterator[tuple[float, bytes]]:
    """Yield (host_stamp, innermost payload) for each packet in a capture.

    Handles both classic pcap and pcapng, selected by file magic.
    """
    with pcap.open("rb") as f:
        magic = f.read(4)
        f.seek(0)
        reader_cls = (
            dpkt.pcapng.Reader if magic == _PCAPNG_MAGIC else dpkt.pcap.Reader
        )
        for stamp, buf in reader_cls(f):
            try:
                pkt = dpkt.ethernet.Ethernet(buf)
            except dpkt.dpkt.UnpackError:
                continue
            # Peel Ethernet -> IP -> UDP down to the raw Velodyne payload.
            while hasattr(pkt, "data"):
                pkt = pkt.data
            yield stamp, pkt


def _to_frame(stamp: object, pts: np.ndarray) -> Frame:
    xyz = np.stack(
        [pts[_F_X], pts[_F_Y], pts[_F_Z]], axis=1
    ).astype(np.float32)
    return Frame(
        stamp=float(stamp.host if hasattr(stamp, "host") else stamp),
        xyz=xyz,
        intensity=pts[_F_INTENSITY].astype(np.float32),
        ring=pts[_F_RING].astype(np.uint8),
    )


def _decode_pcap(pcap: Path, max_frames: int | None) -> list[Frame]:
    cfg = vd.Config(model=vd.Model.VLP32C)
    decoder = vd.StreamDecoder(cfg)
    frames: list[Frame] = []
    for host_stamp, data in _iter_udp_payloads(pcap):
        if len(data) != vd.PACKET_SIZE:
            continue
        result = decoder.decode(host_stamp, data, True)  # as_pcl_structs
        if result is None:
            continue
        stamp, pts = result
        frames.append(_to_frame(stamp, pts))
        if max_frames is not None and len(frames) >= max_frames:
            return frames
    result = decoder.finish(True)  # flush the last partial rotation
    if result is not None:
        stamp, pts = result
        frames.append(_to_frame(stamp, pts))
    return frames


def _cache_path(dataset: int) -> Path:
    return CACHE_DIR / f"dataset_{dataset}.npz"


def _save_cache(path: Path, frames: list[Frame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays: dict[str, np.ndarray] = {
        "stamps": np.array([f.stamp for f in frames], dtype=np.float64)
    }
    for i, f in enumerate(frames):
        arrays[f"xyz_{i}"] = f.xyz
        arrays[f"intensity_{i}"] = f.intensity
        arrays[f"ring_{i}"] = f.ring
    np.savez_compressed(path, **arrays)


def _load_cache(path: Path) -> list[Frame]:
    with np.load(path) as z:
        stamps = z["stamps"]
        return [
            Frame(
                stamp=float(stamps[i]),
                xyz=z[f"xyz_{i}"],
                intensity=z[f"intensity_{i}"],
                ring=z[f"ring_{i}"],
            )
            for i in range(len(stamps))
        ]


def load_frames(dataset: int, max_frames: int | None = None) -> list[Frame]:
    """Load frames for a sample dataset (1-5), decoding + caching on first use."""
    cached = _cache_path(dataset)
    if cached.exists():
        frames = _load_cache(cached)
    else:
        pcap = sample_pcap(dataset)
        if not pcap.exists():
            raise FileNotFoundError(f"sample pcap not found: {pcap}")
        frames = _decode_pcap(pcap, max_frames=None)
        _save_cache(cached, frames)
    if max_frames is not None:
        frames = frames[:max_frames]
    return frames


# Sensors exported from the recorded TWO_LIDAR bags by
# tools/export_bag_npz.py. "falcon" is solid-state (no ring structure) --
# see BoardConfig.vertical_gap_deg before benchmarking it.
BAG_SENSORS = ("vlp32", "falcon")


def _bag_cache_path(bag: str, sensor: str) -> Path:
    return CACHE_DIR / f"bag_{bag}_{sensor}.npz"


def load_bag_frames(bag: str, sensor: str,
                    max_frames: int | None = None) -> list[Frame]:
    """Load frames exported from a recorded ROS 2 bag.

    Unlike `load_frames`, this never decodes anything itself: bags are read
    by `tools/export_bag_npz.py`, which needs ROS and therefore a different
    Python than this package runs on. The export is a prerequisite, and a
    missing one is a workflow step not yet taken.
    """
    if sensor not in BAG_SENSORS:
        raise ValueError(
            f"unknown sensor {sensor!r}; expected one of {BAG_SENSORS}")
    cached = _bag_cache_path(bag, sensor)
    if not cached.exists():
        raise FileNotFoundError(
            f"no exported cache at {cached}. Create it with:\n"
            f"  source /opt/ros/humble/setup.bash\n"
            f"  python3 tools/export_bag_npz.py --bags {bag} "
            f"--sensors {sensor}")
    frames = _load_cache(cached)
    if max_frames is not None:
        frames = frames[:max_frames]
    return frames


if __name__ == "__main__":
    import sys

    ds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    fr = load_frames(ds)
    sizes = [len(f.xyz) for f in fr]
    print(f"dataset {ds}: {len(fr)} frames, "
          f"points/frame min={min(sizes)} max={max(sizes)}")
