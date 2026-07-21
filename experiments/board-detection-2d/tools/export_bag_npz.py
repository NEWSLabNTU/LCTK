#!/usr/bin/env python3
"""Export a recorded ROS 2 bag's PointCloud2 topic to boarddet's npz cache.

This is the ONLY file in this experiment that imports ROS. It runs under
system Python 3.10 with /opt/ros/humble/setup.bash sourced -- NOT inside the
uv venv, which is Python 3.11 and deliberately ROS-free. boarddet never
imports this module; it only reads the .npz files it writes.

    source /opt/ros/humble/setup.bash
    python3 experiments/board-detection-2d/tools/export_bag_npz.py \
        --bags TWO_LIDAR_1 TWO_LIDAR_2 TWO_LIDAR_3 TWO_LIDAR_4 \
        --sensors vlp32 falcon

Output schema matches ingest.py's pcap cache exactly (stamps + per-frame
xyz_i/intensity_i/ring_i), so both sources yield identical Frame objects.
`channel` is stored as `ring`; like intensity it is DIAGNOSTIC ONLY and
algorithm code must never read it.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

_REPO_ROOT = Path(__file__).resolve().parents[3]
_BAG_DIR = _REPO_ROOT / "ros" / "lctk_sample_data" / "bags"
_CACHE_DIR = Path(__file__).resolve().parents[1] / "cache"

SENSOR_TOPICS = {
    "vlp32": "/lidar/vlp32/velodyne_points",
    "falcon": "/lidar/falcon/iv_points",
}

# Both sensors lay out the first 16 bytes identically: x,y,z float32 at
# offsets 0/4/8, intensity uint8 at 12, return_type uint8 at 13, channel
# uint16 at 14. Assert rather than assume -- a silently mis-parsed cloud
# would look like plausible noise downstream.
_EXPECTED_OFFSETS = {"x": 0, "y": 4, "z": 8, "intensity": 12, "channel": 14}


def _check_layout(msg: PointCloud2) -> None:
    offsets = {f.name: f.offset for f in msg.fields}
    for name, want in _EXPECTED_OFFSETS.items():
        if offsets.get(name) != want:
            raise ValueError(
                f"unexpected PointCloud2 layout: field {name!r} at offset "
                f"{offsets.get(name)}, expected {want}. Fields present: "
                f"{[(f.name, f.offset) for f in msg.fields]}")


def _decode(msg: PointCloud2):
    """-> (xyz float32 (M,3), intensity float32 (M,), ring uint8 (M,))."""
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = raw[:, 0:12].copy().view(np.float32).reshape(-1, 3)
    intensity = raw[:, 12].astype(np.float32)
    ring = raw[:, 14:16].copy().view(np.uint16).reshape(-1)
    # is_dense is advertised True, but a NaN here would poison every plane
    # fit downstream, so filter rather than trust the flag.
    keep = np.isfinite(xyz).all(axis=1)
    return xyz[keep], intensity[keep], ring[keep].astype(np.uint8)


def export(bag: str, sensor: str, overwrite: bool = False) -> Path:
    topic = SENSOR_TOPICS[sensor]
    uri = _BAG_DIR / bag
    if not uri.exists():
        raise FileNotFoundError(f"bag not found: {uri} (see bags/README.md)")
    out = _CACHE_DIR / f"bag_{bag}_{sensor}.npz"
    if out.exists() and not overwrite:
        print(f"  {out.name} exists, skipping (use --overwrite to redo)")
        return out

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""))

    stamps: list[float] = []
    arrays: dict[str, np.ndarray] = {}
    i = 0
    while reader.has_next():
        got_topic, data, _ = reader.read_next()
        if got_topic != topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        if i == 0:
            _check_layout(msg)
        xyz, intensity, ring = _decode(msg)
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        arrays[f"xyz_{i}"] = xyz
        arrays[f"intensity_{i}"] = intensity
        arrays[f"ring_{i}"] = ring
        i += 1

    if i == 0:
        raise ValueError(f"no messages on {topic!r} in {bag}")
    arrays["stamps"] = np.array(stamps, dtype=np.float64)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **arrays)
    pts = int(np.mean([len(arrays[f"xyz_{j}"]) for j in range(i)]))
    print(f"  {out.name}: {i} frames, ~{pts} pts/frame")
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags", nargs="+",
                    default=["TWO_LIDAR_1", "TWO_LIDAR_2",
                             "TWO_LIDAR_3", "TWO_LIDAR_4"])
    ap.add_argument("--sensors", nargs="+", default=list(SENSOR_TOPICS),
                    choices=list(SENSOR_TOPICS))
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()
    for bag in args.bags:
        print(bag)
        for sensor in args.sensors:
            export(bag, sensor, overwrite=args.overwrite)


if __name__ == "__main__":
    main()
