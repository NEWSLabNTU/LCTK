"""Bag-sourced frames must be indistinguishable from pcap-sourced ones."""
from __future__ import annotations

import numpy as np
import pytest

from boarddet.ingest import Frame, load_bag_frames


def _write_cache(path, n_frames=3, n_pts=50):
    rng = np.random.default_rng(0)
    arrays = {"stamps": np.arange(n_frames, dtype=np.float64)}
    for i in range(n_frames):
        arrays[f"xyz_{i}"] = rng.normal(size=(n_pts, 3)).astype(np.float32)
        arrays[f"intensity_{i}"] = rng.random(n_pts).astype(np.float32)
        arrays[f"ring_{i}"] = rng.integers(0, 32, n_pts).astype(np.uint8)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)


def test_loads_frames_from_an_exported_cache(tmp_path, monkeypatch):
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    _write_cache(tmp_path / "bag_TESTBAG_vlp32.npz", n_frames=3)

    frames = load_bag_frames("TESTBAG", "vlp32")
    assert len(frames) == 3
    assert all(isinstance(f, Frame) for f in frames)
    assert frames[0].xyz.shape == (50, 3)
    assert frames[0].xyz.dtype == np.float32
    assert frames[1].stamp == 1.0


def test_max_frames_truncates(tmp_path, monkeypatch):
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    _write_cache(tmp_path / "bag_TESTBAG_vlp32.npz", n_frames=5)
    assert len(load_bag_frames("TESTBAG", "vlp32", max_frames=2)) == 2


def test_missing_export_names_the_tool_that_creates_it(tmp_path, monkeypatch):
    """A missing cache is a workflow step not yet run, not a crash -- the
    error must say how to fix it."""
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    with pytest.raises(FileNotFoundError, match="export_bag_npz"):
        load_bag_frames("NOPE", "vlp32")


def test_unknown_sensor_is_rejected(tmp_path, monkeypatch):
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    with pytest.raises(ValueError, match="sensor"):
        load_bag_frames("TESTBAG", "lidar-that-does-not-exist")
