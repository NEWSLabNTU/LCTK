import numpy as np
from boarddet.ingest import load_frames


def test_load_frames_dataset3_first_frames():
    frames = load_frames(3, max_frames=5)
    assert len(frames) == 5
    f = frames[0]
    assert f.xyz.ndim == 2 and f.xyz.shape[1] == 3
    assert f.xyz.dtype == np.float32
    assert len(f.intensity) == len(f.xyz) == len(f.ring)
    # VLP-32C full rotation at 600 rpm: expect tens of thousands of points
    assert f.xyz.shape[0] > 10_000
    # points are in sensor frame, metres: sane range
    r = np.linalg.norm(f.xyz, axis=1)
    assert r.max() < 200.0


def test_load_frames_uses_cache(tmp_path, monkeypatch):
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    frames1 = ingest.load_frames(3, max_frames=2)
    assert (tmp_path / "dataset_3.npz").exists()
    frames2 = ingest.load_frames(3, max_frames=2)
    np.testing.assert_array_equal(frames1[0].xyz, frames2[0].xyz)
