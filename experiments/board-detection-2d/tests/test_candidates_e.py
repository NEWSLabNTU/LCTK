"""TDD for generator "e" (Method E): background-diff candidate generation.

Match criteria mirror test_candidates_b.py exactly."""
from __future__ import annotations

import numpy as np

from boarddet.background import BackgroundModel
from boarddet.board_config import BoardConfig
from boarddet.candidates.background_diff import generate_background_diff
from boarddet.geometry import downsample
from boarddet.synth import make_scene


def _matches_truth(cands, truth):
    return [
        c for c in cands
        if abs(c.plane.normal @ truth.normal) > 0.99
        and abs((c.plane.center - truth.center) @ truth.normal) < 0.05
    ]


def _model_from(points_list, min_sources=1):
    m = BackgroundModel(min_sources=min_sources)
    for i, pts in enumerate(points_list):
        m.observe(downsample(pts, 0.03), source=i)
    m.finalize()
    return m


def test_finds_board_after_reveal():
    """Background learned without the board; the reveal frame's board must
    survive the diff and pass plausible_board_patch."""
    bg_pts, _ = make_scene(rng=np.random.default_rng(0), include_board=False)
    reveal, truth = make_scene(rng=np.random.default_rng(1), include_board=True)
    model = _model_from([bg_pts])
    cands = generate_background_diff(
        downsample(reveal, 0.03), BoardConfig(side_m=1.0), background=model)
    assert _matches_truth(cands, truth)


def test_no_candidates_on_boardless_replay():
    """A second, independently-noised look at the SAME boardless scene must
    yield nothing -- otherwise the diff is just reporting sensor noise."""
    bg_pts, _ = make_scene(rng=np.random.default_rng(2), include_board=False)
    replay, _ = make_scene(rng=np.random.default_rng(3), include_board=False)
    model = _model_from([bg_pts])
    cands = generate_background_diff(
        downsample(replay, 0.03), BoardConfig(side_m=1.0), background=model)
    assert cands == []


def test_consensus_background_keeps_board_of_the_held_out_scene():
    """The LOO claim in miniature: two 'contributor' scenes each with a
    board at a DIFFERENT place, plus a third held-out scene. At
    min_sources=2 the contributors' boards drop out of the background (each
    seen once) while the shared room survives, so the held-out board is
    still found."""
    a, _ = make_scene(rng=np.random.default_rng(4), board_center=(4.0, 2.0, 0.3))
    b, _ = make_scene(rng=np.random.default_rng(5), board_center=(4.0, -2.0, 0.3))
    held, truth = make_scene(rng=np.random.default_rng(6),
                             board_center=(4.0, 0.5, 0.3))
    model = _model_from([a, b], min_sources=2)
    cands = generate_background_diff(
        downsample(held, 0.03), BoardConfig(side_m=1.0), background=model)
    assert _matches_truth(cands, truth)


def test_e_merges_coplanar_stripe_clusters(monkeypatch):
    """E must route through the shared _merge_coplanar_clusters tail (same as
    B), so a board fragmented into ring stripes is merged before gating."""
    import boarddet.candidates.cluster_after_ground as cag
    calls = {"n": 0}
    real = cag._merge_coplanar_clusters

    def spy(*a, **k):
        calls["n"] += 1
        return real(*a, **k)

    monkeypatch.setattr(cag, "_merge_coplanar_clusters", spy)

    bg_pts, _ = make_scene(rng=np.random.default_rng(20), include_board=False)
    reveal, truth = make_scene(rng=np.random.default_rng(21), include_board=True)
    model = _model_from([bg_pts])
    cands = generate_background_diff(
        downsample(reveal, 0.03), BoardConfig(side_m=1.0), background=model)
    assert calls["n"] >= 1
    assert _matches_truth(cands, truth)


def test_non_planar_blob_alone_is_not_a_candidate():
    """The roadmap's own Method-E caveat ('person + board move together')
    is already handled downstream: a non-planar newly-appeared object fails
    plausible_board_patch's flatness gate, so no new gate is needed."""
    bg_pts, _ = make_scene(rng=np.random.default_rng(7), include_board=False)
    rng = np.random.default_rng(8)
    blob = rng.normal([2.0, 1.0, 0.0], [0.25, 0.25, 0.5],
                      size=(1500, 3)).astype(np.float32)
    reveal = np.concatenate([bg_pts, blob])
    model = _model_from([bg_pts])
    cands = generate_background_diff(
        downsample(reveal, 0.03), BoardConfig(side_m=1.0), background=model)
    assert cands == []
