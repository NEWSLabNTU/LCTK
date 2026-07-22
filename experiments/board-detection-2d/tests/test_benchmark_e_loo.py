"""LOO harness: bbox classification grounded in the phase-7 doc's own
confirmed coordinates, and background construction that never sees the
held-out dataset."""
from __future__ import annotations

import numpy as np
import pytest

from boarddet import benchmark_e_loo as loo
from boarddet.bbox_ref import load_bbox
from boarddet.benchmark_e_loo import DEFAULT_BBOX_PATH
from boarddet.ingest import Frame

_BOX = load_bbox(DEFAULT_BBOX_PATH)

# Confirmed true-board centers, phase-7 doc "Pose sanity" table (:782-793).
_CONFIRMED_BOARDS = [
    (2.256, -0.059, 0.074), (2.147, 0.420, 0.076), (2.101, -0.314, 0.074),
    (2.077, -0.605, 0.066), (2.090, -0.829, 0.039),
]
# Documented static clutter attractors (phase-7 doc :572, :633, :~1697).
_CONFIRMED_CLUTTER = [(-1.83, -2.89, -0.1), (4.7, 2.6, -0.1), (-3.3, 3.4, 0.5)]


@pytest.mark.parametrize("center", _CONFIRMED_BOARDS)
def test_confirmed_board_centers_are_in_bbox(center):
    assert _BOX.contains(np.array(center))


@pytest.mark.parametrize("center", _CONFIRMED_CLUTTER)
def test_confirmed_clutter_is_outside_bbox(center):
    assert not _BOX.contains(np.array(center))


@pytest.mark.parametrize("center", _CONFIRMED_CLUTTER)
def test_confirmed_clutter_is_recognized_as_known(center):
    assert loo.near_known_clutter(np.array(center))


def test_true_board_is_not_flagged_as_known_clutter():
    for center in _CONFIRMED_BOARDS:
        assert not loo.near_known_clutter(np.array(center))


def _frames(x: float, n: int = 2) -> list[Frame]:
    """n frames of a small planar patch centred at (x, 0, 0)."""
    ys = np.arange(-0.2, 0.2, 0.02)
    yy, zz = np.meshgrid(ys, ys)
    xyz = np.stack([np.full(yy.size, x), yy.ravel(), zz.ravel()],
                   axis=1).astype(np.float32)
    return [Frame(stamp=float(i), xyz=xyz,
                  intensity=np.zeros(len(xyz), dtype=np.float32),
                  ring=np.zeros(len(xyz), dtype=np.uint8)) for i in range(n)]


def test_save_overlays_writes_pngs_and_is_off_by_default(tmp_path):
    """save_overlays=0 writes no overlay PNGs; >0 writes at most that many
    per fold, and the returned summary is identical either way."""
    from boarddet.board_config import BoardConfig
    sources = {"A": _frames(1.0), "B": _frames(1.0),
               "C": _frames(1.0), "D": _frames(9.0)}
    board = BoardConfig(side_m=1.0)

    off = tmp_path / "off"
    s0 = loo.run_loo(sources, board, off, box=_BOX, min_sources=2,
                     dilation_radius=0, save_overlays=0)
    assert list(off.glob("overlay_*.png")) == []

    on = tmp_path / "on"
    s1 = loo.run_loo(sources, board, on, box=_BOX, min_sources=2,
                     dilation_radius=0, save_overlays=2)
    pngs = list(on.glob("overlay_*.png"))
    assert len(pngs) > 0
    # at most save_overlays per fold (4 folds x 2)
    assert len(pngs) <= 8

    # Numbers unaffected by rendering. median_total_ms is a per-run timing
    # measurement that jitters between runs, so compare every field except
    # that one -- the counts and recall/precision are what must be identical.
    def _counts(summary):
        return {k: {kk: vv for kk, vv in v.items() if kk != "median_total_ms"}
                for k, v in summary["folds"].items()}
    assert _counts(s0) == _counts(s1)


def test_build_background_excludes_the_held_out_dataset():
    """The core LOO invariant. Dataset C's own geometry must never enter
    its own background, or the fold is self-referential."""
    all_frames = {"A": _frames(1.0), "B": _frames(1.0), "C": _frames(9.0)}
    model = loo.build_background(all_frames, held_out="C", voxel=0.06,
                                 dilation_radius=0, min_sources=1)
    assert model.n_sources == 2
    held = all_frames["C"][0].xyz
    assert len(model.foreground_points(held)) == len(held)


def test_unreachable_min_sources_is_rejected_loudly(tmp_path):
    """With N datasets each fold has only N-1 contributors, so
    min_sources > N-1 finalizes an EMPTY background: everything reads as
    foreground and the fold reports a meaninglessly high recall from a
    detector no longer doing background subtraction. Must raise, not run."""
    from boarddet.board_config import BoardConfig
    with pytest.raises(ValueError, match="unreachable"):
        loo.run_loo([1, 3], BoardConfig(side_m=1.0), tmp_path, box=_BOX,
                    min_sources=2)


def test_consensus_drops_a_single_contributors_unique_geometry():
    """With min_sources=2, geometry only one contributor saw (its own
    board) must not become background."""
    all_frames = {"A": _frames(1.0), "B": _frames(1.0), "C": _frames(5.0),
                  "D": _frames(9.0)}
    model = loo.build_background(all_frames, held_out="D", voxel=0.06,
                                 dilation_radius=0, min_sources=2)
    unique = all_frames["C"][0].xyz        # seen by contributor C alone
    shared = all_frames["A"][0].xyz        # seen by contributors A and B
    assert len(model.foreground_points(unique)) == len(unique)
    assert len(model.foreground_points(shared)) == 0


def test_run_loo_accepts_named_sources(tmp_path):
    """Folds are keyed by label, so pcap datasets ('3') and bags
    ('TWO_LIDAR_1') can both be held out by the same harness."""
    from boarddet.board_config import BoardConfig
    sources = {"A": _frames(1.0), "B": _frames(1.0),
               "C": _frames(1.0), "D": _frames(9.0)}
    out = loo.run_loo(sources, BoardConfig(side_m=1.0), tmp_path,
                      box=_BOX, min_sources=2, dilation_radius=0)
    assert set(out["folds"]) == {"A", "B", "C", "D"}


def test_unreachable_min_sources_still_rejected(tmp_path):
    from boarddet.board_config import BoardConfig
    sources = {"A": _frames(1.0), "B": _frames(9.0)}
    with pytest.raises(ValueError, match="unreachable"):
        loo.run_loo(sources, BoardConfig(side_m=1.0), tmp_path,
                    box=_BOX, min_sources=2)


def test_load_sources_rejects_unknown_kind():
    with pytest.raises(ValueError, match="kind"):
        loo.load_sources("floppy-disk", ["1"], "vlp32", None)
