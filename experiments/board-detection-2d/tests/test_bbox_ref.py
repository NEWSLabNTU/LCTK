"""The true-board reference box, loaded from a bbox.json5 rather than
hardcoded, and honouring the box's own rotation."""
from __future__ import annotations

import numpy as np
import pytest

import pathlib

from boarddet.bbox_ref import load_bbox

# Confirmed true-board centres, phase-7 doc "Pose sanity" table.
_BOARDS = [
    (2.256, -0.059, 0.074), (2.147, 0.420, 0.076), (2.101, -0.314, 0.074),
    (2.077, -0.605, 0.066), (2.090, -0.829, 0.039),
]
# Documented static clutter attractors.
_CLUTTER = [(-1.83, -2.89, -0.1), (4.7, 2.6, -0.1), (-3.3, 3.4, 0.5)]

# The pcap rig's reference, as used by stages 3-8 and Method E. This is the
# sample-data session's own crop box. It used to name config/board/bbox.json5,
# which was a different rig's box entirely -- that file had been retuned for a
# Seyond rosbag, so every assertion below was checking the pcap rig's confirmed
# board centres against a box that does not contain them (M-29).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_PCAP_BBOX = _REPO_ROOT / "sessions/sample3-hollow-velodyne/bbox.json5"


def test_reads_json5_with_comments():
    """The real reference file has // comments; strict json cannot parse it."""
    box = load_bbox(_PCAP_BBOX)
    assert np.allclose(box.center, [2.6, 0.0, 0.35])
    assert np.allclose(box.half, [1.55, 1.97, 1.1])


@pytest.mark.parametrize("center", _BOARDS)
def test_confirmed_boards_are_inside(center):
    assert load_bbox(_PCAP_BBOX).contains(np.array(center))


@pytest.mark.parametrize("center", _CLUTTER)
def test_confirmed_clutter_is_outside(center):
    assert not load_bbox(_PCAP_BBOX).contains(np.array(center))


def test_rotation_is_applied(tmp_path):
    """A box rotated 90 deg about z swaps which points fall inside. Without
    rotation handling this test's second assertion passes wrongly."""
    p = tmp_path / "rot.json5"
    p.write_text("""{
        // 90 deg about z, quaternion in (x, y, z, w) order
        "pose": {"translation": [0.0, 0.0, 0.0],
                 "rotation": [0.0, 0.0, 0.7071067811865476, 0.7071067811865476]},
        "size_xyz": [4.0, 1.0, 1.0]
    }""")
    box = load_bbox(p)
    # The long axis now points along world y, not world x.
    assert box.contains(np.array([0.0, 1.5, 0.0]))
    assert not box.contains(np.array([1.5, 0.0, 0.0]))


def test_identity_rotation_is_axis_aligned(tmp_path):
    p = tmp_path / "ident.json5"
    p.write_text('{"pose": {"translation": [0,0,0], "rotation": [0,0,0,1]},'
                 ' "size_xyz": [2.0, 2.0, 2.0]}')
    box = load_bbox(p)
    assert box.contains(np.array([0.9, 0.9, 0.9]))
    assert not box.contains(np.array([1.1, 0.0, 0.0]))


def test_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_bbox(tmp_path / "nope.json5")
