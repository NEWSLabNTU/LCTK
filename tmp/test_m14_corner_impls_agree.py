#!/usr/bin/env python3
"""M-14 (part 3): the two Python `_compute_multi_marker_corners` implementations
(advanced_extrinsic_solver and extrinsic_solver_node) must agree corner-for-corner.

They are duplicated in two packages with no cross-check; a silent divergence would
fold a corner permutation into the extrinsic. This asserts they produce identical
board-frame corners for a representative config, so future drift is caught.

Requires: source install/setup.bash
"""
import sys


class _StubLogger:
    def debug(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass


CONFIG = {
    "board_size": "500mm",
    "board_border_size": "10mm",
    "marker_square_size_ratio": 0.8,
    "num_squares_per_side": 2,
    "marker_ids": [696, 64, 306, 195],
}


def corners_from(cls):
    node = object.__new__(cls)  # skip ROS __init__
    node.aruco_pattern_config = dict(CONFIG)
    node.get_logger = lambda: _StubLogger()
    return node._compute_multi_marker_corners()


def main():
    from advanced_extrinsic_solver.main import AdvancedExtrinsicSolver
    from extrinsic_solver_node.main import EducationalExtrinsicSolver

    adv = corners_from(AdvancedExtrinsicSolver)
    ext = corners_from(EducationalExtrinsicSolver)

    assert set(adv.keys()) == set(ext.keys()), (
        f"marker id sets differ: {sorted(adv)} vs {sorted(ext)}"
    )
    for mid in adv:
        a = [tuple(round(c, 9) for c in pt) for pt in adv[mid]]
        e = [tuple(round(c, 9) for c in pt) for pt in ext[mid]]
        assert a == e, f"marker {mid} corners differ:\n adv={a}\n ext={e}"

    print(f"M-14 PASS: both implementations agree for all {len(adv)} markers")
    print(f"  e.g. marker {CONFIG['marker_ids'][0]}: {adv[CONFIG['marker_ids'][0]]}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # noqa: BLE001
        print(f"M-14 FAIL: {e}")
        sys.exit(1)
