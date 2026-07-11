#!/usr/bin/env python3
"""C-01 verification: solver reads REAL corners from Detection2D.results.

Standalone (no ROS): reproduces the exact parsing logic added to
extrinsic_solver_node and advanced_extrinsic_solver `_detection2d_to_aruco_markers`
and asserts:

1. When a detection carries >=4 results, the 4 corner pixel coords come from
   `results[i].pose.pose.position`, NOT from the axis-aligned bbox.
2. A perspective/rotated marker (whose true corners are NOT an axis-aligned
   rectangle) is recovered exactly -- the old bbox reconstruction would have
   returned a wrong axis-aligned box.
3. When a detection has <4 results, it falls back to the bbox rectangle.
"""
from types import SimpleNamespace


def ns_position(x, y):
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0))))


def make_detection(corners, det_id="aruco_696", carry_results=True):
    """Build a duck-typed Detection2D-like object."""
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    bbox = SimpleNamespace(
        center=SimpleNamespace(position=SimpleNamespace(x=cx, y=cy)),
        size_x=max(xs) - min(xs),
        size_y=max(ys) - min(ys),
    )
    results = [ns_position(x, y) for (x, y) in corners] if carry_results else []
    return SimpleNamespace(bbox=bbox, results=results, id=det_id)


def parse(detection):
    """Exact copy of the C-01 parsing logic in both solvers."""
    bbox = detection.bbox
    center = (bbox.center.position.x, bbox.center.position.y)
    if len(detection.results) >= 4:
        corners = [
            (r.pose.pose.position.x, r.pose.pose.position.y)
            for r in detection.results[:4]
        ]
    else:
        size_x = bbox.size_x
        size_y = bbox.size_y
        cx, cy = center
        corners = [
            (cx - size_x / 2.0, cy - size_y / 2.0),
            (cx + size_x / 2.0, cy - size_y / 2.0),
            (cx + size_x / 2.0, cy + size_y / 2.0),
            (cx - size_x / 2.0, cy + size_y / 2.0),
        ]
    return corners, center


def approx(a, b, eps=1e-9):
    return all(abs(x - y) < eps for p, q in zip(a, b) for x, y in zip(p, q))


def main():
    # A perspective view of a square marker: corners form a trapezoid, NOT an
    # axis-aligned rectangle. This is the case C-01 was getting wrong.
    true_corners = [
        (100.0, 200.0),  # TL
        (340.0, 190.0),  # TR (higher up -> perspective)
        (360.0, 430.0),  # BR
        ( 90.0, 450.0),  # BL
    ]

    # 1 + 2: corners carried in results are recovered exactly.
    det = make_detection(true_corners, carry_results=True)
    corners, _ = parse(det)
    assert approx(corners, true_corners), f"expected real corners, got {corners}"

    # Prove the OLD bbox reconstruction would have been wrong for this marker.
    xs = [c[0] for c in true_corners]
    ys = [c[1] for c in true_corners]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    sx, sy = max(xs) - min(xs), max(ys) - min(ys)
    old_bbox_corners = [
        (cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy - sy / 2),
        (cx + sx / 2, cy + sy / 2), (cx - sx / 2, cy + sy / 2),
    ]
    assert not approx(old_bbox_corners, true_corners), \
        "test is meaningless if bbox already equals true corners"
    max_err = max(abs(a - b) for p, q in zip(old_bbox_corners, true_corners)
                  for a, b in zip(p, q))
    print(f"[1,2] real corners recovered exactly; "
          f"old bbox reconstruction was off by up to {max_err:.1f} px")

    # 3: fallback to bbox when corners absent.
    det2 = make_detection(true_corners, carry_results=False)
    corners2, _ = parse(det2)
    assert approx(corners2, old_bbox_corners), "fallback should equal bbox box"
    print("[3]   fallback to axis-aligned bbox works when results < 4")

    print("\nC-01 consumer logic PASS")


if __name__ == "__main__":
    main()
