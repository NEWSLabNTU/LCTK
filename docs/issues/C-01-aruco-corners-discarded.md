# C-01 · ArUco marker corners discarded; PnP uses an axis-aligned bbox as if it were the corners

- **Severity:** Critical
- **Area:** aruco_locator_node → extrinsic solvers
- **Status:** Open
- **Verified:** Yes (confirmed against live source, 2026-07-09)
- **Location:**
  - `ros/aruco_locator_node/src/main.rs:62-108`
  - `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:378-391`
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1159-1165`

## Problem

`convert_marker_to_detection2d` computes the 4 real marker corners, then discards them and publishes only `calculate_bounding_box(&corners)` (center + `size_x`/`size_y`). `Detection2D` carries no corner field. Both solvers reconstruct "corners" from the bounding box as an axis-aligned rectangle:

```python
corners = [
    (center_x - size_x/2, center_y - size_y/2),  # TL
    (center_x + size_x/2, center_y - size_y/2),  # TR
    (center_x + size_x/2, center_y + size_y/2),  # BR
    (center_x - size_x/2, center_y + size_y/2),  # BL
]
```

The correspondences fed to `cv2.solvePnP` are therefore not the true marker corners.

## Failure scenario

For any marker seen rotated or under perspective — i.e. in essentially every real capture — the reconstructed axis-aligned corners do not match the true corners. The 3D↔2D correspondences are systematically wrong, so the computed extrinsic is biased. This is the core output of the entire toolkit. It only produces a correct result for a fronto-parallel, image-axis-aligned board.

## Suggested fix

Add the 4 corner points to the detection message (extend `Detection2D` usage or publish a custom message / put corners in the result pose array) and consume the real corners in the solvers. Do not derive corners from the bounding box.
