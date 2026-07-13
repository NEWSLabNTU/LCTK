# H-10 · Saving and reloading detections silently re-introduces C-01

- **Severity:** High
- **Area:** advanced_extrinsic_solver
- **Status:** Open
- **Verified:** Yes (confirmed against live source, 2026-07-13)
- **Location:**
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:897-921` (`_serialize_detection2d_array`, the writer)
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1160-1180` (`_detection2d_to_aruco_markers`, the reader)

## Problem

[C-01](./C-01-aruco-corners-discarded.md) was fixed by publishing the four *real* ArUco corner
pixels in `Detection2D.results[]`, because reconstructing corners from the axis-aligned bounding
box discards rotation and perspective and biases the PnP correspondences for any angled view.

The reader honours that. It prefers `results`, and only falls back to the bbox rectangle:

```python
# main.py:1163-1180
# C-01: prefer the real per-corner pixel coordinates. Reconstructing
# corners from `center +/- size/2` discards rotation and perspective,
# biasing the PnP correspondences for any angled view of the board.
if len(detection.results) >= 4:
    corners = [
        (r.pose.pose.position.x, r.pose.pose.position.y)
        for r in detection.results[:4]
    ]
else:
    size_x = bbox.size_x
    ...
```

**But the writer never saves `results`.** `_serialize_detection2d_array` emits only the bounding
box:

```python
# main.py:907-920
"detections": [
    {
        "id": d.id if hasattr(d, "id") else "",
        "bbox": {
            "center": {"x": d.bbox.center.position.x, "y": d.bbox.center.position.y},
            "size_x": d.bbox.size_x,
            "size_y": d.bbox.size_y,
        },
    }
    for d in msg.detections
],
```

So on `load_detections`, every restored detection has `len(detection.results) == 0`, the reader
takes the `else` branch, and the corners are reconstructed from the axis-aligned bbox — **exactly
the bug C-01 fixed**. The comment explaining why that path is wrong sits directly above the branch
the loader is forced into.

Note the asymmetry: `_serialize_detection3d_array` (`main.py:923-956`) *does* preserve `results`.
Only the 2D side drops them.

## Failure scenario

An operator buffers ten good poses, saves them with `dump_detections`, and reloads them the next
day (or on another machine, or to re-solve after a code change). The reloaded calibration is
computed from axis-aligned bbox corners and is systematically wrong for every non-fronto-parallel
board view — while reporting `"Calibration successful"`, because nothing measures reprojection
error ([H-09](./H-09-no-extrinsic-quality-metric.md)).

**A saved calibration cannot even re-solve to the answer it was saved with.** That makes the dump
format unfit for its stated purposes: reproducing a result, sharing a capture, or re-running an
improved solver over old data.

## Suggested fix

Serialize `results` on the 2D side, mirroring what the 3D side already does — i.e. write the four
corner pixels. Bump to `version: 3` and keep the v2 loader path, but have it **warn loudly** that a
v2 file carries no real corners and will produce a biased solve, rather than silently degrading.

Add a round-trip test: buffer a detection with known rotated corners, dump, load, and assert the
corners survive. The existing detection-file fixtures make this cheap.
