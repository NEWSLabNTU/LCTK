# H-08 · ArUco corners are never sub-pixel refined

- **Severity:** High
- **Area:** aruco-detector
- **Status:** Open
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:** `rust/aruco-detector/src/multi_aruco.rs:379-388` (and the duplicate block at `multi_aruco.rs:474-482`)

## Problem

The detector parameters are built like this:

```rust
// rust/aruco-detector/src/multi_aruco.rs:379-388
let parameters = {
    let mut params = aruco::DetectorParameters::create()?;
    params.set_marker_border_bits(border_bits as i32);
    params.set_adaptive_thresh_win_size_min(13);
    params.set_adaptive_thresh_win_size_max(33);
    params.set_adaptive_thresh_win_size_step(2);
    params.set_adaptive_thresh_win_size_step(10);
    params.set_corner_refinement_min_accuracy(0.01);
    params
};
```

`set_corner_refinement_method(...)` is **never called anywhere in the repository**
(`grep -rn "corner_refine\|CORNER_REFINE\|cornerSubPix" rust ros` returns only the two
`set_corner_refinement_min_accuracy` lines above). OpenCV's `DetectorParameters` defaults
`cornerRefinementMethod = CORNER_REFINE_NONE`, so:

- `set_corner_refinement_min_accuracy(0.01)` is a **no-op** — it configures a refiner that
  never runs.
- The corners `detectMarkers` returns are the raw contour/quad intersections, quantised to
  roughly the pixel grid.

## Failure scenario

Corner localisation error is the direct input noise of the PnP solve. Going from
`CORNER_REFINE_NONE` (~0.5–1 px, plus quantisation bias) to `CORNER_REFINE_SUBPIX`
(~0.05–0.1 px on a well-exposed marker) is close to an order of magnitude on the dominant
image-side error term, for a few lines of code. With only 16 corners per pose, and only ~6 DoF
of independent information per pose ([H-07](./H-07-no-pose-diversity-gate.md)), there is no
redundancy to average this away.

## Suggested fix

```rust
params.set_corner_refinement_method(aruco::CORNER_REFINE_SUBPIX);
params.set_corner_refinement_win_size(5);
params.set_corner_refinement_max_iterations(30);
params.set_corner_refinement_min_accuracy(0.01);   // now actually has an effect
```

`CORNER_REFINE_CONTOUR` is the alternative if the markers are small in the image.
Note that sub-pixel refinement operates on the image passed to `detect_markers`, so it will
faithfully refine whatever geometry that image has — it does **not** compensate for
[C-03](./C-03-double-undistortion.md), and the two must be fixed together for the improvement
to be real.

While in this block, also fix the duplicated `set_adaptive_thresh_win_size_step` (see
[L-11](./L-11-detector-param-block-bugs.md)).
