# H-08 · ArUco corners are never sub-pixel refined

- **Severity:** High
- **Area:** aruco-detector
- **Status:** Fixed (2026-07-13)
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Design:** [docs/superpowers/specs/2026-07-13-h08-subpixel-corner-refinement-design.md](../superpowers/specs/2026-07-13-h08-subpixel-corner-refinement-design.md)
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

Corner localisation error is the direct input noise of the PnP solve. With only 16 corners per
pose, and only ~6 DoF of independent information per pose
([H-07](./H-07-no-pose-diversity-gate.md)), there is no redundancy to average it away — it goes
straight into the extrinsic.

## Resolution (2026-07-13)

Refinement is enabled, configurable, and — importantly — moved to where the gradients still
exist.

**Refinement now runs on the RAW frame.** Sub-pixel refinement reads image gradients, and
`undistort` resamples the image bilinearly, blunting exactly those gradients. So the detector no
longer consumes a rectified image: it detects and refines on the unresampled sensor pixels, then
maps the 16 corners into the rectified frame with `undistortPoints` (`R = I`, `P = K`, iterative
`TermCriteria` — the 5-iteration default leaves residual error under strong distortion).

Consequences:

- The published contract is **unchanged** — corners still arrive in the rectified frame, so both
  solvers keep `dist_coeffs = 0` and needed no edits. But it now holds *exactly*, at the point
  level, instead of *approximately*, via a warped image. This retires the fragile implicit
  contract of [M-11](./M-11-solvers-ignore-distortion.md).
- The full-frame `undistort` survives only to draw the debug overlay, so it is **skipped entirely
  when the overlay is off** — saving a full-image warp per frame.
- Detector tuning moved into a new `config/aruco/aruco_detector.json5`, built through a single
  `ArucoDetectorParams::to_opencv_params()`. That is now the only place `DetectorParameters` is
  constructed, which deletes the duplicated block and closes
  [L-11](./L-11-detector-param-block-bugs.md).

### Measured gain

`cargo run -p aruco-detector --example refinement-sweep` — a board rendered at 200 dpi and
downscaled by exact factors, so true corner positions are known analytically and land off the
pixel grid. RMSE in pixels:

| marker size | NONE | **SUBPIX** | CONTOUR |
|---|---|---|---|
| 302 px | 1.129 | **0.923** | 1.380 |
| 216 px | 0.603 | **0.243** | 0.603 |
| 151 px | 0.801 | **0.360** | 0.801 |
| 108 px | 0.973 | **0.603** | 1.108 |
| 76 px | 0.783 | **0.589** | 0.743 |
| 54 px | 1.045 | **0.612** | 0.914 |

**SUBPIX beats NONE at every size, by 25–60%.** CONTOUR is no better than NONE and sometimes
worse, so SUBPIX is the shipped default. `win_size` ∈ {3, 5, 7} changes the result by <0.02 px
across the whole range, so 5 is safe from ~300 px down to ~54 px — comfortably past the ~35 px
the markers subtend at 6 m.

**Honest caveat on the absolute numbers.** The literature's "~0.05–0.1 px" figure is *not* what
this harness measures, and the earlier version of this issue quoted it without evidence. Ground
truth here is itself derived from detecting on the reference render, so the harness's own error
floors these values; the INTER_AREA downscale adds more. The **relative** improvement is the
trustworthy signal. Real-world absolute accuracy should be re-measured on the sample rosbag once
[H-09](./H-09-no-extrinsic-quality-metric.md) provides a reprojection metric.

**Verified:** `rust/aruco-detector/tests/rectify_contract.rs` (5 tests). Both bugs were
re-introduced to confirm the tests actually catch them:

- removing the `undistortPoints` step → `corners_survive_the_distort_detect_undistort_round_trip`
  fails.
- removing `set_corner_refinement_method` (the original H-08 bug) →
  `subpix_refinement_beats_no_refinement` fails with *"SUBPIX and NONE produced the same corners
  (0.0000 px apart), so corner refinement is not running at all"*.
