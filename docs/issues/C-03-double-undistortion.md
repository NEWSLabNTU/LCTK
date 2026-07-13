# C-03 · Image is undistorted twice before ArUco detection

- **Severity:** Critical
- **Area:** aruco_locator_node → aruco-detector
- **Status:** Fixed (2026-07-12)
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:**
  - `ros/aruco_locator_node/src/main.rs:475-508` (`undistort_image`), called at `main.rs:525`
  - `rust/aruco-detector/src/multi_aruco.rs:344-372` (`Detector::detect_markers`), undistort at `multi_aruco.rs:366`
  - `rust/aruco-locator/src/lib.rs:79` (the call that connects them)
  - Also: `rust/aruco-locator/src/lib.rs:115` (`create_visualization` undistorts the already-undistorted overlay image)

## Problem

`process_image` undistorts the incoming frame and hands the result to the detector:

```rust
// ros/aruco_locator_node/src/main.rs:524-529
let processed_mat = Self::undistort_image(&mat, calibration)?;
log_debug!(LOGGER_NAME, "Image undistorted for ArUco detection");

// Detect ArUco markers on processed (undistorted) image
let detection_result = detector.detect_markers(&processed_mat)?;
```

`ArucoDetector::detect_markers` (`rust/aruco-locator/src/lib.rs:79`) forwards straight to
`aruco_detector::multi_aruco::Detector::detect_markers`, which undistorts *again*:

```rust
// rust/aruco-detector/src/multi_aruco.rs:363-372
let mut canvas = Mat::default();

// undistord image
calib3d::undistort(
    mat,
    &mut canvas,
    &camera_matrix,
    &distortion_coefs,
    &core_cv::no_array(),
)?;
```

Both calls pass `newCameraMatrix = no_array()`, so both use the same `K` and both apply the
full radial/tangential correction. The second call therefore applies the distortion correction
to an image that has already had it removed.

## Failure scenario

`undistort` warps the image so that a point at radius `r` moves by the lens correction `δ(r)`.
Running it twice moves the point by roughly `2·δ(r)` instead of `δ(r)` — i.e. the residual
geometric error at a corner is about as large as the lens distortion the calibration was
supposed to remove. Corners are then reported in a doubly-corrected frame, but everything
downstream (`cv2.solvePnP` with `dist_coeffs = np.zeros(5)`) assumes an ideal pinhole under
the same `K`. Every 2D-3D correspondence is systematically wrong.

Two properties make this worse than a constant offset:

1. **The bias is radius-dependent.** Markers near the principal point are barely affected;
   markers near the image border are badly displaced. For a typical wide-FoV automotive lens
   whose correction already moves border pixels by tens of pixels, the residual is of that
   same order.
2. **It poisons the fix for [H-07](./H-07-no-pose-diversity-gate.md).** The standard remedy
   for an ill-conditioned extrinsic is to spread the board across the whole field of view.
   With C-03 open, poses at the image border carry the *largest* bias, so widening FoV
   coverage injects more systematic error rather than less. **C-03 must be fixed before any
   pose-diversity guidance is issued.**

The overlay check hides this: a board sitting near the image centre reprojects fine, which is
exactly the "looks right at the board" symptom.

## Suggested fix

Undistort exactly once. Preferred: delete the undistort inside
`multi_aruco.rs:366` and make `Detector::detect_markers` document that it expects a rectified
image (the node already guarantees one, and `process_image` returns `processed_mat` for the
overlay too). Alternatively, drop `undistort_image` from the node and let the detector own it —
but then `main.rs:743` must stop reusing `processed_mat` as an already-rectified overlay.

Whichever side wins, add a regression test that a synthetic marker rendered through a known
`D` recovers its corners to sub-pixel accuracy after one detector pass.

Also remove the third undistort in `rust/aruco-locator/src/lib.rs:115`
(`create_visualization` is handed the already-rectified `processed_mat`), which makes the
debug overlay disagree with the geometry the solver actually used.

## Resolution (2026-07-12)

Rectification is now explicit, happens exactly once, and is owned by the detector.

- **`aruco-detector`**: new `Detector::rectify()` is the single place `calib3d::undistort` is
  called on the detection path. `detect_markers` and `detect_single_aruco` no longer warp the
  image; both document that they consume an already-rectified frame, and both feed `mat`
  straight to `aruco::detect_markers`.
- **`aruco-locator`**: `ArucoDetector::rectify()` passes through to the detector.
  `create_visualization` no longer undistorts — it clones the (already rectified) image it is
  given, so the drawn corners land where the detector actually found them.
- **`aruco_locator_node`**: its private `undistort_image` is deleted and it calls
  `detector.rectify()` instead. The `CameraCalibration` struct — a *second* copy of `K` and `D`
  living alongside the detector's own — is gone entirely; that duplication is what let the two
  rectification sites drift apart in the first place. The detector's existence already implies
  camera_info has arrived, so it doubles as the readiness check.
- Both CLI entry points (`rust/aruco-locator/src/main.rs`,
  `rust/aruco-detector/examples/detect-aruco.rs`) now call `rectify()` explicitly, since they
  previously relied on the detector doing it implicitly.

**Verified:** `rust/aruco-detector/tests/rectify_contract.rs` pins the contract with three
tests, run against a board rendered from the shipped ArUco config:

- `detect_markers_does_not_rectify` — two detectors differing *only* in their distortion
  coefficients must report identical corners for the same image.
- `rectify_with_zero_distortion_preserves_corners` — rectifying with `D = 0` is a no-op.
- `rectify_with_real_distortion_moves_corners` — guards against the first test passing
  vacuously because the distortion model was inert.

The first test was confirmed to **fail on the old code**: with the internal undistort restored,
marker 696's corner moved from `(112, 112)` to `(84, 84)` — a 40 px displacement on a ~900 px
image, which is the scale of bias every calibration was carrying.
