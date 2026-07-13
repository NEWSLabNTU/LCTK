# L-11 · ArUco detector parameter block sets the same field twice and configures a disabled refiner

- **Severity:** Low
- **Area:** aruco-detector
- **Status:** Fixed (2026-07-13)
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:** `rust/aruco-detector/src/multi_aruco.rs:379-388`, duplicated at `multi_aruco.rs:474-482`

## Problem

```rust
params.set_adaptive_thresh_win_size_min(13);
params.set_adaptive_thresh_win_size_max(33);
params.set_adaptive_thresh_win_size_step(2);
params.set_adaptive_thresh_win_size_step(10);   // overwrites the line above
params.set_corner_refinement_min_accuracy(0.01);// no-op: refinement is CORNER_REFINE_NONE
```

Two defects in five lines:

- `set_adaptive_thresh_win_size_step` is called twice. The `2` is dead; the effective step is
  `10`, so adaptive thresholding tries window sizes `{13, 23, 33}` instead of
  `{13, 15, …, 33}`. Whether that was the intent is unknowable from the code — it reads as an
  editing accident.
- `set_corner_refinement_min_accuracy(0.01)` tunes a refiner that never runs. See
  [H-08](./H-08-no-subpixel-corner-refinement.md).

The whole block is also copy-pasted into a second, near-identical site at `:474-482`.

## Failure scenario

A coarse threshold sweep costs detections on low-contrast or unevenly lit boards — the marker
is simply not found, and the all-or-nothing ID gate (`multi_aruco.rs:407-410`) then discards
the entire frame. Not incorrect, but it silently reduces the pose yield that
[H-07](./H-07-no-pose-diversity-gate.md) needs.

## Suggested fix

Pick one step value deliberately and comment why. Factor the parameter block into a single
constructor shared by both call sites. Fold into the H-08 change.

## Resolution (2026-07-13)

Fixed as part of [H-08](./H-08-no-subpixel-corner-refinement.md). Both copies of the block are
gone: `DetectorParameters` is now constructed in exactly one place,
`ArucoDetectorParams::to_opencv_params()` in `rust/aruco-config/src/detector_params.rs`, and both
`detect_markers` and `detect_single_aruco` call it.

The values moved out of Rust and into `ros/lctk_launch/config/aruco/aruco_detector.json5`, where
`win_size_step` is set once (to 10, the value that was actually taking effect) with a comment
recording that the dead `2` was an editing accident. `min_accuracy` now tunes a refiner that
actually runs.
