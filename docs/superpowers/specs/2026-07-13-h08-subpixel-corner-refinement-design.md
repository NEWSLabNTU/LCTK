# H-08: Sub-pixel ArUco Corner Refinement — Design

- **Date:** 2026-07-13
- **Issue:** [H-08](../../issues/archive/H-08-no-subpixel-corner-refinement.md)
- **Also closes:** [L-11](../../issues/archive/L-11-detector-param-block-bugs.md) (duplicated detector param block)
- **Also resolves:** [M-11](../../issues/archive/M-11-solvers-ignore-distortion.md)'s fragile implicit contract
- **Depends on:** [C-03](../../issues/archive/C-03-double-undistortion.md) (fixed 2026-07-12)
- **Phase:** [Phase 5, Stage 5.0](../../roadmap/phase-5-stable-extrinsic-solution.md)

## Problem

`set_corner_refinement_method` is never called anywhere in the repository, so OpenCV's default
`CORNER_REFINE_NONE` applies: ArUco corners are the raw contour/quad intersections, quantised to
roughly the pixel grid. The `set_corner_refinement_min_accuracy(0.01)` call at
`rust/aruco-detector/src/multi_aruco.rs:386` is a no-op — it tunes a refiner that never runs.

Corner localisation error is the direct input noise of the PnP solve, and with only 16 corners per
pose (carrying ~6 DoF of independent information, see H-07) there is no redundancy to average it
away.

> **Note (post-implementation):** an earlier draft of this spec asserted refinement would take the
> error "from ~0.5–1 px to ~0.05–0.1 px". That figure was received wisdom, not measurement, and the
> harness does not support it — see [Measured results](#measured-results). The improvement is real
> and large (25–60% lower RMSE), but the absolute numbers quoted above were not evidence.

## Design decisions

Four forks, all settled before implementation:

| Decision | Choice | Why |
|---|---|---|
| Where to refine | **Detect on the raw frame, undistort the corner *points*** | Sub-pixel refinement reads image gradients. After C-03 the detector's input is an `undistort`-ed image — i.e. bilinearly resampled, which blunts exactly the gradients SUBPIX depends on. Refining on the raw sensor image and then calling `undistortPoints` on the 16 corners is strictly more accurate: no resampling, and the point-wise undistortion is exact. |
| Refinement method | **`CORNER_REFINE_SUBPIX`, configurable** | Best-studied, best accuracy on well-resolved markers. Made configurable so `CONTOUR` can be swapped in without a rebuild, and so the harness can measure both. |
| Config location | **New `config/aruco/aruco_detector.json5`** | Mirrors the existing `board_detector.json5`. Keeps `aruco_pattern.json5` purely about the printed board — `aruco_generator_node` reads that file to *print* the board and has no business seeing detector tuning. |
| Verification | **Synthetic ground-truth harness** | Deterministic, CI-runnable, needs no rosbag, and independent of H-09 (which does not exist yet). |

## Architecture

The full-frame `undistort` leaves the detection path entirely. It survives only as an overlay
utility.

```
ImageMsg → raw Mat (distorted)
   │
   ├─► detector.detect_markers(raw)
   │      aruco::detect_markers(raw, params{SUBPIX, …})   ← native gradients, no resampling
   │        → corners in DISTORTED pixel coords
   │      calib3d::undistort_points_iter(corners, K, D, R=I, P=K, criteria)
   │        → corners in RECTIFIED pixel coords           ← published
   │
   └─► detector.rectify(raw)   ONLY when debug_overlay_enabled
          → Mat for the overlay
```

Two consequences:

- **The published contract is unchanged.** Corners still arrive in the rectified frame, so both
  solvers keep `dist_coeffs = 0` and need no edits. C-03's invariant now holds *exactly*, at the
  point level, rather than *approximately*, via a resampled image. This retires the fragile
  implicit contract described in M-11 — the rectification is now a property of the corners
  themselves, not of an image that some other stage might or might not have warped.
- **Free perf win.** A full-frame `undistort` on a 1080p image costs several ms per frame. It now
  runs only when the debug overlay is enabled.

### Pitfalls

- `undistort_points` **must** be called with `R = I` and `P = K`. With `P` omitted it returns
  *normalized* coordinates, not pixels — the one easy way to get this silently wrong. Use
  `undistort_points_iter` with an explicit `TermCriteria` rather than `undistort_points_def`,
  whose 5-iteration default leaves residual error under strong distortion.

## The C-03 contract inverts — its test must be rewritten

`Detector::detect_markers` currently promises *"give me a rectified image"*. It will now promise
*"give me a **raw** image; I return rectified corners"*.

So `rust/aruco-detector/tests/rectify_contract.rs::detect_markers_does_not_rectify` — which
asserts corners are identical for `D = 0` versus `D = big` on the same input — becomes **wrong by
design**. It must be *rewritten*, not deleted: the C-03 regression (never resample twice) still
needs a guard.

Its replacement is strictly stronger and subsumes both C-03 and H-08:

```
render ideal board            → ideal_corners   (exactly known: the marker ROI corners)
warp through known K, D       → synthetic distorted image
detect_markers(distorted)     → recovered corners
assert ‖recovered − ideal_corners‖ < tol
```

Round-trip closure. This fails if the image is rectified twice (C-03), if `undistort_points` is
misconfigured (`P` omitted), or if refinement is off (tolerance not met).

Synthesising the distorted image: build a `remap` table by running `undistort_points` over the
whole pixel grid — `map(u,v) = undistort_points((u,v), K, D, R=I, P=K)` — then `remap` the ideal
render through it. Exact, no approximation, and it reuses the same primitive under test in the
opposite direction.

## Configuration

New file `ros/lctk_launch/config/aruco/aruco_detector.json5`:

```json5
{
  // Corner localisation. NONE is OpenCV's default and is what H-08 fixes.
  "corner_refinement": {
    "method": "SUBPIX",      // NONE | SUBPIX | CONTOUR | APRILTAG
    "win_size": 5,
    "max_iterations": 30,
    "min_accuracy": 0.01,
  },
  // Adaptive thresholding sweep for marker candidate detection.
  "adaptive_thresh": {
    "win_size_min": 13,
    "win_size_max": 33,
    "win_size_step": 10,     // L-11: previously set twice (2, then 10); the 2 was dead
  },
}
```

- New `ArucoDetectorParams` type in `aruco-config`, with `to_opencv_params()` behind the existing
  `with-opencv` feature. **This becomes the single construction site for `DetectorParameters`**,
  which deletes the duplicated block at `multi_aruco.rs:379-388` and `:478-487` and closes L-11.
- `aruco_locator_node` gains a mandatory `aruco_detector_config_file` parameter, per the project
  convention that nodes carry no hidden defaults.
- `lctk_launch` supplies it from a new **optional** `aruco_detector_config` key on the marker,
  falling back to the shipped file when absent, so existing YAML configs keep working. This
  back-compat choice is deliberate: H-06 was config schema drift, and this must not repeat it.

## Verified API (OpenCV 4.5.4)

Confirmed by reading the *generated* bindings at
`target/debug/build/opencv-*/out/opencv/{aruco,calib3d}.rs` — not the crate's bundled docs, which
describe the 4.7+ `objdetect` layout and do **not** match what we build against:

- `opencv::aruco::CornerRefineMethod` — `CORNER_REFINE_{NONE=0, SUBPIX=1, CONTOUR=2, APRILTAG=3}`
- `set_corner_refinement_method(i32)`, `set_corner_refinement_win_size(i32)`,
  `set_corner_refinement_max_iterations(i32)`, `set_corner_refinement_min_accuracy(f64)`
- `aruco::DetectorParameters::create() -> Ptr<DetectorParameters>`
- `calib3d::undistort_points_iter(src, dst, K, D, R, P, TermCriteria)`
- `calib3d::init_undistort_rectify_map(...)`, `imgproc::remap(...)`

No fallback to a hand-rolled `cornerSubPix` is needed.

## Testing

In `rust/aruco-detector/tests/`. Deterministic, no rosbag, CI-runnable.

1. **Round-trip accuracy** (above) — the merge gate.
2. **Refinement is not vacuous** — assert `rmse(SUBPIX) < 0.15 px` **and** `rmse(NONE) > 0.30 px`.
   Without the second assertion the first could pass for the wrong reason.
3. **Sweep** — apparent marker size 35→200 px × `win_size` ∈ {3, 5, 7} × method ∈ {NONE, SUBPIX,
   CONTOUR}. Sets the default `win_size`, finds the marker size below which SUBPIX windows collide
   with adjacent corners, and produces the real SUBPIX-vs-CONTOUR curve instead of an assumption.

The board's markers are 192 mm on a 1000 mm plate, so at 1.5–6 m they span roughly 200 px down to
~35 px — the low end is where the methods diverge and where `win_size` collision becomes a risk.

## Measured results

`cargo run -p aruco-detector --example refinement-sweep`. Board rendered at 200 dpi and downscaled
by exact factors, so true corner positions are known analytically and land off the pixel grid.
Corner RMSE, in pixels:

| marker size | NONE | **SUBPIX** | CONTOUR |
|---|---|---|---|
| 302 px | 1.129 | **0.923** | 1.380 |
| 216 px | 0.603 | **0.243** | 0.603 |
| 151 px | 0.801 | **0.360** | 0.801 |
| 108 px | 0.973 | **0.603** | 1.108 |
| 76 px | 0.783 | **0.589** | 0.743 |
| 54 px | 1.045 | **0.612** | 0.914 |

Three things fall out, and two of them contradict what this spec assumed going in:

1. **SUBPIX wins everywhere**, by 25–60%. The default is confirmed.
2. **CONTOUR is not competitive.** It tracks NONE almost exactly and is sometimes worse. The spec
   expected it to overtake SUBPIX at small marker sizes; it does not. Measuring was worth it.
3. **`win_size` barely matters.** Across {3, 5, 7} the result moves by <0.02 px over the whole
   range, so 5 is safe from ~300 px down to ~54 px — well past the ~35 px the markers subtend at
   6 m. The predicted window-collision failure never materialised in the working range.

**Caveat on the absolute values.** Ground truth here is itself derived from detecting on the
reference render, so the harness's own error floors these numbers, and the INTER_AREA downscale
adds more. The *relative* comparison is what this harness can honestly support. Absolute accuracy
should be re-measured on the sample rosbag once H-09 provides a reprojection metric.

## Out of scope

- Publishing raw (distorted) corners and having the solvers model the lens. The corners stay in the
  rectified frame; the solvers are not touched.
- `CORNER_REFINE_APRILTAG`, which swaps the entire marker-detection pipeline rather than refining
  the corners of the existing one. Selectable via config, but not evaluated.
- Any H-09 (reprojection metric) work. Once H-09 lands, re-measure on the sample rosbag and append
  the real-world delta to the H-08 issue doc.
