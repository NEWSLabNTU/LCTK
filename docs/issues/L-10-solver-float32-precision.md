# L-10 · PnP correspondences and intrinsics are cast to float32

- **Severity:** Low
- **Area:** advanced_extrinsic_solver
- **Status:** Fixed (2026-07-13)
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:** `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1068-1069`, `:1298-1330`, `:1341-1342`

## Problem

Every array entering `cv2.solvePnP` is `float32`: the board rotation matrix
(`.astype(np.float32)`, `:1298`), the board position (`:1300`), the model corners (`:1321`),
the image corners (`:1324`), the concatenated buffers (`:1068-1069`), the camera matrix
(`:1341`) and the distortion vector (`:1342`). OpenCV then performs the solve in single
precision.

The rest of the pipeline — `scipy` rotations, the JSON dump (`:723`, `:726` use `float64`) —
is double precision, so this is an isolated downgrade, not a deliberate policy.

## Failure scenario

Not a correctness bug on its own: at metre-scale coordinates, `float32` gives ~1e-7 relative
precision, well below the pixel-noise floor that actually limits the solve. It matters for two
reasons:

1. It is a free loss of headroom, and it will start to matter once the residuals get small
   enough for the improvements in
   [phase 5](../roadmap/phase-5-stable-extrinsic-solution.md) to be measurable.
2. Conditioning is *already* the core problem ([H-07](./H-07-no-pose-diversity-gate.md)). When
   the normal equations are near-singular, single precision eats a meaningful fraction of the
   remaining significant digits, and any `cond(JᵀJ)` diagnostic computed in `float32` will
   itself be unreliable.

## Suggested fix

Use `np.float64` throughout the solver. There is no performance argument at 16·N points.

## Resolution (2026-07-13)

The advanced solver's PnP path is now `float64` throughout — board rotation/position,
model and image corners, the concatenated buffers, the camera matrix and distortion
vector. OpenCV `solvePnP`/`solvePnPRefineLM` accept `CV_64F`, and there is no
performance argument at 16·N points. This removes the isolated single-precision
downgrade and keeps any future `cond(JᵀJ)` diagnostic (H-09) trustworthy.
