# M-11 · Both solvers hardcode `dist_coeffs = 0` and never read `camera_info.d`

- **Severity:** Medium
- **Area:** advanced_extrinsic_solver, extrinsic_solver_node
- **Status:** Open
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:**
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1341-1353`
  - `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:707-723`

## Problem

```python
# advanced_extrinsic_solver/main.py:1341-1353
K = np.array(self.camera_info.k, dtype=np.float32).reshape(3, 3)
dist_coeffs = np.zeros(5, dtype=np.float32)
success, rvec, tvec = cv2.solvePnP(
    object_points, image_points, K, dist_coeffs, flags=cv2.SOLVEPNP_SQPNP,
)
```

`camera_info.d` is subscribed but never used by either solver. Passing zeros is *correct only
under the unstated invariant* that the corners arriving on the wire were detected in an
image that was rectified exactly once with this same `K`. That invariant is:

- **undocumented** — nothing in the solver, the message, or the READMEs states it;
- **currently violated** — the image is rectified twice ([C-03](./C-03-double-undistortion.md));
- **fragile** — anyone adding a second corner producer, or changing
  `undistort`'s `newCameraMatrix` from `no_array()` to an optimal one, silently breaks the
  solve with no error.

## Failure scenario

Any change to the rectification path (a new detector, an alpha-scaled `newCameraMatrix`, a
consumer that publishes raw corners) produces a systematically wrong extrinsic with no warning,
because `solvePnP` is told the lens is perfect.

## Suggested fix

Make the contract explicit. Two coherent options:

- **Rectify once, keep `dist_coeffs = 0`** (matches current intent): fix C-03, and state in the
  `Detection2D` producer docs and the solver docstring that corners are in the rectified frame
  of `camera_info.k`. Add an assertion that `camera_info.d` is non-trivial *and* the producer
  advertises rectification (e.g. via a parameter), so the pairing can't drift.
- **Publish raw corners and let PnP model the lens**: pass `np.array(self.camera_info.d)` into
  `solvePnP`. This is what the (dead) `rust/pnp-solver` already does correctly
  (`rust/pnp-solver/src/lib.rs:68-78`) — see [L-12](./L-12-dead-solver-crates.md).

The second is the more robust design (no resampling of the image, no interpolation blur before
sub-pixel corner refinement), but it is the larger change.
