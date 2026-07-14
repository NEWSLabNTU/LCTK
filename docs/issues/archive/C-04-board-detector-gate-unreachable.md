# C-04 · The board detector accepts nothing: the ICP gate is set below the sensor's noise floor

- **Severity:** Critical
- **Area:** lidar_board_detector → config
- **Status:** Fixed (2026-07-13)
- **Verified:** Yes (reproduced and measured on the shipped sample data, 2026-07-13)
- **Location:**
  - `ros/lctk_launch/config/board/board_detector.json5:23` (`icp_good_fit_threshold`)
  - `ros/lidar_board_detector/src/main.rs:1329` (the acceptance gate)
  - `ros/lidar_board_detector/src/main.rs:1425-1435` (the silent failure branch)
- **Introduced by:** commit `19a82f3` "Adjust lidar_board_detector params"

## Problem

**`just demo` produced zero calibrations, and reported no error.**

In the last recorded run (`play_log/2026-03-09_14-13-41`), the detector published **1,338 board
messages, of which 1,323 were empty**. The solver logged `"Received empty board detection in sync
group"` 1,323 times. The board detector logged exactly **one** warning in the entire run.

The acceptance gate (`main.rs:1329`) is:

```rust
if state.avg_loss < config.icp_good_fit_threshold
   && state.inlier_points.len() >= config.icp_min_inlier_points
```

Reproduced at `log_level=debug`, the measured values are:

```
Board detection failed: final_loss=0.027318, inliers=1038, threshold=0.012000, min_inliers=1000
Board detection failed: final_loss=0.028551, inliers=1053, threshold=0.012000, min_inliers=1000
Board detection failed: final_loss=0.026455, inliers=1040, threshold=0.012000, min_inliers=1000
```

The **inlier count passes** (1038–1062 ≥ 1000). The **loss fails**: ~0.027 against a threshold of
0.012.

### 0.027 is the noise floor, not a bad fit

Three independent lines of evidence say the fit is as good as this sensor can produce:

1. **The ICP asymptotes there.** Over 50 iterations the loss falls 0.0334 → 0.0263, with the gain
   per iteration decaying to ~1e-5. It terminates on `Max iterations reached: 50` while still
   creeping downward. Extrapolating the curve, it converges near 0.026 — it does not approach
   0.012. More iterations will not rescue it.
2. **The sensor cannot do better.** A Velodyne VLP-32C is specified at roughly **±3 cm** range
   accuracy. A mean point-to-model residual of 2.6 cm *is* the sensor noise. The gate was
   demanding a fit tighter than the LiDAR can physically measure.
3. **The project already measured this and wrote it down.** `CLAUDE.md`'s profiling section
   records "ICP quality is consistent across modes (**loss: 0.026–0.029**)" as normal operation —
   the very range the gate rejects.

### How it got here

`git log -L` on the threshold:

```
0.01  →  0.00000001  →  0.024  →  0.028  →  0.026  →  0.028  →  0.012
                        └────── working range ──────┘        ↑
                                                        commit 19a82f3
```

It lived at 0.024–0.028 (consistent with the measured 0.026–0.029 floor), and `19a82f3`
**tightened** it to 0.012. The trailing comment `// Relaxed to handle downsampled point clouds`
was carried along unchanged while the value moved in the opposite direction — so the code reads as
if it had been loosened.

### And the failure is silent

`main.rs:1425` logs the rejection at **`log_debug!`**, invisible at the default `info` level, then
returns `None` — publishing an empty detection array. The pipeline therefore fails **forever,
quietly, while reporting success upstream**. This is [H-09](./H-09-no-extrinsic-quality-metric.md)'s
complaint one layer earlier: nothing in the system can tell you it is not working.

## Failure scenario

Any user running `just demo` or `just calibrate` with the shipped board config gets no board
detections, no transform, and no error message — just an empty RViz and a solver that logs nothing
of interest at the default log level. There is no way to discover the cause without turning on
debug logging and reading ICP internals.

## Resolution (2026-07-13)

`icp_good_fit_threshold: 0.012` → **`0.035`**, with a comment recording *why* the number is what it
is (above the VLP-32C noise floor and above the measured 0.0286 worst case, while still low enough
to reject gross misfits) rather than a stale note about a change that went the other way.

**Verified end-to-end on the shipped sample data:**

| | before | after |
|---|---|---|
| `Board detection successful` | **0** | **1,049** |
| `Received empty board detection` | 1,323 | **0** |
| `PnP solved successfully` | 0 | **1,031** |

The solved extrinsic is plausible: translation `[0.070, −0.021, −0.886] m` — a ~0.89 m,
mostly-vertical LiDAR-to-camera offset. Sync statistics clean (`received=2143, groups=1047,
rejection_rate=0.0%`).

## Follow-ups (not fixed here)

- **The rejection must not be silent.** `main.rs:1425` should log at `warn` (rate-limited), and
  report *which* condition failed and by how much. A detector that emits empty detections forever
  should be loud. Tracked as part of [H-09](./H-09-no-extrinsic-quality-metric.md).
- **`max_icp_iterations: 50` truncates convergence** — the loss is still decreasing when ICP stops.
  The wayside config uses 1000. Worth revisiting, but it is not what caused this bug and was left
  alone to keep the fix to a single variable.
- **A threshold that is a bare number is a trap.** The right long-term shape is a gate expressed
  relative to the sensor's noise model, not a hand-tuned constant that drifts across commits with a
  comment describing the opposite of what happened.
