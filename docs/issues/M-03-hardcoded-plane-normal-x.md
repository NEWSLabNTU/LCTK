# M-03 · Hardcoded plane-normal flip to +X assumes the board always faces sensor-forward-X

- **Severity:** Medium
- **Area:** hollow-board-detector (live path)
- **Status:** Open
- **Verified:** Static review
- **Location:** `rust/hollow-board-detector/src/algo.rs:197-203` (`desired_front = Vector3::x_axis()`)

## Problem

After RANSAC plane fitting, the normal is force-flipped so that `normal · x_axis ≥ 0`. This bakes in the assumption that the board always faces the sensor along +X. Downstream, the board's +Z axis derives from this normal.

## Failure scenario

Reusing the crate on a sensor whose forward axis is not +X (e.g. camera-forward-Z, or Y-forward), or with a board mounted to the side or behind, flips the normal into the wrong hemisphere. Best case ICP loss stays high and the frame is rejected; worst case it converges to a mirrored in-plane orientation. This is on the live production path.

## Suggested fix

Derive the "front" direction from the sensor→board vector (e.g. flip so the normal points toward the sensor origin) instead of a hardcoded axis, or make the desired-front axis a configurable parameter.
