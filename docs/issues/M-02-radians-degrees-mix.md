# M-02 · Advanced solver adjust/pose API mixes radians and degrees

- **Severity:** Medium
- **Area:** advanced_extrinsic_solver services
- **Status:** Not a bug (already unit-labeled)
- **Verified:** Static review
- **Location:** `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:804-806, 822, 868, 878, 1105`

## Problem

- `adjust_transform` applies `delta_roll/pitch/yaw` as **radians** (`R.from_euler("xyz", ...)`, no `degrees=True`).
- `get_pose_info` returns roll/pitch/yaw in **radians** (`as_euler("xyz")`).
- But the `adjust_transform` response string and the `_solve_from_buffer` log report RPY in **degrees** (`as_euler("xyz", degrees=True)`).

## Failure scenario

A client reads "30" from the service response (which is degrees) and sends 30 back to `adjust_transform` intending degrees — but it is applied as 30 radians (~1719°). A silent ~57× error during manual pose refinement.

## Suggested fix

Standardize on one unit across inputs, outputs, and log/response strings — recommend degrees for the human-facing service API — and label the unit in every field name or docstring.

## Status note (2026-07-11)

On review the structured API is internally consistent: `AdjustTransform.srv`
documents `delta_*` as radians, `adjust_transform` applies them as radians, and
`GetPoseInfo.srv` documents/returns radians — the interactive controller sends
radian steps and converts to degrees only for display (`math.degrees(...)`). The
only place degrees appear is the human-readable `adjust_transform` response
string and the solve log, and both already print the `deg` unit explicitly. So
there is no functional radians/degrees mismatch to fix; the value is correct and
labeled. Left as-is.
