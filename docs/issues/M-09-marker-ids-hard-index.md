# M-09 · `marker_ids[0..3]` hard index → IndexError on short config

- **Severity:** Medium
- **Area:** advanced_extrinsic_solver
- **Status:** Open
- **Verified:** Static review
- **Location:** `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1256-1261`

## Problem

The solver hard-indexes `marker_corners[marker_ids[0]]` … `[marker_ids[3]]` from config. A config with fewer than 4 `marker_ids` raises `IndexError` inside the `_solve_from_buffer` service callback, propagating through the executor.

## Failure scenario

A user configures a board with fewer than 4 markers. The solve service crashes mid-request instead of returning a clear validation error.

## Suggested fix

Validate `len(marker_ids) >= 4` (or handle variable marker counts) at config load, with a clear message, before the solve path.
