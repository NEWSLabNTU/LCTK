# L-06 · Pervasive `except Exception: pass` against the project's own guideline

- **Severity:** Low
- **Area:** Python solver nodes
- **Status:** Open
- **Verified:** Static review
- **Location:**
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1437-1448`
  - `ros/lidar_to_lidar_solver/lidar_to_lidar_solver/main.py:365-366`

## Problem

Shutdown / stats paths swallow all exceptions (`except Exception: pass`), hiding real errors. CLAUDE.md's Coding Guidelines explicitly forbid "Pokemon exception handling".

## Failure scenario

A stats or cleanup error at shutdown is swallowed with no diagnostic, masking real problems.

## Suggested fix

Catch specific exceptions, log the error, and only suppress where a documented, benign reason exists.
