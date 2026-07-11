# H-04 · Detector declares params mandatory that the launch adds only "if present" → startup crash

- **Severity:** High
- **Area:** lidar_board_detector ↔ lctk_launch
- **Status:** Open
- **Verified:** Static review
- **Location:**
  - `ros/lidar_board_detector/src/main.rs:308-312` (`.mandatory()?`)
  - `ros/lctk_launch/lctk_launch/config_parser.py:84-85, 367-369`
  - `ros/lctk_launch/launch/calibrate.launch.py:112-115` (adds `aruco_pattern_file` / `bbox_file` only if present)

## Problem

The board detector declares `aruco_pattern_file` and `bbox_file` as `.mandatory()`, but the launch pipeline adds those parameters only when the corresponding config keys are present in the YAML. `aruco_config` and `bbox_config` are marked Optional in the `Marker` dataclass.

## Failure scenario

A user writes a `hollow_board` marker that omits `bbox_config` (or `aruco_config`). The config parses without error, but the detector node exits immediately on the mandatory-parameter check — with no config-time validation to explain why.

## Suggested fix

Either (a) make the parser validate that `aruco_config` and `bbox_config` are present for any `hollow_board` marker used by a detector and fail fast with a clear message, or (b) make the detector params non-mandatory with sensible defaults. Keep the launch and node contracts in sync.
