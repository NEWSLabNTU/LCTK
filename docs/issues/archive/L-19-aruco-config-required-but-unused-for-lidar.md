# L-19 · `aruco_config` is mandatory for LiDAR-only markers but never affects the LiDAR fit

- **Severity:** Low
- **Area:** lctk_launch / config_parser, lidar_board_detector
- **Status:** Fixed (2026-08-28) — W5-E1 (the single-source-target-definition cutover) removed the
  `aruco_config` schema key entirely; see [Resolution](#resolution-2026-08-28) below
- **Verified:** By code trace (2026-08-11) while configuring a two-LiDAR calibration; resolution
  re-verified by code trace 2026-08-28

## Problem

A `hollow_board` marker used by a LiDAR must declare `aruco_config`, or the config parser raises
`Marker '<name>' (used by lidar '<lidar>') is missing 'aruco_config', which the lidar_board_detector
requires.` For a LiDAR-to-LiDAR calibration there is no camera, so the requirement reads as
spurious.

Tracing it: `lidar_board_detector` does load the ArUco pattern and reads `paper_size()` into
`BoardModel::marker_paper_size`. But `marker_paper_size` only feeds the **marker-paper** geometry
(`marker_*_corner()`, `multi_marker_corners()`), which is the camera side of the shared board model.
The LiDAR ICP model is built from `board_width`, `hole_radius` and `hole_center_shift` only; no
`marker_*_corner()` call appears anywhere in the detector or fitter path. The field is carried into
the published detection as metadata and never influences the pose.

So the requirement is structural — `BoardModel` demands the field because it is shared between the
camera and LiDAR detectors — not geometric. The validation enforces a struct's shape, not a real
dependency. The board's physical dimensions are already fully described by the board-geometry
parameters in the detector config.

Consequence is mild (every deployment has an `aruco_pattern.json5` to point at), but it makes
LiDAR-only configs carry a meaningless line and misleads readers into thinking the marker layout
matters to LiDAR detection.

## Suggested fix

Make the ArUco pattern optional for LiDAR-only use: have the detector node default
`marker_paper_size` when no pattern file is supplied, and drop the parser requirement for markers
that no camera observes (the parser already knows which devices observe each marker). Keep the
requirement for any marker used by a camera.

Alternatively, if the shared `BoardModel` is to stay as-is, document in the config schema that
`aruco_config` is required for schema reasons and does not affect LiDAR detection.

## Resolution (2026-08-28)

The single-source-target-definition work (W5-E1) replaced the split `board_config`/`aruco_config`
schema with a single required `target_config` (a Target Definition: plate, cutouts, fiducial
layout, identity) plus `detector_config` (sensor-specific tuning only, no geometry). Verified in
`ros/lctk_launch/lctk_launch/config_parser.py`:

- `_parse_markers` now rejects `type`, `board_config`, and `aruco_config` outright as "retired
  schema key(s)" (lines ~448–466) — the specific mandatory-but-unread `aruco_config` field this
  issue was about no longer exists in the schema at all.
- `_parse_new_marker` requires `target_config` and `detector_config` for **every** marker,
  regardless of which device types observe it (lines ~475–484) — there is no longer a
  LiDAR-specific carve-out to reason about.

Critically, this isn't the same complaint under a new name: `target_config` is a real, consumed
dependency of the LiDAR path, not vestigial metadata. `ros/lidar_board_detector/src/main.rs`
requires `target_config` (`ConfigSource`, lines ~298–318), loads it via
`Self::load_target()` → `ValidatedTarget::parse_json5` (line 1052), and threads the parsed
`target` into the detection callback (`CallbackContext`, used by the ICP `estimator`). The old
`aruco_config` fed only marker-paper geometry the LiDAR side never touched; the new
`target_config` is the LiDAR fitter's actual geometry source. The complaint this issue tracked —
a config key mandatory for a device type that never reads it — is gone by construction, not just
relabeled.

Closing 🟢 and archiving.
