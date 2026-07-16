# Phase 6: Autoware Export

## Overview

LCTK ends today at a `TransformStamped` on a topic and a log line. The user's actual
destination is an Autoware workspace, and the last mile — getting the solved extrinsic
into `sensor_kit_calibration.yaml` with the right direction, frames, and angle
convention — is fully manual and undocumented ([gap-autoware-export](../issues/gap-autoware-export.md)).
This phase ships that last mile.

Design: [2026-07-16-autoware-export-design.md](../superpowers/specs/2026-07-16-autoware-export-design.md)
(includes the 2026-07-16 survey of Autoware `main`/1.5.0, 0.45.1 and 2024.11 config layouts).

## Why now

- All correctness criticals/highs that would poison an exported value are fixed
  (C-01…C-04, H-07…H-10); exporting earlier would have shipped biased numbers.
- The target format is now verified against a real Autoware checkout, both the
  pre-2025 `autoware_individual_params` era and the current folded-into-`autoware_launch`
  era. Same YAML schema in both → one tool.
- No field data required; fully testable headless (golden files + xacro round-trip).

## Deliverables

| # | Deliverable | Acceptance |
|---|-------------|-----------|
| 1 | Frame-algebra core: invert solver `T(optical←lidar)`, compose `T(kit→camera_link)` via existing lidar entry and the fixed optical→camera_link rotation, decompose to xyz+RPY (radians, fixed-axis) | Unit fixtures + RPY round-trip property test pass |
| 2 | `lctk_autoware_export` CLI: patch one entry in `sensor_kit_calibration.yaml`, comment-preserving (`ruamel.yaml`), `--dry-run`, `.bak`, hard errors over guesses | Golden-file test: only the target entry changes |
| 3 | End-to-end validation: exported YAML → `xacro sensor_kit.xacro` → joint origin matches solver transform | e2e test in CI (`just test`) |
| 4 | Book page "Exporting to Autoware" | Documents both eras, frame diagram, worked `just demo` example |

## Ordering constraints

1. **M-01 first or together.** The exporter is the second consumer of the transform
   direction; fixing the publisher's labels later without touching the exporter would
   silently flip the export. Mitigation baked into the design: exporter reads the dump
   JSON rvec/tvec (raw solver output), never the re-labeled TF topic.
2. Dependency: `python3-ruamel.yaml` from **apt**, not pip (CLAUDE.md pip-shadowing
   hazard; installer update belongs to Phase 4's setup work).

## Non-goals

- `sensors_calibration.yaml` (base_link→kit) — LCTK does not measure it.
- Sensor-kit package generation, xacro editing, camera intrinsics export.
- Supporting Autoware versions older than 2024.11 layouts.

## Status

- [x] 1. Frame-algebra core + tests (`ros/lctk_autoware_export/lctk_autoware_export/frames.py`)
- [x] 2. CLI + golden-file tests (`ros2 run lctk_autoware_export export`)
- [ ] 3. xacro round-trip e2e
- [ ] 4. Book page
