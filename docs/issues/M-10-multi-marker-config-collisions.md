# M-10 · Multi-marker camera uses wrong ArUco config; duplicate pairs collide

- **Severity:** Medium
- **Area:** lctk_launch config parser
- **Status:** Open
- **Verified:** Static review
- **Location:** `ros/lctk_launch/lctk_launch/config_parser.py:383-393` (first-marker wins), `414-431` (solvers not de-duped)

## Problem

Two related issues:
1. One `aruco_locator` is created per camera and picks the **first** marker's `aruco_config` (`break`), while a second pair's solver may use a different marker's config — detector and solver then disagree on the pattern, silently, with no warning.
2. Board detectors and aruco locators are de-duped via a `Set`, but solvers are generated per raw pair. Listing the same `[lidar, camera]` pair twice yields two nodes with identical name + namespace — a ROS node collision.

## Failure scenario

A camera observing two boards with different patterns silently mismatches detections; a repeated pair causes a node-name collision at launch.

## Suggested fix

Key the aruco_locator per (camera, marker) when patterns differ, warn on conflicting configs for one camera, and de-duplicate calibration pairs (or error on duplicates) before spawning solver nodes.
