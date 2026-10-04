# 0010. Detection Archives own their projection provenance

- **Date:** 2026-10-04
- **Status:** accepted

## Context

Saved image corners are already undistorted using CameraInfo K and D with P equal to K.
Earlier Detection Archives do not record the matrix, dimensions, or camera frame needed
to interpret those pixels independently of a running graph.

## Decision

Version 6 requires `camera_projection` with model `undistorted_pixels_using_k`, header-owned
camera frame, positive image dimensions, and a finite standard pinhole K matrix. Snapshot
projection metadata, Captures, and the saved transform under one state lock. A change in
K, dimensions, or camera frame starts a new capture epoch.

Writer, restore, exporter, and validation enforce version 6. Remove legacy migration and
reject older archives without a CameraInfo fallback. Share the small projection invariant
through `lctk_quality`; persistence stays in its existing owning packages.

## Consequences

An archive can be interpreted without external camera settings. Projection uses K with
zero distortion because the saved pixels have already been undistorted. Old archives need
new capture rather than an unverifiable provenance claim. Shared validation and parity
tests constrain consumer drift without moving the full persistence layer.
