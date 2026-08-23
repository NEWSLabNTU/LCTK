# 0003. Calibration targets share one interface and are selected once per launch

- **Date:** 2026-08-23
- **Status:** accepted
- **Accepted design:** [Selectable calibration targets](../superpowers/specs/2026-08-21-selectable-calibration-targets.md)
- **Implementation plan:** [Phase 8](../roadmap/phase-8-selectable-calibration-targets.md)

## Context

LCTK was built around one 1000 mm hollow Calibration Target. Its plate geometry, three asymmetric
cutouts, four-marker ArUco layout and detector settings became distributed across Rust types, Python
geometry, ROS nodes and several configuration files. A new 600 mm solid target has different LiDAR
geometry and a single full-face ArUco marker, but uses the same corner-aligned diamond frame.

Treating the new hardware as changed constants would preserve the old duplication and hide a more
important difference: the hollow target's cutouts identify its in-plane orientation to LiDAR, while
a solid square has four geometrically equivalent quarter-turns. Retiring the hollow target
immediately would also discard a field-validated rollback path and make existing recordings harder
to interpret before the new estimator and sensor presets have real-world evidence.

## Decision

Both physical targets remain supported behind one deep Calibration Target interface. An immutable
**Target Definition** is the single source of physical truth: plate geometry, surface kind,
fiducial layout, canonical frame and LiDAR Orientation Reference. Sensor noise, point density,
operating range and crop boxes remain outside it as **Detector Tuning** and deployment concerns.

The operator selects exactly one Target Definition for each sensor before launch. A launched
calibration graph does not switch targets at runtime and does not let one sensor observe multiple
targets concurrently. Launch validates this invariant. Different launches may select different
targets.

The target module hides two internal surface adapters:

- `solid`, for the 600 mm plate with `DICT_5X5_1000` marker ID 1, a 480 mm marker and 60 mm margin;
- `perforated`, for the existing 1000 mm plate and its explicit asymmetric circular cutouts.

Callers consume validated target geometry, marker corners, target models and Target Identity; they
do not branch on hollow-board classes or reconstruct layouts themselves. Target Identity binds the
semantic Target Definition, revision and board-frame convention. Detectors and locators publish it,
solvers require their connected observations to agree, and Detection Archive version 5 records it.
Version-4 transforms remain directly exportable, but version-4 captures require explicit
operator-selected target migration before re-solving.

LiDAR pose estimation respects what each surface can actually observe. A common square-and-plane
estimator supplies geometric alignment. For the solid target, the square owns in-plane translation
and yaw, the plane owns normal offset and tilt, and the selected board-up corner must satisfy
`board_up dot sensor_up >= 0.90`. Interior planar points cannot invent an unobserved in-plane pose.
For the hollow target, the common estimator may seed and diagnose the result, but asymmetric-cutout
evidence and the existing `BoardIcpIterator` remain final authority. The iterator stays maintained
and tested; it is not exposed as a production solid-target fallback without a later decision.

Each supported sensor-target pair has its own Detector Tuning preset. True geometry derives from
the Target Definition, while point-count, voxel, clustering, plane and acceptance operating points
are measured per sensor and target.

The solid presets begin experimental. This label informs operators but never blocks launch,
publication, saving or export. Promotion is per sensor-target preset and uses real recordings:
moving-board bags for coverage, orientation continuity, overlays and independent-window extrinsic
consistency; short static and board-absent captures where jitter and false-positive evidence are
otherwise unavailable. Synthetic clouds are test fixtures, not field evidence. Promotion requires
a published evidence report and operator/maintainer sign-off; any confirmed quadrant flip keeps the
preset experimental until fixed.

## Considered options

**Retire the hollow target immediately.** Rejected because it removes the known-good hardware,
existing-data regression path and estimator rollback before solid-target field validation exists.

**Build a second solid-board pipeline.** Rejected because physical geometry, marker layout, identity,
archive compatibility and solver rules would again diverge across parallel implementations.

**Support simultaneous or runtime-switched targets.** Rejected because the workflow needs one
operator-selected target per launch. Multi-target camera routing and ambiguous identity ownership
would add interface and state without a present use case.

**Use unconstrained six-degree-of-freedom ICP for the solid plate.** Rejected because a planar solid
square does not provide evidence for every update such an optimizer can produce. A low residual
would not make the invented degrees of freedom observable.

## Consequences

Target geometry changes concentrate in one module and one manifest, while existing hardware and
recordings remain usable. Rust and Python can share semantic-identity and geometry goldens. Mixed
target data fails explicitly instead of being silently reinterpreted.

The cost is two maintained surface adapters, archive migration, per-sensor/per-target tuning and a
larger regression matrix. The solid target also remains dependent on correct mounting-up
configuration and real field evidence. Supporting simultaneous targets, retiring the hollow
adapter, or enabling hollow ICP for solid production each requires a new architectural decision.
