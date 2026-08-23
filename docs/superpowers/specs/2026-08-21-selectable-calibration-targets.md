# Spec: Selectable calibration targets

- **Date:** 2026-08-21
- **Status:** Accepted
- **Selected direction:** Keep both the existing hollow target and the new solid target behind one
  interface
- **Decision record:** [ADR 0003](../../adr/0003-selectable-calibration-targets.md)
- **Implementation plan:** [Phase 8](../../roadmap/phase-8-selectable-calibration-targets.md)
- **Related:** `2026-08-13-corner-aligned-board-frame.md`,
  `2026-08-14-lidar-to-camera-solver-diamond-frame.md`, H-11, L-19, M-17, M-21

## Status of decisions

This document is the accepted detailed design. Text marked **Decision** records choices settled
during operator review. Numeric operating thresholds explicitly delegated to field data are tuning
work, not unresolved architecture. Implementation still requires a separate implementation plan.

## Problem

LCTK currently models one physical calibration target: a 1000 mm square plate with three circular
holes and a 500 mm, four-marker ArUco sheet in its lower quarter. That physical description is split
between the board detector and ArUco pattern configs, while Rust and Python independently derive
parts of the geometry.

A second target is entering service: a 600 mm solid square whose entire face is a centred ArUco
sign. It has one 480 mm marker, ID 1, from `DICT_5X5_1000`, surrounded by a 60 mm white margin. The
plate remains diamond-mounted and uses the existing corner-aligned frame.

Changing only constants is unsafe:

- `BoardShape` hardcodes exactly three holes;
- Rust and Python marker-corner code hardcode a 2x2/four-marker layout;
- `board_width` and `side_m` duplicate one physical dimension;
- the current frame-convention guard cannot distinguish two targets using the same frame;
- version-4 Detection Archives recompute marker geometry from the currently loaded config and do
  not identify the target that produced their Captures;
- a solid square has four equivalent geometric orientations in LiDAR XYZ data, because the removed
  holes were the only asymmetric LiDAR-visible features.

## Goals

1. Support either physical target, selected per calibration workflow.
2. Put all physical target facts in one Target Definition.
3. Give LiDAR detection, camera detection, marker generation, solvers and archives one consistent
   Target Identity.
4. Preserve the existing corner-aligned board frame.
5. Preserve existing hollow-target recordings and hardware as regression and rollback paths.
6. Make the solid target's weak in-plane observability explicit and fail closed on ambiguity.
7. Keep sensor-specific Detector Tuning and deployment-specific crop boxes outside the Target
   Definition.

## Non-goals

- Supporting non-square plates.
- Supporting non-ArUco fiducials.
- Dynamically changing a target while nodes are running.
- Letting one sensor observe more than one Target Definition in one launched calibration graph.
- Estimating target geometry from observations.
- Porting the superseded `extrinsic_solver_node`; its existing Stage 3 deletion remains separate.
- Retuning the new target without recorded or live field data.

## Domain language

The canonical terms are recorded in the repository `CONTEXT.md`:

- **Calibration Target** — physical plate, fiducial layout, canonical frame and identity.
- **Target Definition** — immutable physical description; no sensor tuning or crop box.
- **Target Identity** — versioned binding preventing cross-target reinterpretation.
- **Detector Tuning** — sensor/range-specific detection and acceptance settings.
- **LiDAR Orientation Reference** — physical evidence identifying the target's named in-plane axes
  to a LiDAR.

`HollowBoard`, `BoardShape`, `aruco_config` and `board_config` are implementation-era names, not
general domain terms.

**Decision.** New domain and configuration language uses `Calibration Target`, `Target Definition`
and `target_config`. Misleading implementation names become `calibration-target`,
`calibration-target-detector` and `target_geometry.py`; `MarkerType.HOLLOW_BOARD` is removed.
Stable ROS names including `lidar_board_detector` and `calibration_board_detections` remain because
both physical targets are boards and renaming their wire-facing interface adds migration without
clarifying meaning.

## Settled physical profiles

### Solid 600 mm target

**Decision.** The new target has:

- target ID `solid_600_aruco_1` and revision 1;
- a 600 mm square plate with no cutouts;
- `corner_aligned_plate_center_v1` frame convention;
- plate centre as origin;
- `+Y` from centre toward the physical top corner;
- `+X` from centre toward the physical left corner, from the board's viewpoint;
- `+Z` toward the observing sensor;
- one `DICT_5X5_1000` marker, ID 1;
- a 600 mm paper/sign face, centred on and flush with the plate;
- a 60 mm white margin on each side;
- a derived marker side of `600 - 2*60 = 480 mm`;
- diamond/corner-up mounting as a required physical invariant.

In the current `[right, top, left, bottom]` correspondence order, the marker corners in board-local
metres are:

```text
right   (-0.339411254970,  0,                 0)
top     ( 0,                 0.339411254970, 0)
left    ( 0.339411254970,  0,                 0)
bottom  ( 0,                -0.339411254970, 0)
```

The plate's centre-to-corner radius is `0.6/sqrt(2) = 0.424264068712 m`.

### Existing hollow 1000 mm target

**Decision.** Existing physical geometry and measured paper placement remain supported:

- target ID `hollow_1000_aruco_4` and revision 1;
- a 1000 mm square plate;
- three 150 mm-radius circular cutouts;
- cutout centres in corner-aligned board coordinates:
  `(+282.842712, 0) mm`, `(0, +282.842712) mm`, `(-282.842712, 0) mm`;
- `corner_aligned_plate_center_v1` frame convention;
- a 500 mm ArUco paper centred at `(0, -353.553391) mm` in
  `(toward_left_corner, toward_top_corner)` coordinates;
- marker IDs `[696, 64, 306, 195]`, `DICT_5X5_1000`, 10 mm outer border, 2x2 cells,
  `marker_fill_ratio = 0.8`, and one border bit.

Explicit cutout centres replace the old `hole_center_shift * sqrt(2)` derivation.

## Architecture

```text
                            +-------------------+
                            | Target Definition |
                            +---------+---------+
                                      |
                    +-----------------+------------------+
                    |                 |                  |
                    v                 v                  v
          LiDAR target detector  ArUco locator     ArUco generator
             detection + ID      detection + ID       printed sign
                    |                 |
                    +--------+--------+
                             v
                 LiDAR-camera solver
                   local definition
                   identity equality
                   marker geometry
                             |
                             v
                   Detection Archive v5
```

### `calibration-target` module

**Decision.** `rust/hollow-board-config` becomes the neutral `rust/calibration-target` module. It is
the single Rust implementation of Target Definition parsing, validation, identity, plate geometry,
paper-to-board mapping and marker layout.

Its external interface remains small:

```rust
let target = CalibrationTarget::from_json5(bytes)?;

target.identity() -> &TargetIdentity
target.marker_corners_by_id() -> &IndexMap<u32, [Point3<f64>; 4]>
target.posed(pose) -> PosedTarget<'_>

posed.closest_points(points) -> Vec<Correspondence>
posed.{center, top_corner, bottom_corner, left_corner, right_corner, axes}()
```

Callers do not inspect cutouts, derive marker sizes, rotate paper coordinates, calculate hashes or
branch on target kind.

The internal surface seam has two real adapters:

```rust
SolidSquareSurface
PerforatedSquareSurface
```

The solid adapter projects onto the closed diamond plate. The perforated adapter performs the same
projection, then pushes points inside cutouts onto the nearest rim. Adapter selection occurs once
when the Target Definition is validated; the public interface exposes no board-specific class.

### `calibration-target-detector` module

**Decision.** `rust/hollow-board-detector` becomes `rust/calibration-target-detector`. Its interface
owns pose estimation and rejection:

```rust
let estimator = TargetPoseEstimator::new(target, detector_tuning)?;

estimator.estimate(points, sensor_up)
    -> Result<TargetDetection, TargetRejectReason>
```

The implementation hides plane fitting, fixed-square coverage fitting, initial pose, orientation
selection, optional cutout refinement, covariance estimation and acceptance gates.

The ROS node remains an adapter: PointCloud2 conversion, crop selection, parameters, publication,
logging and RViz output stay in `lidar_board_detector`.

### Python target geometry

**Decision.** `lidar_to_camera_solver.board_geometry` becomes `target_geometry`. It imports no
`rclpy` and exposes validated immutable values rather than raw JSON dictionaries:

```python
target = load_target(path)

target.identity
target.marker_corners_by_id
```

Rust and Python remain separate implementations because they run in different packages and
languages. Their shared interface test is target-keyed world-coordinate golden data under
`fixtures/targets/`.

## Target Definition schema

**Decision.** Both profiles use one explicit JSON5 schema. No missing-field fallback is allowed.

```json5
{
  schema_version: 1,
  target_id: "solid_600_aruco_1",
  revision: 1,
  board_frame_convention: "corner_aligned_plate_center_v1",

  plate: {
    side: "600mm",
    surface: {
      kind: "solid",
    },
  },

  fiducial: {
    kind: "square_aruco_grid",
    dictionary: "DICT_5X5_1000",
    marker_ids: [1],
    paper_side: "600mm",
    paper_center: {
      toward_left_corner: "0mm",
      toward_top_corner: "0mm",
    },
    outer_border: "60mm",
    cells_per_side: 1,
    marker_fill_ratio: 1.0,
    border_bits: 1,
  },

  lidar_orientation_reference: {
    kind: "mounting_up",
    local_axis: "+y",
  },
}
```

The hollow profile changes `surface.kind` to `perforated`, supplies explicit
`circular_cutouts`, uses its existing ArUco layout, and declares
`lidar_orientation_reference.kind = "asymmetric_cutouts"`.

**Decision.** LiDAR Orientation Reference is explicit physical truth, not inferred from surface
kind and not an estimator-strategy switch. `asymmetric_cutouts` requires validation that the cutout
set is not invariant under a quarter turn. `mounting_up` requires a named local axis, valid sensor-up
direction and an unambiguous corner-up observation. Camera orientation remains independently
observable from the ArUco marker.

Validation rejects:

- unknown schema, frame, surface, fiducial or LiDAR-orientation-reference kind;
- unknown fields;
- empty target ID or revision zero;
- non-finite or non-positive dimensions;
- cutouts with non-positive radius, overlap, or any part outside the plate;
- paper corners outside the plate;
- fiducial geometry intersecting a cutout;
- zero cells or a marker count other than `cells_per_side^2`;
- duplicate or out-of-dictionary marker IDs;
- `2*outer_border >= paper_side`;
- `marker_fill_ratio` outside `(0, 1]`;
- `border_bits < 1`;
- `plate_features` geometry that remains symmetric under a quarter turn;
- `mounting_up` without a valid sensor-up direction at pose-estimation time.

## Pose observability and estimator adapters

### Common path

**Decision.** Both target profiles use one plane and known-size outer-square fit. Physical side
length comes only from the Target Definition. The fitted square centre and coverage are retained
rather than discarded before pose refinement.

Both bbox and bbox-free detection paths feed the same target pose estimator after selecting points.
This makes their pose semantics identical and closes the current gap where the bbox-free detector
fits a square while the bbox path starts later refinement from a plane centroid.

### Perforated target

**Decision.** The existing target uses the common square-and-plane estimator as its initial pose and
alignment gate. That base estimate remains available as a debug diagnostic, including the correction
between base and final poses, but it is not a selectable production result for the hollow profile.

The hollow adapter refines the common pose with cutout-aware `BoardIcpIterator`, which remains final
authority. It must compare quarter-turn hypotheses and require a configurable minimum separation
between the best and second-best losses. Failure of cutout evidence rejects the observation; it does
not silently publish the common estimate. This preserves the hollow target as the known-good
regression and rollback path while still giving it the deeper common initializer.

### Solid target

**Decision.** ArUco ink is invisible to XYZ-only LiDAR geometry. A solid square therefore cannot
derive an absolute quarter-turn from shape. Its adapter uses the physical mounting invariant:

1. fit the known-size outer square;
2. orient `+Z` toward the sensor;
3. derive the four possible board-`+Y` corner directions from the fitted square geometry;
4. choose the candidate maximizing `board_up dot sensor_up`;
5. require that alignment to be at least `0.90`;
6. derive `+X = Y x Z`;
7. constrain refinement so filled-surface ICP cannot rotate into an equivalent quadrant.

The board-up candidate must originate from fitted plate corners. Constructing `+Y` from projected
sensor-up and then checking their dot product would be circular. For orthogonal square diagonals, a
threshold above `1/sqrt(2)` also guarantees a unique winning corner; `0.90` accepts a board up to
about 25.8 degrees from sensor-up and rejects the edge-aligned `0.707` case. This is a hard gate,
replacing the current warning-only lowest-corner ambiguity check.

The estimator must report weaker in-plane/yaw information in covariance or an explicit
observability diagnostic. ICP loss alone is not evidence of correct orientation.

**Decision.** Solid-target refinement is evidence-separated rather than one unconstrained 6-DoF
closest-point update:

- the known-size outer-square fit owns both in-plane translations and rotation about the board
  normal;
- the plane fit owns translation along the board normal and the two normal-tilt rotations;
- the selected quarter-turn never changes, and final board-up alignment must remain at least `0.90`;
- insufficient outer-edge evidence rejects the observation rather than allowing interior plate
  points to invent in-plane position or yaw.

The implementation may alternate the two fits internally, but each residual updates only the
degrees of freedom it observes.

**Decision.** `BoardIcpIterator` is retained and remains tested. The hollow target's cutout-aware
adapter continues to need it, and preserving it provides an implementation-level rollback path if
field evidence later shows the solid estimator needs reconsideration. This spec does not authorize
deleting the iterator. Production configuration does not expose it as a solid-target fallback; any
such experiment remains internal until field evidence and a spec amendment justify that option.

Threshold values beyond the accepted `0.90` alignment gate are derived from field data rather than
fixed by this architecture.

## Runtime Target Identity

**Decision.** Add:

```text
lctk_interfaces/msg/CalibrationTargetIdentity.msg

uint32 schema_version
string target_id
uint32 revision
string semantic_sha256
string board_frame_convention
```

The SHA-256 covers one canonical encoding of the validated Target Definition's semantic values, not
the JSON5 source bytes. Comments, whitespace, object-key order and equivalent length spellings such
as `"600mm"` and `"0.6m"` do not change identity. Physical geometry, fiducial layout, marker-ID
placement, frame convention, target ID or revision changes do.

The canonical encoding uses a fixed field order, normalized enum strings and integer micrometres for
lengths. Marker-ID order remains significant because it assigns markers to grid cells. Circular
cutouts are sorted by normalized centre and radius because their source-list order has no geometric
meaning. Rust and Python assert the canonical bytes and resulting hash against the same golden
fixtures; neither invents its own serialization.

`lidar_board_detector` and `aruco_locator_node` each publish their identity once with RELIABLE,
TRANSIENT_LOCAL, KEEP_LAST(1) QoS. Identity topics are relative and routed by launch to the exact
solver input; the current absolute `/lctk/board_frame_convention` topic is retired because multiple
publishers or profiles can race there.

A LiDAR-camera solver waits for:

```text
lidar_identity == camera_identity == locally_loaded_identity
```

A LiDAR-LiDAR solver waits for both LiDAR identities and requires equality. Absence, malformed
identity, or mismatch is fatal before accepting a Detection Pair. Identity remains immutable for a
node lifetime.

**Decision.** Each sensor uses exactly one Target Definition in one launched calibration graph. The
operator selects that definition before launch. Different launches may select different targets,
but one camera or LiDAR never observes both profiles concurrently. ArUco locator instances therefore
remain per camera rather than becoming per `(camera, target)`.

Launch validation rejects a sensor connected to calibration pairs naming different Target
Definitions. This keeps identity routing unambiguous and avoids changing ArUco detection to tolerate
unrelated marker IDs from a second target in the same image.

## Detection Archive version 5

**Decision.** Version 5 stores full Target Identity alongside the existing board-frame convention,
Captures, Quality Verdict and optional Adjusted Transform.

Loading a Detection Archive for re-solving requires exact identity equality with the currently
loaded Target Definition. Version 4 is never reinterpreted implicitly.

The migration command accepts an explicit target:

```bash
ros2 run lidar_to_camera_solver migrate_detections \
    --input detections-v4.json \
    --output detections-v5.json \
    --target-config hollow_1000_aruco_4_v1.json5
```

Migration verifies that archived marker IDs belong to the selected target before binding its
identity. This proves compatibility of IDs, not physical provenance; operator confirmation remains
required and the command says so.

**Decision.** `lctk_autoware_export` accepts a valid version-4 solved transform because it does not
recompute correspondences. Restoring or re-solving version 4 still requires explicit migration to
version 5 with an operator-selected Target Definition. For version 5 the exporter validates Target
Identity structurally. It does not need a Target Definition merely to export an already-solved
transform.

## Launch and configuration interface

**Decision.** One target path reaches every observer and solver:

```yaml
markers:
  calibration_board:
    target_config: $(find-pkg-share lctk_launch)/config/targets/solid_600_aruco_1_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/solid_600/velodyne.json5
    bbox_config: $(find-pkg-share lctk_launch)/config/board/bbox.json5
    aruco_detector_config: $(find-pkg-share lctk_launch)/config/aruco/aruco_detector.json5
    pairs:
      - [top_lidar, front_center]
```

`MarkerType.HOLLOW_BOARD`, `board_config` and `aruco_config` disappear. A single-value marker-type
enum adds no leverage; the Target Definition describes what the target is.

Per-LiDAR Detector Tuning override remains available. Detector presets may vary by sensor and target
because point-count, voxel, cluster and acceptance operating points differ. Geometry-dependent
extent and diagonal calculations derive from `plate.side` and are not repeated in tuning files.

**Decision.** Provide an explicit Detector Tuning preset for every supported sensor-target pair.
Target geometry, including plate side and derived diagonal/extent expectations, comes only from the
Target Definition. Point-count floors, voxel size, clustering radius, plane tolerance and fit gates
remain preset-owned because they depend on target area, sensor sampling and range/noise behavior.
In particular, the 600 mm plate presents only 36% of the area of the 1000 mm plate, so the new
Velodyne and Seyond operating points must be measured rather than inferred from one shared preset.

## LiDAR-camera solving policy

**Decision.** Continuous one-Capture solving remains available for both Target Definitions. A
solid-target result is publishable, savable and exportable under the same operator-controlled
workflow as a hollow-target result. The software reports available Quality Verdict and degeneracy
evidence, but does not refuse a result merely because it came from one Capture or one Marker.

Manual multi-Capture solving remains available when the operator wants more observations and pose
diversity. It is guidance and evidence, not a mandatory promotion path. Target Definition therefore
does not encode a minimum Capture count and launch does not silently change `solver_mode` based on
the selected target.

The existing hollow recordings and examples continue to select the hollow profile. New examples
select the solid profile. No existing recording is relabelled as the new target.

## Expected file impact

### Core Rust modules

- `rust/hollow-board-config/` -> `rust/calibration-target/`
  - replace fixed `BoardShape` with validated target profiles;
  - express cutouts explicitly;
  - generalize marker-grid expansion;
  - add Target Identity;
  - retain and generalize diamond projection tests.
- `rust/hollow-board-detector/` -> `rust/calibration-target-detector/`
  - remove physical geometry from Detector Tuning;
  - add common square-fit path and two internal pose adapters;
  - replace `BoardModelParams` with validated target input.
- `rust/board-cluster-detector/`
  - stop deserializing `side_m`;
  - receive target side at the detection interface;
  - preserve the fitted square pose for downstream estimation.
- `rust/aruco-config/`
  - keep detector-independent dictionary/pattern values;
  - move physical paper placement into Target Definition;
  - validate generic `n x n` layouts without four-marker indexing.
- `rust/aruco-detector/` and `rust/aruco-locator/`
  - cover the 1x1 marker profile and remove old-ID assumptions.
- `rust/aruco-generator/`
  - render the fiducial embedded in a Target Definition.

### ROS packages

- `ros/lidar_board_detector/`
  - load Target Definition and Detector Tuning separately;
  - use generic target estimator;
  - publish target-sized Detection3D and RViz geometry;
  - omit cutout markers for solid profile;
  - publish Target Identity.
- `ros/aruco_locator_node/`
  - load Target Definition;
  - remove hardcoded warning for `[696, 64, 306, 195]`;
  - publish Target Identity.
- `ros/aruco_generator_node/`
  - accept Target Definition instead of standalone pattern config.
- `ros/lidar_to_camera_solver/`
  - replace `board_geometry.py` with `target_geometry.py`;
  - use generic marker layout;
  - gate on LiDAR, camera and local Target Identity;
  - write/read Detection Archive version 5.
- `ros/lidar_to_lidar_solver/`
  - compare the two LiDAR Target Identities.
- `ros/lctk_interfaces/`
  - add `CalibrationTargetIdentity.msg` and generated bindings.
- `ros/lctk_autoware_export/`
  - validate version-5 identity structure while retaining safe version-4 transform export.
- `ros/lctk_launch/`
  - parse `target_config` and `detector_config`;
  - route identity topics;
  - enforce one coherent Target Definition per generated solver.

### Configuration and fixtures

- add `config/targets/solid_600_aruco_1_v1.json5`;
- add `config/targets/hollow_1000_aruco_4_v1.json5`;
- replace standalone physical `config/aruco/aruco_pattern.json5`;
- remove `board_width`, `hole_radius`, `hole_center_shift` and `side_m` from Detector Tuning;
- split presets as needed under `config/board/{solid_600,hollow_1000}/`;
- update all example YAMLs without changing which target their recorded data contains;
- move shared marker and surface goldens under `fixtures/targets/<target_id>/`.

### Documentation and tracking

- update `CLAUDE.md`, package READMEs and book user/developer guides;
- preserve historical archived issue/spec terminology;
- repair current docs that call all targets hollow boards;
- track field-only tuning and validation work explicitly rather than claiming it headlessly complete.

## Verification contract

### Interface tests

- both target manifests validate in Rust and Python;
- invalid schema table produces field-specific errors;
- shared Rust/Python goldens agree by target ID and marker ID;
- new marker side is exactly 480 mm;
- new marker corners match the coordinates stated above;
- generic layout accepts 1x1 and 2x2, rejects wrong/duplicate marker counts;
- solid and perforated closest-point projection properties hold;
- target identity matches across languages;
- comments, whitespace, key order and equivalent length units preserve identity;
- any semantic geometry, fiducial, frame, ID or revision change changes identity.

### Detector tests

- both bbox and bbox-free paths use the same pose semantics;
- perforated adapter selects correct quarter-turn and rejects weak best/second-best separation;
- solid adapter derives `+Y` from fitted corners and requires `board_up dot sensor_up >= 0.90`;
- solid refinement cannot cross into an equivalent quadrant;
- Detection3D and RViz geometry use target side;
- covariance/diagnostics expose solid-target weak directions.

### Runtime tests

- missing, malformed and mismatched Target Identity stop each solver;
- identities are routed per generated detector/solver, not globally;
- one-marker ArUco detection preserves detector corner order;
- Detection Archive v5 round-trips and rejects a different target;
- explicit v4 migration verifies marker IDs and records operator-assumed target;
- legacy sample data still runs against hollow profile.

### Field gates for the solid target

**Decision.** Validation status applies independently to each sensor-target Detector Tuning preset,
not to the solid Target Definition as one all-or-nothing unit. For example, the Velodyne-solid
preset may be validated while the Seyond-solid preset remains experimental. Experimental status is
operator-facing documentation and metadata; it does not block launch or suppress results.

**Decision.** Real sensor recordings, not fabricated point clouds, provide field evidence. Existing
hand-held solid-board bags are replayed deterministically and evaluated over labelled board-visible
and, where present, board-absent intervals. Their motion supplies range, tilt and image-position
diversity for detection coverage, quadrant continuity, LiDAR-camera overlay and independent
time-window/subset extrinsic consistency.

The historical static hollow-board bags are a reference, not a controlled A/B benchmark: motion,
duration and target size differ, so raw recall and jitter are not directly comparable. Short
supplemental real solid-board recordings fill only evidence the moving bags cannot provide:
a stationary interval for pose jitter and a board-absent/clutter interval for false detections.
Synthetic clouds remain unit/property-test fixtures and never count as field-validation evidence.

Promotion uses a published evidence report plus explicit operator/maintainer sign-off. Universal
recall, jitter and extrinsic-consistency thresholds are not invented before the first real solid
datasets establish credible baselines; later evidence may justify adding preset-specific limits.
Any confirmed 90-degree quadrant flip blocks promotion of that preset to validated status until the
cause is fixed. This is a validation-status rule, not a runtime block.

- replay representative VLP-32C and Seyond recordings across their supported ranges;
- tune point-count, voxel, cluster, square-fit and acceptance settings per operating profile;
- label visible/absent intervals and report detection coverage and false detections where observable;
- use a supplemental stationary interval to measure translation and rotation stability;
- rotate/tilt through allowed placements and report quadrant discontinuities;
- run LiDAR-camera overlay validation;
- compare extrinsics solved from independent time windows/subsets;
- run LiDAR-to-LiDAR repeatability validation when both sensors observe the target;
- collect replacement sample data before considering hollow-target retirement.

Low ICP or reprojection residual alone does not satisfy these gates.

## Review decisions

Questions are resolved one at a time during the grilling session.

1. **Resolved:** each sensor uses one operator-selected Target Definition per launch; simultaneous
   targets are unsupported.
2. **Resolved:** Target Identity hashes a shared canonical encoding of semantic values; source
   formatting does not affect identity.
3. **Resolved:** use `Calibration Target` / `target_config`, rename hollow-specific internal modules,
   and preserve accurate, stable ROS names containing `board`.
4. **Resolved:** `lidar_orientation_reference` explicitly states `mounting_up` or
   `asymmetric_cutouts`; validation checks the claimed physical evidence.
5. **Resolved:** derive board-up candidates from fitted square corners, select the maximum dot
   product with sensor-up, and hard-require alignment `>= 0.90`.
6. **Resolved:** solid refinement separates square-observed in-plane translation/yaw from
   plane-observed normal offset/tilt; `BoardIcpIterator` remains for the hollow adapter and rollback.
6a. **Resolved:** retain `BoardIcpIterator` internally and for hollow production use; do not expose a
    solid-target runtime fallback.
6b. **Resolved:** the common estimator seeds and diagnoses hollow detection, but cutout-aware
    `BoardIcpIterator` remains final authority with no silent fallback.
7. **Resolved:** continuous one-Capture solving remains publishable, savable and exportable for the
   solid target; quality reporting may warn, but does not overrule the operator.
8. **Resolved:** version-4 Autoware export remains accepted; restoring or re-solving version 4
   requires explicit target-binding migration to version 5.
9. **Resolved:** use separate Detector Tuning presets per supported sensor-target pair; derive true
   geometry from the Target Definition but keep sensor/noise/point-density thresholds explicit.
10a. **Resolved:** validation is per sensor-target Detector Tuning preset; experimental status is
    informative and never a runtime refusal.
10b. **Revised and resolved:** real moving solid-board bags provide the primary field evidence;
    historical hollow bags are reference data, not a controlled A/B benchmark. Short real static
    and board-absent supplements cover jitter and false-positive evidence that motion cannot.
10c. **Resolved:** promotion uses a published evidence report plus explicit operator/maintainer
    sign-off; fabricated data is test-only, and numeric limits wait for real baselines.
10d. **Resolved:** any confirmed quadrant flip blocks validated status until fixed; experimental
    runtime use remains available.

## Decision record

[ADR 0003](../../adr/0003-selectable-calibration-targets.md) records why both targets share one
interface, why selection is fixed per launch, and why the hollow adapter remains until the solid
target has replacement field evidence.
