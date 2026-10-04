# Extrinsic validation against held-out observations

- **Date:** 2026-10-04
- **Status:** Shared pipeline implemented; local reference evaluator verified; student exercise specified
- **Area:** `lctk_extrinsic_validation`, Detection Archive, runtime frame binding
- **Scope:** Offline LiDAR-camera validation after the calibration workflow ends

## Publication boundary

The validator is a student implementation exercise. Keep `lctk_extrinsic_validation`
on its own local feature branch; publish shared pipeline changes, tests, this contract,
and operator documentation to `origin/main`. Rebase `origin/2026-lab1` onto the updated
main branch and publish lab-specific documentation updates there. A main or lab checkout
does not ship the evaluator package until students implement it. The CLI below defines
the intended interface of that implementation.

## Purpose and interpretation

Evaluate a fixed extrinsic against Detection Pairs from separate recordings reserved for
validation. Project Calibration Target corners recovered from LiDAR observations into the
camera and compare them with observed image corners.

This measures agreement on held-out observations. Without independent ground truth, the
result does not establish absolute translation or rotation accuracy. The residual includes
errors from the candidate extrinsic, camera intrinsics, camera corner detection, LiDAR
target detection, target geometry, and synchronization. Shared systematic errors can remain
undetected. Coverage helps the operator interpret a small residual from a narrow dataset.

### Terms

- **Candidate Transform:** The fixed extrinsic selected for evaluation, including a saved
  manual adjustment when present.
- **Validation Dataset:** Detection Pairs collected from separate reserved recordings with
  unchanged sensor mounting and camera settings.
- **Validation Reprojection Error:** Euclidean pixel distance between an observed camera
  corner and the corresponding target corner projected using the Candidate Transform.
- **Validation Report:** Reprojection metrics, dataset coverage, provenance, and scoring
  completeness. Completeness describes whether observations could be scored; it is not an
  accuracy verdict.

These terms will join the canonical glossary in `AGENTS.md` during implementation.

## Operator workflow

1. Finish the normal calibration workflow and save its Detection Archive, including the
   selected transform.
2. Keep the sensor mounting and camera settings fixed. Record separate bags with varied
   Board Placements for validation.
3. Play those recordings through the existing `lidar_board_detector` and
   `aruco_locator_node`. Retain Detection Pairs through the existing manual or assisted
   capture workflow and save one or more validation Detection Archives.
4. Run the offline evaluator with the calibration archive, validation archives, and the
   validation Target Definition.
5. Inspect the report's completeness, placement coverage, and reprojection errors.

The existing capture workflow may compute an estimate while preparing validation archives.
The evaluator ignores that estimate and the archives' Quality Verdicts.

### Command

```bash
ros2 run lctk_extrinsic_validation validate \
  --extrinsic calibration.json \
  --validation validation1.json validation2.json \
  --target-config target.json5 \
  --output validation_report.json
```

The initial interface accepts Detection Archives only. Bag playback uses the existing
workflow. Additional transform formats and a dedicated capture node are outside this
implementation.

## Inputs and compatibility

### Candidate Transform

`--extrinsic` names a version-6 Detection Archive. Its saved `transform.rvec` and
`transform.tvec` are required, finite three-element vectors. The evaluator uses exactly
those values, whether they originated from a Solved Estimate or an Adjusted Transform.
The source archive must contain an observation that establishes its LiDAR header frame;
an empty source cannot provide the candidate's frame provenance and is rejected.

The archive stores the raw transform **camera optical coordinates from LiDAR coordinates**:

```text
X_camera = Rodrigues(rvec) * X_lidar + tvec
```

Translations are in metres; the rotation vector is in radians. The published TF's inversion
convention remains owned by the current publication/export code. Evaluation consumes the
archive's raw transform directly.

### Validation archives

Every input must be version 6 with the supported board-frame convention and valid Target
Identity. All validation archives must match the supplied Target Definition. The source
calibration archive may identify a different Calibration Target: the candidate relates
sensor coordinates, while validation geometry comes from the supplied target.

One invocation evaluates one validation Target Definition. Missing whole markers are
allowed. A visible known marker must supply all four valid corners in the established
detector/target corner order.

### Projection compatibility

All source and validation archives must agree on:

- Camera `K`, compared exactly after numeric normalization, without a tolerance.
- Image width and height.
- Camera optical frame label and LiDAR frame label.
- Projection model `undistorted_pixels_using_k`.

This first implementation deliberately requires unchanged camera settings. Different
intrinsics can be meaningful in a broader validation design, but accepting them is outside
the selected workflow. The source archive also requires projection metadata even though
its observations are not numerically evaluated.

Every saved camera detection header must agree with its archive's camera frame. LiDAR
detection headers must consistently identify the archive's LiDAR frame. Frame labels must
be nonempty and distinct. Header consistency cannot prove that a publisher correctly names
its coordinates or that the physical mounting is unchanged.

### Separation and duplicate handling

Compare validation observations against **all** Captures in the source calibration archive,
including Captures that robust estimation may have excluded from its numerical fit. An
exact shared observation is an input error.

Exact duplicates among validation inputs are retained once, in input order, and counted in
the report. Deduplication does not make a report incomplete. Observation comparison uses a
canonical representation of the saved Detection Pair, including both sensor headers and
payloads; archive-level transforms, quality, and input path do not affect equality.

This catches exact reuse, including copied archives. It does not prove independent bag
origin or identify near-duplicate observations with different saved content. The operator
owns recording separation.

## Detection Archive version 6

Version 6 adds required projection provenance:

```json
{
  "version": 6,
  "camera_projection": {
    "model": "undistorted_pixels_using_k",
    "frame_id": "camera_optical_frame",
    "width": 1920,
    "height": 1080,
    "k": [1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0]
  }
}
```

The snippet illustrates the new fields; an actual archive also contains the existing
board-frame convention, Target Identity, detection count, Detection Pairs, and optional
transform and quality fields.

Validate supported model, nonempty frame, positive integer dimensions, and a finite valid
3-by-3 camera matrix with positive focal lengths and the expected homogeneous row. Saved
metadata must describe the buffered camera observations. Archive snapshots copy projection
metadata, Captures, and the selected transform under the same state lock.

The model records an existing coordinate convention: the ArUco detector undistorts corners
using CameraInfo `K` and `D`, with `P = K`, in
`rust/aruco-detector/src/multi_aruco.rs`. Saved image corners therefore remain pixels under
`K`. Evaluation projects with that `K` and zero distortion. Applying CameraInfo `D` again
would compare different coordinate models. Version 6 does not need to save `D` to score
these already-undistorted corners.

A change in `K`, image dimensions, or camera frame starts a new capture epoch. The runtime
clears retained Captures and derived state so one archive cannot mix projection models.

The archive writer, restore path, validator, and Autoware exporter require version 6.
Earlier versions are rejected with a clear diagnostic. Remove the legacy migration command,
module, helper code, tests dedicated to migration, and maintained migration documentation.
There is no compatibility mode or external CameraInfo fallback.

## Runtime frame ownership

Remove device `frame_id` from the session schema, parser, shipped manifests, templates, and
maintained documentation. Configured labels can disagree with the actual coordinates.
Detection headers become the source of archive and published transform labels.

### LiDAR-camera solver

- Bind the LiDAR frame from the synchronized board detection header and camera frame from
  the synchronized ArUco detection header.
- Require CameraInfo's frame to match the camera detection frame before accepting Captures.
- Remove configured `parent_frame` and `child_frame` solver parameters.
- Require nonempty, distinct frame labels before solving or publishing.
- Restore frame binding from consistent archive headers and projection metadata.
- Use the bound LiDAR frame for assisted review scene/cache coordinates.

### LiDAR-LiDAR solver

Remove configured `lidar1_frame` and `lidar2_frame` parameters. Derive labels from the two
ordered synchronized detection headers, preserving the configured logical device ordering.
Require nonempty, distinct labels.

### Epoch changes and source labels

When a bound frame changes, invalidate cached Detection Pairs, retained Captures where
applicable, estimates, adjustments, and publication state. Advance the generation used by
ongoing work so a result computed in the preceding epoch cannot publish. A new valid pair
may establish the replacement binding; no numerical result survives a frame change.

Bag and live sources preserve their publisher headers. The `pcap_avi` session source
generates labels from device names: the LiDAR device name and the camera device name with
`_optical_frame` appended. Reject generated collisions. Direct source launch arguments
remain publisher settings where needed; session-level overrides are removed.

Session checking reports that frames come from sensor headers at runtime. `reference_frame`
continues to select a logical device and retains its existing meaning.

## Scoring algorithm

For each unique validation Capture:

1. Read and validate the LiDAR observation of the Calibration Target's board position and
   quaternion.
   Recover target-local marker corners from the supplied Target Definition.
2. Match camera detections to known marker IDs. Preserve the current correspondence order
   through a shared pure helper used by the solver and evaluator.
3. Transform target-local corners into LiDAR coordinates using the observed board pose.
4. Apply the fixed Candidate Transform and project through `K` with zero distortion.
5. Compute each matched corner's Euclidean pixel error.

No solve, refinement, covariance weighting, or residual-based outlier rejection occurs.
Recognizable marker IDs absent from the Target Definition provide no correspondences. An
unparseable marker ID, malformed known marker, duplicate known marker ID, invalid board
pose, or absence of usable correspondences makes the Capture unscorable. Invalid numeric
values never enter metric computation.

If any matched point has nonpositive camera depth, the entire Capture is unscorable.
Nonfinite projections are also unscorable. Finite projections outside image bounds remain
in the metrics: the evaluator does not hide large errors by clipping or discarding points.

The shared correspondence extraction must preserve the current calibration solver's
selection rules and ordering. Validation can apply stricter report completeness rules
without silently changing solver admissions.

## Metrics and completeness

Each scored Capture records matched marker and corner counts, corner RMS in pixels, and
maximum corner error. Each unscorable Capture records explicit failure reasons. Both retain
source archive path, original capture index, and camera/LiDAR timestamps.

### Aggregation

Let `e_ij` be corner `j`'s Euclidean pixel error in Capture `i`, and `n_i` its corner count:

```text
capture_mse_i = sum_j(e_ij²) / n_i
placement_mse_p = mean(capture_mse_i for Captures i in Placement p)
placement_balanced_rms = sqrt(mean(placement_mse_p over Placements p))
```

Placement-balanced RMS is the headline metric. Equal weighting across Placements prevents
many observations at one static placement from dominating the result. Partial visibility
does not increase a Capture's weight just because it supplies more corners.

Also report pooled corner RMS, median/p95/maximum per-Capture RMS, and per-archive summaries.
Use NumPy's linear percentile convention for p95. State metric units and populations in the
report. Per-archive summaries apply the same completeness rules to that archive's retained
unique Captures; duplicate-only archives have no retained scoring population.

Reuse existing placement grouping: 5 cm position tolerance and 5 degrees plane-normal
tolerance, greedy representatives in first-seen order, ignoring in-plane rotation and normal
sign. Record grouping tolerances in the report.

Compute placement membership once from all retained Captures with valid board poses, and
assign stable placement IDs. Reuse this grouping for coverage, per-archive summaries, and
usable-subset diagnostics. Subset metrics average only Placements with at least one
scorable Capture; they do not regroup the subset after exclusions.

### Coverage

Report distinct placement count, normal span, depth range, and lateral span using existing
`lctk_quality` geometry semantics. Depth range is the range of radial distances from the
LiDAR; lateral span is the LiDAR Y extent. Label those definitions to avoid implying camera
optical depth. Coverage includes retained Captures with valid board poses even if their
image projection cannot be scored. Record this population and its count explicitly.

### Complete and incomplete reports

A report is complete only when every unique retained validation Capture is scorable and
the dataset contains at least one Capture. Large residuals alone do not make it incomplete.

When complete, publish full-dataset metrics. When incomplete, full-dataset aggregate metrics
are `null`; publish any metrics from scorable Captures under explicitly named usable-subset
diagnostics. Never present the usable subset as the full Validation Dataset.

When zero Captures are usable, still write the report with failure counts and null metrics.
An empty validation dataset is incomplete with an explicit empty-dataset diagnostic.

## Report and CLI behavior

Use a versioned JSON report containing:

- Completeness and input/retained/scorable/unscorable/duplicate counts.
- Source and validation paths and SHA-256 hashes of the bytes read.
- Source and validation Target Identities, fixed candidate vectors, and projection metadata.
- Full-dataset metrics or null values, plus usable-subset diagnostics when needed.
- Coverage definition, population counts, placement grouping, and per-archive summaries.
- Per-Capture results or failures and original source indices/timestamps.

Read each archive once so hashing and evaluation refer to the same content. Emit finite
JSON: absent or unavailable metrics are `null`, never NaN or infinity. Write output
atomically using a temporary sibling file and replacement.

- Exit **0** after writing a complete report, regardless of error magnitude.
- Exit **nonzero** after writing an incomplete report, including zero usable Captures.
- Reject unreadable, incompatible, or overlapping inputs before scoring. Archive envelope,
  version, identity, projection, frame binding, and candidate-transform failures are input
  errors; a report is not required. Individual Capture payload defects are scoring failures
  and produce an incomplete report. A malformed detection collection or a declared count
  inconsistent with its length is an archive envelope error.

There is no automatic accuracy threshold or pass/fail verdict in this version.

## Implementation boundaries

- Add the Python ament package `ros/lctk_extrinsic_validation` with a thin CLI and a pure,
  ROS-independent evaluator. Use system-compatible NumPy, SciPy, and OpenCV dependencies.
- Reuse `lctk_target` for Target Definition loading and identity checks.
- Reuse `lctk_quality` residual, placement, and diversity computations. Introduce small
  shared pure helpers for correspondence construction and projection metadata validation.
- Keep archive persistence ownership in the current packages. Share the projection
  metadata invariant and add parity tests for writer/restore/export/validation behavior.
- Decode saved observations without constructing generated ROS messages or instantiating
  a Detection Buffer. Evaluation must run without a ROS graph.
- Preserve existing numeric solver and Autoware transform conventions.

Update maintained user guides, package READMEs, the book navigation, and canonical domain
terms. Record decisions in ADRs for fixed held-out evaluation, version-6 provenance, and
header-owned frames. Historical documents remain historical.

## Acceptance conditions

### Public test seams

Exercise these boundaries through their observable behavior:

1. The offline evaluation API: archive inputs and a Target Definition produce a report or
   an input error, without a ROS graph.
2. The `validate` CLI: files, arguments, output JSON, and process exit status.
3. Detection Archive encode/restore and Autoware load/export interfaces: version,
   projection provenance, frame consistency, and transform preservation.
4. Solver observation/CameraInfo callbacks and their published transform, archive, and
   review outputs: frame binding, epoch reset, and suppression of stale work.
5. Session parsing, Node Plan construction, session checking, and source launch parameters:
   removed overrides and generated publisher labels.

Existing public correspondence and quality interfaces supply regression coverage. Tests
assert known geometric outcomes and user-visible behavior rather than private helper calls.

### Offline evaluation

- Synthetic correct transforms produce near-zero pixel residuals; perturbed transforms
  increase residuals. The saved candidate remains unchanged and no solve path is invoked.
- Partial marker visibility, corner ordering, finite out-of-bounds projections, malformed
  detections, duplicate marker IDs, invalid poses, and nonpositive depth follow the stated
  scoring rules.
- Tests distinguish placement-balanced weighting from pooled corner weighting and exercise
  multiple archives, empty input, incomplete input, and zero usable Captures.
- Exact training overlap is rejected; validation duplicates are counted and retained once.
- Different source/validation targets work; mismatched validation target identity fails.
- Projection/frame/version incompatibilities fail before scoring.
- Report serialization, atomic replacement, provenance hashes, and exit codes obey the
  contract. Adjusted saved transforms are evaluated exactly as stored.

### Archive and runtime integration

- Every maintained consumer accepts valid v6 and rejects older versions consistently.
- Camera projection metadata is a coherent snapshot of the archived capture epoch.
- Actual headers label transforms and archives; invalid labels block acceptance/publication.
- Frame, intrinsics, or dimension changes invalidate previous state and suppress stale
  results. Restored archives establish binding and later live changes invalidate it.
- Assisted review uses the bound frame. LiDAR-LiDAR device ordering stays correct.
- Session schema rejects obsolete frame overrides; `pcap_avi` generated labels and collision
  handling are exercised. Maintained sessions parse without the removed field.
- Correspondence extraction preserves existing calibration numeric behavior and selection.
- The retired migration command is absent from installed entry points and maintained docs.

### Repository gates

Wire the new test directory into `just test`; deliberately break an assertion and confirm
that the recipe exits nonzero, then restore it. Before claiming implementation complete,
run `just build`, `just test`, and `just lint-py`, and check documentation links. Preserve
pre-existing user changes and exclude build-generated lockfile churn from the feature.

## Deferred work

Other extrinsic input formats, direct bag orchestration in the evaluator, multiple
validation targets per invocation, changed-camera-intrinsic evaluation, independent
ground-truth acquisition, uncertainty estimates, and automatic accuracy thresholds are
outside this version.

## Implementation process

Implementation uses subagents and test-driven development at the public seams above.
Assign disjoint file ownership and agree shared helper contracts before parallel edits.
Each behavior proceeds as one failing test, observed failure, minimal implementation,
and observed passing test. Continue in vertical slices; retain the red/green evidence.
Establish existing correspondence behavior before extracting shared code.

Use GPT-6 Luna with maximum reasoning for general implementation work. Consult GPT-6.1 Sol
with medium reasoning before nontrivial implementation or architectural commitment, and
again before declaring completion. Advisors receive this spec and only relevant,
prefiltered files. Review integrated changes, then run the repository gates above.
