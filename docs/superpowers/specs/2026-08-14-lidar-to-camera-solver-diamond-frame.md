# Spec: `lidar_to_camera_solver` on the corner-aligned board frame (Phase 2)

- **Date:** 2026-08-14
- **Status:** Stage 1 and Stage 2 implemented; Stage 3 pending
- **Phase 1:** `2026-08-13-corner-aligned-board-frame.md` (landed, field-validated on the two-LiDAR rig)
- **Closes:** H-11. **Incidentally closes:** M-12 (at Stage 2), M-14
- **Related:** M-13, M-21, L-10

## Glossary of names this spec fixes

Everything below is a decision, not a description. Implementations must use these exact names.

| Thing | Name |
|---|---|
| New package, its ROS node, and its console entry point | `lidar_to_camera_solver` |
| Package it is created from (by `git mv`) | `advanced_extrinsic_solver` |
| Package deleted without ever being ported | `extrinsic_solver_node` |
| Mode parameter | `solver_mode` |
| Mode values | `continuous` (default), `manual` |
| Launch/justfile selector being **removed** | `use_advanced_solver` |
| Shared geometry module inside the new package | `board_geometry.py` |
| Frame-convention topic (absolute) | `/lctk/board_frame_convention` |
| Frame-convention identifier | `corner_aligned_plate_center_v1` |
| Detection file format version | 3 → **4** |
| Cross-language fixture | `marker_corners_world.golden.json` |
| Marker paper placement config key | `paper_placement`, in `aruco_pattern.json5` |

## Problem Statement

Phase 1 redefined what a published board pose means: the board model's local axes now run corner to
corner with the origin at the plate centre, matching how the board is physically hung. The LiDAR side
was ported, validated on the TWO_LIDAR recordings, and shipped.

The camera side was not. Both `extrinsic_solver_node` and `advanced_extrinsic_solver` still build
board-local marker coordinates in the old edge-aligned frame with the origin at a plate corner. Their
`_compute_multi_marker_corners` implementations are AST-identical to each other and stale in exactly
the same way.

That convention appears on **both sides** of one product. The published pose is `T_sensor←board`; the
solver supplies board-local marker coordinates to it. Changing only the LiDAR side produces two
errors simultaneously:

- a **45° in-plane rotation**, which is undetectable — the 2×2 ArUco grid is symmetric, so PnP still
  solves cleanly with low reprojection error;
- an **origin shift of ~707 mm** (the plate's half-diagonal for a 1000 mm board), which probably
  would be caught.

Half the failure is silent. Any LiDAR-camera calibration run from the current tree is wrong. This is
present-tense, not a latent risk.

Three further problems sit alongside it and are cheaper to fix now:

**The geometry exists twice and the code around it has drifted.** `extrinsic_solver_node` uses
`cv2.SOLVEPNP_ITERATIVE` with no refinement pass, float32 throughout, and discards the board pose's
6×6 covariance. `advanced_extrinsic_solver` uses `cv2.SOLVEPNP_SQPNP` followed by
`cv2.solvePnPRefineLM` (or a weighted SciPy Levenberg–Marquardt refinement when covariance weights
exist), float64 throughout, and propagates the covariance onto per-corner weights. L-10 was fixed
only in the latter. M-12 documents the estimator asymmetry and is still open. The weaker of the two
is the shipped default and the one `just demo` runs.

**A saved calibration cannot say which convention produced it.** The version-3 detection file stores
no board-local coordinates — corners are recomputed at load time from `aruco_pattern.json5` — so
files written before and after Phase 1 are indistinguishable, and either reloads under whatever
convention the loading code believes in.

**`lctk_autoware_export` does not check the format version at all.** It reads only
`transform.rvec`/`transform.tvec` and ignores `version` entirely — its own test fixtures declare
`"version": 2` and pass. It then writes that transform into a `sensor_kit_calibration.yaml` that ends
up on a vehicle.

## Solution

Consolidate the camera side into `lidar_to_camera_solver`: one package, one geometry implementation
ported to the corner-aligned frame, two named operating modes.

Make the phase boundary loud. The `lidar_board_detector` already publishes
`corner_aligned_plate_center_v1` on the latched topic `/lctk/board_frame_convention`;
`lidar_to_camera_solver` subscribes and refuses to start unless it matches. **Absence of the tag is
failure, not consent.**

Bump the detection format to version 4 so a stored calibration records the convention that produced
it, and give `lctk_autoware_export` the version check it has never had.

Three stages, each independently verifiable:

- **Stage 1** — `git mv advanced_extrinsic_solver → lidar_to_camera_solver`, extract
  `board_geometry.py`, add the guard, port to the corner-aligned frame, assert against the golden,
  bump to version 4. This is the stage that ends the silent 45° error.
- **Stage 2** — add `solver_mode: continuous` on the migrated backend.
- **Stage 3** — delete `advanced_extrinsic_solver` and `extrinsic_solver_node`.

Stage 2 does not begin until the repository owner has visually confirmed Stage 1 against recorded
camera data.

## User Stories

**Operator — correctness and safety**

1. As a calibration operator, I want a LiDAR-camera calibration run from a current checkout to produce a correct extrinsic, so that I am not shipping a silently rotated transform.
2. As a calibration operator, I want `lidar_to_camera_solver` to refuse to start when it disagrees with `lidar_board_detector` about the board frame, so that I get a failure instead of a plausible wrong answer.
3. As a calibration operator, I want that refusal to print both the identifier I expected (`corner_aligned_plate_center_v1`) and the one I received, so that I know what to change rather than merely that something failed.
4. As a calibration operator, I want the solver to refuse to start when nothing has been published on `/lctk/board_frame_convention` at all, so that starting before a detector cannot be mistaken for agreement.
5. As a calibration operator, I want the solver to wait 10 seconds for that announcement rather than failing instantly, so that ordinary launch races do not stop me working.
6. As a calibration operator, I want the solver to keep validating every message on that topic after startup, so that a detector restarting on an older build is caught rather than trusted because the first check passed.
7. As a calibration operator, I want a saved detection file to record the frame convention that produced it, so that I can tell whether an old calibration is still meaningful.
8. As a calibration operator, I want a version-3 file to be rejected with a clear message rather than silently reinterpreted, so that a stale calibration cannot quietly become a wrong one.
9. As a calibration operator, I want an explicit command to convert a version-3 file to version 4, so that I can keep calibrations I still trust without hand-editing JSON.
10. As a calibration operator, I want `lctk_autoware_export` to refuse a file whose version it does not understand, so that a stale-convention transform cannot reach a vehicle's sensor kit.
11. As a calibration operator doing LiDAR-to-LiDAR calibration, I want `lidar_to_lidar_solver` to keep working untouched throughout, so that this work does not block the calibration I am actually doing.

**Operator — one solver, two modes**

12. As a calibration operator, I want a single LiDAR-camera solver package rather than two, so that I do not have to know which of two similar things I am running.
13. As a calibration operator, I want the two behaviours named `continuous` and `manual` rather than "default" and "advanced", so that the name tells me what it does.
14. As a calibration operator, I want to select the behaviour with `solver_mode`, so that the choice is visible in my launch command and my justfile invocation.
15. As a calibration operator, I want `solver_mode` not to collide with the existing `mode` argument (`offline`/`realtime`), so that a launch file mentioning both is readable.
16. As a calibration operator, I want `continuous` to be the default, so that the quickest path to a sanity check is unchanged.
17. As a calibration operator, I want `interactive_solver_controller` to keep working against the migrated solver, so that my multi-pose workflow survives the move.
18. As a calibration operator, I want the service paths documented in the Autoware export guide to match the migrated node's name, so that the guide is not quietly wrong.
19. As a calibration operator, I want only one solver node running per camera-LiDAR pair, so that two publishers cannot fight over `extrinsic_transform`.

**Operator — confidence in the result**

20. As a calibration operator, I want to verify the port by seeing the projected point cloud land on the board in a camera image, so that I have direct evidence rather than a residual number.
21. As a calibration operator, I want to be told explicitly that reprojection RMS cannot detect this class of failure, so that I do not accept a low number as proof.
22. As a calibration operator, I want the validation to use sample dataset 3, which I already have, so that verifying the port does not require a new capture session.

**Maintainer — one implementation**

23. As a maintainer, I want exactly one Python implementation of the board's marker geometry, so that the frame port lands once rather than twice.
24. As a maintainer, I want `board_geometry.py` to import nothing from `rclpy`, so that it can be tested without a running ROS graph.
25. As a maintainer, I want the marker paper's placement read from `paper_placement` in `aruco_pattern.json5`, so that Python and Rust consume the same measured number instead of each deriving it.
26. As a maintainer, I want the migration performed as `git mv`, so that the frame port reads as an honest diff against known-good code rather than a wall of new lines.
27. As a maintainer, I want the package, the ROS node and the console entry point all named `lidar_to_camera_solver`, so that a reader is not tracking three aliases for one thing.
28. As a maintainer, I want `advanced_extrinsic_solver` and `extrinsic_solver_node` deleted once their behaviour is absorbed, so that nobody maintains three solvers.
29. As a maintainer, I want `continuous` mode rebuilt on the migrated backend, so that it inherits SQPnP, LM refinement, float64 and covariance weighting instead of preserving the weaker estimator.
30. As a maintainer, I want that estimator change identified as a behaviour change rather than a refactor, so that it is measured before it is trusted.

**Maintainer — the contract between languages**

31. As a maintainer, I want `board_geometry.py` asserted against the same `marker_corners_world.golden.json` the Rust `hollow-board-config` tests use, so that the two cannot drift apart unnoticed.
32. As a maintainer, I want that fixture moved out of the Rust crate's `tests/fixtures/` to a neutral location both languages read, so that it is not a guest in one side's test tree.
33. As a maintainer, I want the golden keyed by ArUco marker id, so that the binding whose corruption causes a silent quarter-turn is pinned explicitly.
34. As a maintainer, I want the golden expressed in world coordinates at a stated physical mounting, so that it means the same thing in both languages and survives the port unchanged.
35. As a maintainer, I want `corner_aligned_plate_center_v1` defined once per language, so that the tag and the geometry cannot be changed independently.

**Maintainer — the guard**

36. As a maintainer, I want the convention comparison written as a pure function over the received string, so that it is testable without constructing a node.
37. As a maintainer, I want the subscriber QoS to be `RELIABLE` + `TRANSIENT_LOCAL` + `KEEP_LAST(1)`, matching the publisher exactly, so that the guard does not fail closed because of its own bug.
38. As a maintainer, I want that QoS pairing stated in a comment with the reason, so that the L-07 durability-mismatch defect is not repeated.
39. As a maintainer, I want the guard's failure to exit non-zero, so that the launch system can observe it.

**Maintainer — the saved format**

40. As a maintainer, I want the format bumped to version 4 in the same change that alters what a stored pose means, so that version and meaning stay in step.
41. As a maintainer, I want the 6×6 pose covariance that the version-3 serializer discards to be stored, so that saving and reloading a buffer does not silently change the solve.
42. As a maintainer, I want every reader of the format — the solver's `load_detections` service and `lctk_autoware_export` — enumerated and updated, so that a version check in one place is not undermined by a consumer that ignores it.

## Implementation Decisions

### The package: `lidar_to_camera_solver`

Created by `git mv ros/advanced_extrinsic_solver ros/lidar_to_camera_solver`, then renaming symbols
inside — **not** authored fresh. History is preserved and the frame port appears as a reviewable diff
against code known to work.

The package name, the ROS node name and the `setup.py` console entry point are all
`lidar_to_camera_solver`. This deliberately breaks two things that must be fixed in the same commit
as the move:

- `interactive_solver_controller` hardcodes `NODE_NAME = "advanced_extrinsic_solver"` and discovers
  the solver by scanning for `/{NODE_NAME}/get_pose_info`, building all ten service client paths from
  it.
- The Autoware export guide documents the dump service path literally, including the old node name.

`advanced_extrinsic_solver` also imports `lctk_interfaces.srv` without declaring the dependency in its
`package.xml`. The new package declares it.

### Modes: `solver_mode`

One parameter, `solver_mode`, with values `continuous` and `manual`. Default `continuous`.

It is deliberately **not** named `mode`: that name already denotes the `offline`/`realtime` processing
mode across `calibrate.launch.py`, the justfile and CLAUDE.md, and two unrelated meanings in one
launch file would be actively confusing.

`use_advanced_solver` is **removed outright, with no deprecated alias**. It is declared in
`calibrate.launch.py`, branched on to choose between the two packages, forwarded from
`demo.launch.py`, and defaulted to `"false"` in four places in the justfile. All become `solver_mode`,
with the justfile variable reading `solver_mode := "continuous"`.

A deliberate consequence: with the boolean gone, `extrinsic_solver_node` becomes **unreachable from
the config-driven launch path**. It can still be started directly with `ros2 run`, but nothing in
`just lidar-camera` or `just demo` can select it. This retires it safely at Stage 1 without any effort
spent porting or guarding code that is deleted at Stage 3.

Only one mode runs per node, so two publishers on one `extrinsic_transform` topic is structurally
impossible.

### `board_geometry.py`

The genuinely shared, genuinely identical code moves into one module inside the new package:
`_compute_multi_marker_corners`, `_load_aruco_pattern_config`, `_parse_dimension`,
`_detection2d_to_aruco_markers`, and the rotation-matrix-to-quaternion conversion — roughly 250 lines.

It imports nothing from `rclpy`, so it is importable and unit-testable without a graph. Stage 2 folds
`continuous` mode into the same package and imports the same module directly. **No separate shared
package is created**: after Stage 3 there is exactly one consumer, and a package with one user is a
liability rather than an abstraction. If a third consumer ever appears, extracting it then is trivial.

The module gains the **paper-coordinate adapter** the Rust `BoardModel::marker_paper_point` already
has. Marker positions are computed in the paper's own `(u, v)` coordinates and mapped onto the plate
through a stated placement, rather than being emitted directly as board-local coordinates. The
placement comes from `paper_placement` in `aruco_pattern.json5` — which both solvers already load, and
which already carries the measured value. **No new launch parameter is required**, and `board_width`
— which the Python side does not read today and does not need — stays out of it.

### The frame-convention guard

`lidar_board_detector` publishes `corner_aligned_plate_center_v1` on `/lctk/board_frame_convention`,
once, with `KEEP_LAST(1)` + `RELIABLE` + `TRANSIENT_LOCAL`, holding the publisher handle so the latched
sample survives.

`lidar_to_camera_solver` subscribes with **exactly matching QoS**: `RELIABLE`, `TRANSIENT_LOCAL`,
`KEEP_LAST(1)`. This is called out explicitly because the repository's one recorded lesson here is
L-07, a durability mismatch in `tf_tree_broadcaster.py` that silently delivered nothing; the surviving
comment there explains why *not* to use `TRANSIENT_LOCAL`, and following it blindly would give this
guard a volatile subscriber that receives no tag, concludes "absent", and refuses to start — turning
a guard into an outage.

Behaviour:

- wait up to **10 seconds** at startup for a tag;
- on timeout, fail;
- on mismatch, fail, naming both expected and received;
- validate **every** subsequent message too, so a detector restarting on a stale build is caught.

Failure is raised during node construction. Both current solvers construct the node outside any
`try` block, so an exception in `__init__` propagates and exits non-zero — unlike
`calibration_judge`, whose `main()` swallows its own fatal and exits 0. Follow the solver pattern, not
the judge's.

The comparison itself is a pure function over the received string, so the decision table
(match / mismatch / absent) is testable without a graph.

### The version-4 detection format

Version 3 gains one field: the frame-convention identifier, using the same string the detector
publishes, so there is one vocabulary rather than two.

**Version 3 files are rejected**, with a message naming the conjugation and pointing at an explicit
one-shot conversion command. Automatic migration on load is rejected as a design: it would make a
file's meaning depend on which build opened it, which is the same class of silent-difference problem
this entire phase exists to remove. Note the existing loader already rejects unknown versions with a
generic message and accepts 1, 2 and 3 — 1 and 2 with a loud warning.

The version-3 serializer drops the board pose's 6×6 covariance, so a reloaded buffer always solves
with uniform weight 1.0 and quietly differs from the live buffer it was saved from. Since the format
is being opened anyway, **version 4 stores it**, rather than costing another version bump later.

`lctk_autoware_export` gains a version check. It is currently version-blind, and it writes into a file
that reaches a vehicle, making it the single most important place for the check to exist. Its own
fixtures, which declare `"version": 2`, are updated.

### Staging and commit order

**Stage 1**, six commits in this order:

1. `git mv` and rename, updating `interactive_solver_controller`, the Autoware export guide,
   `CLAUDE.md`, `package.xml`, `setup.py`, the root `Cargo.toml` member list and the justfile test
   path in the same commit;
2. extract `board_geometry.py`, no behaviour change;
3. add the frame-convention guard;
4. port `board_geometry.py` to the corner-aligned frame;
5. add the golden assertion and move `marker_corners_world.golden.json` to its neutral home,
   updating the Rust test's `include_str!`;
6. bump to version 4 and add the `lctk_autoware_export` version check.

The guard precedes the port, which is the property that matters: at no point does a ported solver
exist without the check that its counterpart agrees. It follows the move so it is written once, in its
final home, rather than into a package about to be renamed.

**Stage 2** adds `solver_mode: continuous` to `lidar_to_camera_solver`. Because it is built on the
migrated backend, it inherits SQPnP + `solvePnPRefineLM`, float64, and covariance weighting — which the
superseded `extrinsic_solver_node` lacked. This closes M-12's asymmetry as a side effect. It is a
**behaviour change to the continuous path, not a refactor**, and is measured before and after on
dataset 3 rather than assumed.

**Stage 3** deletes `advanced_extrinsic_solver` and `extrinsic_solver_node` and every reference to
them: the root `Cargo.toml` members, `lctk_launch`'s `package.xml` exec-depends, both stale package
READMEs, the duplicate `extrinsic_solver_node.launch.xml` files each ships, `CLAUDE.md`, `README.md`,
`CONTRIBUTING.md`, and the book's architecture and build-system directory trees.

### The golden fixture

`marker_corners_world.golden.json` and its independent Python generator move out of
`rust/hollow-board-config/tests/fixtures/` to a neutral location both languages read, with the Rust
test updated to match. A contract between two implementations should not live inside one of them, and
a fixture buried in a crate's test directory is one somebody eventually tidies away.

### What does not change

`lctk_quality` needs no changes: `placements.py` reads the board normal as rotation column 2, which
Phase 1 left as the normal, and its placement-distinctness thresholds compare poses to each other, so
a constant conjugation and constant origin shift cancel. `lidar_to_lidar_solver` is untouched. The
ArUco 2D detection path is image-space and unaffected.

The advanced solver's `_pose_weight` lever arms are wrong by ~707 mm *today*, because Phase 1 moved
the board pose's origin to the plate centre while the corners stayed corner-origin. Porting
`board_geometry.py` fixes this implicitly; no separate edit is needed, but it belongs in the test
surface.

## Testing Decisions

### What makes a good test here

Tests must assert **externally observable geometry** — where marker corners are in the world — and
never board-local coordinates, because those are exactly what this change redefines.

Two properties matter more than usual, both inherited from Phase 1:

- **Convention sensitivity.** A test built only from rotation-invariant quantities — inter-corner
  distances, dot products, reprojection residuals — cannot detect an in-plane relabelling and is
  therefore blind to this entire class of defect. Phase 1 found 51 assertions with exactly this flaw,
  all of them compiled out of both sanctioned build profiles besides.
- **Survivability.** Any assertion re-baselined at the moment of the port cannot verify the port.
  `marker_corners_world.golden.json` must pass **byte-identical** before and after, exactly as it did
  on the Rust side.

### Seams

**Primary seam: `board_geometry.py`'s public API.** The frame convention is entirely observable here;
everything downstream either consumes it or is image-space. Testing here rather than at the node
avoids requiring a ROS runtime for pure arithmetic.

**Existing cross-language seam: `marker_corners_world.golden.json`.** This is the contract itself. The
Rust `hollow-board-config` tests already assert against it; adding the Python assertion completes it
and is the single highest-value test in this spec.

**Two narrow pure-function seams:** the convention comparison, and version-4 load/save. Both are
functions over plain values, testable without a graph.

**Explicitly not seams:** the node lifecycle, the `cv2.solvePnP` call, and
`interactive_solver_controller`. The first two inherit correctness from the primary seam; the third is
unchanged by this work beyond a name.

### Modules tested

- **`board_geometry.py`** — the golden assertion keyed by ArUco marker id, plus a check that a stated
  `paper_placement` is honoured rather than plumbed through and ignored (slide the placement, assert
  every corner moves by exactly that world vector and the plate does not).
- **The convention comparison** — matching, mismatching, and absent, with absent asserted to *fail*.
- **The version-4 format** — round trip including the restored covariance, rejection of version 3,
  and `lctk_autoware_export` refusing a file it does not understand.

### Prior art

Phase 1's `board_frame.rs` and `boundary_projection.rs` are the established pattern for
convention-sensitive geometry assertions under randomised poses. `marker_layout_golden.rs` is the
established pattern for a fixture-backed cross-language contract, including an independent generator
that does not import the implementation it checks. `advanced_extrinsic_solver`'s pytest directory is
the established Python test location and is already in the justfile's `test` recipe; the new package
inherits it through the `git mv`, and the recipe's hardcoded path list is updated in the same commit.

Note that `extrinsic_solver_node` has no test directory and is in no test target. It does not acquire
one — it is deleted at Stage 3.

### Verification beyond unit tests

End-to-end verification runs against sample dataset 3 (pcap + 270-frame avi at 1920×1080), in which a
VLP-32C and a camera observe the same board simultaneously. Frame inspection confirms it is the same
physical board Phase 1 validated against: square plate hung as a diamond, three holes with none at the
bottom, and the ArUco sheet in the plate's lower quarter with its top corner at the plate centre —
matching `paper_placement`'s measured value.

The gate is **visual**: the projected point cloud must land on the board in the image, via
`pointcloud_image_overlay`. **Reprojection RMS is explicitly not a gate**, and this must be recorded
wherever the gate is described. A 45° in-plane error leaves reprojection error low because the marker
grid is symmetric; the observable signature is the ~707 mm origin shift, or the picture. Phase 1's
decisive evidence was likewise an operator looking at where an axis arrow pointed, not a number in a
log.

**The repository owner runs this check**, and Stage 2 blocks on their confirmation. Stage 2 then
carries its own before-and-after comparison on the same dataset, because it changes the estimator on
the continuous path.

## Out of Scope

- **Porting `extrinsic_solver_node`.** It is left untouched, becomes unreachable from the
  config-driven launch path when `use_advanced_solver` is removed, and is deleted at Stage 3 without
  ever being ported or guarded.
- **Merging the two packages as a single refactor.** Investigation established they do not share a
  backend — different PnP estimators, different precision, different covariance handling. A direct
  merge would have forced a numerical decision in the middle of a frame port. Staging avoids it.
- **RANSAC and outlier rejection** (the remainder of M-12). Neither implementation uses it today;
  adding it is a separate, separately-measured change.
- **Full REP-103 alignment** of the board frame — making X the normal to match `board-cluster-detector`.
  Unchanged from Phase 1: still deferred, still reduces to a column permutation and one sign flip,
  still requires changes to the quality metric and the detection publisher.
- **The crop-box (`bbox`) detection path**, which remains unexercised and unmeasured (M-17).
- **M-21**, ICP's unreachable stable-pose exit and the inert `icp_pose_weight_threshold`.
- **Two defects found while investigating**: `advanced_extrinsic_solver` importing `lctk_interfaces`
  without declaring it (fixed incidentally by the new package's `package.xml`, but worth its own note),
  and `debug_mode` declared but never read in *both* solvers. Filed separately.
- **Python linting.** `ruff` is not installed on the development machine, so `just lint`'s Python steps
  and `just lint-py` exit 127 without linting anything. Tracked separately.

## Further Notes

**Why the guard is a runtime check rather than documentation.** The convention appears on both sides
of the `T_sensor←board` product, so a mismatch is not merely undetected — it is *undetectable* by the
quality metric an operator would naturally consult. Documentation cannot prevent a failure whose whole
character is that it looks like success.

**Why absence must fail.** `TRANSIENT_LOCAL` means a late-joining subscriber receives the latched
sample — but only while a publisher is alive. A solver started before any `lidar_board_detector`, or
after the bag has finished and the detector has exited, sees nothing. This was confirmed empirically
during Phase 1 validation: `ros2 topic echo /lctk/board_frame_convention --once` returned nothing once
the detector had exited. Treating that as consent would make the guard useless precisely in the case
it exists for. It also means the launch system offers no ordering guarantee — `calibrate.launch.py`
builds a flat node list with no `TimerAction` or `OnProcessStart` anywhere — which is exactly why the
10-second wait exists.

**On the estimator asymmetry.** What ships as the default today is the weaker of the two solvers:
float32, `SOLVEPNP_ITERATIVE` with no refinement, and the board covariance discarded. Stage 2 resolves
this as a side effect of building `continuous` on the migrated backend, which is a better outcome than
porting the weaker estimator forward. It is called out here so the improvement is *measured* rather
than discovered.

**On the guard's position in the sequence.** An earlier decision had the guard landing first, before
anything else. It now lands third, after the move and the extraction. The property that decision
protected — that no ported solver ever exists without the check that its counterpart agrees — is
preserved, because the guard still precedes the port. What changed is only that it is written once, in
its final home, instead of into a package about to be renamed.
