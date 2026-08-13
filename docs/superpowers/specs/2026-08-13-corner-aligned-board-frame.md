# Spec: corner-aligned canonical board frame

- **Date:** 2026-08-13
- **Status:** ready for implementation
- **Supersedes remediation option 6 of:** `2026-08-12-initial-board-pose-inplane-rotation.md`
- **Related issues:** M-14, M-17, M-19, L-20, L-21

## Problem Statement

An operator calibrating a LiDAR against a camera must hand-set a magic number,
`initial_inplane_rotation_deg`, to `45.0` in each rig's board-detector config. If they leave it at the
shipped default of `0.0`, the board detector silently finds nothing — ICP never converges, no
detections are published, and nothing explains why.

Nothing in the configuration, the logs, or the documentation tells an operator that this number
exists, what it means, or that `45.0` is the only value that works. The two rig presets carry `45.0`;
the template preset carries `0.0`. An operator starting from the template gets a detector that
appears to run and produces no output.

The underlying cause is that the calibration board's software model disagrees with the physical
board by exactly 45°. The model is a square whose local axes run along its **edges**, while every
name in that model — top corner, bottom corner, left and right corners, the three hole positions —
describes a **diamond**, in which the axes run corner to corner. The board is physically hung as a
diamond. The magic number has been bridging that gap.

ICP cannot recover from the error on its own: 45° sits exactly halfway between two of the square's
four 90°-symmetric orientations, and points landing on the board's interior carry no information
about rotation within the board plane. There is no gradient to follow.

## Solution

Redefine the board model's canonical local frame so its in-plane axes run along the **diagonals**,
corner to corner, matching the diamond naming the model already uses and the way the board is
physically mounted. Move the frame's origin from a corner to the plate centre.

After the change, `initial_inplane_rotation_deg` is `0.0` for every supported rig, and the correct
value is the default. The parameter survives as a genuine escape hatch for a future rig whose board is
not mounted diamond-wise, rather than as a correction every operator must discover.

Because the change alters the meaning of every published board pose, and the camera-side solvers
independently reimplement the board's geometry, the work is phased:

- **Phase 1 (this spec)** — the board model, the detector node, all configs, and a guard that makes
  the phase boundary loud rather than silent.
- **Phase 2 (separate spec)** — the two camera-side solver implementations and the saved-detection
  file format, to land when camera validation data exists.

## User Stories

**Operator — configuration and setup**

1. As a calibration operator, I want the shipped board-detector defaults to work on my rig, so that I don't have to discover an undocumented magic number before the detector produces anything.
2. As a calibration operator, I want `initial_inplane_rotation_deg` to be `0.0` in every preset, so that all three configs agree and none of them is quietly wrong.
3. As a calibration operator, I want the documentation for that parameter to say plainly that `0.0` is correct for every supported rig, so that I stop treating it as a tuning dial.
4. As a calibration operator, I want the parameter's documentation to explain what it actually encodes — the board's roll within its own plane relative to corner-up mounting — so that I know when a non-zero value would ever be justified.
5. As a calibration operator starting from the template config, I want the same behaviour as the rig presets, so that copying the template is a safe starting point.
6. As a calibration operator, I want board-detector configs that no longer carry contradictory advice such as "try ±45 or ±90", so that I don't waste a session sweeping a parameter that has one correct value.

**Operator — diagnosis and confidence**

7. As a calibration operator, I want the board drawn in RViz as a diamond outline that traces the actual plate corners, so that I can see at a glance whether the detector has found the board in the right orientation.
8. As a calibration operator, I want the board's axis arrows drawn from the plate centre, so that the displayed frame matches where the pose actually is.
9. As a calibration operator, I want the up axis arrow to point at the physically up-most corner of the board, so that I can confirm the frame convention visually in seconds without reading code.
10. As a calibration operator, I want the board outline to be derived from the same accessors the detector uses, so that the picture cannot drift away from the maths.
11. As a calibration operator, I want the ambiguity warning about which corner is lowest to describe the situation it actually detects, so that I am not told a diamond-mounted board is at risk when it is the well-conditioned case.
12. As a calibration operator, I want the published board pose's uncertainty to be expressed about the plate centre, so that the reported translation and rotation uncertainties are not inflated by a long lever arm from a corner.

**Operator — safety across the phase boundary**

13. As a calibration operator, I want the detector to announce which board-frame convention it is publishing, so that any consumer can tell whether it understands the data.
14. As a calibration operator running camera calibration during the phase gap, I want the solver to refuse to start rather than produce a plausible-looking result, so that I never ship a silently rotated extrinsic.
15. As a calibration operator, I want that refusal to name the reason — the board frame changed and the camera solver has not been updated — so that I know what to do rather than merely that something failed.
16. As a calibration operator doing LiDAR-to-LiDAR calibration, I want that path to keep working throughout, so that the phase boundary does not block the work I am actually doing.

**Maintainer — a model that explains itself**

17. As a maintainer, I want the board model's local coordinates to match its accessor names, so that reading `top_corner` tells me where the top corner is without a mental 45° rotation.
18. As a maintainer, I want the model's frame convention documented at the module level — origin, axis directions, and the paper's coordinate system — so that the next person does not have to reverse-engineer it from arithmetic.
19. As a maintainer, I want the board's three-hole asymmetry to be legible in the coordinates, so that it is obvious this is the only feature capable of resolving the square's 90° ambiguity.
20. As a maintainer, I want the known counter-intuitive naming — the left corner appearing on an observer's right — recorded explicitly, so that nobody "fixes" it and breaks downstream corner ordering.
21. As a maintainer, I want the pose origin at the plate centre, so that it matches the cluster detector's convention and the two can eventually be unified.
22. As a maintainer, I want the post-ICP corner fixup reduced to a rotation that moves no physical point, so that its purpose is obvious rather than entangled with an origin relocation.

**Maintainer — trustworthy tests**

23. As a maintainer, I want the board model's geometric contract expressed as real tests rather than assertions that are compiled out of every build, so that the contract is actually enforced.
24. As a maintainer, I want those tests to check coordinates against the model's own axes, so that they can detect a convention error rather than being blind to rotation.
25. As a maintainer, I want the geometry tests run under several randomised board poses, so that they cannot pass by accident on an identity pose.
26. As a maintainer, I want a property test asserting the new frame is an exact re-parameterisation of the old one, so that the migration's central claim is discharged mechanically rather than argued.
27. As a maintainer, I want the boundary projection verified against a brute-force reference over many random points, so that sign errors, wrong vertex snapping, and quadrant-folding bugs are caught.
28. As a maintainer, I want the marker-corner test to assert physical world positions rather than frame-relative ones, so that it must pass unchanged across the frame change and cannot be re-baselined into false agreement.
29. As a maintainer, I want the marker golden keyed by marker identity, so that it pins which physical marker sits where — the binding whose corruption produces a silent quarter-turn.
30. As a maintainer, I want a test that actually calls the marker-layout routine, so that we stop shipping a test that recomputes its own expectations and verifies nothing.
31. As a maintainer, I want the board model's tests to exercise the same code path the detector runs, so that a passing suite means something about production.
32. As a maintainer, I want the ICP test fixtures to generate points that lie on the board, so that they stop silently feeding half their points off the plate.

**Maintainer — the change itself**

33. As a maintainer, I want the boundary test rewritten honestly for a diamond rather than hidden behind a rotation applied internally, so that the model has one convention rather than a public one and a private one.
34. As a maintainer, I want the duplicated correspondence routine collapsed to a single implementation, so that this rewrite lands once rather than twice.
35. As a maintainer, I want the marker paper's placement stated as an explicit measured offset in configuration, so that both the Rust and the future Python implementations read the same number instead of deriving it independently.
36. As a maintainer, I want the change sequenced so that every step before the semantic flip is a provable no-op, so that the flip is isolated and reviewable.
37. As a maintainer, I want the deferred camera-side work recorded in the issue tracker, so that it is not lost between phases.
38. As a maintainer, I want an accessor whose returned rotation would silently disagree with the physical marker paper removed rather than shipped, so that no future caller adopts a wrong-by-45° API.

## Implementation Decisions

### The canonical frame

With `W` the board width, `R = W/√2` the half-diagonal, `s` the hole centre shift, and `d = s√2`:

- **Origin** — the plate centre, which becomes the pose translation.
- **+Z** — the board normal, pointing toward the sensor. **Unchanged.**
- **+Y** — from the centre toward the top corner.
- **+X** — `Y × Z`, which is from the centre toward the left corner.

| accessor | new local (x, y) |
|---|---|
| board centre | (0, 0) |
| top corner | (0, +R) |
| bottom corner | (0, −R) |
| left corner | (+R, 0) |
| right corner | (−R, 0) |
| left hole centre | (+d, 0) |
| right hole centre | (−d, 0) |
| top hole centre | (0, +d) |

Right-handedness holds: with `u` the up-diagonal and `v` the perpendicular in-plane direction,
`Y × Z = u × (u×v) = −v`, and `X × Y = (−v) × u = u × v = Z`.

**Z deliberately remains the normal.** Full REP-103 alignment with the cluster detector — where X is
the normal — is a separate, later change. Adopting it now would silently break the quality-metric
module and the detection publisher, both of which read the third rotation column as the normal.

### Why the magic number becomes zero

The two conventions are related by an exact conjugation: the new rotation is the old rotation
composed with a −45° in-plane rotation, and the new translation is the old board centre. The
detector's initial-pose construction already builds a base rotation whose up axis is the sensor's up
direction projected into the board plane; the shipped presets then post-multiply it by +45°. Under the
conjugation those cancel exactly, leaving the base rotation untouched.

**Consequence: the detector's rotation construction does not change at all.** Only the model's
interpretation of local coordinates does. Had the frame been defined with +X toward the top corner
instead of +Y, the required value would have been ∓90 rather than 0.

### The boundary test

In the old frame the board is an axis-aligned box in local coordinates, so membership factorises per
coordinate and the nearest point is found by clamping each coordinate independently. Neither property
survives the rotation: in the new frame the square is a diamond, `|x| + |y| ≤ R`.

The existing "is the point outside" test compares the clamped position against the original for exact
float equality. That must go — there is no componentwise operation whose fixed points are a diamond,
and an exact float comparison used as a geometric predicate is fragile regardless.

The projection is replaced with a true L¹-ball projection. This snippet encodes the decision more
precisely than prose:

```
outside  ⟺  |x| + |y| > R

project(x, y, R):
    a, b = |x|, |y|
    if a + b <= R: return (x, y)
    t = (a + b - R) / 2                  # perpendicular foot, folded to the first quadrant
    pa, pb = a - t, b - t
    if pa < 0: pa, pb = 0, R             # foot left the segment: snap to a vertex
    elif pb < 0: pa, pb = R, 0           # at most one can go negative, since pa + pb == R > 0
    return (copysign(pa, x), copysign(pb, y))
```

`copysign` rather than `signum`, because `signum(0.0)` is `1.0`.

Projection onto the nearest point of the plate is a metric operation on an unchanged physical set, so
this returns the identical world point as the old implementation — not an analogue. In-plane
observability is unchanged in kind: interior points contribute nothing, edge points constrain one
degree of freedom, vertex-snapped points constrain two.

### Modules modified

- **Board configuration module** — the canonical frame, all eight geometry accessors, the boundary
  projection, a raw-meters point primitive, and a new public inverse mapping a world point back to
  local plane coordinates. The marker accessors are routed through a new paper-coordinate adapter so
  the marker layout's own arithmetic is untouched. The marker-pose accessor is **deleted**: its
  returned rotation would now differ by 45° from the physical paper, and it has no callers.
- **Board detector node** — the initial pose's translation collapses to the plate centroid, which makes
  its board-width parameter unused and therefore removable, giving a compiler-enforced migration. The
  post-ICP fixup becomes rotation-only. The RViz board outline changes from a cube primitive to a line
  strip through the four corner accessors, so it cannot drift from the model. A dead marker-drawing
  routine is removed. The node publishes the frame-convention tag.
- **ArUco pattern configuration** — gains the marker paper's centre offset relative to the plate
  centre, as an explicit measurement. Both languages read this one value rather than deriving the
  paper's placement from the board width independently.
- **Board detector presets** — both rig presets set the in-plane rotation to zero.

### The correspondence routine's duplication

The routine currently exists twice, textually identical, split by a feature flag — and each copy
contains both inner dispatch arms, making the outer split redundant. Both consumers enable the
feature, so the module's own tests exercise the copy production never runs.

It collapses to one shared per-point body plus two thin wrappers differing only in their iterator.
Bound lists and the return type are preserved verbatim so no caller changes. This must land **before**
the frame change, so the rewrite happens once against a tested body.

### The frame-convention tag

The detector publishes a convention identifier on a latched (transient-local) topic, so late-joining
subscribers receive it. Phase 2's camera solvers subscribe and fail at startup on mismatch, naming the
reason.

A latched topic rather than the alternatives: detections travel on a stock third-party message type
with no free field, so the tag cannot ride along without a wrapper message and an interface-package
regeneration; and a node parameter would couple consumers to a detector node name that the launch
system generates per sensor-marker pair. **Absence of the tag must be treated as failure, not as
consent** — a solver that starts before any detector must not interpret silence as agreement.

The same identifier becomes the tag in Phase 2's saved-file format bump, so this is not throwaway work.

### Sequencing

Each step before the flip is a provable no-op, gated on the previously-written tests still passing:

1. Land the world-coordinate marker golden **on the existing convention**.
2. Collapse the duplicated correspondence routine.
3. Introduce the paper-coordinate adapter as an identity, and add the paper offset to configuration.
4. **Flip the frame.** Gate: the new suite passes *and* the golden from step 1 passes unchanged.
5. Detector node changes.
6. Frame tag.
7. Presets to zero, and the parameter's documentation rewritten.
8. Tolerance constants split and tightened — isolated, so a newly-surfaced violation is attributable.
9. Remove the dead bbox parser and correct the two configs carrying an obsolete quaternion ordering.
10. ICP fixture generation made diamond-aware.
11. Documentation, and the Phase 2 issue.

### Related defects folded in

- The ambiguity warning's explanatory comment states the opposite of the truth: it names a
  diamond-mounted board as the marginal case, but a diamond's corner heights are well separated and the
  warning has never fired for one. The ill-conditioned case is an axis-aligned board, which an earlier
  stage already rejects.
- A dead configuration parser reads quaternions in the opposite component order from the live path,
  and two configs carry values in that obsolete order. Invisible today because the rotation in question
  maps a symmetric crop box onto itself.

## Testing Decisions

### What makes a good test here

Tests must assert **externally observable geometry** — where points are in the world, whether a
projection is genuinely the nearest point, whether the published frame's axes point where they claim.
They must not assert intermediate representations or local coordinates in isolation, because those are
exactly what this change redefines.

Two properties matter more than usual:

- **Convention sensitivity.** The existing geometric assertions are all world-frame distances and dot
  products, which are rotation-invariant and therefore blind to precisely this class of error. New
  tests must dot each accessor against the model's *own* axes.
- **Survivability.** Any test that gets re-baselined at the moment of the flip cannot verify the flip.
  The marker golden must be expressed in world coordinates at a stated physical mounting, so the same
  file must pass before and after.

Tests must also run under several randomised poses. Identity-pose-only tests pass by accident.

### Seams

**Primary seam: the board configuration module's public geometry API.** The frame convention is
entirely observable here — accessors, the correspondence routine, and the marker layout. Everything
downstream either consumes this (the ICP iterator, the detector node, the quality metric) or
duplicates it (the camera solvers). Testing here rather than lower avoids new seams; testing here
rather than higher avoids requiring a ROS runtime and recorded data for what is pure geometry.

**Secondary seam, unavoidable: the marker-corner golden file.** This is a cross-language contract
between the Rust implementation and the two Python reimplementations, so it cannot be exercised
in-process. A checked-in fixture is the seam; the Rust side asserts against it in Phase 1, the Python
side in Phase 2.

**Explicitly not new seams.** The ICP iterator and the detector node both inherit the convention from
the primary seam and get no new test surface. Their existing tests remain as they are.

### Modules tested

- **Board configuration module** — the bulk of the work. Frame pinning against the model's own axes;
  round-trip between local coordinates and world points including negative coordinates; origin
  identity; diagonal and edge lengths; a randomised comparison of the boundary projection against a
  brute-force nearest-point reference; and a property test asserting the new frame is an exact
  re-parameterisation of the old, implemented by keeping a small reference version of the previous
  projection inside the test module. That last test discharges the migration's central claim
  mechanically and is worth keeping permanently.
- **Marker layout** — the world-coordinate golden, keyed by marker identity. This replaces an existing
  test that never calls the routine it is named for.
- **ICP integration fixtures** — point generation restricted to the plate, plus at least one assertion
  stronger than "the loop terminated": from a perturbed seed, the converged pose should map the corner
  accessors onto the true corners within tolerance.

### Prior art

- The board configuration module's existing inline test module is the established seam for the
  correspondence routine; extend it rather than adding a new test target.
- The cluster detector's golden-parity harness is the established pattern for fixture-backed tests,
  including how tolerances and known mismatches are recorded alongside the fixtures.
- The detector node's in-binary unit test module is the established pattern for testing node-level
  helpers without a ROS runtime.
- The advanced solver's pytest directory is the established Python test location and is already wired
  into the project test target; Phase 2's parity test belongs there.

### Verification beyond unit tests

End-to-end verification runs against the two-LiDAR recordings, which carry both a spinning and a
solid-state LiDAR and on which both presets currently work. Four gates: converged plate corner world
positions unchanged within ICP tolerance; the published pose differing from the previous one by
exactly the known conjugation; ICP loss at zero no worse than the previous loss at forty-five; and a
visual check that the board renders as a diamond with the up axis arrow pointing at the up-most
corner.

**Not verified in Phase 1:** anything camera-side. The available recordings contain no camera stream,
so the marker layout is covered only at unit level and no end-to-end camera extrinsic is measured. The
crop-box detection path is likewise unexercised.

## Out of Scope

- **The camera-side implementation.** Both solver reimplementations of the marker layout, their frame
  tag checks, and the saved-detection format version bump are Phase 2, tracked separately.
- **Full REP-103 alignment** — making X the normal to match the cluster detector exactly. After this
  change it reduces to a column permutation plus one sign flip, which is the main argument for doing
  this work now. It additionally requires changes to the quality metric, the detection publisher, the
  LiDAR-to-LiDAR same-face handling, and another format bump.
- **Feeding the cluster detector's already-correct diamond pose directly into ICP.** The cluster
  detector computes a correct corner-aligned pose and discards it; consuming it would remove the need
  for the sensor up-axis parameter entirely. Blocked on REP-103 alignment.
- **Eliminating the duplicated plane fit.** The cluster detector fits a plane and discards it; the node
  refits the same points. Worth removing, unrelated to the frame.
- **Renaming the mirrored left/right accessors.** The new frame makes the counter-intuitive naming
  glaring, but renaming would ripple into downstream corner ordering. Documented, deliberately deferred.
- **Verifying the crop-box detection path.** The template preset's existing zero becomes correct as a
  side effect, which should fix that path's seed, but it is not run here and is not measured. The
  tracking issue stays open, marked fixed-but-unverified.
- **Enabling debug assertions in the release-derived build profile.** Tracked separately; the tests
  added here are strictly better than the assertions they replace.

## Further Notes

**The evidence base.** Every board in this repository is diamond-mounted. Stance — the normalised
maximum of the two diagonals' alignment with the up axis, approximately 1.0 for a corner-standing
board and 0.71 for an edge-aligned panel — was computed across twenty-five golden fixtures spanning all
five sample datasets and falls in 0.9986–1.0000, with independent confirmation from pre-gate overlay
renders for both recorded rigs. The forty-five degrees is a convention bug, not a per-rig mounting
parameter.

**The empirical anchor and its one residual risk.** Detection fails at zero and works at forty-five;
this is behavioural, not a matter of visual judgement. The proof that the new value is zero consumes
that positive sign. A negative forty-five would produce a geometrically identical diamond with
different corner labelling — the silent quarter-turn failure mode — and would mean the up axis should
point at the bottom corner instead. The visual gate distinguishes the two in seconds and must not be
skipped.

**Why the phase gap needs a guard rather than a note.** The published pose is the transform from board
coordinates to sensor coordinates, and the camera solvers supply board-local marker coordinates to it.
The convention therefore appears on both sides of that product. Changing only one side produces an
in-plane forty-five degree error — undetectable, because the symmetric two-by-two marker grid still
solves cleanly with low reprojection error — plus an origin shift of roughly seven hundred millimetres,
which probably would be caught. Half the error is silent, which is why documentation alone is
insufficient.

**On the assertions being replaced.** The board configuration module carries fifty-one debug
assertions intended as its geometry contract. They are compiled out of both sanctioned build
commands, and every one is rotation-invariant, so even executing they would hold identically under any
in-plane relabelling. That is exactly why the forty-five degree mismatch was invisible to the
mechanism meant to guard against it. Their replacement with convention-sensitive tests is a
substantive part of this work, not incidental cleanup.
