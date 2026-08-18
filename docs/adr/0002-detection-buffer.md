# 0002. The detection buffer owns every estimate derived from its captures

- **Date:** 2026-08-18
- **Status:** accepted
- **Implementation:** complete (2026-08-18)
- **Review that produced it:** [2026-08-15 architecture review](./2026-08-15-architecture-review.md), candidate 2
- **Related plan:**
  [LiDAR-to-camera solver diamond-frame Phase 2](../superpowers/specs/2026-08-14-lidar-to-camera-solver-diamond-frame.md)

## Context

The manual `lidar_to_camera_solver` has one central domain concept but no module for it. Its
detection buffer is a bare list guarded by a node-wide lock, while ten ROS service callbacks mutate
the list, decide when to solve, classify board placements, format status, preserve detections, and
manage the transform derived from those detections. The numerical implementation is also embedded in
the node: correspondence construction, covariance weighting, PnP, refinement, diversity checks, and
quality assessment all read and write node fields directly.

This shape has produced concrete invariant failures:

- `load_detections_callback()` can call `_solve_from_buffer()` while holding a non-reentrant lock,
  and `_solve_from_buffer()` attempts to acquire the same lock;
- removal or replacement can leave a transform publishing after a capture used to derive it has
  disappeared;
- an empty buffer can leave solved/current pose fields queryable;
- a failed or not-yet-ready solve can retain quality derived from older contents;
- `_assess_quality()` computes the operator-facing quality verdict, after which the success path
  overwrites it with `"Calibration successful"`;
- tests cannot construct the domain concept, so they bind node methods onto fake objects or use
  `LidarToCameraSolver.__new__()` to bypass ROS construction.

The [project glossary](../../CONTEXT.md) distinguishes a **Capture** from a **Board Placement**. A
Capture is one deliberately retained synchronized Detection Pair. Several Captures can observe one
Board Placement: those repeated frames may average down frame noise, but they add no new geometry.
The buffer must preserve that distinction everywhere.

## Decision

`lidar_to_camera_solver` gains one deep in-process module,
`lidar_to_camera_solver.detection_buffer`, whose main type is `DetectionBuffer`.

The Detection Buffer owns:

- the ordered Captures;
- Capture validation and preparation;
- distinct-Board-Placement classification;
- 3D-to-2D correspondence construction;
- board-pose covariance propagation and correspondence weighting;
- solve readiness and optional pose-diversity refusal;
- SQPnP and its weighted or unweighted LM refinement;
- quality assessment;
- the Solved Estimate and all status derived from the exact current Captures.

It imports no `rclpy`, creates no ROS graph entities, logs nothing, opens no files, and publishes
nothing. It may use generated ROS message types at its interface. All operator text, ROS response
population, timestamps, frame labels, and publication remain adapter work in the node.

The module lives inside `ros/lidar_to_camera_solver/lidar_to_camera_solver/`. It is not a shared
package. `lidar_to_lidar_solver` is explicitly out of scope and remains untouched.

### External interface

The intended interface is small. Exact field spelling may change during implementation, but the
operations and their semantics are fixed by this decision:

```python
@dataclass(frozen=True)
class DetectionPair:
    aruco: Detection2DArray
    board: Detection3DArray


class DetectionBuffer:
    def __init__(
        self,
        *,
        camera_matrix: np.ndarray,
        marker_corners_by_id: Mapping[
            int,
            Sequence[tuple[float, float, float]],
        ],
        min_frames_required: int,
        min_normal_spread_deg: float,
        min_depth_range_m: float,
        enforce_pose_diversity: bool,
    ) -> None: ...

    def capture(self, pair: DetectionPair) -> BufferUpdate: ...
    def restore(
        self,
        pairs: Iterable[DetectionPair],
        *,
        append: bool,
    ) -> BufferUpdate: ...
    def remove(self, index: int) -> BufferUpdate: ...
    def clear(self) -> BufferUpdate: ...
    def snapshot(self) -> BufferSnapshot: ...
```

There is no public `solve()`, placement-count function, pose-weight function, or refinement
function. Accepted mutations automatically solve, refuse, fail, or report that they are not ready.
Tests cross the same interface as callers.

`DetectionPair` wraps the existing `Detection2DArray` and `Detection3DArray`. The module validates
and normalises them immediately into owned internal values. It retains detached copies of the wire
messages only because version-4 Detection Archives preserve those messages. A returned snapshot
must not expose mutable internal state.

Captures keep their ordered zero-based indices. The existing ROS list/remove contract is index-based;
there is no persistent Capture ID.

### Mutation and solve are separate outcomes

A `BufferUpdate` reports two independent facts:

1. whether the requested mutation was accepted; and
2. the solve state of the resulting buffer.

Adding the first valid Capture is a successful mutation even when the buffer needs more frames. A
failed solve does not rewrite a successful mutation into a failed capture, and a rejected mutation
does not disturb the previous snapshot.

The solve outcome is a tagged value with these meanings:

- **Empty** — no Captures exist;
- **NotReady** — the Captures are valid but fewer than `min_frames_required` exist;
- **Refused** — enough Captures exist, but an enabled policy deliberately rejected publication,
  currently only enforced pose diversity;
- **Failed** — preparation or the numerical solve failed for the accepted current contents;
- **Solved** — a numerical estimate exists for the current contents, with its Quality Verdict.

`Solved` does not mean trustworthy. A numerically solved but geometrically degenerate calibration is
still `Solved`, with `quality.is_degenerate == True`. Only `Refused` means configured policy blocked
an estimate.

Expected input and numerical errors become typed mutation/solve outcomes. Programming errors are not
swallowed as domain failures.

### Current-buffer invariant

A Solved Estimate is active only when it was derived from the exact current buffer revision. Each
accepted mutation increments the revision and atomically replaces all derived state. If the new
contents are Empty, NotReady, Refused, or Failed, the buffer exposes no active Solved Estimate.

An older estimate may be mentioned in diagnostics, but it is never eligible for publication and is
not the current solution. The node must stop publishing and clear its Adjusted Transform whenever a
successful mutation leaves no active Solved Estimate.

Rejected mutations do not increment the revision and leave the current Captures, Solved Estimate,
quality, and Adjusted Transform unchanged.

### Capture admission

`capture()` rejects a Detection Pair before mutation unless it can prepare at least four valid
3D-to-2D correspondences. Rejection includes:

- absent board detection or absent board-pose result;
- invalid or non-finite board pose/quaternion;
- absent real ArUco corners;
- no marker ID present in configured marker geometry;
- fewer than four usable correspondences after filtering.

This is structural admission, not a quality gate. A valid duplicate or low-diversity Capture is
accepted.

### Captures and Board Placements

An accepted Capture is retained even when it belongs to an existing Board Placement. Its update
classifies it as a new or duplicate placement so the adapter can tell the operator to move the
board. Duplicate Captures contribute to the solve exactly as they do today; changing their weighting
or replacing them with per-placement representatives is a separate, measured estimator change.

Solve readiness remains based on `min_frames_required`, preserving current behaviour. Distinct
Board Placements drive quality and diversity. Pose diversity refuses a solve only when
`enforce_pose_diversity` is enabled. This ADR does not silently change the gate to require N distinct
placements.

Placement groups are recomputed from the complete candidate buffer after every accepted mutation.
Removing a representative Capture can therefore regroup remaining Captures correctly. The
implementation delegates placement semantics to `lctk_quality` rather than defining another set of
tolerances.

### Mutation semantics

`capture(pair)` validates one pair, appends it, classifies its placement, and performs one solve
attempt.

`restore(pairs, append=...)` is atomic. It validates and prepares every incoming Capture before
changing state. If one is malformed, nothing changes. On success it appends to or replaces the
current Captures and performs one solve attempt, not one solve per restored Capture.

`remove(index)` rejects an invalid zero-based index without changing state. A valid removal
reclassifies placements and performs one solve attempt. Removing the last Capture yields Empty and
clears every derived value.

`clear()` yields Empty, clears all Captures and derived state, and increments the revision once when
state changed. Clearing an already-empty buffer is an accepted no-op and leaves its revision
unchanged. The ROS clear adapter separately calls `DetectionPairSource.discard_cached_pair()`;
pair-source cache ownership does not move into this module.

`snapshot()` is side-effect free. It returns detached immutable state sufficient for status, list,
publication, and archive encoding.

### Synchronisation and performance

The module owns one internal reentrant lock. Every mutation and its solve attempt are synchronous
and atomic. The node owns no lock for buffer state. `capture()`, `restore()`, and `remove()` may block
while PnP and quality assessment run; that is part of the interface.

The current node uses a single-threaded executor, so this contract is conservative rather than a new
source of concurrency. A future latest-wins worker may replace the synchronous implementation without
changing the interface, but no worker and no continuous acquisition policy are part of this ADR.

### Camera model lifecycle

The ROS node constructs the Detection Buffer lazily from the first `CameraInfo`. Calls made before
that retain the existing `"No camera info available"` refusal at the adapter.

Repeated CameraInfo messages with element-for-element identical intrinsic matrices do nothing. If
any intrinsic-matrix element changes, a new calibration session starts: the node replaces/clears the
Detection Buffer, stops publication, and clears the Adjusted Transform. CameraInfo is static
configuration rather than a noisy measurement, so no numerical tolerance is applied. Captures made
under different camera models must never be mixed.

The camera matrix is float64. Distortion remains zero because the ArUco observations on the wire are
already rectified; changing that convention is outside this decision.

### Manual adjustment

Manual transform adjustment is not Detection Buffer state. The buffer owns the Solved Estimate;
the node's adjustment state owns the Adjusted Transform anchored to it.

After a successful buffer mutation:

- `Solved` rebases the Adjusted Transform to the new Solved Estimate with zero manual delta;
- Empty, NotReady, Refused, or Failed clears the Adjusted Transform and stops publication.

`reset_transform` returns to the current Solved Estimate without re-solving unchanged Captures.
Rejected buffer mutations leave the existing adjustment untouched because the buffer revision did
not change.

### Detection Archive adapter

`detection_format.py` is extended to encode and decode the complete version-4 Detection Archive:
buffer snapshot, Quality Verdict, and optional Adjusted Transform. The format version remains 4.
`DetectionBuffer` knows neither JSON nor file paths; service callbacks own file opening and error
rendering.

Load semantics are:

- replacement load: restore Captures, solve them once, then restore the archived Adjusted Transform
  only when the restored buffer has a current Solved Estimate;
- append load: ignore the archived Adjusted Transform, solve the combined Captures once, and use
  zero adjustment from the combined Solved Estimate;
- replacement that cannot solve: do not publish the archived Adjusted Transform, because no current
  Solved Estimate anchors it;
- malformed archive or Capture: reject the entire load without changing live state.

These rules eliminate the current state in which a loaded current transform can exist while no
solved baseline exists, and prevent an appended archive's transform from claiming to describe a
different combined buffer.

### ROS adapter responsibilities

The node remains responsible for:

- `DetectionPairSource` and its freshness/refusal diagnosis;
- CameraInfo acquisition and camera-model change detection;
- ROS service request/response types;
- translating typed buffer outcomes into operator messages and log levels;
- the Adjusted Transform and manual delta operations;
- `TransformStamped` construction, frame labels, clock stamps, publication, and markers;
- file I/O around `detection_format.py`;
- discarding the pair source's cached pair when clearing.

Service paths and `.srv` definitions do not change. Existing operator behaviour is preserved unless
this ADR explicitly strengthens an invariant.

### Test surface

Tests construct `DetectionBuffer` directly and cross only its public interface. The existing
reach-through patterns are deleted rather than layered underneath interface tests.

Required behavioural coverage:

- twenty noisy Captures of a static board remain one Board Placement;
- moving or tilting the board creates a placement; in-plane spin does not;
- a duplicate is retained and classified without increasing placement count;
- one accepted Capture can return mutation success plus NotReady;
- structurally unusable Capture rejection leaves revision and state unchanged;
- enough valid Captures produce Solved and preserve the Quality Verdict;
- degenerate quality remains Solved unless diversity enforcement refuses it;
- covariance weighting improves the public solved result in the existing synthetic bad-pose case;
- invalid removal is atomic;
- valid removal recomputes or invalidates the estimate and never exposes the old one;
- clear removes Captures, estimate, quality, and correspondence count;
- restore append/replace validates atomically and solves exactly once;
- a structurally rejected restore leaves the previous snapshot unchanged;
- archive replacement/append follows the Adjusted Transform rules above;
- changed camera intrinsics clear node-level calibration state;
- no test binds unbound node methods, fabricates fake `self`, uses node `__new__()`, or calls private
  PnP/refinement helpers.

## Consequences

**Locality.** Capture mutation, placement rules, solve state, numerical estimation, and quality move
behind one interface. The next fix to those invariants lands once instead of being coordinated across
service callbacks.

**Leverage.** Manual service callbacks become adapters: obtain/parse input, call one buffer method,
render the returned update. Tests gain the same leverage through the same seam.

**Stronger state.** A transform cannot remain active after its source Captures change. Mutation
success no longer gets confused with solve success, and a numerical solve no longer gets confused
with a trustworthy calibration.

**Synchronous cost.** Mutations block during solving. This preserves current manual-mode behaviour
and keeps asynchronous policy out of the first extraction.

**More typed values.** The module introduces snapshots and tagged outcomes instead of sharing node
fields and status strings. This is deliberate interface size spent to remove ordering constraints
from callers.

**Ruled out.** A list-only wrapper: it would leave solve, quality, and invalidation scattered and
would fail the deletion test. A generic cross-solver calibration package: LiDAR-to-LiDAR uses a
different estimator and is future work. A public estimator seam: only one implementation exists.

## Explicitly out of scope

- `lidar_to_lidar_solver` changes;
- `solver_mode` and continuous/latest-pair policy wiring;
- Stage 2 of the diamond-frame Phase 2 plan;
- asynchronous/latest-wins solving;
- changing duplicate-Capture weighting;
- changing placement tolerances;
- requiring N distinct placements before every solve;
- new robust estimation or outlier rejection;
- detection archive version 5;
- ROS service-definition changes.

The next stage may use this module to implement `solver_mode: continuous`, but this ADR neither
implements nor activates that policy.
