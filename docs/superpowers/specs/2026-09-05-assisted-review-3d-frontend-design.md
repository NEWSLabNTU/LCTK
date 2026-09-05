# Assisted review page — 3D frontend, and the evidence that feeds it

- **Date:** 2026-09-05
- **Status:** Approved, not yet implemented
- **Area:** `lidar_to_camera_solver`, `lidar_board_detector`, `lctk_launch`
- **Supersedes nothing.** `continuous` and `manual` are untouched; this is the `assisted`
  review surface only.
- **Design precursor:** `docs/superpowers/specs/2026-08-31-assisted-extrinsic-solver-design.md`

## The problem

The assisted review page exists so an operator can see *why* a capture was bad and decide
which pairs to drop. Two things stop it doing that job.

**It shows a picture that does not belong to the capture.** Reported from a live session on
`solid600-handheld-vlp`: the drawn ArUco outlines do not sit on the ArUco markers.

**It shows one picture at a time.** The decision the operator actually makes — "have I
covered enough distinct board placements, and which of these is the outlier?" — is a
question about the *set* of captures in space. A vertical list of thumbnails cannot answer
it. `lctk_quality.diversity` computes the answer numerically, and the page renders it as a
line of text.

## Two defects, verified

Both are real and both are in the shipped code, but they do not both fire on every session —
see "Which sessions each defect reaches" below before reproducing either. Neither affects
the calibration: the corners on the wire are correct and the archive stores corners, not
pictures. Only the review image lies — which still matters, because the review image is what
drives the drop decisions.

### Defect 1 — the frame is never matched to the pair

`main.py:524`, in the image callback:

```python
self._preview_store.set_latest(frame)     # no stamp, one slot
```

`main.py:780`, at capture:

```python
self._preview_store.capture(
    pair_id, corners=aruco_corner_quads(aruco), reprojected=None
)
```

`PreviewStore.capture` reads `self._latest` — whatever arrived most recently. The corners
come from `aruco`, which *is* synchronized against the board detection by
`DetectionPairSource`. The image is synchronized against nothing.

The error has a direction. The ArUco detection only reaches the solver after
`aruco_locator_node` has processed that frame, while raw images keep arriving on the same
topic. So `_latest` is normally **several frames newer** than the frame the corners were
measured in, and the gap is unbounded — if the detector lags, it grows.

Observable signature: offset vanishes when the board is still, grows with board speed, and
points along the direction of motion.

### Defect 2 — the corners are in a different frame than the image

`rust/aruco-detector/src/multi_aruco.rs:316` maps refined corners into the **rectified**
frame with `undistortPoints`, so downstream PnP pairs them with `K` and zero distortion.
That is correct and deliberate.

`main.py:477` subscribes the solver to `solver.camera_topic`, which is whatever `image_topic`
the session names — for `sample3-hollow-velodyne` that is the **raw** image. `preview.
annotate()` then draws rectified corners onto raw, distorted pixels. Nothing in the solver
checks which of the two it was handed.

`aruco_locator_node` gets this right for its own overlay. `ros/aruco_locator_node/src/main.
rs:489`:

> *The full-frame rectification survives only to draw the debug overlay in the same frame
> the corners are reported in*

The solver never applied that reasoning.

Observable signature: offset is near zero at the principal point and grows radially toward
the image edges — where markers sit when the board is off-centre. Present even when the
board is perfectly still.

### Telling them apart

| | Defect 1 | Defect 2 |
|---|---|---|
| Board held still | offset gone | offset remains |
| Worst where | anywhere | image edges; exactly zero at the principal point |
| Direction | along board motion | radial |

### Which sessions each defect reaches

Defect 2 fires only where the solver is handed a **distorted** stream. Several sessions feed
a pre-rectified one, on which `undistortPoints` is a no-op and drawing the published corners
straight onto the frame is correct. This was checked against the actual `camera_info`, not
inferred from topic names.

| Session | Stream | Distortion | Defect 1 | Defect 2 |
|---|---|---|---|---|
| `sample3-hollow-velodyne` (`just demo`) | `image_raw` | k1 = 0.1008, k2 = −0.3424, k3 = 0.4139 | yes | **yes** |
| `solid600-handheld-vlp` | ZED `rect/image` | all zeros | yes | no |
| `solid600-handheld-seyond` | ZED `rect/image` | all zeros | yes | no |
| `vlp32-zed-hollow` | ZED `rect/image` | all zeros | yes | no |
| `seyond-left`, `seyond-right` | `image_raw` | live rig, not verified here | yes | likely |

Measured on the demo session's own `camera_info.yaml` at 1920×1080, mapping a 9×9 grid of raw
pixels through `undistortPoints`:

```
displacement px: median 7.0  mean 15.7  max 87.5
  image centre  960,540  -> 960.0,540.0    off by  0.0 px
  mid-edge     1919,540  -> 1880.1,544.8   off by 39.2 px
  corner       1919,1079 -> 1849.1,1047.8  off by 76.5 px
```

Zero at the centre, 76 px at the corner. That gradient is why the reported symptom was
"*some* of the annotations" rather than all of them.

**Consequence for verification.** The originally reported session,
`solid600-handheld-vlp`, is a ZED with zero distortion, so what was observed there is
defect 1 alone. **Defect 2 cannot be reproduced or verified on any ZED session in this
tree.** Use `sample3-hollow-velodyne` for that, and see the testing section — the golden
unit test is what actually pins it, because a hand-held recording cannot isolate one defect
from the other.

## What this changes

1. A single module owns "find the message that goes with this pair", and both the camera
   frame and the LiDAR cloud go through it.
2. The review page becomes a 3D scene of every captured placement, with the ArUco preview
   as the per-pair detail rather than the primary view.
3. `plane_inliers` becomes a first-class topic so the scene has real sensor points in it.
4. The three stillness gates become editable from the page, under a loopback rule.

---

## Module: `StampedRing` (internal seam)

Recent messages keyed by stamp. Two adapters use it — frames and clouds — so this is a real
seam rather than a hypothetical one.

```python
put(stamp: float, value: T) -> None
match(stamp: float) -> T | None
```

Behind those two methods:

- bounded by **time** (`review_evidence_seconds`), with a fixed count cap as a memory
  backstop rather than a second tunable — a stalled clock must not be able to grow the ring
  without limit
- **exact stamp equality first**, then nearest within tolerance, then `None`
- thread-safe; `put` is called from a subscription callback and `match` from the pair
  callback
- a non-monotonic stamp clears the ring rather than corrupting the ordering, matching the
  rule `StabilityTracker` already applies

The exact-first rule is not an optimisation. Both producers stamp their output from the
source message, so equality is the normal case and the nearest-match path is the exception.

### Why exact equality is available

**Camera.** `ros/aruco_locator_node/src/main.rs:604` copies `msg.header.stamp` from the
source image into the detection header.

**LiDAR.** `ros/lidar_board_detector/src/main.rs:1330` builds `Detection3DArray { header:
msg.header.clone(), ... }`, and `:1992` builds the debug `PointCloud2` with the same
`header` threaded down from the same `msg`. One callback, one source cloud, one stamp.

`vision_msgs/Detection3D` prescribes exactly this in the message file:

> *Source data that generated this classification are not a part of the message. If you
> need to access them, use an exact or approximate time synchronizer in your code, as this
> message's header should match the header of the source data.*

Embedding the cloud inside the detection was considered and is not available: Humble's
`Detection3D` has no `source_cloud` field.

## Module: `EvidenceStore` (external seam)

Replaces `PreviewStore`, which was shallow — a locked dict whose caller supplied the frame,
and therefore supplied the wrong one.

```python
observe_frame(stamp, height, width, encoding, step, data) -> None
observe_cloud(stamp, points) -> None
observe_intrinsics(camera_matrix, distortion) -> None
capture(pair_id, stamp, corners) -> CaptureEvidence
get(pair_id) -> CaptureEvidence | None
drop(pair_id) -> None
clear() -> None
```

```python
@dataclass(frozen=True)
class CaptureEvidence:
    preview_jpeg: bytes | None
    cloud_xyz: bytes | None          # packed little-endian float32 xyz
    missing: tuple[str, ...]         # e.g. ("camera frame",) — shown on the page
```

Behind those seven methods: two `StampedRing`s, stamp matching and tolerance policy,
**rectification**, annotation, JPEG encode, float32 packing, LRU eviction by `pair_id`, and
all the locking.

**Deletion test.** Delete `EvidenceStore` and `main.py` grows two ring buffers, the matching
rule, the locking, the rectify step, annotate and encode; `review_server.py` grows two
lookup paths. The complexity reappears in two places, so the module earns its keep.

### Refuse rather than lie

If no frame or cloud matches within tolerance, that slot in `CaptureEvidence` is `None` and
its name appears in `missing`. The page renders the pair without it and says which evidence
was unavailable.

A borrowed frame from a neighbouring sweep is worse than no frame, because it looks
plausible. This also preserves the existing rule from `preview.py`: **a preview must never
be able to break a capture.** Every failure path returns falsy; calibration correctness
never depends on a picture being available.

### Rectification

`capture()` undistorts the selected frame with the intrinsics from `observe_intrinsics`
before drawing, putting the pixels in the same frame the corners are reported in. This runs
**once per capture**, not once per frame — a capture happens every few seconds, an image
arrives 30 times a second.

**Skip it when `D` is all zeros**, which is the case on every ZED session in this tree. Not
as an optimisation: `cv2.undistort` under zero distortion is geometrically an identity but
still resamples the image bilinearly, and blurring a review frame for no reason is exactly
the cost that motivated detecting on raw pixels in the first place
(`multi_aruco.rs`, H-08). The zero-distortion branch must therefore be a real branch, not a
call that happens to be harmless.

`EvidenceStore` deliberately does **not** try to infer whether the stream it was handed is
raw or already rectified. It applies the distortion the camera published, which is the only
self-consistent rule: a rectified stream publishes `D = 0` and gets no correction, a raw
stream publishes real coefficients and gets corrected. Guessing from a topic name is how
C-03 (double rectification) happened.

Rejected alternative: subscribe to `aruco_locator_node`'s `image_with_detections`
(`main.rs:341`), which is already rectified, already annotated, and already stamped from the
source image. It is provably aligned, but it requires `debug_overlay_enabled`, costs a
full-frame warp at camera rate rather than one per capture, and **still needs the same
stamped ring** because it arrives asynchronously. It pays more and saves nothing.

Rejected alternative: feed the image into Conflux as a third stream. Architecturally the
cleanest, but `sync_queue_size: 100` of 720p frames is roughly 270 MB, and a missing image
would then block a capture.

### Memory

| Payload | Per item | Ring | Total |
|---|---|---|---|
| Camera frame, half resolution | ~690 KB | 1.0 s | ~21 MB |
| `plane_inliers` cloud | ~3.6 KB | 1.0 s | ~360 KB |

Frames are stored at half resolution and the corners are scaled to match. Full resolution
at 30 fps for one second is ~80 MB, which is not reasonable on the Jetson, and the review
page renders these as thumbnails regardless.

## Interface: `NodeFacade`

Grows from five methods to eight. Each addition is a distinct capability; none is a
pass-through.

```python
state() -> dict                                          # unchanged shape + scene_revision
scene() -> dict                                          # new
preview(pair_id) -> bytes | None                         # unchanged
cloud(pair_id) -> bytes | None                           # new
drop(pair_id) -> tuple[bool, str]                        # unchanged
export_archive(path) -> tuple[bool, str]                 # unchanged
export_autoware(dry_run) -> tuple[bool, str, dict|None]  # unchanged
set_stability_params(values) -> tuple[bool, str]         # new
```

The existing contract holds: no method raises, failures come back as `(False, reason)` so
an operator sees the reason on the page instead of a stack trace in a log.

## The wire

Splitting by *change rate* rather than by topic keeps the poll small without pushing
geometry math into an untested layer.

| Endpoint | Fetched | Carries |
|---|---|---|
| `GET /api/state` | every 500 ms | status, stillness, diversity, per-pair RMS, `scene_revision` |
| `GET /api/scene` | only when `scene_revision` changes | world-space quads for every pair, plus the camera pose |
| `GET /api/pair/<id>/cloud.bin` | once per pair, memoized by id | packed float32 xyz |
| `GET /api/pair/<id>/preview.jpg` | on selection | unchanged |
| `POST /api/pair/<id>/drop` | on action | unchanged |
| `POST /api/params` | on action | the three stability gates |

`scene_revision` increments on any buffer mutation. It is the same signal that already
invalidates a pending Autoware preview, so the invariant is not new.

### Geometry stays in Python

`DetectionBuffer._prepare_pair` already computes exactly what the 3D view needs
(`detection_buffer.py:432`):

```python
world_corners = (rotation @ local_corners.T).T + position
```

`/api/scene` reuses that rather than re-deriving it, which gives a property worth having:
**the 3D view and the solve read the same geometry, so they cannot disagree about where a
marker is.** The plate outline is derived the same way, from the target's plate side.

This also puts the only real math on the server, where the repo has a test runner. The
frontend receives coordinates and draws them.

### One camera pose

The camera frustum and the camera's axis marker are both derived from a single pose in the
scene payload. During prototyping these were two independent literals in the mock renderer
and consequently did not line up. The wire format makes that class of bug unrepresentable:
there is one pose, and the frustum apex is its origin by construction.

---

## Frontend modules

Three deep modules and a thin orchestrator. No build step, no npm, no bundler — a build
step inside a colcon package is a new failure mode with no offsetting benefit.

| Module | Interface | Implementation hides |
|---|---|---|
| `ReviewApi` | `state` `scene` `cloud` `drop` `exportArchive` `autowarePreview` `autowareWrite` `setParams` | fetch, JSON vs binary, cloud memoization, errors normalized to `{ok, detail}` |
| `SceneModel` | `sync(app)` `focus(id)` `frameAll()` `pick(x, y)` | three.js objects, buffer geometry, **disposal**, camera animation, raycasting |
| `Chrome` | `render(app)` | sidebar list, footer gauges, dropdown menu, detail pane, DOM updates |

`sync(app)` takes the whole application state and computes the delta itself — which pairs
are new, which were dropped, which changed colour. Callers never do add/remove bookkeeping.
That is the leverage the interface buys.

`main.js` is then roughly thirty lines: poll, build `app`, call `Chrome.render(app)` and
`SceneModel.sync(app)`.

### One owner for selection

```js
app = { server, scene, selectedId, layers }
```

Lives in `main.js`. Both modules render *from* it; neither mutates it. Two owners of the
selected id would drift, and the drift would show as the sidebar highlighting one pair
while the viewport focuses another.

### Vendored three.js

Pinned, committed under `web/vendor/`, served by the node. The review page is used in
vehicle bays and over forwarded ports; a CDN dependency means a blank viewport whenever
there is no internet, with no error to explain it.

## Layout

Approved as prototype D, kept alongside this document at
`assets/2026-09-05-assisted-review-layout/layout.html`. Open it in a browser directly; it
needs no server.

Its `scene.js` is a **throwaway canvas-2D mock**, not the proposed renderer — it exists so
the layout could be judged against a live viewport rather than a grey rectangle. The real
viewport is three.js, per `SceneModel` above. The mock's fake data is four captured pairs in
the LiDAR frame.

- **Header** — brand left, `☰` dropdown right. Menu holds export actions, view toggles, and
  the advanced parameter panel.
- **Left sidebar** — text-only list of captures; id, RMS in a tinted pill, range and inlier
  count. Toggle rides the sidebar's own right edge and slides to the viewport edge when
  collapsed, so it stays reachable.
- **Main** — 3D viewport holding every captured pair at once. Legend top-right, camera
  actions top-left.
- **Detail** — selecting a pair focuses the camera on it and opens a right-hand pane with
  the ArUco preview and its numbers.
- **Footer** — the four diversity gauges (placements, normal span, depth range, lateral
  span) followed by stillness, solve and sync cells, all in one rail.

The diversity gauges lead because the degenerate-capture failure is the one that inverts
every other metric: a single placement filmed nine times scores ±0.22° / ±9 mm, the most
confident number the metric can produce, for a capture that is worthless. See
`ros/lctk_quality/lctk_quality/placements.py`.

### Viewport controls

| Input | Action |
|---|---|
| Left drag | pan, tracking the cursor 1:1 at any zoom |
| Left click, no movement | select the pair under the cursor |
| Right drag | orbit; horizontal inverted, so dragging right swings the scene right |
| Wheel | dolly |

The canvas suppresses its context menu, otherwise the orbit binding pops it.

---

## The `plane_inliers` change

`debug/plane_inliers` is presently created only when `enable_debug` is set, which also
creates ten other publishers and a per-frame full-cloud publish. The 3D view needs this one
cloud and none of the others.

Change, in `ros/lidar_board_detector/src/main.rs`:

- lift `plane_inliers` out of `Option<BoardDebugPublishers>` so it is always created
- give it RELIABLE QoS rather than `debug_qos`. It becomes one of LCTK's own topics,
  produced and consumed by LCTK on both ends, and the convention for those is RELIABLE
- keep the topic name `debug/plane_inliers`. It appears in existing RViz configurations and
  a rename buys nothing

The other ten debug publishers stay gated behind `enable_debug`, unchanged.

### Things checked while designing this

- Both publish sites — the bbox_free path (`~1450`) and the bbox path (`~1583`) — already
  pass the same `header`. Nothing to change.
- `plane_inliers` publishes after the plane fit, but the pose gate can still reject
  afterwards (`main.rs:1320`), emitting an empty detection array. So a cloud may exist with
  no corresponding pair. This is harmless: lookups are keyed by the stamp of pairs that were
  actually captured, and the ring simply holds a few extra entries.
- Cross-topic arrival order is not guaranteed by DDS. The ring handles it; nothing relies on
  the cloud arriving before the detection, though in practice it does, since both are
  published from the same callback.

## Parameter writes

`POST /api/params` is the review server's first ability to change **node state** while the
graph is running. Today it can write a detections archive and an Autoware YAML, and the
Autoware write is deliberately two requests so the operator sees the diff before confirming.

The server is unauthenticated. It binds `127.0.0.1` unless `review_bind_host` says
otherwise, and logs a warning naming the exposure when it does. That default is what makes
this acceptable, so:

- **exactly three parameters are settable**: `stability_window_s`,
  `stability_max_translation_m`, `stability_max_rotation_deg`. The whitelist is explicit and
  nothing else is reachable through this route, ever.
- **ranges are validated server-side**, not only in the browser. `stability_window_s` must
  be finite and strictly positive, matching the rule `StabilityTracker` already enforces.
- **all parameter writes are refused when `review_bind_host` is not a loopback address**,
  and the page states why the panel is disabled.

The last rule means the feature is unavailable on a rig where the page was deliberately
exposed to a network. That is the intended trade.

A change applies to future captures only. Pairs already in the buffer were admitted under
the gate in force when they were taken, and the page says so next to the Apply button.

---

## Implementation chunks

Ordered so each ships something verifiable on its own, and so the correctness fix lands
first.

| # | Chunk | Ships |
|---|---|---|
| 1 | `StampedRing` + `EvidenceStore`; both defects fixed | the existing page starts telling the truth. No 3D |
| 2 | Rust ungate + RELIABLE QoS; solver subscribes; `cloud.bin` | clouds available and matched |
| 3 | Flask `static_folder`; existing page moved to `web/` verbatim | no behaviour change, pure move |
| 4 | `/api/scene`; `ReviewApi` + `SceneModel`; three.js viewport | 3D viewport |
| 5 | `Chrome` — prototype D's layout | the new page |
| 6 | `POST /api/params`, loopback-gated | live retuning |

Chunk 1 is independently valuable: if everything after it stalled, it would still be worth
landing, because it repairs a page that is currently misleading its operator.

Chunk 3 is deliberately a no-op move. Doing the file relocation and the rewrite in one step
would make a rendering regression indistinguishable from a packaging mistake.

## Testing

Per `CLAUDE.md`, every new suite has an assertion broken deliberately and a non-zero exit
confirmed before it is trusted. `ros/lidar_to_camera_solver/test` is already wired into the
`test` recipe, so no recipe change is needed; the `lidar_board_detector` change is covered by
`cargo nextest`.

**The test that matters.** A synthetic frame with known lens distortion and known marker
corners, asserting that the drawn corners land on the marker. This test fails against
today's code. It is precisely the check whose absence allowed defect 2, and it pins the fix
so the defect cannot return quietly.

It must use **non-zero** distortion, and it must place a marker away from the principal
point. With `D = 0` the whole defect is invisible, and at the image centre it is exactly zero
even at full distortion — either mistake yields a test that passes against the broken code.
Use the demo session's real coefficients, where the error reaches 76 px at the frame corner.

Pair it with the inert case: the same assertion under `D = 0` must also pass, both before and
after the fix. That is what stops a future "fix" from rectifying a stream that was already
rectified, which is the C-03 double-correction bug pointed the other way.

Also:

| Suite | Covers |
|---|---|
| `test_stamped_ring.py` | exact match; nearest within tolerance; miss beyond tolerance; eviction by time; eviction by count; non-monotonic stamp clears |
| `test_evidence_store.py` | the distortion golden above; capture with no matching frame yields `missing`, not a neighbouring frame; cloud packing round-trips; `drop` removes both payloads |
| `test_review_server.py` | the three new routes; `cloud.bin` returns 404 rather than a neighbouring sweep; `POST /api/params` rejects an out-of-whitelist key, an out-of-range value, and a non-loopback bind |
| `test_scene_payload.py` | `scene_revision` increments on mutation; quads match `DetectionBuffer`'s own object points; one camera pose, with the frustum derived from it |

Frontend code is not unit-tested — there is no JS runner in this workspace and adding one is
out of scope. This is the reason geometry lives on the server: the untested layer receives
coordinates and draws them, and holds no math that could be silently wrong.

## What this does not do

- **`continuous` and `manual` are untouched.** `solver_mode` remains the switch.
- **No authentication.** The loopback default and the parameter whitelist are the whole
  security story. Anything more is a separate design.
- **No raw-scene point cloud.** Only `plane_inliers` — the points the detector actually fit
  the board to. Showing the full sweep would make "the detector fit the wrong cluster"
  diagnosable, at roughly 400× the bytes, and is deferred.
- **`just smoke` still does not exercise assisted mode.** Unchanged by this work, and worth
  its own issue.

## Open items

- **`review_max_previews` versus a time bound.** The rings need a time bound; the per-pair
  evidence cache needs a count bound. Proposal: keep `review_max_previews` for the cache and
  add one `review_evidence_seconds` for both rings. Two knobs rather than four, and the time
  bound is the one that governs whether a match can be found at all.
- **`stamp_s` is the node clock, not the message header stamp** (`main.py`, around line
  694), in the stillness tracker. This design uses header stamps for evidence matching, so
  the two now use different clocks. Not a defect — the tracker measures wall-clock stillness
  — but it should not be assumed they agree. Noted rather than changed.
- **The 8.0° rotation gate remains a noise-tolerance workaround**, against a measured
  3.91° floor for a 600 mm plate at ~7 m. Chunk 6 makes it retunable from the page, which is
  what will let it be measured properly against a closer capture.
