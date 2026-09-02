# Assisted extrinsic solver — design

- **Date:** 2026-08-31
- **Status:** Approved, not yet implemented
- **Area:** `lidar_to_camera_solver`, `lctk_launch`
- **Supersedes nothing.** `continuous` and `manual` both stay, unchanged and selectable.

## The problem

Capturing a multi-pose LiDAR-camera calibration today is three jobs done at once by one
person. From `CLAUDE.md`'s manual-mode workflow and
`interactive_solver_controller`'s key bindings, the operator must:

1. hold the board still in a new pose,
2. decide *by eye* that it is still enough and different enough from the poses already
   captured, and
3. reach for a keyboard and press `Space`.

Step 2 is the expensive one, and the tooling gives no help with it. The TUI shows a buffer
count and a pose table; it never shows the image the corners were measured in. When a
capture turns out to be bad — motion blur, a partly occluded marker, glare across the
plate — nothing in the pipeline can say *why*, because **no solver in the tree subscribes
to an image at all**. `lidar_to_camera_solver` consumes `Detection2DArray` and
`Detection3DArray` only.

The result is a capture session whose quality is not knowable until after it is over, and
whose failures are not diagnosable at all.

## What this adds

A third `solver_mode`, `assisted`, that moves the two mechanical judgements into the node
and the one real judgement into a browser:

- **the node decides when the board is still** and whether the pose is new,
- **the node queues the pair itself**, so the operator's hands stay on the board,
- **a web page shows the queue with image previews**, so the operator can see what was
  captured, drop the bad ones, re-solve, and export.

`continuous` and `manual` are untouched. `solver_mode` remains the switch, so the original
paths stay runnable for comparison.

## Non-goals

- Replacing `interactive_solver_controller`. The TUI stays; `manual` still drives it.
- Changing the estimator. Assisted mode feeds the same `DetectionBuffer` and gets the same
  SQPnP → LM → covariance-weighted → M-12-filtered solve.
- Authentication. The review server is unauthenticated by design; the mitigation is that it
  binds loopback unless explicitly opened. See *Security* below.
- A new archive format. Export writes the existing version-5 archive.

## Prerequisite: the corner-order hack must go first

Commit `3e6b873` added this to `detection_buffer.py`:

```python
# Quick fix: rotate the corner order by 1 position (90 degrees)
pixels = np.roll(pixels, shift=1, axis=0)
```

It is a hardcoded quarter-turn between the 2D corner pixels the detector emits and the
board-local 3D corners the target geometry emits. In `manual` mode an operator watching the
overlay would notice a quarter-turned result within a few captures. In `assisted` mode the
node captures unattended, so a wrong corner convention would be baked into an entire
session before anyone looked.

**This is fixed before assisted mode ships**, as its own change, in whichever single place
the two conventions actually disagree — not by rolling one of them at the point of use.
This is the same failure class as
[M-14](../../issues/archive/M-14-corner-order-brittle.md), which is explicit that the
duplication of corner-order knowledge across implementations is the underlying defect.

## Architecture

`main.py` is already ~1100 lines. The new work goes into three new modules in the same
package, each with one job and a testable boundary:

| module | responsibility | depends on ROS? |
|---|---|---|
| `stability.py` | `StillnessTracker` — is the board being held still? | no |
| `preview.py` | `PreviewStore` — latest camera frame, and a JPEG snapshot per queued pair | yes (one subscription) |
| `review_server.py` | Flask app + daemon thread; the review and export API | no ROS types |
| `main.py` | wires the three into the existing buffer under `solver_mode=assisted` | yes |

Two of the three are ROS-free and therefore properly unit-testable, which is the point of
the split.

### Dataflow

Everything upstream of `StillnessTracker` is unchanged.

```
image ──┬──► aruco_locator_node ──► aruco_detections ──┐
        │                                               ├──► DetectionPairSource ──► pair
        └──► PreviewStore (latest frame only)           │      (Conflux, sync window,
                                                        │       freshness + skew + epoch)
lidar ─────► lidar_board_detector ──► board_detections ─┘
                                                              │
                                          ┌───────────────────▼───────────────────┐
                                          │ StillnessTracker                      │
                                          │  |Δt| < max_translation_m and         │
                                          │  |Δθ| < max_rotation_deg across       │
                                          │  the last window_s seconds of pairs   │
                                          └───────────────────┬───────────────────┘
                                                              │ held still
                                          ┌───────────────────▼───────────────────┐
                                          │ novelty gate                          │
                                          │  lctk_quality.distinct_placements     │
                                          │  (5 cm / 5 deg)                       │
                                          └───────────────────┬───────────────────┘
                                                              │ new placement
                                                DetectionBuffer.add(pair)
                                                + PreviewStore.capture(pair_id)
                                                              │
                                            SQPnP → LM → covariance → M-12 rejection
                                                              │
                      ┌───────────────────────────────────────┼───────────────────────┐
                      ▼                                       ▼                       ▼
            /…/extrinsic_transform                   Flask review page          Export
            (TF semantics, T_lidar←camera)           list · preview · drop      ├─ detections.json v5
                      │                              diversity meter            └─ Autoware YAML
                      ▼                              re-solve                      (diff, then confirm)
            pointcloud_image_overlay
```

### `stability.py` — `StillnessTracker`

Pure numpy, no ROS, no OpenCV. Holds a bounded deque of recent board poses.

```python
tracker.push(position, quaternion, stamp_s) -> StillnessState
```

`StillnessState` reports `is_still`, the measured `translation_span_m` and
`rotation_span_deg` over the window, how many frames are in it, and — when not still —
a short human reason (`"board moving: 23 mm over 10 frames"`). That string goes straight
to the live page, so the operator learns what the gate wants instead of guessing.

Stillness alone is not enough to queue. After `is_still` fires, a **cooldown** suppresses
re-firing until the board has *left* the placement, otherwise one long hold would queue
every frame in it. The novelty gate below is the second, independent guard.

### The novelty gate

Reuses `lctk_quality.placements.distinct_placements` with its existing defaults
(`DEFAULT_POSITION_TOL_M = 0.05`, `DEFAULT_ORIENTATION_TOL_DEG = 5.0`). A candidate is
queued only if it forms a new placement against everything already buffered.

This matters more than it looks. `lctk_quality.diversity`'s module docstring records that
on a real field capture, reprojection RMSE and subset resampling both *invert* — a
degenerate single-placement capture filmed nine times scores **better** on both — and only
placement diversity separates good from degenerate. An auto-queueing capture loop without
this gate would generate exactly that degenerate capture, and every quality number would
applaud it.

The page therefore shows the diversity meter live: placements captured, normal span,
depth range, lateral span, each against `MIN_PLACEMENTS = 10`,
`MIN_NORMAL_SPAN_DEG = 20.0`, `MIN_DEPTH_RANGE_M = 1.5`, `MIN_LATERAL_SPAN_M = 1.0`.
That turns "am I done?" from a guess into a reading.

### `preview.py` — `PreviewStore`

Subscribes to the same image topic the camera's `aruco_locator_node` consumes, with the
mode-derived QoS (`RELIABLE` offline, `BEST_EFFORT` realtime).

The subscription callback is deliberately trivial — it stores the latest frame and returns,
following the `ArcSwap` pattern `CLAUDE.md` prescribes for high-rate sensor data with slow
downstream processing. Nothing decodes or encodes in the callback.

When a pair is queued, `capture(pair_id)` takes the latest frame, draws the detected ArUco
corners and (once an estimate exists) the reprojected board points, JPEG-encodes it once,
and stores the bytes against the pair id. Previews are evicted with their pair and bounded
by `review.max_previews`.

**A missing frame never blocks a capture.** If no image has arrived, the pair is still
queued and the preview reports "no frame". Calibration correctness does not depend on the
preview path.

### `review_server.py` — the Flask app

Flask **2.0.1 from apt** (`python3-flask`, already installed). This is not a pip
dependency: `CLAUDE.md` Known Issue 3 records pip `--user` installs shadowing apt's
`setuptools`, `numpy`, `scipy` and `anyio` and breaking the build four separate times. The
apt package sidesteps that entirely.

Served in a daemon thread. Handlers never touch `DetectionBuffer` directly; they submit
commands to the node and wait, so the buffer keeps exactly one locking discipline.

| route | purpose |
|---|---|
| `GET /` | the single review page |
| `GET /api/state` | queue summary, per-pair RMS, diversity meter, live stillness state, sync status |
| `GET /api/pair/<id>/preview.jpg` | the JPEG snapshot |
| `POST /api/pair/<id>/drop` | remove one pair |
| `POST /api/resolve` | re-solve from what remains |
| `POST /api/export` | write the artifacts (below) |

`GET /api/state` includes `pair_source.status_line()` verbatim, so a stalled or skewed
synchronizer is visible on the page rather than only in the log.

### Export

Two artifacts, in order:

1. **`detections.json`, version 5** — the existing archive: kept pairs, solved transform
   (raw `T_optical←lidar`), quality report, and the full Target Identity.
2. **Autoware `sensor_kit_calibration.yaml`** — via the existing `lctk_autoware_export`
   logic, which owns the frame arithmetic.

The Autoware write is **two-step and never one-click**: the first request returns the diff
that would be applied, the second applies it. The existing exporter's `.bak` behaviour is
kept. This file reaches a vehicle; a preview is not optional ceremony.

The exporter needs `--target`, `--camera-frame` and `--lidar-frame`. These are node
parameters, and the export route refuses with a clear message when they are unset rather
than guessing.

## Parameters

Per the repo convention that nodes take explicit config and hardcode no defaults, all of
these come from the calibration config through `calibrate.launch.py`:

| parameter | meaning |
|---|---|
| `stability.window_s` | seconds of pose history the pose must stay within tolerance across (at least 3 pairs must land in it) |
| `stability.max_translation_m` | translation span allowed across the window |
| `stability.max_rotation_deg` | rotation span allowed across the window |
| `stability.cooldown_s` | minimum gap between two auto-captures |
| `novelty.position_tol_m` | new-placement position tolerance (defaults to `lctk_quality`'s) |
| `novelty.orientation_tol_deg` | new-placement orientation tolerance |
| `review.bind_host` | default `127.0.0.1` |
| `review.port` | review server port |
| `review.jpeg_quality` | preview encode quality |
| `review.max_previews` | preview cache bound |
| `export.autoware_target` | path to `sensor_kit_calibration.yaml`; unset disables Autoware export |
| `export.camera_frame`, `export.lidar_frame` | entry names in that file |

## Error handling

| situation | behaviour |
|---|---|
| Target Identity mismatch between archive/target and the running node | the existing gate refuses the pair; the page names the mismatch |
| synchronizer stale, skewed, or replaying | `pair_source.status_line()` shown verbatim on the page |
| no camera frame yet | pair is queued anyway; preview reads "no frame" |
| solve fails or is refused | the existing `Failed` / `Refused` / `NotReady` outcome is rendered with its reason |
| Autoware export parameters unset | export refuses and says which parameter is missing |
| `review.bind_host` is not loopback | `WARN` at startup naming the exposure |

## Security

The review server has **no authentication**. Anyone who can reach the port can read the
queue, the camera previews, and the solved extrinsic, and can trigger an export that writes
`sensor_kit_calibration.yaml` on the host.

The mitigation is the bind address: loopback by default, and opening it is a deliberate
parameter change that logs a warning. This is a field tool on a rig network, not a service;
that is the level of protection being claimed, and it should not be mistaken for more.

## Testing

| unit | how it is tested |
|---|---|
| `StillnessTracker` | pure unit tests: a still board fires once, a moving board never fires, behaviour exactly at the tolerance, the cooldown suppresses a long hold, the window fills correctly |
| novelty gate | that a second pose inside 5 cm / 5° is refused and one outside is accepted |
| `PreviewStore` | encode and draw against a synthetic frame; capture with no frame available yields the "no frame" result rather than raising |
| `review_server` | Flask's test client against a fake node facade — list, drop, re-solve and both export steps, with no ROS running |
| export | extends the existing `lctk_autoware_export` tests for the two-step diff-then-write path |
| mode plumbing | `assisted` is accepted by `parse_solver_mode` and by `calibrate.launch.py`'s validation, and `continuous` / `manual` still behave as before |

Per `CLAUDE.md`'s testing practice, each new recipe or suite is verified by breaking an
assertion deliberately and confirming a non-zero exit before it is trusted.

## Risks

- **The corner-order hack.** Handled as a prerequisite above; assisted mode must not ship on
  top of it.
- **Auto-capture invites a degenerate capture.** Mitigated by the novelty gate and the live
  diversity meter, but an operator who ignores the meter can still produce a bad calibration
  quickly. The meter is deliberately prominent.
- **`main.py` growth.** Mitigated by putting the new work in separate modules and keeping
  `main.py`'s addition to wiring.
