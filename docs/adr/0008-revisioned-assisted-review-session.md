# 0008. The assisted review page is a revisioned session with progressive evidence hydration

- **Date:** 2026-09-07
- **Status:** accepted

## Context

The assisted-review page polls `/api/state` every 500 ms.  The response currently
rebuilds the same Detection Buffer projection, replaces the capture list and
footer, rebuilds the Three.js scene, and fetches the selected preview and every
cloud before the first useful render.  This makes a new browser wait for scene
and evidence I/O, causes static DOM to be rewritten on every heartbeat, and
fetches a preview again whenever the operator returns to a pair.

There are two different kinds of change in this page.  Live stillness and
synchronization status may change on every heartbeat.  Capture quality and the
scene change only when the Detection Buffer, camera pose, or matched evidence
changes.  A delayed camera frame or `plane_inliers` cloud may complete evidence
for one Capture after the Capture has already been accepted.  Dropping a
Capture may also change every RMS value because the Solved Estimate belongs to
the whole Detection Buffer.

The browser and server therefore need a shared vocabulary for change without
changing the existing endpoint paths or JSON fields.  A notification transport
would add another lifetime and reconnect protocol to an unauthenticated page;
the current requirement is satisfied by the existing heartbeat plus conditional
HTTP and client-side caching.

## Decision

The review server exposes revision metadata alongside the existing state and
scene representations:

- a new `session_epoch` changes whenever the node review session starts;
- `capture_revision` identifies the current Detection Buffer projection,
  including quality values;
- `scene_revision` identifies world geometry and camera pose;
- `evidence_revisions` identifies delayed evidence changes per Capture; and
- `state_revision` covers the complete `/api/state` representation, including
  live status.

`/api/state` and `/api/scene` return an `ETag` derived from the session epoch
and the representation's revisions.  A matching `If-None-Match` receives `304`
with no body.  Mutation endpoints remain uncached.  Existing paths and fields
remain valid; revision fields are additive.

The node keeps a small read model for revision tokens and caches the expensive
capture/quality projection by Detection Buffer revision.  It takes the
Detection Buffer snapshot and evidence metadata under the node lock, then
builds or reads the projection from that coherent key outside the lock.  The
Detection Buffer remains the owner of the Solved Estimate and Quality Verdict;
the read model only projects them for review.

The browser owns one `ReviewSession` module.  `ReviewApi` is only an HTTP
adapter: it sends validators, handles `304` as “retain the previous payload”,
and returns asset bytes.  `ReviewSession` owns state, scene, preview bytes,
cloud buffers, one in-flight request per asset, bounded eviction, selected-pair
generations, and object-URL lifetime.  A missing asset is retryable and is not
cached as a permanent success or failure.  Preview bytes are keyed by session
epoch, Capture ID, and evidence revision; an object URL exists only while a
preview is displayed.

After the first successful state response, the page renders the list and
footer immediately.  Scene, cloud, and preview hydration run independently and
render each successful result as it arrives.  Chrome and SceneModel use the
revision, selection, layer, and asset keys relevant to their work; an unchanged
heartbeat does not replace capture rows or rewrite static labels.  Async
results carry the session epoch/revision and local generation and are discarded
when stale.  Confirmed parameters are retained separately from local,
unapplied drafts and are reconciled only after a successful Apply.

The heartbeat remains 500 ms.  No WebSocket, Server-Sent Events, JavaScript
build step, CDN dependency, authentication, raw LiDAR sweep, or solver-mode
change is introduced.

## Consequences

The first paint is useful even when a camera preview, cloud, or scene request is
slow or unavailable.  Returning to a retained pair is local and immediate, and
late evidence updates only the affected pair unless a Detection Buffer mutation
changes the joint quality projection.  Conditional requests and dirty
rendering reduce repeated work while live status remains observable.

The server still performs a cheap heartbeat and may return `200` when live
status changes.  Revision metadata must be maintained when captures, camera
pose, parameters, or evidence change, and cache eviction/object URLs require
explicit lifecycle tests.  The page remains pull-based, so a browser that is
disconnected during a change catches up on its next heartbeat.
