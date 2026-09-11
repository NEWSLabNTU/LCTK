# 0008. The assisted review page is a revisioned session with split reads and SSE invalidation

- **Date:** 2026-09-11 (amended)
- **Status:** accepted

> Amendment: the revisioned read model, ETags, progressive hydration, and
> client-owned caches from the 2026-09-07 decision are retained. The original
> 500 ms full-state heartbeat is superseded by split projections plus SSE
> invalidation and a small revisions fallback, as recorded below.

## Context

The assisted-review page used to poll `/api/state` every 500 ms. Even with
revision checks, that heartbeat still transferred and decoded a combined
representation whenever live status changed, and it made the frontend ask the
server to reconsider unrelated projections. A new browser also had to wait for
scene and evidence I/O before useful content could appear.

There are two different kinds of change in this page.  Live stillness and
synchronization status may change on every heartbeat.  Capture quality and the
scene change only when the Detection Buffer, camera pose, or matched evidence
changes.  A delayed camera frame or `plane_inliers` cloud may complete evidence
for one Capture after the Capture has already been accepted.  Dropping a
Capture may also change every RMS value because the Solved Estimate belongs to
the whole Detection Buffer.

The browser and server therefore need a shared vocabulary for change and a
transport that does not send a capture list for a live-only change. The page is
unauthenticated and runs in forwarded vehicle-bay ports, so the transport must
remain same-origin, dependency-free, bounded, and recoverable without a
durable event log.

## Decision

The review server exposes independently revisioned representations alongside the
existing state and scene representations:

- a new `session_epoch` changes whenever the node review session starts;
- `capture_revision` identifies the current Detection Buffer projection,
  including quality values;
- `captures_revision` identifies the complete capture projection: pair list,
  quality/solve values, diversity gauges, and evidence availability. It
  advances for a Detection Buffer or evidence change while `capture_revision`
  retains its existing Detection Buffer-only meaning;
- `live_revision` identifies stillness, synchronization, identity/error,
  confirmed parameter, and export-availability status;
- `scene_revision` identifies world geometry and camera pose;
- `evidence_revisions` remains available in the compatibility state/pair
  projection to identify delayed evidence changes per Capture; and
- `state_revision` covers the complete `/api/state` representation, including
  live status.

`/api/state` remains an atomic bootstrap and compatibility response. New clients
use `/api/captures`, `/api/live`, and `/api/scene` as independent projections.
`/api/revisions` returns only the complete revision vector
`{session_epoch, live_revision, captures_revision, scene_revision}`. Each
projection has an ETag derived only from its own revision(s) and the session
epoch; a matching `If-None-Match` receives `304` with no body. Mutation
endpoints remain uncached. Existing paths and fields remain valid; revision
fields are additive.

`/api/events` is a same-origin Server-Sent Events stream. It sends a compact
`revisions` event containing the complete vector, including immediately on
connect and again after reconnect. Events are invalidation hints, not payload:
they contain no pair list, evidence bytes, or raw LiDAR. The server coalesces
pending notifications to the newest vector in a bounded latest-value slot per
subscriber, emits periodic keepalive comments, disables proxy buffering, and
removes disconnected subscribers. No durable event history or per-client
full-state computation is introduced. Clients use native EventSource reconnect
and poll `/api/revisions` at a low rate while the stream is unavailable.

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

The browser opens the event stream after the bootstrap response. A live-only
vector change fetches `/api/live` only; a capture/evidence change fetches
`/api/captures`; a scene change fetches `/api/scene`. Each fetch has its own
ETag, in-flight request, retry/backoff, and response-revision guard. A response
records the revision it actually returned, not merely the hint that caused the
request. A hint arriving during a fetch causes another refresh when the
response is behind. If the epoch changes, all projections, ETags, assets, and
outstanding generations are cleared and a fresh bootstrap is required.

No WebSocket, JavaScript build step, CDN dependency, authentication, raw LiDAR
sweep, or solver-mode change is introduced. The old 500 ms full-state heartbeat
is retained only for compatibility callers; it is not used by the new page's
steady-state synchronization.

## Consequences

The first paint is useful even when a camera preview, cloud, or scene request is
slow or unavailable.  Returning to a retained pair is local and immediate, and
late evidence updates only the affected pair unless a Detection Buffer mutation
changes the joint quality projection.  Conditional requests and dirty
rendering reduce repeated work while live status remains observable.

The initial paint is useful as soon as bootstrap state arrives. Live-only
changes transfer no capture body, and capture/evidence changes do not transfer
live or scene bodies unless their own revisions changed. The revisions fallback
keeps a disconnected browser convergent without reintroducing the full-state
heartbeat's bandwidth cost. Cross-projection ordering is intentionally
eventual, while each capture snapshot is atomic across pair/quality/gauge and
evidence metadata.

The server must publish notifications at the read-model mutation seams and
maintain the independent revisions. The browser must handle reconnects,
coalesced hints, delayed responses, and epoch replacement. These lifecycle
rules and the guarantee that live-only updates transfer zero capture bytes are
covered by backend, Flask, and browser tests.
