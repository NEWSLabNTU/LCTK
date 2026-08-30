# Assisted capture and review: bag-driven placement ingestion with a review surface

- **Status:** Design, awaiting approval
- **Date:** 2026-08-31
- **Follows:** [Phase 8](../../roadmap/phase-8-single-source-target-definition.md) (W7-B)
- **Related:** [H-07](../../issues/archive/H-07-no-pose-diversity-gate.md) (pose diversity),
  [H-09](../../issues/archive/H-09-no-extrinsic-quality-metric.md) (quality metric),
  [M-12](../../issues/archive/M-12-no-robust-estimation-or-refinement.md) (pose rejection)

## The problem

Capturing a calibration today is a four-terminal manual loop: launch the graph, play a
bag, watch RViz until a detection looks stable, press `Space`, repeat. The operator is
the frame selector, and the only correctness signal available at capture time is "does
the overlay look right".

That costs an operator's full attention for the length of every session, and it gets
worse with the 600 mm solid target. One centred ArUco yields **4 coplanar corners per
placement** — exactly PnP's minimum — against 16 spread over a metre for the perforated
board. The solid target therefore leans much harder on having many well-spread
placements, which is precisely the part that is manual today.

## The proposal

The operator records **one bag per board placement** and feeds them to the pipeline in
sequence. The system picks a representative frame from each bag, solves incrementally,
and presents a review surface where a human accepts or rejects placements with the
overlay in front of them.

This splits the operator's job in two, and the split is the point: **capture becomes
unattended, judgement becomes reviewable after the fact.**

## The load-bearing constraint: stability is not correctness

The obvious auto-selection rule — "take frames once the pose stabilises" — is unsound
here, and it is worth stating plainly before any of the design depends on it.

Within a single bag the board does not move. What varies frame to frame is detector
noise, so "stability" measures **repeatability, not correctness**. This pipeline's
characteristic failure is a *stable wrong answer*:

- The M-14 quarter-turn slip is perfectly stable. Every frame agrees, and all of them
  are rotated 90° about the board normal.
- A plane fit that locked onto a wall behind the board is stable for as long as the wall
  is there.
- A mis-associated ArUco ID is stable for the whole bag.

A stability-gated selector would rubber-stamp every one of these with high confidence.

So stability is used for exactly one thing it is valid for — **choosing which frame
within a bag to keep** — and correctness is decided by signals that can actually
disagree with a confident wrong answer:

| signal | where it already lives | what it catches |
|---|---|---|
| per-pose reprojection RMS vs the current estimate | `lctk_quality` `residuals.per_pose_rms_px`; M-12's gate | a placement that disagrees with the others |
| quarter-turn loss separation | `perforated.rs` `loss_separation_m` | an ambiguous in-plane orientation |
| pose diversity | `lctk_quality` `diversity`, H-07 gate | a degenerate, under-constrained set |
| cross-placement consistency | the solve itself | a self-consistent but wrong placement |

The first three are already computed and thrown away at the end of a session. This
design's main job is to *retain and surface* them, not to invent new ones.

## What already exists

Most of the machinery is present; the gap is orchestration and presentation.

| capability | status |
|---|---|
| detection archive v5, save/load/append with identity | `detection_format.py`, `restore(append=)` |
| pose diversity measurement and refusal | `lctk_quality/diversity.py`, `compute_diversity` |
| per-pose reprojection RMS | `lctk_quality/residuals.py`, `per_pose_rms_px` |
| pose-outlier rejection and re-solve | `detection_buffer.py`, `reject_outlier_poses` (M-12) |
| quarter-turn hypothesis margin | `perforated.rs`, `PerforatedPoseEstimate.loss_separation_m` |
| 2D/3D overlay rendering | `pointcloud_image_overlay` |
| buffer add/remove/list/clear over services | `lctk_interfaces/srv/*`, driven by the TUI |
| deterministic evidence schema | `lctk_quality/evidence.py` |

**Genuinely new surface**, and it should be costed as such:

1. **A bag reader.** Nothing in the repo reads bags programmatically today — the runbook
   notes `EvidenceCollector` "is a library and a schema. There is no `ros2 run` that
   turns a bag into a report." This is new, not glue.
2. **Per-bag session boundaries** in the solver's notion of a capture.
3. **A review server** and its client.

## Design

### Stage 1 — ingest

```
ros2 run lctk_capture ingest --target <target.json5> --detector <preset.json5> \
    --bag placement_01.bag --bag placement_02.bag ... \
    --out session.json
```

Per bag: replay, run the existing detector and locator, collect synchronized pairs, and
select **one representative frame** — the medoid of the detected board poses, not the
mean, so the kept frame is a real observation rather than a synthetic average. Record
alongside it the dispersion of the poses it was chosen from, which is the honest
"stability" number and belongs in the report rather than in a gate.

A bag that yields no synchronized pairs, or whose dispersion exceeds a stated threshold,
is **reported as such and kept out of the buffer** — not silently dropped. A silently
skipped placement is the same failure class as a silently skipped test.

Output is a detection archive (v5, carrying identity) plus a sidecar record per bag:
frame chosen, pose dispersion, detection count, rejection reason if any.

### Stage 2 — solve incrementally

Feed placements into `DetectionBuffer` in order. After each, record the solve state:
estimate, per-pose RMS for every placement, diversity, and which placements M-12
rejected. This is a loop over machinery that already exists and needs no new solver
behaviour.

The incremental record is what makes the review surface useful: it shows not just the
final numbers but *when* a placement started disagreeing.

### Stage 3 — review

A local web server, served from the machine that ran the ingest, presenting per
placement:

- the 2D/3D overlay image
- per-pose reprojection RMS against the current estimate
- the quarter-turn loss separation (perforated targets)
- the pose dispersion from Stage 1
- whether M-12 rejected it

**Sorted worst-first, not chronologically.** The value is turning "review 40 placements"
into "review the 3 the solver is unsure about". Accepting or rejecting rewrites the
archive and re-solves; the diversity gate reports live, so the operator can see when
removing a placement would make the set degenerate.

The server is a review surface for a local session, not a service: bound to localhost,
no auth, no persistence beyond the session archive.

## Why a web UI rather than extending the TUI

The overlay is the one signal a human reads faster than any metric, and the TUI cannot
show it. The existing TUI stays exactly as it is for live, at-the-rig work — this is a
second surface for after-the-fact review, not a replacement.

## Deliberate non-goals

- **No automatic acceptance.** The system narrows what a human looks at; it never
  decides a calibration is good. Given that the characteristic failure is a confident
  wrong answer, an auto-accept path would be the wrong thing to build.
- **No live capture.** Ingest is offline, over recorded bags. Live capture stays with
  the TUI.
- **No new detector or solver behaviour.** Every metric surfaced here is already
  computed today.

## Open questions

1. **Bag reader implementation.** `rosbag2_py` in the ingest node, or shell out to
   `ros2 bag play` and subscribe as the graph does today? Playing is closer to the
   validated path and reuses the real node graph; reading directly is faster and
   deterministic but re-implements the sync semantics `lctk_sync` owns. Leaning toward
   playing, on the grounds that a second synchronization implementation is exactly the
   defect H-14 recorded.
2. **Dispersion threshold** for "this bag is too unstable to use". Should be measured on
   real recordings before being fixed, not guessed — the same mistake C-04 made by
   setting a gate below the sensor noise floor.
3. **Does this subsume W7-B or feed it?** W7-B needs real-bag evidence with an operator;
   this tooling would produce exactly that evidence. Sequencing matters: building it
   before any real-data session risks designing against imagined failure modes.

## Recommended sequencing

W7-B first, manually, on one real session. That produces the bags this tooling would
consume and reveals which failure modes actually occur. Then build ingest against real
recordings rather than against a guess about them.
