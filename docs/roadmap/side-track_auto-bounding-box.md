# Side Track: Automatic Bounding Box for Board Detection

## Problem

The board detection pipeline (`Pointcloud → Bounding Box Filter → RANSAC Plane → PCA Initial
Pose → ICP Refinement → Pose`) depends on a hand-tuned crop box (`bbox.json5`, adjusted with
`filter_box_tuner`). Every new sensor placement or board position requires manual re-tuning
before detection works at all. Phase 7 (§2.1) already states the constraint explicitly: the
dynamic-target detector must work **without** manual bounding box filtering.

This doc collects candidate strategies for making the bounding box appear automatically. They
are not mutually exclusive — the recommended path combines a bootstrap strategy (find the board
with no prior) with a tracking strategy (keep the box locked on cheaply once found).

Current relevant facts:

- The detector API takes bare `&[na::Point3<f64>]` — no intensity, no ring/beam index
  (`hollow-board-detector/src/detector.rs`). Strategies needing intensity or beam structure
  require plumbing those through from the `PointCloud2` in `lidar_board_detector`.
- Board geometry is known precisely (`board_detector.json5`): a planar hollow board with a
  known hole pattern — a strong, discriminative prior.
- The camera side already produces an independent detection of the same target (ArUco corners
  + PnP pose), usable as a spatial hint once even a coarse extrinsic exists.
- ICP loss noise floor is ~0.026 (VLP-32C ±3 cm) — any candidate-scoring gate must sit above
  it (see C-04 postmortem).

---

## Candidate Strategies

### A. Plane-first global search (no ROI at all)

Skip the crop box; make plane extraction the filter. Run multi-plane extraction over the full
cloud (iterative RANSAC with inlier removal, or normal-based region growing), then score each
plane segment against the board prior:

- Inlier patch extent ≈ board dimensions (PCA eigenvalues of the segment).
- Planarity (third eigenvalue small).
- Height band sanity (board center is 0.5–2 m up, not the ground plane).
- Optional strongest cue: run one cheap ICP iteration against the hollow-board model and use
  the hole pattern to disambiguate the board from walls/signs of similar size.

The winning segment's oriented PCA box (plus margin) *becomes* the bbox handed to the existing pipeline unchanged.

- **Pros:** No extra sensors, no extra state, works on the very first frame, minimal
  architecture change (auto-bbox is a drop-in front stage).
- **Cons:** Full-cloud RANSAC is the expensive path — ground plane and walls dominate and must be peeled off first. Mitigate by downsampling (voxel grid) for the search stage only, then running the normal pipeline on the full cloud inside the found box.
- **Effort:** Medium. Pure-Rust, testable with `cargo test` on recorded clouds; fits the
  existing `rust/` layer cleanly (candidate crate: `board-finder` or a module in
  `hollow-board-detector`).

### B. Ground removal + Euclidean clustering

Cheaper variant of A. Remove the dominant ground plane (one RANSAC pass), then Euclidean-cluster the remainder. Filter clusters by bounding-box size ≈ board dims and planarity; each surviving cluster seeds a candidate bbox, validated by the existing RANSAC→PCA→ICP stages (which already gate on fit quality).

- **Pros:** Fast, simple, well-understood. Clustering is O(n log n) with a k-d tree.
- **Cons:** Fails when the board leans against or stands near another structure (merges into
  one cluster). A person holding the hand-held Phase-7 target merges with the target — needs
  the plane/hole scoring from A as a second stage anyway.
- **Effort:** Low–medium.

### C. Camera-seeded ROI (cross-modal hint)

Use the ArUco detection the camera pipeline already produces. Given the marker's PnP pose in
the camera frame and *any* coarse extrinsic (previous calibration, CAD/URDF mounting values, or a first pass from strategy A), transform the board pose into the LiDAR frame and place a
board-sized box (generous margin, e.g. 2×) around it.

- **Pros:** Nearly free — both detections already exist; the solver already synchronizes them
  via Conflux. Naturally tracks a moving board, which is exactly the Phase-7 probing scenario. Margin can shrink as the extrinsic estimate converges.
- **Cons:** Chicken-and-egg: needs a coarse extrinsic to bootstrap. In practice mounting
  values are known to ~10 cm / a few degrees, which a generous margin absorbs; otherwise
  strategy A/B bootstraps frame one. Also couples the LiDAR detector to the camera pipeline —
  keep it as an optional ROI *hint* topic so `lidar_board_detector` still runs standalone.
- **Effort:** Medium (new hint topic + subscription; box math is trivial).

### D. Temporal tracking (auto-bbox after first lock)

Whatever finds the board first, keep the box locked on for free: after each successful
detection, re-center the bbox on the detected board pose (with margin scaled to observed frame-to-frame motion, or a constant-velocity prediction). On N consecutive detection failures, fall back to the bootstrap search (A/B/C).

- **Pros:** Turns the expensive global search into a once-per-session cost. Essential for the
  10 Hz+ moving-target requirement of Phase 7. Trivial state machine: `SEARCHING → TRACKING`.
- **Cons:** Not a bootstrap by itself; abrupt occlusion or teleporting target forces re-search.
- **Effort:** Low. This one is worth doing regardless of which bootstrap wins.

### E. Motion / background subtraction

For static-mount scenarios (wayside, parked vehicle): accumulate a background occupancy grid
(voxel hash) over the first few seconds *without* the board, then flag voxels that newly become occupied when the operator walks the board in. The changed-voxel blob seeds the bbox.

- **Pros:** Extremely discriminative in cluttered static scenes where A/B struggle; no camera
  dependency.
- **Cons:** Requires an operator workflow step (board absent, then present) — a procedure
  change, not just software. Useless when the sensor platform itself moves. Person + board move
  together, so still needs plane/hole scoring to crop person out.
- **Effort:** Medium.
- **Status:** implemented and validated on real data — 88.4% recall at 100%
  precision, breaking phase 7's ~44–50% ceiling. See
  [`side-track_method-e-background-subtraction.md`](side-track_method-e-background-subtraction.md).

### F. 2D projection + image-space detection (ties to the pointcloud-to-image side track)

Project the cloud to a structured 2D image and detect the board there, reusing cheap 2D ops
(`side-track_pointcloud-to-image.md`):

- **Range image** (spherical projection — native for the VLP-32C): the board appears as a
  smooth, connected, roughly rectangular depth patch with characteristic holes (depth
  discontinuities or dropouts). Detect via connected components on depth continuity + shape
  filter; back-project the patch to 3D for the bbox. Dense, no empty-pixel problem, O(n).
- **BEV grid**: the upright board collapses to a short line segment of high height-extent in
  bird's-eye view — a distinctive signature vs. walls (long lines) and poles (dots). Very cheap
  scan for candidates; weaker at estimating vertical extent, so pair with a height channel.
- A learned detector on either representation (e.g. PointPillars-style) is possible but
  overkill for one known rigid target; keep classical.

- **Pros:** Fast (2D ops on small images), and the range image especially exploits the sensor's
  native structure. Good fit for the Jetson (could even use OpenCV).
- **Cons:** Range image needs ring/beam metadata plumbed through (currently dropped). BEV
  discretization adds resolution/precision tradeoffs. More new code than A/B.
- **Effort:** Medium–high, but shares infrastructure with the pointcloud-to-image side track if
  that proceeds anyway.

---

## Recommendation

Layered combination, in implementation order:

1. **D (tracking) first** — small, independent win; eliminates re-tuning whenever the board or
   truck shifts slightly, even while bootstrap remains manual.
2. **A (plane-first global search) as the bootstrap**, with B's ground-removal as its first
   step and the hole-pattern ICP score as the disambiguator. Pure Rust, no new sensor plumbing,
   directly satisfies Phase-7 §2.1.
3. **C (camera hint)** as an optional accelerator once Phase 7's dynamic pipeline lands — it is
   the natural way to keep the box on a hand-held moving target between LiDAR detections.
4. Revisit **F (range image)** if A's full-cloud search proves too slow on the Jetson at 10 Hz;
   it is the principled fast path, and the pointcloud-to-image side track would fund the shared
   projection code.

`bbox.json5` and `filter_box_tuner` stay as the manual override/escape hatch (`bbox_mode:
auto | manual | hint`), defaulting to `auto`.

## Open Questions

- Does `lidar_board_detector` need intensity/ring fields plumbed from `PointCloud2` now, to
  keep F viable later? (Cheap to add while touching the message path.)
- Candidate scoring threshold: must be validated against the ~0.026 ICP noise floor so the
  auto-search never "accepts nothing" silently (C-04 lesson) — log every rejected candidate
  with its score.
- Multi-board scenes (two rigs calibrating simultaneously): search must return all candidates
  above threshold, not argmax.
