# Initial board pose and the in-plane (roll) rotation: why both LiDAR presets need `initial_inplane_rotation_deg: 45.0`

- **Date:** 2026-08-12
- **Branch investigated:** `feat/bbox-free` (HEAD `8ca3d8d`, plus `33e4ab9` rename in history)
- **Type:** investigation write-up (source walkthrough), no code changed
- **Related:** [M-17](../../issues/M-17-initial-pose-rewrite-unverified-bbox-path.md),
  [M-14](../../issues/M-14-corner-order-brittle.md),
  [M-03 (archived)](../../issues/archive/M-03-hardcoded-plane-normal-x.md)

> **Update 2026-08-13 — two of this document's findings have been acted on.** The core diagnosis
> below is unchanged and `initial_inplane_rotation_deg: 45.0` is still required. What changed:
>
> 1. **`board_cluster_detector::pose::board_pose` now reports a REP-103 / Autoware frame**
>    (X forward, Y left, Z up; the sensor-facing normal is −X). The axis that aims at the up-most
>    corner is now **Z**, not X. This is a relabel only — the numbers, and the corner winding, are
>    unchanged, as the golden-parity suite confirms. Passages below that call that axis "X" are
>    marked inline.
> 2. **The divergent third initial pose in the library is gone.** `algo::fit_board_icp`,
>    `algo::fit_board_icp_with_iterator`, `Detector::detect*`, and their `-135°` constants were
>    deleted, along with the unused `board-fitter`/`board-fitter-config` crates. §5's "live trap"
>    paragraph is superseded — see the note there. Exactly one in-plane convention now exists
>    repo-wide: `sensor_up_axis` + `initial_inplane_rotation_deg`, in the node.
>
> Still open: the 45° fix itself, and the sign question in §7.

**Why this file lives in `docs/superpowers/specs/`:** repo convention (CLAUDE.md) puts one file per
finding in `docs/issues/`, but every issue file must be indexed in `docs/issues/README.md`, and this
task forbids editing existing files — an unindexed issue file would break that convention. The
closest existing issue, M-17, already owns the "the initial-pose rewrite was never proven
equivalent" finding; this document is the evidence that settles it. `docs/superpowers/specs/` is the
designated home for design/analysis docs, so it goes here. If this is later promoted to a tracker
entry, it should be filed as an update to M-17 rather than a new ID.

---

## Verdict

**The hypothesis is correct, and the source proves it exactly.** The board's *model* is a square
whose local X and Y axes run along its four **edges**, and every accessor in that model
(`top_corner`, `bottom_corner`, `left_corner`, `right_corner`, and the three hole centers) is named
for a **diamond**-mounted board — the local diagonal `(1,1)` is the physical "up". The live initial
pose in `compute_initial_pose_from_plane` instead builds the board frame with local **Y = sensor
"up" projected onto the plane** and **X = Y × Z**, i.e. a frame whose *edges* are vertical and
horizontal — an **axis-aligned** square. The physical board is hung corner-up (diamond), which the
`bbox_free` Stage-1 gate independently enforces (`stance_floor: 0.9`, "~1 for a board standing on a
corner, ~0.71 for an axis-aligned flat panel"). The two are exactly 45° apart about the plane
normal, so `initial_inplane_rotation_deg: 45.0` is not a fudge factor — it is the missing model↔world
convention, and it numerically reproduces the *previous* implementation, which had `Rz(-45°)`
hard-coded into its lifting rotation (removed by commit `162a28e`, which simultaneously added the
config knob). Nothing anywhere estimates the in-plane rotation from data on the path that feeds ICP:
the one component that *does* estimate it (`board_cluster_detector::pose::board_pose`, which aims
the board Z axis at the up-most corner — a diamond convention; called X before the 2026-08-13
REP-103 relabel) has its rotation **discarded**;
only its point set is forwarded. ICP cannot be relied on to recover the missing 45° because 45° is
precisely the half-way saddle between two of the square's four 90°-symmetric attractors.

---

## 1. Pipeline shape and where the initial pose enters

Both detection modes share Stages 2 and 3; only Stage 1 differs.

| Stage | bbox mode | bbox_free mode |
|---|---|---|
| 1 — cluster selection | `filter_points_by_bbox` (`main.rs:903`) | `select_board_cluster` (`main.rs:906`) |
| 2 — plane | `detect_plane_ransac` (`main.rs:963`), or PCA when `skip_ransac` (`main.rs:936-945`) | same |
| 3a — voxel downsample | `main.rs:991-1031` | same |
| 3b — initial pose + ICP | `detect_icp` (`main.rs:1046`) | same |

The mode switch is a single `match` on `bbox_free_cfg` at `ros/lidar_board_detector/src/main.rs:900-922`,
and both arms produce nothing but a `Vec<Point3<f64>>`. **The initial-pose derivation is therefore
identical in both modes** — there is exactly one call site:

```rust
// ros/lidar_board_detector/src/main.rs:1451-1460
// Step 3: Create initial pose using plane normal-based alignment
let initial_pose = Self::compute_initial_pose_from_plane(
    plane_model,
    plane_inlier_points,
    board_width.as_meters(),
    &config.sensor_up_axis,
    config.initial_inplane_rotation_deg,
    header,
    board_debug_publishers,
)?;
```

Note both shipped LiDAR presets set `skip_ransac: true`
(`board_detector_seyond.json5:54`, `board_detector_velodyne.json5:56`), so on those rigs Stage 2 is
`compute_plane_from_points` (PCA over the selected cluster, `main.rs:945`) rather than RANSAC. That
changes only the plane, never the in-plane angle.

---

## 2. The initial-pose derivation, step by step

Source: `ros/lidar_board_detector/src/main.rs:1707-1846`. The doc comment states the intent
plainly (`main.rs:1709-1712`):

> The board's local frame: Z = board normal (toward sensor), Y ≈ world "up" projected
> onto the board plane, X = cross(Y, Z).

**Step 1 — translation seed = cluster centroid** (`main.rs:1730-1734`): the arithmetic mean of the
plane inlier points. No shape information is used.

**Step 2 — normal sign** (`main.rs:1745-1753`): the plane normal is flipped to point toward the
sensor origin (this is the M-03 fix).

**Step 3 — rotation** (`main.rs:1771-1816`). This is the whole of the in-plane story:

```rust
// ros/lidar_board_detector/src/main.rs:1771-1790
let rotation = {
    let board_z = plane_normal;
    let up = sensor_up_axis.as_vector();

    // Project world "up" onto the board plane (remove component along board_z)
    let up_projected = up - up.dot(&board_z) * board_z;

    let board_y = if up_projected.norm() > 1e-6 {
        up_projected.normalize()
    } else {
        // "up" is parallel to the board normal — pick an arbitrary perpendicular
        let alt = if board_z.x.abs() < 0.9 { na::Vector3::x() } else { na::Vector3::y() };
        (alt - alt.dot(&board_z) * board_z).normalize()
    };

    let board_x = board_y.cross(&board_z);
```

`sensor_up_axis` is a three-valued enum resolving to a unit world axis
(`rust/hollow-board-detector/src/config.rs:11-26`); `"x"` for the Seyond Falcon, `"z"` for the
Velodyne. **This is the only place `sensor_up_axis` enters the rotation, and it only selects which
world axis becomes the board's local +Y.** The in-plane angle is therefore *entirely determined* by
the plane normal and the up axis — no point-cloud shape, no PCA, no corner search, no hole search.

Then the config offset (`main.rs:1800-1815`):

```rust
let base_rotation = na::UnitQuaternion::from_matrix(&na::Matrix3::from_columns(&[
    board_x, board_y, board_z,
]));

// Apply configurable in-plane rotation offset around the board normal.
// Corrects a fixed rotational bias visible in RViz (set via initial_inplane_rotation_deg).
if initial_inplane_rotation_deg.abs() > 1e-6 {
    let angle_rad = initial_inplane_rotation_deg.to_radians();
    let offset = na::UnitQuaternion::from_axis_angle(
        &na::Unit::new_normalize(board_z),
        angle_rad,
    );
    offset * base_rotation
} else {
    base_rotation
}
```

The axis is the board normal (pointing *at* the sensor), so a positive angle is counter-clockwise as
seen from the LiDAR — matching the comment at `rust/hollow-board-detector/src/config.rs:84-88`.

**Step 4 — translation** (`main.rs:1818-1828`): the pose origin is placed so the *model's* center
lands on the cluster centroid, using `board_center_board = (w/2, w/2, 0)` — i.e. the pose origin is
a **corner** of the board, not its center. This is the same convention the model uses
(`BoardModel::board_center`, below).

### What the initial pose is *not*

- It is **not** PCA on the cluster. (`debug/pca_eigenvectors` and the log line "Published initial
  board pose markers from PCA" at `main.rs:1473` are stale names — the code has no PCA in this
  function.)
- It is **not** a min-area-rect / extent-ratio / corner fit.
- It does **not** use the hole pattern.

---

## 3. The board model's canonical orientation — diamond, by naming

`rust/hollow-board-config/src/lib.rs` defines the model. Everything is placed by
`board_plane_point(x, y)` (`lib.rs:49-53`), which walks `x` along the board's local X axis and `y`
along local Y from the pose origin. The board occupies `[0, w] × [0, w]` — the ICP correspondence
routine clamps into exactly that box (`lib.rs:331-342` and the non-parallel twin at `lib.rs:575-586`).
So **local X and Y run along the board's edges**.

The named accessors (`lib.rs:62-97`):

| accessor | local (x, y) | line |
|---|---|---|
| `bottom_corner` | `(0, 0)` | `lib.rs:66-68` |
| `top_corner` | `(w, w)` | `lib.rs:62-64` |
| `left_corner` | `(w, 0)` | `lib.rs:70-72` |
| `right_corner` | `(0, w)` | `lib.rs:74-76` |
| `left_circle_center` | `(w/2 + s, w/2 − s)` | `lib.rs:78-83` |
| `right_circle_center` | `(w/2 − s, w/2 + s)` | `lib.rs:85-90` |
| `top_circle_center` | `(w/2 + s, w/2 + s)` | `lib.rs:92-97` |

These names are only self-consistent in a **diamond** orientation. "Bottom" at `(0,0)` and "top" at
`(w,w)` means the local `(1,1)` **diagonal** is the physical vertical; "left" at `(w,0)` and "right"
at `(0,w)` are then the two side corners, and the three holes sit left / right / **above** the
center — a triangle with one hole up, which is what the physical board has. If the board frame were
axis-aligned (edges vertical/horizontal), `(0,0)` would be a *bottom-left* corner and `(w,w)` a
*top-right* corner, and no naming of "left"/"right" corners would make sense at all.

The same diamond naming propagates into the ArUco layout: `multi_marker_corners`
(`lib.rs:123-162`) names its four markers `bottom` `(0,0)`, `left` `(+square, 0)`, `right`
`(0, +square)`, `top` `(+square, +square)` and emits each marker's corners as
`[right, top, left, bottom]` (`lib.rs:121-122`, `lib.rs:145-151`).

Independent physical confirmation that the rig really is diamond-mounted (not just the naming):

- `docs/roadmap/phase-7-projection-board-detection.md:325` — "the true diamond-mounted board (one
  diagonal near vertical) [vs] the axis-aligned clutter panels".
- `docs/roadmap/phase-7-projection-board-detection.md:251`, `:579`, `:645` — real captures raster as
  a "clean diamond outline **with two dark hole blobs**".
- `docs/superpowers/plans/2026-07-17-projection-board-detection-experiment.md:16` — "Board prior is
  geometry only: **diamond (square rotated 45°)**".
- The shipped `bbox_free` gate: `stance_3d` returns "~1 for a board standing on a corner, ~0.71 for
  an axis-aligned flat panel" (`rust/board-cluster-detector/src/pose.rs:94-106`), it is applied at
  `rust/board-cluster-detector/src/detector.rs:201-205`, and both presets set `stance_floor: 0.9`
  (`board_detector_seyond.json5`, `board_detector_velodyne.json5`). **Stage 1 in these configs will
  only ever hand ICP a corner-standing board.**

---

## 4. Why 45.0 is required — the mechanism

Take the simplest concrete case (Velodyne, `sensor_up_axis: "z"`), a vertical board facing the
sensor along −X, i.e. `board_z = (−1, 0, 0)`.

**Base frame (`initial_inplane_rotation_deg = 0`):**
`up_projected = (0,0,1)` (already in-plane) ⇒ `board_y = (0, 0, 1)`, and
`board_x = board_y × board_z = (0, −1, 0)`.
The model's bottom→top diagonal `(1,1,0)_local` maps to `(board_x + board_y)/√2 = (0, −0.707, 0.707)`
— **45° off vertical**. The board's *edges* are vertical/horizontal: the model is drawn as an
axis-aligned square. That is the wrong shape for a corner-standing board.

**With `initial_inplane_rotation_deg = 45.0`:** rotating the base frame by +45° about `board_z` gives
`board_x = (0, −0.707, 0.707)`, `board_y = (0, 0.707, 0.707)`, so the diagonal maps to `(0, 0, 1)` —
**straight up**. The model is now a diamond, matching reality. (Verified numerically; the same
construction with a `+X`-up sensor produces the analogous result, which is why the Seyond preset
also needs 45.)

### This exactly restores the pre-`162a28e` hard-coded behavior

Before commit `162a28e` ("feat(board-det): bbox_free seyond pose fix + reject/foreground
diagnostics", 2026-08-04) the same function built the rotation from a **fixed lifting rotation with
`−45°` baked in** (`162a28e^:ros/lidar_board_detector/src/main.rs:1658-1688`):

```rust
// Step 3: Let the xy-plane projections of board normal and plane normal overlap
// This decreases the chance of falling into local minimum
let rotation = {
    // Create lifting rotation: -90° around Y-axis, then -45° around Z-axis
    let lifting_rotation = na::UnitQuaternion::from_euler_angles(0.0, -FRAC_PI_2, 0.0)
        * na::UnitQuaternion::from_euler_angles(0.0, 0.0, -std::f64::consts::FRAC_PI_4);
    let lifted_normal = lifting_rotation * na::Vector3::z_axis();
    // ... planar_rotation = rotation_between(lifted_normal, (n.x, n.y, 0)) ...
    planar_rotation * lifting_rotation
};
```

Evaluating `Ry(−90°)·Rz(−45°)`: `board_x = (0, −0.707, 0.707)`, `board_y = (0, 0.707, 0.707)`,
`board_z = (−1, 0, 0)` — i.e. **the legacy seed is exactly the new base frame rotated +45° about the
board normal**, and its `(1,1)` diagonal is exactly world-up. The `planar_rotation` that follows is
`rotation_between` two vectors that both lie in the world XY plane, so it is a rotation about world
**Z** and preserves "diagonal is up" (this is also why the legacy math broke on the X-up Seyond: it
hard-assumed the gravity axis is Z, and it never tilts the frame to match a non-vertical plane).

So `initial_inplane_rotation_deg: 45.0` is a faithful, generalized re-expression of the `−FRAC_PI_4`
that used to be hard-coded. The commit message records only "config: sensor_up_axis=x,
initial_inplane_rotation_deg=45, and per-gate unit comments" — the fact that 45° was *restoring a
lost model convention* rather than *correcting a rig-specific bias* was never written down, which is
why it now reads as a mystery constant. The field's own doc comment reinforces the misreading:
"Use this to correct a fixed in-plane rotational offset visible in RViz … Try ±45 or ±90"
(`rust/hollow-board-detector/src/config.rs:84-88`).

### The knock-on: the shipped default config is now 45° wrong

`ros/lctk_launch/config/board/board_detector.json5:112` ships `initial_inplane_rotation_deg: 0.0`,
and it is the config used by `sample_data.yaml`, `vehicle.yaml` and the L2L example's camera pair.
Per the algebra above, that is **not** equivalent to the legacy hard-coded seed — it is the legacy
seed minus 45°. This is a stronger statement than M-17's "equivalence was never demonstrated": for
the shipped bbox-mode default, the seed provably *changed*. Whether the sample-data calibration still
converges is a separate (empirical) question — it may, because the sample-data board is closer and
denser — but the seed is not the one the legacy path used.

---

## 5. What *does* estimate in-plane rotation — and why it is thrown away

`board_cluster_detector::pose::board_pose` (`rust/board-cluster-detector/src/pose.rs:32-92`) fits a
genuine, data-driven in-plane rotation, and it uses the **diamond** convention:

```rust
// rust/board-cluster-detector/src/pose.rs:54-71
// Board X axis: center -> up-most corner, projected in-plane.
let (top_i, _) = corners_3d_vec.iter().enumerate().fold(...);   // argmax over corner·up
let top = corners_3d_vec[top_i];
let mut x = top - center;
x -= n * x.dot(&n);
x = x.normalize();
let y = n.cross(&x);
```

Board X points at the **up-most corner** — i.e. along a diagonal of the physical square, which is
exactly the "diamond top" (`docs/superpowers/plans/2026-07-29-boarddet-pipeline-finalize.md:271`).
It is computed for every candidate (`detector.rs:194-199`) and used for the stance/isolation gates
and for picking the winner.

But `DetectOutcome` documents it as gate-only — *"`detection` (the square-fit pose) is used here only
for gating/selection"* (`rust/board-cluster-detector/src/detector.rs:54`) — and the node consumes
only the points:

```rust
// ros/lidar_board_detector/src/main.rs:1239-1240
match outcome.selected_points {
    Some(pts) if !pts.is_empty() => Ok(Some(pts)),
```

`outcome.detection` (with its rotation and its four CCW-wound 3D corners) is never read by
`ros/lidar_board_detector/src/main.rs`. So on the `bbox_free` path the pipeline **computes a correct
diamond-oriented pose, discards it, and then re-derives an axis-aligned one from the plane normal
plus a config constant.** Note also the two frames are not the same convention even after the 45°
fix — `board_pose`'s up axis lies along a *diagonal*, `compute_initial_pose_from_plane`'s axes lie
along *edges* — so they cannot be swapped in blindly (see remediation). (Since the 2026-08-13
relabel that up axis is `board_pose`'s Z; this passage said X when written.)

> **Superseded 2026-08-13.** A third, divergent initial pose used to exist in the library, off the
> node's path: `hollow_board_detector::algo::fit_board_icp` seeded `board_x` toward the in-plane
> direction of the sensor origin, had no `initial_inplane_rotation_deg` at all, and placed the pose
> origin at the centroid rather than at a corner. A fourth lived in
> `algo::fit_board_icp_with_iterator`, which seeded ICP from a PCA of the plane inliers plus a
> hardcoded `-135°`. Both functions, all three `Detector::detect*` methods, and
> `detection::FitBoardIcp` have been **deleted** — none had a caller outside the crate's own tests.
> The node was already driving `BoardIcpIterator` itself and calls only `detector.config()` /
> `detector.aruco_pattern()`, so nothing on the production path changed. `Detector` survives as a
> config carrier. The trap this paragraph warned about no longer exists.

---

## 6. Can ICP recover a 45° in-plane error on its own?

Short answer: it is the single worst starting angle, and nothing in the loop is designed to escape it.

- The ICP is plain point-to-model Gauss–Newton-ish: correspondences from
  `BoardModel::find_correspondences` (`hollow-board-config/src/lib.rs:169-412`), Kabsch fit, SLERP
  damping (`rust/hollow-board-detector/src/algo.rs:845-970`). It is strictly **local** — one
  correspondence set, one closed-form update, damped by `icp_damping_factor`
  (`algo.rs:938-946`). There is no multi-start, no annealing, no symmetry search.
- The **square outline is 4-fold symmetric**, so the in-plane error to the nearest symmetric copy is
  at most 45°. A 45° seed sits exactly on the ridge between two attractors: the outline term is
  balanced and provides (to first order) no preferred direction of rotation.
- Correspondences for interior points are the points themselves (`lib.rs:397`, the "no clamp, no
  hole" branch), so they contribute **zero** rotational gradient. Only points near the edges or
  inside the three holes pull. On a sparse far board that is a small minority of points.
- The three holes *do* break the 4-fold symmetry (rotating the layout by 90° maps
  left→top→right→*empty*), and they are honored in the correspondence search
  (`lib.rs:350-398`). But their pull is a second-order effect competing against a saturated,
  symmetric outline term, and it only exists if hole-interior points are present at all.
- The accept gate is `avg_loss < icp_good_fit_threshold` (`main.rs:1561-1563`; 0.035 in both
  presets). A 45°-misaligned square-vs-diamond overlap leaves a large fraction of points outside the
  model, so the mean residual stays well above the ~0.026–0.031 m noise floor and the frame is
  rejected — with the log naming `final_loss > icp_good_fit_threshold` (`main.rs:1690-1702`). That
  is precisely the "detection does not work without 45.0" symptom.
- `icp_outlier_threshold` (0.050 m, `algo.rs:886-895`) also *removes* the very correspondences that
  carry the large-error signal, further weakening any recovery.
- `square_icp_residual_max` (0.45) and `icp_rejection_threshold` are unrelated to this: the former is
  a Stage-1 coverage residual for the fixed-square fit (`bbox_free.rs:35`,
  `board-cluster-detector/src/square_fit.rs`), the latter is an early-*success* exit inside the ICP
  loop (`algo.rs:978-982`).

---

## 7. Is the square's 90° symmetry handled anywhere?

Only *after* ICP, and only as a bookkeeping fixup — never as a search:

```rust
// ros/lidar_board_detector/src/main.rs:1593-1599, 1634-1637
let up_vector = config.sensor_up_axis.as_vector();
let height = |c: &na::Point3<f64>| c.coords.dot(&up_vector);
let (lowest_index, lowest_corner) = corners.iter().enumerate()
    .min_by(|a, b| height(a.1).total_cmp(&height(b.1))).unwrap();
...
let fixup_rotation = {
    let angle = FRAC_PI_2 * lowest_index as f64;
    na::UnitQuaternion::from_axis_angle(&board_normal, angle)
};
```

The converged pose is re-indexed so the lowest corner becomes the model's `bottom_corner`, rotating
the frame by `90° × index`. The ambiguity is acknowledged but only *warned* about
(`main.rs:1601-1622`), and the warning text itself names "a diamond-mounted board" as the case where
the two lowest corners tie. That is the M-14 finding
([`docs/issues/M-14-corner-order-brittle.md`](../../issues/M-14-corner-order-brittle.md)), whose
recommended fix — *"Break the symmetry in the target, not in the code… Score all 4 candidate in-plane
rotations by ICP loss against the asymmetric hole layout"* — is precisely the machinery that would
also make `initial_inplane_rotation_deg` unnecessary. It is still open.

Also worth noting: for a true diamond, `hs[1] - hs[0]` in the warning heuristic
(`main.rs:1609-1621`) is genuinely near zero — the two side corners are at the *same* height — so the
M-14 warning is expected to fire on **every** frame on these rigs, and the 90° fixup's choice between
them is a coin flip driven by noise.

---

## 8. What is unresolved, and how to settle it

The source settles the *mechanism* (§4 is algebra, not inference). Two things it does not settle:

1. **Sign/quadrant on the Seyond rig.** `+45°` is derived above for the `up = +Z`, board-facing-−X
   case; for `sensor_up_axis: "x"` the physical rig has `+X` up and `+Z` forward, and the operator
   arrived at `45.0` empirically. `−45°` (equivalently `+45°` composed with the 90° post-fixup)
   produces a *geometrically identical diamond* — the square is 90°-symmetric — but a **different
   corner labeling**, which matters downstream because the ArUco correspondence maps image corners
   to the model's `bottom/left/right/top` markers. A wrong choice is a silent quarter-turn in the
   extrinsic (M-14's failure mode), not a detection failure. **Experiment:** publish the four ArUco
   marker centers from the LiDAR-side `BoardModel` and compare against the camera's per-ID marker
   centers for the same frame; the labeling that minimizes ID-wise disagreement is the correct sign.
   The H-09 per-pose reprojection residual already in `lctk_quality` makes this measurable without
   new plumbing.
2. **Whether the shipped `board_detector.json5` (0.0) still calibrates.** §4 shows its seed is 45°
   from the legacy one. **Experiment:** run the sample-data bbox path at `0.0` and at `45.0` and
   compare final ICP `avg_loss` and the solved extrinsic; that is also the measurement M-17 asks for.

Cheap instrumentation that would make all of this self-evident: log the angle between the initial
pose's `board_x` and the world up axis, plus the initial `avg_loss` (currently hard-coded to
`f64::INFINITY` at `main.rs:1680`, so the seed's quality is invisible), and keep publishing
`debug/initial_board_marker` (`main.rs:1463-1474`) — in RViz the axis-aligned-vs-diamond mismatch is
visible at a glance.

---

## 9. Remediation options (not implemented)

1. **Document the constant where it is defined.** Change nothing but the comments in
   `hollow-board-detector/src/config.rs:84-88` and the three configs: say that 45° expresses "the
   board is mounted diamond-wise; the model's local axes run along its edges", not "a rig-specific
   RViz bias". *Trade-off:* zero risk, zero robustness gain; leaves the default config at the wrong
   value.
2. **Make the board mounting an explicit enum** (`board_mounting: "diamond" | "axis_aligned"`) that
   maps internally to 45° / 0°. *Trade-off:* still a fixed prior, but it becomes a physical fact
   about the rig rather than a magic number, and it is checkable against the `stance_floor` gate
   already in the same file. Small, safe change.
3. **Seed from the Stage-1 pose that `bbox_free` already computed.** Forward `outcome.detection`
   through `select_board_cluster` and use its rotation. *Trade-off:* removes the constant entirely on
   the bbox_free path and is data-driven per frame — but requires a convention conversion
   (`board_pose`'s X is a *diagonal*; the model's X is an *edge*, so a fixed ∓45° still appears, just
   in code where it can be justified and tested), and it does nothing for the bbox path.
4. **Search the four symmetry-equivalent seeds and keep the best ICP residual.** Run ICP from
   `base + {0, 90, 180, 270}° + offset`, or from `{0, 45, 90, 135}°` if you want mounting-agnostic
   behavior, and keep the lowest `avg_loss`. *Trade-off:* this is M-14's recommended fix and also
   removes the mounting prior; the three-hole layout makes the four outcomes genuinely
   distinguishable. Costs up to 4× ICP (~100 ms → ~400 ms per frame at the profiled rate — real, but
   only when the first seed fails, if implemented as a fallback ladder). Highest robustness payoff.
5. **Estimate the in-plane angle from the hole pattern.** Project the cluster into the plane, locate
   the three hole voids (they are already visible in the projection rasters — phase-7 `:579`), and
   solve the 2D correspondence to the model's three hole centers. *Trade-off:* fully data-driven and
   resolves the 90° ambiguity outright, but needs enough points to see the voids (fails on the sparse
   far board — the very case where the presets already had to lower `icp_min_inlier_points`), so it
   should be a first-choice estimator with option 4 as the fallback.
6. **Redefine the model as a diamond** (local axes along the diagonals, corners at
   `(±d, 0), (0, ±d)`). *Trade-off:* conceptually cleanest and matches every "top/left/right/bottom"
   name in the codebase, but it is the widest blast radius in this repo: `board_plane_point`, the
   `[0,w]²` clamp in `find_correspondences`, `multi_marker_corners`, the two Python
   `_compute_multi_marker_corners` reimplementations (M-14), the detection-file format, and the
   Autoware export all assume the corner-origin `[0,w]²` box. Not worth it unless combined with 4/5.
