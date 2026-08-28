# boarddet Pipeline Finalize Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the three pose-correctness / robustness defects in the `board-detection-2d` runtime path and lock the B/E generators + evidence into shape so the pipeline can be ported into the ROS `lidar_board_detector` node with confidence.

**Architecture:** The detector is a per-frame function `detect(points, board, generator, background) -> DetectOutcome`. This plan (a) adds a NaN/inf sanitize stage, (b) makes `board_pose` orientation-correct (up-axis aware, sensor-facing normal, canonical corner winding), (c) collapses generators B and E onto a single shared cluster-and-gate tail so they differ only in their foreground-extraction front stage, (d) captures the production operating point as a reusable preset, and (e) adds the missing real-data / realistic-sim regression tests that protect the finalize decision.

**Tech Stack:** Python 3, `uv` project, numpy, open3d, opencv, scipy, `velodyne_decoder` (real-pcap ingest), pytest (+ nextest is Rust-only — this experiment is pytest-only).

## Global Constraints

- Work inside `experiments/board-detection-2d/`. Run everything through `uv`: `cd experiments/board-detection-2d && uv run pytest`.
- This experiment is **linted by pytest only** — ruff runs on `ros/` only. Do not add ruff config here.
- **Do NOT change any `BoardConfig` dataclass default.** Many tests are pinned "byte-identical" against the current defaults (`square_icp=False`, `isolation=False`, `stance_floor=0.0`, `flatness_rms_max=0.035`). Production flags go in a *preset* (Task 4), not the defaults.
- VLP-32C range-noise floor is ~0.026–0.031 m measured plane-fit RMS. Every metric gate in meters must sit **above** it (this is the C-04 bug class: a gate below the floor silently accepts nothing). Do not lower `_FLATNESS_RMS_MAX`, `flatness_rms_max`, or ICP thresholds under it.
- `board_pose`'s new `up` parameter must **default to `(0, 0, 1)`** so existing callers and pinned tests stay green.
- Commit after each task with a conventional-commit subject. End every commit message body with:
  `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`
- Current branch is `feat/method-e-background-subtraction`; commit there.

---

### Task 1: NaN/inf sanitize stage

Raw VLP-32C `PointCloud2` carries invalid returns as NaN. `fit_plane`'s SVD propagates a NaN into the plane normal, silently poisoning every downstream projection and pose. The experiment never hit this (cached npz frames are clean); a live ROS frame hits it immediately. Drop non-finite rows at the very front of `detect()`.

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/geometry.py` (add `finite_only`)
- Modify: `experiments/board-detection-2d/src/boarddet/detector.py:104-106` (call it before `downsample`)
- Test: `experiments/board-detection-2d/tests/test_geometry.py`, `experiments/board-detection-2d/tests/test_detector.py`

**Interfaces:**
- Consumes: nothing new.
- Produces: `finite_only(points: np.ndarray) -> np.ndarray` — returns the rows of `points` (an `(N,3)` array) where all three coordinates are finite.

- [ ] **Step 1: Write the failing unit test**

Add to `experiments/board-detection-2d/tests/test_geometry.py`:

```python
def test_finite_only_drops_non_finite_rows():
    from boarddet.geometry import finite_only
    pts = np.array([
        [1.0, 2.0, 3.0],
        [np.nan, 0.0, 0.0],
        [0.0, np.inf, 0.0],
        [4.0, 5.0, 6.0],
        [0.0, 0.0, -np.inf],
    ], dtype=np.float32)
    out = finite_only(pts)
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out, [[1, 2, 3], [4, 5, 6]])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_geometry.py::test_finite_only_drops_non_finite_rows -v`
Expected: FAIL with `ImportError: cannot import name 'finite_only'`.

- [ ] **Step 3: Implement `finite_only`**

Add to `experiments/board-detection-2d/src/boarddet/geometry.py` (near `downsample`):

```python
def finite_only(points: np.ndarray) -> np.ndarray:
    """Drop rows with any non-finite (NaN/inf) coordinate.

    Raw LiDAR PointCloud2 encodes invalid returns as NaN; fit_plane's SVD
    would otherwise propagate a NaN normal and poison every downstream pose.
    """
    points = np.asarray(points)
    if len(points) == 0:
        return points
    return points[np.isfinite(points).all(axis=1)]
```

- [ ] **Step 4: Run the unit test to verify it passes**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_geometry.py::test_finite_only_drops_non_finite_rows -v`
Expected: PASS.

- [ ] **Step 5: Write the failing integration test**

Add to `experiments/board-detection-2d/tests/test_detector.py`:

```python
def test_detect_tolerates_non_finite_points():
    import numpy as np
    from boarddet.board_config import BoardConfig
    from boarddet.detector import detect
    from boarddet.synth import make_scene
    pts, _ = make_scene(rng=np.random.default_rng(0))
    poisoned = np.vstack([pts, np.full((5, 3), np.nan, dtype=pts.dtype)])
    out = detect(poisoned, BoardConfig(), generator="b")
    # A NaN-free run on the same seed detects; the poisoned run must not
    # crash and must not return a NaN pose.
    if out.detection is not None:
        assert np.isfinite(out.detection.center).all()
        assert np.isfinite(out.detection.rotation).all()
```

Note: confirm `make_scene`'s exact return signature in `src/boarddet/synth.py` before running; if it returns only points, drop the `, _`.

- [ ] **Step 6: Run it to verify it fails**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_detector.py::test_detect_tolerates_non_finite_points -v`
Expected: FAIL — either a crash inside SVD or a NaN pose (the bug).

- [ ] **Step 7: Wire `finite_only` into `detect`**

In `experiments/board-detection-2d/src/boarddet/detector.py`, add the import and call it as the first thing in `detect()`. Change:

```python
    t0 = time.perf_counter()
    dn = downsample(points, voxel)
```

to:

```python
    t0 = time.perf_counter()
    points = finite_only(points)
    dn = downsample(points, voxel)
```

and add `finite_only` to the existing geometry import line:

```python
from .geometry import PlaneModel, downsample, finite_only, project_to_plane
```

- [ ] **Step 8: Run both new tests + the full suite**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_geometry.py tests/test_detector.py -v`
Expected: PASS. Then `uv run pytest -q` — expected: all pass (237+ green).

- [ ] **Step 9: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/geometry.py \
        experiments/board-detection-2d/src/boarddet/detector.py \
        experiments/board-detection-2d/tests/test_geometry.py \
        experiments/board-detection-2d/tests/test_detector.py
git commit -m "fix(boarddet): drop non-finite points before plane fit"
```

---

### Task 2: board_pose up-axis aware + sensor-facing normal + canonical winding

Three coupled defects in `board_pose` ([pose.py:21-34](../../../experiments/board-detection-2d/src/boarddet/pose.py)):
1. The board X axis is picked by raw world-Z (`corners_3d[:, 2]`), ignoring `board.up_axis` — wrong by ~90° on a z-forward rig (Seyond Falcon).
2. The plane normal's sign (from SVD) is arbitrary, so the board's facing flips unpredictably; a calibration board normal should point toward the sensor.
3. Corner winding is not canonicalized here, so the ICP path (corners from `fit_fixed_square`, fixed cyclic order) and the non-ICP path (corners CCW-sorted in the scorer) emit different orderings — a consumer pairing these with ArUco corners by index gets a mirrored map depending on which path ran.

Fix all three inside `board_pose` by taking an `up` argument (default `(0,0,1)`), orienting the normal toward the sensor at the origin, choosing X toward the up-most corner along `up`, and re-sorting corners into a single canonical CCW order.

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/pose.py:21-34`
- Modify: `experiments/board-detection-2d/src/boarddet/detector.py:195,222` (pass `up`)
- Test: `experiments/board-detection-2d/tests/test_pose.py`

**Interfaces:**
- Consumes: `PlaneModel` (has `.center`, `.normal`, `.u`, `.v`), `ScoreResult` (has `.corners_2d`, `.score`).
- Produces: `board_pose(plane: PlaneModel, result: ScoreResult, up: np.ndarray = (0.,0.,1.)) -> BoardDetection`. `BoardDetection.rotation` columns are `[board_x, board_y, normal]`; `normal` points toward the sensor origin; `corners_3d` are CCW about `normal` starting from the corner nearest `board_x` (the up-most corner). `_stance` in detector.py still reads diagonals as `corners_3d[2]-corners_3d[0]` / `corners_3d[3]-corners_3d[1]`, which this ordering preserves.

  > **Note (2026-08-13).** This describes the **Python** `board_pose`, which still uses these axis
  > names and is unchanged. The **Rust** port in `board-cluster-detector` was relabelled to REP-103
  > on 2026-08-13 — same vectors, same winding, but its columns are now X forward / Y left / Z up
  > with the sensor-facing normal as −X. Do not assume the axis letters match across the two.

- [ ] **Step 1: Write the failing tests**

Add to `experiments/board-detection-2d/tests/test_pose.py`:

```python
def _square_scoreresult(corners_2d):
    # Minimal ScoreResult carrying only what board_pose reads.
    from boarddet.scorer import ScoreResult
    return ScoreResult(
        score=1.0, corners_2d=np.asarray(corners_2d, dtype=float),
        side_lengths=np.ones(4), fill_ratio=1.0, angle_err_deg=0.0,
        raster=np.zeros((1, 1), dtype=np.uint8), origin=np.zeros(2),
        cell_m=0.02,
    )


def test_board_pose_x_axis_follows_up_axis():
    from boarddet.geometry import PlaneModel
    from boarddet.pose import board_pose
    # Plane in the x=const plane: u=+y, v=+z, normal=+x (points away from
    # sensor at origin -> board_pose must flip it to -x).
    plane = PlaneModel(center=np.array([4.0, 0.0, 0.0]),
                       normal=np.array([1.0, 0.0, 0.0]),
                       u=np.array([0.0, 1.0, 0.0]),
                       v=np.array([0.0, 0.0, 1.0]))
    # A diamond: corners up/down/left/right in (u,v). Up corner is +v.
    corners_2d = np.array([[0.0, 0.7], [0.7, 0.0], [0.0, -0.7], [-0.7, 0.0]])
    det = board_pose(plane, _square_scoreresult(corners_2d),
                     up=np.array([0.0, 0.0, 1.0]))
    # X axis (col 0) must point from center toward the highest (max-z) corner.
    top = det.corners_3d[np.argmax(det.corners_3d @ np.array([0., 0., 1.]))]
    x_expected = top - det.center
    x_expected = x_expected / np.linalg.norm(x_expected)
    assert det.rotation[:, 0] @ x_expected > 0.999
    # Normal (col 2) must face the sensor at the origin: normal . (-center) > 0
    assert det.rotation[:, 2] @ (-det.center) > 0.0


def test_board_pose_uses_given_up_not_world_z():
    from boarddet.geometry import PlaneModel
    from boarddet.pose import board_pose
    # z-forward rig: gravity along +y. Board faces the sensor along +z-ish.
    plane = PlaneModel(center=np.array([0.0, 0.0, 4.0]),
                       normal=np.array([0.0, 0.0, 1.0]),
                       u=np.array([1.0, 0.0, 0.0]),
                       v=np.array([0.0, 1.0, 0.0]))
    corners_2d = np.array([[0.0, 0.7], [0.7, 0.0], [0.0, -0.7], [-0.7, 0.0]])
    det = board_pose(plane, _square_scoreresult(corners_2d),
                     up=np.array([0.0, 1.0, 0.0]))
    up = np.array([0.0, 1.0, 0.0])
    top = det.corners_3d[np.argmax(det.corners_3d @ up)]
    x_expected = top - det.center
    x_expected = x_expected / np.linalg.norm(x_expected)
    assert det.rotation[:, 0] @ x_expected > 0.999


def test_board_pose_winding_is_canonical_ccw():
    from boarddet.geometry import PlaneModel
    from boarddet.pose import board_pose
    plane = PlaneModel(center=np.array([4.0, 0.0, 0.0]),
                       normal=np.array([1.0, 0.0, 0.0]),
                       u=np.array([0.0, 1.0, 0.0]),
                       v=np.array([0.0, 0.0, 1.0]))
    corners_2d = np.array([[0.0, 0.7], [0.7, 0.0], [0.0, -0.7], [-0.7, 0.0]])
    # Same geometry, corners handed in scrambled order:
    scrambled = corners_2d[[2, 0, 3, 1]]
    det_a = board_pose(plane, _square_scoreresult(corners_2d))
    det_b = board_pose(plane, _square_scoreresult(scrambled))
    # Canonical ordering => identical corner sequence regardless of input order.
    np.testing.assert_allclose(det_a.corners_3d, det_b.corners_3d, atol=1e-9)
    # And it is CCW about the (sensor-facing) normal: signed area > 0 in the
    # (board_x, board_y) basis.
    r = det_a.rotation
    xy = (det_a.corners_3d - det_a.center) @ r[:, :2]
    area = 0.5 * np.sum(xy[:, 0] * np.roll(xy[:, 1], -1)
                        - np.roll(xy[:, 0], -1) * xy[:, 1])
    assert area > 0.0
```

- [ ] **Step 2: Run the new tests to verify they fail**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_pose.py -v`
Expected: the three new tests FAIL (current `board_pose` takes no `up`, does not flip the normal, does not canonicalize winding).

- [ ] **Step 3: Rewrite `board_pose`**

Replace the body of `experiments/board-detection-2d/src/boarddet/pose.py`'s `board_pose` with:

```python
def board_pose(plane: PlaneModel, result: ScoreResult,
               up: np.ndarray = (0.0, 0.0, 1.0)) -> BoardDetection:
    up = np.asarray(up, dtype=float)
    up = up / np.linalg.norm(up)
    corners_3d = unproject(result.corners_2d, plane)
    center = corners_3d.mean(axis=0)

    # Orient the plane normal toward the sensor at the origin. SVD fixes the
    # normal only up to sign; a calibration board's normal must face the
    # sensor for a consistent optical-frame convention.
    n = plane.normal / np.linalg.norm(plane.normal)
    if n @ center > 0.0:          # points away from origin -> flip toward it
        n = -n

    # Board X axis: center -> up-most corner (the diamond "top"), projected
    # in-plane. Uses the caller's `up` (world up in the sensor frame), NOT
    # raw world-Z, so a z-forward rig (Falcon) is handled correctly.
    top = corners_3d[np.argmax(corners_3d @ up)]
    x = top - center
    x = x - (x @ n) * n
    x = x / np.linalg.norm(x)
    y = np.cross(n, x)            # right-handed: (x, y, n)

    # Canonical winding: sort corners CCW about n in the (x, y) basis so both
    # the ICP and non-ICP paths emit one consistent ordering for ArUco
    # correspondence. atan2 starts near the +x (up-most) corner.
    rel = corners_3d - center
    ang = np.arctan2(rel @ y, rel @ x)
    order = np.argsort(ang)
    corners_3d = corners_3d[order]

    rotation = np.stack([x, y, n], axis=1)
    return BoardDetection(center=center, rotation=rotation,
                          corners_3d=corners_3d,
                          score=result.score, result=result)
```

- [ ] **Step 4: Pass `up` from `detect` at both call sites**

In `experiments/board-detection-2d/src/boarddet/detector.py`, `up` is already computed at the top of `detect()` (lines 102-103). Update both `board_pose` calls:

- Line ~195 (ICP path): `det = board_pose(cand.plane, refined_res)` → `det = board_pose(cand.plane, refined_res, up)`
- Line ~222 (non-ICP path): `det = board_pose(cand.plane, res)` → `det = board_pose(cand.plane, res, up)`

- [ ] **Step 5: Run the new tests + full suite**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_pose.py -v`
Expected: PASS (including the pre-existing `test_pose_recovers_truth`, which uses `up=(0,0,1)` implicitly and `abs(...)` on the normal, so it stays valid).
Then: `uv run pytest -q`
Expected: all pass. If any detector/scorer test that asserts a *signed* normal direction (not `abs`) fails, that test was relying on the old arbitrary sign — inspect it; the sensor-facing convention is the correct behavior, update the test's expectation to the sensor-facing sign.

- [ ] **Step 6: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/pose.py \
        experiments/board-detection-2d/src/boarddet/detector.py \
        experiments/board-detection-2d/tests/test_pose.py
git commit -m "fix(boarddet): board_pose up-axis, sensor-facing normal, canonical winding"
```

---

### Task 3: Unify B/E onto one shared cluster-and-gate tail

Generators B (`generate_cluster_after_ground`) and E (`generate_background_diff`) differ **only** in how they produce the foreground point set: B strips big planes (RANSAC), E diffs against a prebuilt `BackgroundModel`. Everything after — anisotropic DBSCAN, coplanar-cluster merge, `plausible_board_patch` gate — should be one shared code path. Currently E skips `_merge_coplanar_clusters`, an inconsistency: ring-gap fragmentation survives the background diff too (the E module docstring says so), so E benefits from the same merge. Extract the shared tail and have both call it.

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/candidates/cluster_after_ground.py` (add `_cluster_and_gate`, call it from `generate_cluster_after_ground`)
- Modify: `experiments/board-detection-2d/src/boarddet/candidates/background_diff.py` (call `_cluster_and_gate`)
- Test: `experiments/board-detection-2d/tests/test_candidates_b.py`, `experiments/board-detection-2d/tests/test_candidates_e.py`

**Interfaces:**
- Consumes: `_anisotropic_scaled`, `_merge_coplanar_clusters`, `plausible_board_patch`, `Candidate`.
- Produces: `_cluster_and_gate(fg: np.ndarray, board: BoardConfig, *, cluster_eps: float, cluster_min_points: int, vertical_gap_deg: float, rejects: list | None) -> list[Candidate]` — clusters a foreground set, merges coplanar stripe clusters, and gates each group through `plausible_board_patch`.

- [ ] **Step 1: Write the failing test (E now runs the merge)**

Add to `experiments/board-detection-2d/tests/test_candidates_e.py`:

```python
def test_e_merges_coplanar_stripe_clusters(monkeypatch):
    """E must route through the shared _merge_coplanar_clusters tail (same as
    B), so a board fragmented into ring stripes is merged before gating."""
    import boarddet.candidates.cluster_after_ground as cag
    calls = {"n": 0}
    real = cag._merge_coplanar_clusters

    def spy(*a, **k):
        calls["n"] += 1
        return real(*a, **k)

    monkeypatch.setattr(cag, "_merge_coplanar_clusters", spy)

    import numpy as np
    from boarddet.background import BackgroundModel
    from boarddet.board_config import BoardConfig
    from boarddet.candidates.background_diff import generate_background_diff
    # Board-present frame vs an empty background: board survives the diff.
    bg = BackgroundModel(min_sources=1)
    bg.observe(np.zeros((10, 3)) + 999.0, source="empty")  # far away
    bg.finalize()
    board_pts = _diamond_points(side=1.0, center=np.array([3.0, 0.0, 0.0]))
    generate_background_diff(board_pts, BoardConfig(), background=bg,
                             cluster_min_points=10)
    assert calls["n"] >= 1
```

Note: reuse whatever board-point helper `test_candidates_e.py` already defines (e.g. `_diamond_points` / a replay fixture) rather than the placeholder name above — match the file's existing fixtures.

- [ ] **Step 2: Run it to verify it fails**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_candidates_e.py::test_e_merges_coplanar_stripe_clusters -v`
Expected: FAIL — `_merge_coplanar_clusters` is never called on the E path today (assert `calls["n"] >= 1` fails).

- [ ] **Step 3: Extract the shared tail in `cluster_after_ground.py`**

Add to `experiments/board-detection-2d/src/boarddet/candidates/cluster_after_ground.py`:

```python
def _cluster_and_gate(fg: np.ndarray, board: BoardConfig, *,
                      cluster_eps: float, cluster_min_points: int,
                      vertical_gap_deg: float,
                      rejects: list[RejectReason] | None) -> list[Candidate]:
    """Shared B/E tail: anisotropic DBSCAN -> coplanar-stripe merge -> gate.

    B and E differ only in how `fg` (the foreground point set) is produced;
    from here down they are identical.
    """
    if len(fg) < cluster_min_points:
        return []
    scaled = _anisotropic_scaled(fg.astype(np.float64), cluster_eps,
                                 vertical_gap_deg)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scaled))
    labels = np.asarray(pc.cluster_dbscan(eps=cluster_eps,
                                          min_points=cluster_min_points))
    out: list[Candidate] = []
    for group_pts in _merge_coplanar_clusters(fg, labels, board):
        cand = plausible_board_patch(group_pts, board, rejects=rejects)
        if cand is not None:
            out.append(cand)
    return out
```

Then replace the tail of `generate_cluster_after_ground` (from the `if len(rest) < cluster_min_points` line to the end) with:

```python
    return _cluster_and_gate(
        rest, board, cluster_eps=cluster_eps,
        cluster_min_points=cluster_min_points,
        vertical_gap_deg=vertical_gap_deg, rejects=rejects)
```

- [ ] **Step 4: Route E through the shared tail**

Replace the body of `generate_background_diff` in `experiments/board-detection-2d/src/boarddet/candidates/background_diff.py` (keep the docstring and signature) with:

```python
    fg = background.foreground_points(points)
    from .cluster_after_ground import _cluster_and_gate
    return _cluster_and_gate(
        fg, board, cluster_eps=cluster_eps,
        cluster_min_points=cluster_min_points,
        vertical_gap_deg=vertical_gap_deg, rejects=rejects)
```

Remove the now-unused imports (`open3d as o3d`, `_anisotropic_scaled`) from `background_diff.py` if nothing else uses them.

- [ ] **Step 5: Run the E + B suites**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_candidates_b.py tests/test_candidates_e.py tests/test_detector.py tests/test_detector_e.py -v`
Expected: PASS. The new spy test passes; B behavior is unchanged (it already used the merge). If a pre-existing E test pinned an exact candidate *count* that the added merge changes, verify the new count is correct (merge should only ever join stripe fragments of one physical surface) and update that pinned expectation.

- [ ] **Step 6: Run the full suite**

Run: `cd experiments/board-detection-2d && uv run pytest -q`
Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/candidates/cluster_after_ground.py \
        experiments/board-detection-2d/src/boarddet/candidates/background_diff.py \
        experiments/board-detection-2d/tests/test_candidates_e.py
git commit -m "refactor(boarddet): B and E share one cluster-and-gate tail"
```

---

### Task 4: Production operating-point preset

The finalize decision is "run the pipeline at the recommended operating point." That point is a specific set of flags (`square_icp=True`, `stance_floor=0.9`, `isolation=True`, `flatness_rms_max=0.045`, per-rig `up_axis`), not the `BoardConfig` defaults (which are frozen for byte-identical tests). Capture it as one preset function so the benchmarks, the real-data tests (Task 5), and the eventual ROS port all consume one source of truth instead of re-listing flags.

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/presets.py`
- Test: `experiments/board-detection-2d/tests/test_presets.py`

**Interfaces:**
- Consumes: `BoardConfig`.
- Produces: `production_config(side_m: float = 1.0, up_axis=(0.,0.,1.), cluster_min_points: int = 30) -> BoardConfig` — a `BoardConfig` with the recommended-operating-point flags set.

- [ ] **Step 1: Write the failing test**

Create `experiments/board-detection-2d/tests/test_presets.py`:

```python
def test_production_config_operating_point():
    from boarddet.presets import production_config
    cfg = production_config()
    assert cfg.square_icp is True
    assert cfg.isolation is True
    assert cfg.stance_floor == 0.9
    assert cfg.flatness_rms_max == 0.045
    assert cfg.up_axis == (0.0, 0.0, 1.0)


def test_production_config_per_rig_overrides():
    from boarddet.presets import production_config
    cfg = production_config(side_m=1.2, up_axis=(0.0, 1.0, 0.0),
                            cluster_min_points=20)
    assert cfg.side_m == 1.2
    assert cfg.up_axis == (0.0, 1.0, 0.0)
    assert cfg.cluster_min_points == 20
    # Defaults must be untouched by the preset (regression guard for the
    # "never change BoardConfig defaults" constraint).
    from boarddet.board_config import BoardConfig
    assert BoardConfig().square_icp is False
    assert BoardConfig().isolation is False
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_presets.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'boarddet.presets'`.

- [ ] **Step 3: Create the preset**

Create `experiments/board-detection-2d/src/boarddet/presets.py`:

```python
"""Named BoardConfig presets.

BoardConfig's dataclass defaults are frozen "stage-6 byte-identical" for the
pinned test suite. The production operating point lives here instead, as the
single source of truth the benchmarks, real-data regression tests, and the
ROS port all consume. Rationale for each flag is in
docs/roadmap/phase-7-projection-board-detection.md (recommended operating
point) and docs/roadmap/side-track_method-e-background-subtraction.md.
"""
from __future__ import annotations

from .board_config import BoardConfig


def production_config(side_m: float = 1.0,
                      up_axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
                      cluster_min_points: int = 30) -> BoardConfig:
    """The recommended operating point for real VLP-32C frames.

    - square_icp: fixed-side square fit (raw minAreaRect angle is near-random
      on sparse frames; median error 43 deg -> 7 deg).
    - stance_floor=0.9: reject non-diamond-stance panels.
    - isolation: reject embedded (wall-continuation) clutter.
    - flatness_rms_max=0.045: stage-6 recall recovery, still above the
      ~0.031 m VLP-32C noise floor.
    - up_axis / cluster_min_points: per-rig (z-forward Falcon -> (0,1,0);
      far/sparse board -> 20).
    """
    return BoardConfig(
        side_m=side_m,
        up_axis=up_axis,
        cluster_min_points=cluster_min_points,
        square_icp=True,
        stance_floor=0.9,
        isolation=True,
        flatness_rms_max=0.045,
    )
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_presets.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d/src/boarddet/presets.py \
        experiments/board-detection-2d/tests/test_presets.py
git commit -m "feat(boarddet): production_config operating-point preset"
```

---

### Task 5: Real-pcap per-fold recall-floor regression test

The 88.4% / 100% headline that justifies finalizing Method E currently has **zero** automated protection — every detection test uses synthetic scenes. A refactor could tank real-world recall and the suite stays green. Add a data-gated regression test that runs the real LOO benchmark on the cached sample pcaps and asserts a per-fold recall floor (catches a ds5-class collapse), a pooled recall floor, and a precision floor. This is an environment-gated integration test — it skips **only** when the sample pcaps or `velodyne_decoder` are genuinely absent, with an explicit reason (not a silent skip).

**Files:**
- Create: `experiments/board-detection-2d/tests/test_realdata_recall.py`

**Interfaces:**
- Consumes: `benchmark_e_loo.load_sources`, `benchmark_e_loo.run_loo`, `bbox_ref.load_bbox`, `presets.production_config`, `ingest.DATA_DIR`.
- Produces: nothing (test only).

- [ ] **Step 1: Write the test**

Create `experiments/board-detection-2d/tests/test_realdata_recall.py`:

```python
"""Real-pcap regression floor for the Method E operating point.

Env-gated: skips (with a reason) only when the sample pcaps or
velodyne_decoder are absent. When data is present it MUST run — it is the
only automated guard on the finalize headline number.
"""
import pytest

pytestmark = pytest.mark.realdata


def _have_pcaps():
    try:
        from boarddet.ingest import DATA_DIR
    except Exception:
        return False
    return all((DATA_DIR / n / "lidar.pcap").exists() for n in "12345")


@pytest.mark.skipif(not _have_pcaps(),
                    reason="sample pcaps ros/lctk_sample_data/data/{1..5} absent")
def test_methode_loo_recall_floor(tmp_path):
    pytest.importorskip("velodyne_decoder",
                        reason="velodyne_decoder not installed")
    from boarddet.benchmark_e_loo import load_sources, run_loo
    from boarddet.bbox_ref import load_bbox, Path  # Path re-exported? else import
    from boarddet.presets import production_config

    from boarddet.benchmark_e_loo import DEFAULT_BBOX_PATH
    sources = load_sources("pcap", ["1", "2", "3", "4", "5"],
                           sensor="vlp32", max_frames=40)
    board = production_config()
    summary = run_loo(sources, board, tmp_path, box=load_bbox(DEFAULT_BBOX_PATH),
                      min_sources=3)

    folds = summary["folds"]
    recalls = {k: v["recall"] for k, v in folds.items()}
    # No fold may collapse to near-zero (the ds5-overlap failure mode).
    assert min(recalls.values()) >= 0.35, recalls
    # Pooled recall over all frames must hold the operating-point level.
    total_true = sum(v["n_true_board"] for v in folds.values())
    total_frames = sum(v["n_frames"] for v in folds.values())
    assert total_true / total_frames >= 0.80, recalls
    # Precision: accepted detections should overwhelmingly be true-board.
    total_dets = sum(v["n_detections"] for v in folds.values())
    assert total_dets > 0
    assert total_true / total_dets >= 0.95, {
        "true": total_true, "dets": total_dets}
```

Note: `max_frames=40` keeps the test to tens of seconds. Confirm `load_sources`'s `max_frames` reaches `load_frames` (it does — `load_sources` forwards it). If `from boarddet.bbox_ref import Path` is wrong, import `Path` from `pathlib`; only `DEFAULT_BBOX_PATH` and `load_bbox` are needed from the boarddet modules.

- [ ] **Step 2: Register the `realdata` marker**

Add to `experiments/board-detection-2d/pyproject.toml` under the pytest config (create `[tool.pytest.ini_options]` if absent):

```toml
[tool.pytest.ini_options]
markers = [
    "realdata: integration test that needs the sample pcaps + velodyne_decoder",
]
```

- [ ] **Step 3: Run the test**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_realdata_recall.py -v -m realdata`
Expected on this machine (pcaps shipped in git): PASS. If it FAILS on the recall floor, that is a real signal — the operating point or an upstream task regressed real recall; investigate before proceeding, do not lower the floor to make it pass.

- [ ] **Step 4: Confirm it does not slow the default suite excessively**

Run: `cd experiments/board-detection-2d && uv run pytest -q`
Expected: all pass; note the added wall-time. If the real-data test dominates runtime, document running the fast suite with `-m "not realdata"` in the experiment README.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d/tests/test_realdata_recall.py \
        experiments/board-detection-2d/pyproject.toml
git commit -m "test(boarddet): real-pcap LOO recall-floor regression guard"
```

---

### Task 6: Pose-accuracy test on the realistic raycast sim

"100% precision" in the benchmark is center-inside-a-3.94 m-box, which is blind to pose accuracy — yet calibration consumes the board *corners* for PnP. There is no corner ground truth on real pcaps, so assert pose accuracy on the ray-based sim (`boarddet.sim`), which casts real VLP-32C beams and carries exact truth (`BoardMeta.center`/`.normal`/`.corners`). Build a controlled single-board scene, detect, and assert tight center + normal + corner error.

**Files:**
- Create: `experiments/board-detection-2d/tests/test_sim_pose_accuracy.py`

**Interfaces:**
- Consumes: `sim.make_diamond_board`, `sim.Rect`, `sim.render`, `sim.Vlp32cSensor`, `detector.detect`, `presets.production_config`.
- Produces: nothing (test only).

- [ ] **Step 1: Write the test**

Create `experiments/board-detection-2d/tests/test_sim_pose_accuracy.py`:

```python
import numpy as np
import pytest

from boarddet.detector import detect
from boarddet.presets import production_config
from boarddet.sim import Rect, Vlp32cSensor, make_diamond_board, render


def _scene_with_board(center, normal, side=1.0):
    ground = Rect(center=np.array([0.0, 0.0, -1.2]),
                  normal=np.array([0.0, 0.0, 1.0]),
                  u_axis=np.array([1.0, 0.0, 0.0]),
                  half_u=20.0, half_v=20.0)
    rect, corners = make_diamond_board(center, normal, up_hint=[0.0, 0.0, 1.0],
                                       side=side)
    return [ground, rect], corners


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_sim_pose_accuracy(seed):
    rng = np.random.default_rng(seed)
    sensor = Vlp32cSensor()
    center = np.array([4.0, 0.0, 0.2])
    normal = np.array([-1.0, 0.0, 0.0])  # faces sensor at origin
    scene, truth_corners = _scene_with_board(center, normal, side=1.0)
    frame = render(scene, sensor, range_noise_std=0.01,
                   dropout_grazing=0.1, dropout_random=0.01, rng=rng)

    out = detect(frame.points, production_config(), generator="b")
    assert out.detection is not None, "board not detected in clean sim scene"
    det = out.detection
    # Center within a few cm.
    assert np.linalg.norm(det.center - center) < 0.08
    # Normal aligned (sign-invariant, since sensor-facing vs truth may differ
    # in sign convention).
    truth_n = normal / np.linalg.norm(normal)
    assert abs(det.rotation[:, 2] @ truth_n) > 0.98
    # Every detected corner matches some truth corner within ~10 cm.
    for c in det.corners_3d:
        assert np.linalg.norm(truth_corners - c, axis=1).min() < 0.10
```

- [ ] **Step 2: Run it**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_sim_pose_accuracy.py -v`
Expected: PASS for all three seeds. If detection fails, first confirm the sim board is within `production_config`'s gates (a 4 m, corner-standing diamond passes stance/flatness/extent); if a threshold is genuinely too tight for sim fidelity, widen the *assertion tolerance*, not the detector gate.

- [ ] **Step 3: Commit**

```bash
git add experiments/board-detection-2d/tests/test_sim_pose_accuracy.py
git commit -m "test(boarddet): pose-accuracy floor on the raycast sim"
```

---

### Task 7: Single-session warmup-path test (the real deployment code path)

The 88.4% number comes from *cross-dataset* LOO background construction, which the roadmap doc explicitly flags is **not** the single-session "empty room, then walk the board in" path a production node runs. That deployment path (`observe` board-free frames → `finalize` → `detect` on board-present frames, `min_sources=1`) has no real-data evidence and no test. Add one on the sim: build an empty room (ground + walls + clutter, **no board**), observe several frames as one source, finalize with `min_sources=1`, then detect on a frame with the board revealed.

**Files:**
- Create: `experiments/board-detection-2d/tests/test_background_warmup.py`

**Interfaces:**
- Consumes: `background.BackgroundModel`, `sim.*`, `detector.detect`, `presets.production_config`.
- Produces: nothing (test only).

- [ ] **Step 1: Write the test**

Create `experiments/board-detection-2d/tests/test_background_warmup.py`:

```python
import numpy as np

from boarddet.background import BackgroundModel
from boarddet.detector import detect
from boarddet.presets import production_config
from boarddet.sim import Rect, Vlp32cSensor, make_diamond_board, render


def _empty_room():
    ground = Rect(center=np.array([0.0, 0.0, -1.2]),
                  normal=np.array([0.0, 0.0, 1.0]),
                  u_axis=np.array([1.0, 0.0, 0.0]),
                  half_u=20.0, half_v=20.0)
    wall = Rect(center=np.array([10.0, 0.0, 0.5]),
                normal=np.array([-1.0, 0.0, 0.0]),
                u_axis=np.array([0.0, 1.0, 0.0]),
                half_u=8.0, half_v=3.0)
    clutter = Rect(center=np.array([3.0, 3.0, 0.0]),
                   normal=np.array([-1.0, -1.0, 0.0]) / np.sqrt(2),
                   u_axis=np.array([1.0, -1.0, 0.0]) / np.sqrt(2),
                   half_u=0.5, half_v=0.5)
    return [ground, wall, clutter]


def test_single_session_warmup_then_detect():
    sensor = Vlp32cSensor()
    room = _empty_room()

    # Warm-up: observe several board-FREE frames as one live source.
    bg = BackgroundModel(voxel=0.06, dilation_radius=1, min_sources=1)
    for seed in range(5):
        rng = np.random.default_rng(seed)
        frame = render(room, sensor, range_noise_std=0.01,
                       dropout_random=0.01, rng=rng)
        bg.observe(frame.points, source="live")
    bg.finalize()
    assert bg.n_voxels > 0

    # Reveal the board: same room + a diamond board walked in.
    center = np.array([4.0, 0.0, 0.2])
    rect, _ = make_diamond_board(center, np.array([-1.0, 0.0, 0.0]),
                                 up_hint=[0.0, 0.0, 1.0], side=1.0)
    rng = np.random.default_rng(99)
    frame = render(room + [rect], sensor, range_noise_std=0.01,
                   dropout_random=0.01, rng=rng)

    out = detect(frame.points, production_config(), generator="e", background=bg)
    assert out.detection is not None, "board not found via warmup background"
    assert np.linalg.norm(out.detection.center - center) < 0.15
```

- [ ] **Step 2: Run it**

Run: `cd experiments/board-detection-2d && uv run pytest tests/test_background_warmup.py -v`
Expected: PASS. This proves the deployment path works: a board-free warmup background lets E isolate the revealed board with `min_sources=1` (no self-suppression, because the board was absent during warmup). If it fails to detect, check that the background genuinely suppressed the static room (`bg.n_voxels > 0`) and that the board sits outside those voxels (it is new geometry, so it must).

- [ ] **Step 3: Commit**

```bash
git add experiments/board-detection-2d/tests/test_background_warmup.py
git commit -m "test(boarddet): single-session warmup-then-detect deployment path"
```

---

## Self-Review

**Spec coverage** (against the two-reviewer findings + user corrections):
- R1 #4 NaN filter → Task 1. ✅
- R1 #1/#2/#5/#6 board_pose (up-axis, winding, normal sign) → Task 2. ✅ (R1 #2 90°-flip is handled by `stance_floor` upstream + up-axis argmax; user confirmed diamond stance makes the top corner unambiguous.)
- User correction "B and E differ by one filter branch, let user choose" → Task 3 unifies the shared tail; both remain selectable via `generator=` + `mode` intent. ✅
- R1 #8 `min_score` dead under ICP / operating point → Task 4 preset makes the chosen path explicit. ✅
- R2 #1 headline unprotected → Task 5. ✅
- R2 #3 precision = crop-box, pose accuracy unmeasured → Task 6. ✅
- R2 #2 deployment path unvalidated → Task 7. ✅
- **Deferred (user's call):** R1 #7 surface buried constants into config + json5 loader → postponed to the ROS-merge step (existing `board_detector.json5` is the old hollow-board detector's; it will be rewritten, not field-merged). R2 #4/#5 min_sources geometry rule → documented in the preset; final pin lands with config work. Noted here so the port task picks them up.

**Placeholder scan:** Each code step carries real code. Two spots depend on names in existing files (Task 1 `make_scene` signature; Task 3 the E test's board-point fixture) — both are flagged with an explicit "confirm the real name" note rather than left as a silent guess.

**Type consistency:** `finite_only(points)->ndarray` (Task 1) used in detector.py. `board_pose(plane, result, up=(0,0,1))` (Task 2) called with `up` at both detector.py sites. `_cluster_and_gate(fg, board, *, cluster_eps, cluster_min_points, vertical_gap_deg, rejects)` (Task 3) called identically from B and E. `production_config(side_m, up_axis, cluster_min_points)` (Task 4) consumed by Tasks 5/6/7. `run_loo(sources, board, out_dir, *, box, ..., min_sources)` matches `benchmark_e_loo.py`'s real signature. `BackgroundModel(voxel, dilation_radius, min_sources)` + `.observe(points, source)` + `.finalize()` + `.n_voxels` match `background.py`. `render(scene, sensor, range_noise_std, dropout_grazing, dropout_random, rng)` and `make_diamond_board(center, normal, up_hint, side, ...)` and `Rect(center, normal, u_axis, half_u, half_v, holes=...)` match `sim/`.

## ROS-Port Carry-Forward Notes

Executed via subagent-driven-development; final whole-branch review clean (0 Critical, 0 Important). Two non-blocking items surfaced for the eventual ROS port — capture them when this pipeline is ported into `ros/lidar_board_detector/`:

1. **Corner-ordering seam:** `board_pose` canonicalizes `corners_3d` to CCW winding, but `det.result.corners_2d` keeps the scorer/fit order. Nothing in the experiment pairs them by index, but the ArUco correspondence in the port MUST take corners from `corners_3d` — never `zip(corners_3d[i], result.corners_2d[i])`.
2. **Far-board density knob:** `production_config()` defaults `cluster_min_points=30` (correct for the near ~2 m pcap board). A far/sparse board (the ~9 m VLP bag) needs `production_config(cluster_min_points=20)` at the call site — the default does not auto-adapt. Fold this into the per-rig config surfacing (the deferred R1 #7 constants work).

Also still deferred to the port (per user's decision): surface the ~20 buried tuning constants into `board_detector.json5` + a `BoardConfig` loader (that file today describes the old hollow-board ICP detector and will be rewritten, not field-merged), and pin `min_sources` to a documented capture-geometry rule.
