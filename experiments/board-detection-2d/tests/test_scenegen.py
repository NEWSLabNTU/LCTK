"""Tests for domain-randomized scene generation (Task 30): diamond board
orientation (review must-fix #2), board count, and the embedded/
free-standing clutter mix that teaches a CNN the isolation discriminator.

Task 35 adds tests for the broadened free-standing distractor population
(small scatter clusters, large free-standing structures, non-square/
arbitrary-orientation panels) -- see `SceneGenConfig`'s "Task 35" knobs and
`random_scene`'s scatter-cluster / large-clutter blocks in scenegen.py."""
from __future__ import annotations

import numpy as np

from boarddet.sim.primitives import Box, Cylinder, Rect
from boarddet.sim.raycast import render
from boarddet.sim.scenegen import (
    Scene,
    SceneGenConfig,
    make_diamond_board,
    random_scene,
)
from boarddet.sim.sensor import Vlp32cSensor

# ---------------------------------------------------------------------------
# diamond orientation (review must-fix #2)
# ---------------------------------------------------------------------------


def test_diamond_board_corners_point_up_down_left_right():
    """At zero extra in-plane jitter, a diamond board's 4 corners must sit
    along the plane's own up/down/left/right axes (v, -v, u, -u) at the
    diagonal distance side/sqrt(2) -- a diamond, not an axis-aligned
    square whose corners would sit at 45 deg off those axes instead."""
    center = np.array([3.0, 0.0, 0.2])
    normal = np.array([-1.0, 0.0, 0.0])
    up_hint = np.array([0.0, 0.0, 1.0])
    side = 1.0
    rect, corners = make_diamond_board(center, normal, up_hint, side,
                                       inplane_rot_rad=0.0)
    half_diag = side / np.sqrt(2.0)
    u, v, n = rect.u_axis, rect.v_axis, rect.normal
    # recover the UN-rotated (u, v) used to build the 45-deg diamond stance
    # by rotating back -45 deg
    u0 = np.cos(-np.pi / 4) * u + np.sin(-np.pi / 4) * v
    v0 = np.cos(-np.pi / 4) * v - np.sin(-np.pi / 4) * u
    expected = np.stack([
        center + half_diag * v0,   # top
        center + half_diag * u0,   # right
        center - half_diag * v0,   # bottom
        center - half_diag * u0,   # left
    ])
    # corners are returned in (+u+v, +u-v, -u-v, -u+v) order == (top, right,
    # bottom, left) at inplane_rot_rad=0 (see make_diamond_board docstring)
    assert np.allclose(corners, expected, atol=1e-9)
    # diagonal (vertical) extent = side*sqrt(2), not the side length itself
    vertical_extent = float((corners[0] - corners[2]) @ v0)
    assert np.isclose(vertical_extent, side * np.sqrt(2.0), atol=1e-9)
    assert np.isclose(np.linalg.norm(n), 1.0)


def test_diamond_board_rect_is_axis_aligned_square_in_its_own_rotated_frame():
    """The Rect primitive itself stays a plain axis-aligned square in ITS
    OWN (u, v) frame (half_u == half_v == side/2); "diamond" is purely the
    45-deg rotation of that frame relative to world up/right."""
    rect, _ = make_diamond_board(center=(2.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
                                 up_hint=(0.0, 0.0, 1.0), side=1.2)
    assert np.isclose(rect.half_u, 0.6)
    assert np.isclose(rect.half_v, 0.6)


def test_diamond_board_hollow_has_four_holes_matching_real_board_geometry():
    rect, _ = make_diamond_board(center=(2.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
                                 up_hint=(0.0, 0.0, 1.0), side=1.0, hollow=True,
                                 hole_radius=0.15, hole_shift=0.2)
    assert len(rect.holes) == 4
    for (_, radius) in rect.holes:
        assert np.isclose(radius, 0.15)


def test_diamond_board_plain_by_default_has_no_holes():
    rect, _ = make_diamond_board(center=(2.0, 0.0, 0.0), normal=(-1.0, 0.0, 0.0),
                                 up_hint=(0.0, 0.0, 1.0), side=1.0)
    assert rect.holes == []


# ---------------------------------------------------------------------------
# random_scene: board count + geometry sanity
# ---------------------------------------------------------------------------


def test_random_scene_produces_requested_board_count_range():
    rng = np.random.default_rng(0)
    cfg = SceneGenConfig()
    counts = {len(random_scene(rng, cfg).boards) for _ in range(60)}
    assert counts.issubset({0, 1, 2, 3})
    assert len(counts) > 1  # domain randomization actually varies the count


def test_random_scene_board_prim_index_matches_placed_rect():
    rng = np.random.default_rng(3)
    scene = random_scene(rng, SceneGenConfig())
    for board in scene.boards:
        prim = scene.primitives[board.prim_index]
        assert np.allclose(prim.center, board.center)
        assert np.allclose(prim.normal, board.normal)


def test_random_scene_boards_have_diamond_diagonal_extent():
    rng = np.random.default_rng(4)
    cfg = SceneGenConfig(board_side_m=1.0, board_side_jitter_frac=0.0,
                        board_inplane_rot_jitter_deg=0.0)
    scene = random_scene(rng, cfg)
    for board in scene.boards:
        diag = np.linalg.norm(board.corners[0] - board.corners[2])
        assert np.isclose(diag, np.sqrt(2.0), atol=1e-6)


def test_random_scene_returns_a_scene_instance():
    rng = np.random.default_rng(5)
    scene = random_scene(rng, SceneGenConfig())
    assert isinstance(scene, Scene)
    assert len(scene.primitives) > 0


# ---------------------------------------------------------------------------
# clutter mix: embedded (coplanar-with-wall) vs free-standing
# ---------------------------------------------------------------------------


def test_random_scene_clutter_has_both_embedded_and_free_standing():
    rng = np.random.default_rng(6)
    cfg = SceneGenConfig(n_clutter_range=(2, 6))
    saw_embedded = False
    saw_free = False
    for _ in range(30):
        scene = random_scene(rng, cfg)
        for c in scene.clutter:
            if c.embedded:
                saw_embedded = True
            else:
                saw_free = True
    assert saw_embedded, "no embedded (coplanar-with-wall) clutter ever generated"
    assert saw_free, "no free-standing clutter ever generated"


def test_random_scene_single_scene_has_the_mix_when_clutter_count_allows():
    """The mix must be per-scene, not just across many scenes -- guaranteed
    at least one of each when n_clutter >= 2 (see random_scene)."""
    rng = np.random.default_rng(7)
    cfg = SceneGenConfig(n_clutter_range=(4, 4))
    for _ in range(10):
        scene = random_scene(rng, cfg)
        embedded = [c for c in scene.clutter if c.embedded]
        free = [c for c in scene.clutter if not c.embedded]
        assert len(embedded) >= 1
        assert len(free) >= 1


def test_embedded_clutter_is_actually_coplanar_with_its_wall():
    rng = np.random.default_rng(8)
    cfg = SceneGenConfig(n_clutter_range=(4, 4), clutter_embedded_frac=1.0)
    scene = random_scene(rng, cfg)
    for c in scene.clutter:
        if not c.embedded:
            continue
        wall = scene.primitives[c.wall_index]
        panel = scene.primitives[c.prim_index]
        assert np.allclose(panel.normal, wall.normal, atol=1e-9)
        # panel center must lie exactly on the wall's plane
        offset = (panel.center - wall.center) @ wall.normal
        assert abs(offset) < 1e-9


def test_free_standing_clutter_is_not_on_any_wall_plane():
    rng = np.random.default_rng(9)
    cfg = SceneGenConfig(n_clutter_range=(4, 4), clutter_embedded_frac=0.0)
    scene = random_scene(rng, cfg)
    walls = [p for p in scene.primitives if getattr(p, "half_u", None)
            == cfg.wall_half_width_m]
    for c in scene.clutter:
        assert not c.embedded
        panel = scene.primitives[c.prim_index]
        for wall in walls:
            offset = (panel.center - wall.center) @ wall.normal
            assert abs(offset) > 1e-3


# ---------------------------------------------------------------------------
# rendering sanity: boards actually produce a coherent hit region
# ---------------------------------------------------------------------------


def test_random_scene_renders_and_boards_get_hit_pixels():
    rng = np.random.default_rng(11)
    sensor = Vlp32cSensor()
    scene = random_scene(rng, SceneGenConfig(board_count_weights={3: 1.0}))
    assert scene.boards, "forced-3-board config produced no boards"
    frame = render(scene.primitives, sensor, azimuth_steps=720,
                  rng=np.random.default_rng(12))
    for board in scene.boards:
        mask = frame.hit_prim_id == board.prim_index
        assert mask.sum() > 0, "a generated board produced zero hit pixels"


def test_board_count_distribution_tracks_weights():
    """Scene-count distribution roughly follows board_count_weights, and
    0-board (empty) scenes really occur for the precision anchor."""
    cfg = SceneGenConfig()  # {0:.15, 1:.45, 2:.25, 3:.15}
    counts = [len(random_scene(np.random.default_rng(s), cfg).boards)
              for s in range(800)]
    frac = {k: counts.count(k) / len(counts) for k in (0, 1, 2, 3)}
    assert frac[0] > 0.05, "no empty scenes -- precision anchor missing"
    assert frac[1] == max(frac.values()), "1-board should be the dominant case"
    # loose bounds around the configured weights (sampling + min-sep dropouts)
    assert 0.08 < frac[0] < 0.24
    assert 0.35 < frac[1] < 0.55
    assert counts.count(4) == 0, "board count must be capped at 3"


def test_boards_never_overlap_in_range_image():
    """No two boards' azimuth-column extents overlap in the rendered range
    image (extent-aware separation), so they never merge into one ambiguous
    blob. This is the guarantee that matters -- checked on the actual render,
    not just center angles, since a near board subtends more azimuth."""
    sensor = Vlp32cSensor()
    cfg = SceneGenConfig(board_count_weights={3: 1.0})  # force crowding
    checked = 0
    for seed in range(250):
        scene = random_scene(np.random.default_rng(seed), cfg)
        if len(scene.boards) < 2:
            continue
        checked += 1
        sf = render(scene.primitives, sensor, azimuth_steps=1800,
                   rng=np.random.default_rng(seed + 1))
        spans = []
        for b in scene.boards:
            m = sf.hit_prim_id == b.prim_index
            if m.any():
                spans.append((int(sf.cols[m].min()), int(sf.cols[m].max())))
        for i in range(len(spans)):
            for j in range(i + 1, len(spans)):
                lo = max(spans[i][0], spans[j][0])
                hi = min(spans[i][1], spans[j][1])
                assert lo > hi, f"boards overlap in range image: {spans[i]} vs {spans[j]}"
    assert checked > 50  # actually exercised multi-board scenes


def test_empty_scene_renders_without_boards():
    """A 0-board scene still renders a valid range image (clutter only)."""
    from boarddet.sim.dataset import render_labeled_scene, DatasetConfig
    sensor = Vlp32cSensor()
    cfg = DatasetConfig(scenegen=SceneGenConfig(board_count_weights={0: 1.0}),
                        azimuth_steps=720)
    sample = render_labeled_scene(np.random.default_rng(3), sensor, cfg)
    assert sample.boards == []
    assert np.isfinite(sample.image[..., 0]).any(), "empty scene has no returns"


# ---------------------------------------------------------------------------
# physical placement: boards always FACE the sensor, never edge-on (thin bar)
# ---------------------------------------------------------------------------


def test_boards_always_face_sensor_never_edge_on():
    """Every generated board's incidence-from-face-on must stay under the
    hard safety cap, so the LiDAR never sees the board edge-on as a thin bar.
    Guards the facing-with-tilt invariant against future config drift."""
    cfg = SceneGenConfig()
    worst = 0.0
    for seed in range(300):
        scene = random_scene(np.random.default_rng(seed), cfg)
        for b in scene.boards:
            ray = b.center / np.linalg.norm(b.center)   # sensor -> board
            n = b.normal / np.linalg.norm(b.normal)
            incidence = np.degrees(np.arccos(np.clip(abs(float(ray @ n)), 0.0, 1.0)))
            worst = max(worst, incidence)
            assert incidence <= cfg.board_max_incidence_deg + 1e-6, (
                f"board edge-on risk: incidence {incidence:.1f} deg "
                f"exceeds cap {cfg.board_max_incidence_deg}")
    # sanity: realized tilt tracks the configured jitter (a few deg of slack
    # for the sensor height/tilt offset shifting the actual viewing ray), and
    # is clearly non-flat.
    assert worst <= cfg.board_normal_jitter_deg + 5.0
    assert worst > 10.0, "boards are barely tilted; expected visible facing-tilt"


def test_board_max_incidence_is_safely_below_edge_on():
    """The safety cap itself must stay well short of 90 deg (edge-on)."""
    assert SceneGenConfig().board_max_incidence_deg < 75.0


# ---------------------------------------------------------------------------
# vertical laser coverage: VLP-32C thins toward FOV edges, so every board must
# be placed where laser channels reach >= board_min_vertical_coverage of it
# ---------------------------------------------------------------------------


def test_boards_get_enough_vertical_laser_coverage():
    """Every generated board is placed so that laser channels span at least
    board_min_vertical_coverage of its vertical extent, with >= the minimum
    ring count -- guards against boards in the sparse top/bottom of the FOV."""
    sensor = Vlp32cSensor()
    el = np.degrees(sensor.elevations)
    cfg = SceneGenConfig()
    n_checked = 0
    for seed in range(300):
        scene = random_scene(np.random.default_rng(seed), cfg, sensor)
        for b in scene.boards:
            n_checked += 1
            e = np.degrees(np.arctan2(b.corners[:, 2],
                                      np.hypot(b.corners[:, 0], b.corners[:, 1])))
            top, bot = float(e.max()), float(e.min())
            in_band = el[(el >= bot) & (el <= top)]
            assert len(in_band) >= cfg.board_min_laser_rings
            coverage = (in_band.max() - in_band.min()) / (top - bot)
            assert coverage >= cfg.board_min_vertical_coverage - 1e-9, (
                f"board vertical coverage {coverage:.2f} < "
                f"{cfg.board_min_vertical_coverage}")
    assert n_checked > 200


# ---------------------------------------------------------------------------
# Task 35: diverse free-standing distractors -- small scatter, large
# structures, non-square/arbitrary orientation
# ---------------------------------------------------------------------------


def _rect_diag(rect: Rect) -> float:
    return 2.0 * float(np.hypot(rect.half_u, rect.half_v))


def _box_diag(box: Box) -> float:
    return 2.0 * float(np.linalg.norm(box.half_sizes))


def _cylinder_extent(cyl: Cylinder) -> float:
    return float(np.hypot(cyl.height, 2.0 * cyl.radius))


def _incidence_deg(prim) -> float:
    """Angle between the primitive's own normal/axis and the sensor-facing
    ray to its center -- 0 deg = face-on (like the board), 90 deg = edge-on.
    Only meaningful for planar Rects (what "facing" means for a Box/Cylinder
    is ambiguous, so callers only use this on Rect clutter)."""
    center = prim.center
    ray = center / np.linalg.norm(center)
    return float(np.degrees(np.arccos(np.clip(abs(float(ray @ prim.normal)), 0.0, 1.0))))


def test_scatter_clusters_produce_small_free_standing_poles_and_panels():
    """Forcing exactly one scatter cluster of exactly 5 members must yield
    a mix of thin `Cylinder` poles and small `Rect` panels, all within the
    configured small (0.03-1.5 m) size range -- the pole/bracket real FP
    population (Stage-10: 28% of the CNN's clutter, ~0.2-0.5 m)."""
    rng = np.random.default_rng(20)
    cfg = SceneGenConfig(
        board_count_weights={0: 1.0}, n_clutter_range=(0, 0),
        n_boxes_range=(0, 0), n_cylinders_range=(0, 0),
        n_large_clutter_range=(0, 0),
        n_scatter_clusters_range=(1, 1), scatter_cluster_size_range=(5, 5),
    )
    saw_pole = False
    saw_panel = False
    for _ in range(30):
        scene = random_scene(rng, cfg)
        for prim in scene.primitives:
            if isinstance(prim, Cylinder):
                saw_pole = True
                assert cfg.scatter_pole_radius_range_m[0] - 1e-9 <= prim.radius \
                    <= cfg.scatter_pole_radius_range_m[1] + 1e-9
                assert cfg.scatter_pole_height_range_m[0] - 1e-9 <= prim.height \
                    <= cfg.scatter_pole_height_range_m[1] + 1e-9
            elif isinstance(prim, Rect) and getattr(prim, "half_u", None) not in \
                    (cfg.ground_half_extent_m, cfg.wall_half_width_m):
                saw_panel = True
                diag = _rect_diag(prim)
                assert diag < 1.0, f"scatter panel diag {diag} unexpectedly large"
    assert saw_pole, "no scatter poles (Cylinder) generated"
    assert saw_panel, "no scatter panels (Rect) generated"


def test_large_clutter_produces_big_free_standing_panels_and_boxes():
    """Forcing large-clutter panels must yield diagonals well into the
    1.5-3 m range (the 1.71 m-median "other_clutter" real FP population,
    Stage-10: 72% of the CNN's clutter) -- previously uncovered by the
    narrower panel/box ranges."""
    rng = np.random.default_rng(21)
    cfg = SceneGenConfig(
        board_count_weights={0: 1.0}, n_clutter_range=(0, 0),
        n_boxes_range=(0, 0), n_cylinders_range=(0, 0),
        n_scatter_clusters_range=(0, 0),
        n_large_clutter_range=(1, 1), large_clutter_panel_frac=1.0,
    )
    diags = []
    for _ in range(30):
        scene = random_scene(rng, cfg)
        for c in scene.clutter:
            if c.kind == "large_panel":
                diags.append(_rect_diag(scene.primitives[c.prim_index]))
    assert len(diags) > 20
    assert max(diags) > 1.5, f"large panels never exceeded 1.5 m diag: max={max(diags)}"
    assert min(diags) >= 2.0 * cfg.large_panel_half_extent_range_m[0] / \
        cfg.clutter_max_aspect_ratio - 1e-6


def test_large_clutter_boxes_are_big():
    rng = np.random.default_rng(22)
    cfg = SceneGenConfig(
        board_count_weights={0: 1.0}, n_clutter_range=(0, 0),
        n_boxes_range=(0, 0), n_cylinders_range=(0, 0),
        n_scatter_clusters_range=(0, 0),
        n_large_clutter_range=(1, 1), large_clutter_panel_frac=0.0,
    )
    saw_large_box = False
    for _ in range(20):
        scene = random_scene(rng, cfg)
        for prim in scene.primitives:
            if isinstance(prim, Box) and _box_diag(prim) > 1.0:
                saw_large_box = True
                assert np.all(prim.half_sizes >= cfg.large_box_half_size_range_m[0] - 1e-9)
    assert saw_large_box, "no large free-standing boxes generated"


def test_new_distractor_panels_have_varied_non_square_aspect_ratio():
    """Scatter/large panels must include genuinely non-square rectangles
    (half_u != half_v, ratio up to ~clutter_max_aspect_ratio), unlike the
    board which is always a plain square diamond."""
    rng = np.random.default_rng(23)
    cfg = SceneGenConfig(
        board_count_weights={0: 1.0}, n_clutter_range=(0, 0),
        n_boxes_range=(0, 0), n_cylinders_range=(0, 0),
        n_scatter_clusters_range=(2, 2), scatter_cluster_size_range=(4, 4),
        scatter_pole_frac=0.0,  # force panels, not poles, for this check
        n_large_clutter_range=(1, 1), large_clutter_panel_frac=1.0,
    )
    ratios = []
    for _ in range(15):
        scene = random_scene(rng, cfg)
        for c in scene.clutter:
            if c.kind in ("scatter_panel", "large_panel"):
                r = scene.primitives[c.prim_index]
                lo, hi = sorted((r.half_u, r.half_v))
                ratios.append(hi / lo)
    assert len(ratios) > 20
    assert max(ratios) > 1.5, "no meaningfully non-square panels generated"
    assert min(ratios) < 1.3, "no near-square panels generated -- aspect should vary"


def test_new_distractor_panels_are_not_all_sensor_facing():
    """Unlike the board (capped at board_max_incidence_deg from face-on),
    the new scatter/large panels must include high-incidence (edge-on/
    tilted-away) orientations -- confirms they are NOT sensor-facing
    constrained."""
    rng = np.random.default_rng(24)
    cfg = SceneGenConfig(
        board_count_weights={0: 1.0}, n_clutter_range=(0, 0),
        n_boxes_range=(0, 0), n_cylinders_range=(0, 0),
        scatter_pole_frac=0.0,
        n_scatter_clusters_range=(2, 2), scatter_cluster_size_range=(4, 4),
        n_large_clutter_range=(1, 1), large_clutter_panel_frac=1.0,
    )
    incidences = []
    for _ in range(15):
        scene = random_scene(rng, cfg)
        for c in scene.clutter:
            if c.kind in ("scatter_panel", "large_panel"):
                incidences.append(_incidence_deg(scene.primitives[c.prim_index]))
    assert len(incidences) > 20
    assert max(incidences) > 70.0, (
        "no near-edge-on panels generated -- orientation looks constrained")


def test_clutter_meta_kind_tags_cover_new_distractor_types():
    """`ClutterMeta.kind` distinguishes the pre-existing wall/free panels
    from the Task-35 scatter/large panels, for downstream bucketing."""
    rng = np.random.default_rng(25)
    cfg = SceneGenConfig(
        board_count_weights={0: 1.0},
        n_clutter_range=(4, 4), clutter_embedded_frac=0.5,
        n_scatter_clusters_range=(1, 2), scatter_cluster_size_range=(3, 5),
        scatter_pole_frac=0.0,
        n_large_clutter_range=(1, 2), large_clutter_panel_frac=1.0,
    )
    kinds = set()
    for _ in range(20):
        scene = random_scene(rng, cfg)
        for c in scene.clutter:
            kinds.add(c.kind)
    assert kinds == {"wall_panel", "free_panel", "scatter_panel", "large_panel"}


def test_enriched_clutter_size_spans_small_to_large():
    """With default config, a batch of scenes' Rect/Box/Cylinder clutter
    sizes must span from small (<0.5 m) to large (>1.5 m) -- the real FP
    range Stage-10 diagnosed (0.2-1.71 m+), vs the old narrow panel/box/
    cylinder-only range."""
    cfg = SceneGenConfig()
    sizes = []
    for seed in range(60):
        scene = random_scene(np.random.default_rng(seed), cfg)
        board_prim_idx = {b.prim_index for b in scene.boards}
        for i, prim in enumerate(scene.primitives):
            if i in board_prim_idx:
                continue
            if isinstance(prim, Rect) and getattr(prim, "half_u", None) == \
                    cfg.ground_half_extent_m:
                continue  # skip the ground plane
            if isinstance(prim, Rect) and getattr(prim, "half_u", None) == \
                    cfg.wall_half_width_m:
                continue  # skip walls
            if isinstance(prim, Rect):
                sizes.append(_rect_diag(prim))
            elif isinstance(prim, Box):
                sizes.append(_box_diag(prim))
            elif isinstance(prim, Cylinder):
                sizes.append(_cylinder_extent(prim))
    assert len(sizes) > 50
    assert min(sizes) < 0.5, f"no small (<0.5m) clutter: min={min(sizes):.2f}"
    assert max(sizes) > 1.5, f"no large (>1.5m) clutter: max={max(sizes):.2f}"
