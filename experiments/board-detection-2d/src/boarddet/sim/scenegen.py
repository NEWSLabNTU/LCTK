"""Domain-randomized scene generation for the ray-based simulator (Task 30).

Builds a `Scene` (a list of `boarddet.sim.primitives` ready for
`raycast.render`, plus board/clutter metadata for labeling and tests):
a ground plane, 2-4 walls, 1-3 DIAMOND calibration boards (see the module
docstring in `range_image.py` and review must-fix #2 -- the recorded board
is a 45-deg-rotated diamond, `side*sqrt(2)` diagonal, NOT an axis-aligned
square), a mix of embedded (coplanar-with-a-wall) and free-standing panel
clutter (so a CNN trained on this data learns the isolation discriminator
Stage 8 identified, not just "flat surface = board"), boxes, and cylinders.
Sensor tilt/height are randomized per scene by transforming every primitive
from a convenient "nominal, level" frame into the actual (tilted, raised)
sensor frame before it's handed to `raycast.render` (which always casts
from the origin).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .primitives import Box, Cylinder, Rect

# ---------------------------------------------------------------------------
# metadata
# ---------------------------------------------------------------------------


@dataclass
class BoardMeta:
    """A generated diamond board's pose + geometry, in the SAME (sensor)
    frame as the primitive handed to raycast.render -- everything a label
    generator or a train/eval script needs, without re-deriving it."""

    prim_index: int
    center: np.ndarray    # (3,)
    normal: np.ndarray    # (3,) unit
    u_axis: np.ndarray    # (3,) unit -- the diamond's own (rotated) edge direction
    v_axis: np.ndarray    # (3,) unit
    side: float           # square side length (diagonal extent = side*sqrt(2))
    corners: np.ndarray   # (4,3): the 4 diamond tips, in (+u+v, +u-v, -u-v, -u+v) order
    hollow: bool


@dataclass
class ClutterMeta:
    """A generated clutter panel's provenance: embedded (coplanar with an
    existing wall -- indistinguishable from that wall by depth alone, so
    a CNN must learn isolation, not just "flat surface") vs free-standing
    (isolated in space, like a real board)."""

    prim_index: int
    embedded: bool
    wall_index: int | None  # index into scene.primitives of the wall it's on, else None


@dataclass
class Scene:
    primitives: list           # ready for raycast.render(scene.primitives, sensor, ...)
    boards: list[BoardMeta]
    clutter: list[ClutterMeta]
    sensor_roll_rad: float
    sensor_pitch_rad: float
    sensor_height_offset_m: float


@dataclass
class SceneGenConfig:
    # ground
    ground_z_range_m: tuple[float, float] = (-1.5, -1.0)
    ground_half_extent_m: float = 20.0
    # walls
    n_walls_range: tuple[int, int] = (2, 4)
    wall_distance_range_m: tuple[float, float] = (6.0, 14.0)
    wall_half_width_m: float = 8.0
    wall_half_height_m: float = 3.0
    wall_center_z_m: float = 0.5
    # boards
    n_boards_range: tuple[int, int] = (1, 3)
    board_range_m: tuple[float, float] = (2.0, 8.0)
    board_azimuth_range_deg: tuple[float, float] = (-75.0, 75.0)
    board_height_range_m: tuple[float, float] = (-0.4, 0.6)
    board_side_m: float = 1.0
    board_side_jitter_frac: float = 0.1
    board_normal_jitter_deg: float = 20.0
    board_inplane_rot_jitter_deg: float = 15.0
    board_hollow_prob: float = 0.3
    board_hole_radius_m: float = 0.15
    board_hole_shift_m: float = 0.20
    # clutter panels: MIX of embedded (coplanar-with-wall) + free-standing
    n_clutter_range: tuple[int, int] = (2, 6)
    clutter_embedded_frac: float = 0.5
    clutter_panel_half_extent_range_m: tuple[float, float] = (0.2, 0.8)
    clutter_free_range_m: tuple[float, float] = (2.0, 10.0)
    clutter_free_azimuth_range_deg: tuple[float, float] = (-160.0, 160.0)
    clutter_free_height_range_m: tuple[float, float] = (-0.8, 1.0)
    # boxes / cylinders
    n_boxes_range: tuple[int, int] = (0, 3)
    box_half_size_range_m: tuple[float, float] = (0.15, 0.5)
    n_cylinders_range: tuple[int, int] = (0, 3)
    cylinder_radius_range_m: tuple[float, float] = (0.08, 0.25)
    cylinder_height_range_m: tuple[float, float] = (0.8, 2.0)
    clutter_range_m: tuple[float, float] = (2.0, 10.0)
    clutter_azimuth_range_deg: tuple[float, float] = (-160.0, 160.0)
    clutter_height_range_m: tuple[float, float] = (-0.8, 1.0)
    # sensor pose randomization
    sensor_tilt_jitter_deg: float = 3.0
    sensor_height_jitter_m: float = 0.1


# ---------------------------------------------------------------------------
# small geometry helpers
# ---------------------------------------------------------------------------


def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    return v / np.linalg.norm(v)


def _rodrigues(v: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    return (v * np.cos(angle) + np.cross(axis, v) * np.sin(angle)
           + axis * (axis @ v) * (1.0 - np.cos(angle)))


def _jitter_direction(rng: np.random.Generator, direction: np.ndarray,
                      jitter_deg: float) -> np.ndarray:
    """Rotate `direction` by a random angle in [0, jitter_deg] about a
    random axis perpendicular to it -- a small random cone jitter."""
    direction = _unit(direction)
    if jitter_deg <= 0.0:
        return direction
    arbitrary = np.array([1.0, 0.0, 0.0])
    if abs(direction @ arbitrary) > 0.9:
        arbitrary = np.array([0.0, 1.0, 0.0])
    axis0 = _unit(np.cross(direction, arbitrary))
    spin = rng.uniform(0.0, 2.0 * np.pi)
    axis = _unit(np.cos(spin) * axis0 + np.sin(spin) * np.cross(direction, axis0))
    angle = np.radians(rng.uniform(0.0, jitter_deg))
    return _unit(_rodrigues(direction, axis, angle))


def _plane_basis(normal: np.ndarray, up_hint: np.ndarray):
    n = _unit(normal)
    v = np.asarray(up_hint, dtype=np.float64) - (up_hint @ n) * n
    v = _unit(v)
    u = np.cross(v, n)
    return u, v, n


def make_diamond_board(center, normal, up_hint, side: float,
                       inplane_rot_rad: float = 0.0, hollow: bool = False,
                       hole_radius: float = 0.15, hole_shift: float = 0.20,
                       ) -> tuple[Rect, np.ndarray]:
    """Build a diamond-oriented board `Rect` (a square whose own edge axes
    are rotated 45 deg + `inplane_rot_rad` from `up_hint`, so its corners
    point up/down/left/right -- see `range_image.py`'s docstring and
    Task-29 review must-fix #2). Returns `(rect, corners)`; `corners` is
    `(4,3)` in `(+u_rot+v_rot, +u_rot-v_rot, -u_rot-v_rot, -u_rot+v_rot)`
    order, which -- at `inplane_rot_rad=0` -- equals the closed-form
    `center +/- half_diag*v` (top/bottom) and `center +/- half_diag*u`
    (right/left), `half_diag = side/sqrt(2)`, matching `boarddet.synth`'s
    existing diamond convention exactly.
    """
    center = np.asarray(center, dtype=np.float64)
    u, v, n = _plane_basis(np.asarray(normal, dtype=np.float64),
                           np.asarray(up_hint, dtype=np.float64))
    theta = np.pi / 4.0 + inplane_rot_rad
    u_rot = _unit(np.cos(theta) * u + np.sin(theta) * v)

    # board_detector.json5: "suppose the center of board is (0, 0); the
    # center of hole will be at (+/- shift, +/- shift)" -- the 4 diagonal
    # quadrant positions, not the 4 axis positions.
    holes = []
    if hollow:
        for hu, hv in ((hole_shift, hole_shift), (hole_shift, -hole_shift),
                      (-hole_shift, hole_shift), (-hole_shift, -hole_shift)):
            holes.append(((hu, hv), hole_radius))

    half = side / 2.0
    rect = Rect(center=center, normal=n, u_axis=u_rot,
               half_u=half, half_v=half, holes=holes)
    v_rot = rect.v_axis
    corners = np.stack([
        center + half * u_rot + half * v_rot,
        center + half * u_rot - half * v_rot,
        center - half * u_rot - half * v_rot,
        center - half * u_rot + half * v_rot,
    ])
    return rect, corners


# ---------------------------------------------------------------------------
# sensor-frame transform: build every primitive in a convenient "nominal,
# level" frame, then transform into the actual (tilted/raised) sensor frame
# raycast.render always casts rays from (the origin).
# ---------------------------------------------------------------------------


def _sensor_rotation(roll: float, pitch: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return ry @ rx


class _FrameTransform:
    """world(nominal) -> sensor(actual) transform: p_sensor = R @ p_nom +
    (0, 0, -height_offset); directions rotate only (R @ d)."""

    def __init__(self, roll: float, pitch: float, height_offset: float):
        self.R = _sensor_rotation(roll, pitch)
        self.height_offset = height_offset

    def point(self, p) -> np.ndarray:
        return self.R @ np.asarray(p, dtype=np.float64) + np.array(
            [0.0, 0.0, -self.height_offset])

    def direction(self, d) -> np.ndarray:
        return self.R @ np.asarray(d, dtype=np.float64)


def _polar(rng_range: float, az_rad: float, z: float) -> np.ndarray:
    return np.array([rng_range * np.cos(az_rad), rng_range * np.sin(az_rad), z])


# ---------------------------------------------------------------------------
# random_scene
# ---------------------------------------------------------------------------


def random_scene(rng: np.random.Generator, cfg: SceneGenConfig | None = None) -> Scene:
    cfg = cfg if cfg is not None else SceneGenConfig()

    roll = np.radians(rng.uniform(-cfg.sensor_tilt_jitter_deg, cfg.sensor_tilt_jitter_deg))
    pitch = np.radians(rng.uniform(-cfg.sensor_tilt_jitter_deg, cfg.sensor_tilt_jitter_deg))
    height_offset = rng.uniform(-cfg.sensor_height_jitter_m, cfg.sensor_height_jitter_m)
    xf = _FrameTransform(roll, pitch, height_offset)

    primitives: list = []

    # --- ground -------------------------------------------------------
    ground_z = rng.uniform(*cfg.ground_z_range_m)
    ground = Rect(
        center=xf.point([0.0, 0.0, ground_z]),
        normal=xf.direction([0.0, 0.0, 1.0]),
        u_axis=xf.direction([1.0, 0.0, 0.0]),
        half_u=cfg.ground_half_extent_m, half_v=cfg.ground_half_extent_m,
    )
    primitives.append(ground)

    # --- walls ----------------------------------------------------------
    n_walls = int(rng.integers(cfg.n_walls_range[0], cfg.n_walls_range[1] + 1))
    wall_dirs = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
    rng.shuffle(wall_dirs)
    wall_indices: list[int] = []
    for dx, dy in wall_dirs[:n_walls]:
        dist = rng.uniform(*cfg.wall_distance_range_m)
        wall_center_nom = np.array([dx * dist, dy * dist, cfg.wall_center_z_m])
        wall_normal_nom = np.array([-dx, -dy, 0.0])  # faces back toward origin
        # in-plane "horizontal" axis for a vertical wall facing +/-x or +/-y
        wall_u_nom = np.array([-dy, dx, 0.0])
        wall = Rect(
            center=xf.point(wall_center_nom),
            normal=xf.direction(wall_normal_nom),
            u_axis=xf.direction(wall_u_nom),
            half_u=cfg.wall_half_width_m, half_v=cfg.wall_half_height_m,
        )
        wall_indices.append(len(primitives))
        primitives.append(wall)

    # --- diamond boards ---------------------------------------------------
    n_boards = int(rng.integers(cfg.n_boards_range[0], cfg.n_boards_range[1] + 1))
    boards: list[BoardMeta] = []
    for _ in range(n_boards):
        board_range = rng.uniform(*cfg.board_range_m)
        az = np.radians(rng.uniform(*cfg.board_azimuth_range_deg))
        height = rng.uniform(*cfg.board_height_range_m)
        center_nom = _polar(board_range, az, height)
        facing_sensor_nom = -_unit(center_nom)
        normal_nom = _jitter_direction(rng, facing_sensor_nom, cfg.board_normal_jitter_deg)
        side = cfg.board_side_m * (1.0 + rng.uniform(-cfg.board_side_jitter_frac,
                                                       cfg.board_side_jitter_frac))
        inplane_jitter = np.radians(rng.uniform(-cfg.board_inplane_rot_jitter_deg,
                                                 cfg.board_inplane_rot_jitter_deg))
        hollow = bool(rng.random() < cfg.board_hollow_prob)
        rect_nom, corners_nom = make_diamond_board(
            center_nom, normal_nom, up_hint=[0.0, 0.0, 1.0], side=side,
            inplane_rot_rad=inplane_jitter, hollow=hollow,
            hole_radius=cfg.board_hole_radius_m, hole_shift=cfg.board_hole_shift_m,
        )
        rect = Rect(
            center=xf.point(rect_nom.center),
            normal=xf.direction(rect_nom.normal),
            u_axis=xf.direction(rect_nom.u_axis),
            half_u=rect_nom.half_u, half_v=rect_nom.half_v,
            holes=rect_nom.holes,
        )
        corners = np.stack([xf.point(c) for c in corners_nom])
        boards.append(BoardMeta(
            prim_index=len(primitives), center=rect.center, normal=rect.normal,
            u_axis=rect.u_axis, v_axis=rect.v_axis, side=side,
            corners=corners, hollow=hollow,
        ))
        primitives.append(rect)

    # --- clutter panels: MIX of embedded (coplanar-with-wall) + free-standing
    n_clutter = int(rng.integers(cfg.n_clutter_range[0], cfg.n_clutter_range[1] + 1))
    # guarantee at least one of each when there's room for both AND the
    # config doesn't pin the fraction to a hard 0/1 boundary -- the mix is
    # essential (brief), not left to chance for small n_clutter, but a
    # frac of exactly 0.0/1.0 is an explicit "all one kind" request and
    # must be honored (e.g. by tests isolating one branch).
    if not wall_indices or cfg.clutter_embedded_frac <= 0.0:
        n_embedded = 0
    elif cfg.clutter_embedded_frac >= 1.0:
        n_embedded = n_clutter
    elif n_clutter >= 2:
        n_embedded = int(np.clip(round(n_clutter * cfg.clutter_embedded_frac),
                                 1, n_clutter - 1))
    else:
        n_embedded = 1 if rng.random() < cfg.clutter_embedded_frac else 0
    embedded_flags = [True] * n_embedded + [False] * (n_clutter - n_embedded)
    rng.shuffle(embedded_flags)

    clutter: list[ClutterMeta] = []
    for embedded in embedded_flags:
        if embedded and wall_indices:
            wall_idx = int(rng.choice(wall_indices))
            wall = primitives[wall_idx]
            half_u = rng.uniform(*cfg.clutter_panel_half_extent_range_m)
            half_v = rng.uniform(*cfg.clutter_panel_half_extent_range_m)
            # random offset within the wall's own extent, in the wall's own
            # (u, v) coords, so the panel is a strict sub-patch of the wall
            max_du = max(wall.half_u - half_u, 0.0)
            max_dv = max(wall.half_v - half_v, 0.0)
            du = rng.uniform(-max_du, max_du)
            dv = rng.uniform(-max_dv, max_dv)
            panel_center = wall.center + du * wall.u_axis + dv * wall.v_axis
            panel = Rect(center=panel_center, normal=wall.normal,
                        u_axis=wall.u_axis, half_u=half_u, half_v=half_v)
            clutter.append(ClutterMeta(prim_index=len(primitives),
                                       embedded=True, wall_index=wall_idx))
            primitives.append(panel)
        else:
            c_range = rng.uniform(*cfg.clutter_free_range_m)
            c_az = np.radians(rng.uniform(*cfg.clutter_free_azimuth_range_deg))
            c_height = rng.uniform(*cfg.clutter_free_height_range_m)
            center_nom = _polar(c_range, c_az, c_height)
            normal_nom = _jitter_direction(rng, -_unit(center_nom), 60.0)
            half_u = rng.uniform(*cfg.clutter_panel_half_extent_range_m)
            half_v = rng.uniform(*cfg.clutter_panel_half_extent_range_m)
            u_nom, _, n_nom = _plane_basis(normal_nom, [0.0, 0.0, 1.0])
            panel = Rect(center=xf.point(center_nom), normal=xf.direction(n_nom),
                        u_axis=xf.direction(u_nom), half_u=half_u, half_v=half_v)
            clutter.append(ClutterMeta(prim_index=len(primitives),
                                       embedded=False, wall_index=None))
            primitives.append(panel)

    # --- boxes ------------------------------------------------------------
    n_boxes = int(rng.integers(cfg.n_boxes_range[0], cfg.n_boxes_range[1] + 1))
    for _ in range(n_boxes):
        b_range = rng.uniform(*cfg.clutter_range_m)
        b_az = np.radians(rng.uniform(*cfg.clutter_azimuth_range_deg))
        b_height = rng.uniform(*cfg.clutter_height_range_m)
        center_nom = _polar(b_range, b_az, b_height)
        yaw = rng.uniform(0.0, 2.0 * np.pi)
        cz, sz = np.cos(yaw), np.sin(yaw)
        r_nom = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        half_sizes = rng.uniform(*cfg.box_half_size_range_m, size=3)
        box = Box(center=xf.point(center_nom), R=xf.R @ r_nom, half_sizes=half_sizes)
        primitives.append(box)

    # --- cylinders ----------------------------------------------------------
    n_cyl = int(rng.integers(cfg.n_cylinders_range[0], cfg.n_cylinders_range[1] + 1))
    for _ in range(n_cyl):
        c_range = rng.uniform(*cfg.clutter_range_m)
        c_az = np.radians(rng.uniform(*cfg.clutter_azimuth_range_deg))
        base_z = rng.uniform(*cfg.clutter_height_range_m)
        center_nom = _polar(c_range, c_az, base_z)
        radius = rng.uniform(*cfg.cylinder_radius_range_m)
        height = rng.uniform(*cfg.cylinder_height_range_m)
        cyl = Cylinder(base=xf.point(center_nom), axis=xf.direction([0.0, 0.0, 1.0]),
                       radius=radius, height=height)
        primitives.append(cyl)

    return Scene(primitives=primitives, boards=boards, clutter=clutter,
                sensor_roll_rad=roll, sensor_pitch_rad=pitch,
                sensor_height_offset_m=height_offset)
