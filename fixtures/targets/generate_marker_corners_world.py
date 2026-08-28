#!/usr/bin/env python3
"""Generate ``marker_corners_world.golden.json``.

This is the *independent* side of a cross-language contract. It derives every world
coordinate in the fixture from two things only:

  1. each Target Definition manifest's stated geometry (plate side, the printed
     marker paper's size/placement/cell layout, and the marker ID list), read as
     plain JSON5 data, and
  2. a stated physical mounting of the plate (a pose in the world), carried forward
     unchanged from ``fixtures/board/generate_marker_corners_world.py``.

It deliberately does **not** import, call, or shell out to the ``calibration-target``
Rust crate or the ``lctk_target`` Python package -- it does not use `ValidatedTarget`,
`load_target`, or any function from either. Its whole value is that it can disagree
with them. It uses the general-purpose ``json5`` package only to read the manifest's
raw fields (numbers and strings) off disk, then does all geometry itself from the
physical definition below.

## The physical definition (first principles, not implementation)

A Target Definition's plate is a square, and the crate's canonical local frame is
"corner-aligned": origin at the plate's centre, `+X` toward the plate's own *left*
corner, `+Y` toward its *top* corner, `+Z` along the plate normal (this is a schema
convention stated for every manifest here and in ``ros/lctk_launch/config/targets/``,
not an implementation detail). The `fiducial.paper_center` field is the printed
marker-paper square's centre, stated as a `(toward_left_corner, toward_top_corner)`
offset from the plate centre along those same two axes -- i.e. directly in local
`(x, y)`.

The printed paper is itself a square, but glued with its *edges* -- not its
corners -- parallel to the plate's edges. Since the plate's corner-aligned `+X`/`+Y`
axes point at the plate's corners, the paper's own edges run at 45 degrees to them:
the paper's edge from its bottom corner to its left corner runs along
`(+X + +Y) / sqrt(2)`, and from its bottom corner to its right corner along
`(+Y - +X) / sqrt(2)`. Call these the paper's own `u` (toward its left corner) and
`v` (toward its right corner) axes. The paper's own bottom corner -- the origin of
its `(u, v)` coordinates -- then sits at `paper_center - (u_axis + v_axis) *
(paper_side / 2)`, i.e. one full paper half-diagonal below the paper centre along
`+Y`.

Marker cells tile the paper in a `cells_per_side` x `cells_per_side` grid, inset by
`outer_border` on every edge, each cell holding one marker of side
`square_size * marker_fill_ratio`, centred in its cell. `marker_ids` lists them in
"x-major order by (x, y)" (documented in
``book/src/user-guide/configuration.md``): index 0 is `(u_cell=0, v_cell=0)`, index 1
is `(u_cell=1, v_cell=0)`, and so on, wrapping to the next row (`v_cell += 1`) every
`cells_per_side` markers -- i.e. `u_cell = index % cells_per_side`,
`v_cell = index // cells_per_side`.

Each marker cell's four corners are named the same way the plate's own four corners
are: `bottom` is the cell corner nearest the paper's own bottom corner (lowest `u`
and `v`); `left` is reached by moving one marker-length further in `+u` (toward the
paper's left corner); `right` by moving one marker-length further in `+v` (toward the
paper's right corner); `top` by moving a marker-length in both.

## The physical mounting

A vertical plate 3 m in front of the sensor, hung as a diamond: the plate's
diagonals run straight up and horizontally, so the up-most point of the board is a
corner. The normal points back at the sensor (-X). This is the same mounting
``fixtures/board/generate_marker_corners_world.py`` used; the physical board did not
move when the board-frame convention changed, so neither did this statement of where
it is mounted.

Regenerate this fixture only when a manifest's *physical* geometry changes (plate
side, paper placement, cell layout, marker IDs) -- never to make a failing test pass.
Do not re-baseline it from `calibration-target` or `lctk_target` output: run this
script and compare its output to the committed golden by hand; a disagreement is a
finding about the golden or the implementation, not something to paper over here.

Usage:
    python3 generate_marker_corners_world.py > /tmp/candidate.json
    # then diff /tmp/candidate.json against the committed
    # fixtures/targets/marker_corners_world.golden.json numerically (exact text
    # formatting need not match).

Coverage: both targets the committed golden covers, ``hollow_1000_aruco_4`` and
``solid_600_aruco_1``. Both fit the same general grid/paper-placement formula above
without needing target-specific special-casing (`solid_600_aruco_1` is simply the
`cells_per_side=1`, `marker_fill_ratio=1.0`, `paper_center=(0, 0)` case of the same
rules), so nothing is short-changed here.
"""

import json
import math
import sys
from pathlib import Path

import json5

FIXTURES_DIR = Path(__file__).resolve().parent

# Target Definition manifests the committed golden covers, in the order the golden
# lists them.
MANIFESTS = [
    "solid_600_aruco_1_v1.json5",
    "hollow_1000_aruco_4_v1.json5",
]

# ---------------------------------------------------------------------------
# The stated physical mounting -- see module docstring. Unchanged from
# fixtures/board/generate_marker_corners_world.py.
# ---------------------------------------------------------------------------
PLATE_CENTER = (3.0, 0.5, 1.2)
NORMAL = (-1.0, 0.0, 0.0)  # toward the sensor at the origin
UP_DIAGONAL = (0.0, 0.0, 1.0)  # plate centre -> up-most (top) corner


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def add(p, v):
    return (p[0] + v[0], p[1] + v[1], p[2] + v[2])


def sub(p, q):
    return (p[0] - q[0], p[1] - q[1], p[2] - q[2])


def scale(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def normalize(v):
    n = math.sqrt(sum(c * c for c in v))
    return scale(v, 1.0 / n)


def parse_length_m(value: str) -> float:
    """Parse a manifest length string ("0.5m", "150mm", "-0.353553391m") to metres,
    rounded to the nearest micrometre.

    Parsing the unit suffix is conversion only -- reading the number the manifest
    states, not a geometric derivation -- so doing it here rather than importing the
    crate's own parser does not compromise independence. The micrometre rounding is
    likewise not implementation trivia: every Target Definition schema and manifest
    in this repository documents length semantics as micrometre-quantized (see e.g.
    ``rust/calibration-target/src/lib.rs``'s "Target Identity defines length
    semantics at a micrometre", and the committed golden's own `_comment`, "canonical
    micrometre-normalized Target Definitions"). Skipping this step would compare
    this script's full-precision decimal arithmetic against every consumer's
    micrometre-quantized arithmetic and manufacture a ~0.4 um phantom disagreement
    out of nothing.
    """
    value = value.strip()
    if value.endswith("mm"):
        meters = float(value[: -len("mm")]) / 1000.0
    elif value.endswith("m"):
        meters = float(value[: -len("m")])
    else:
        raise ValueError(f"length {value!r} does not end in 'mm' or 'm'")
    return round(meters * 1_000_000.0) / 1_000_000.0


def load_manifest(filename: str) -> dict:
    with open(FIXTURES_DIR / filename, "r", encoding="utf-8") as handle:
        return json5.load(handle)


def marker_corners_for_manifest(manifest: dict, left_dir, up_dir):
    """World-frame `[right, top, left, bottom]` corners for every marker id.

    `left_dir`/`up_dir` are the world-frame unit vectors for the manifest's
    canonical local `+X` (toward the plate's left corner) and `+Y` (toward its top
    corner) axes -- i.e. the physical mounting applied to the schema's own
    corner-aligned frame definition.
    """
    fiducial = manifest["fiducial"]

    paper_side_m = parse_length_m(fiducial["paper_side"])
    paper_center_left_m = parse_length_m(fiducial["paper_center"]["toward_left_corner"])
    paper_center_top_m = parse_length_m(fiducial["paper_center"]["toward_top_corner"])
    outer_border_m = parse_length_m(fiducial["outer_border"])
    cells_per_side = int(fiducial["cells_per_side"])
    marker_fill_ratio = float(fiducial["marker_fill_ratio"])
    marker_ids = list(fiducial["marker_ids"])

    paper_center_world = add(
        add(PLATE_CENTER, scale(left_dir, paper_center_left_m)),
        scale(up_dir, paper_center_top_m),
    )

    # The paper's own edge directions: 45 degrees to the plate's corner-aligned
    # axes (see module docstring).
    u_dir = normalize(add(left_dir, up_dir))  # paper centre -> its left corner
    v_dir = normalize(sub(up_dir, left_dir))  # paper centre -> its right corner

    half_paper_m = paper_side_m / 2.0
    paper_bottom_world = sub(paper_center_world, scale(add(u_dir, v_dir), half_paper_m))

    def paper_to_world(pu, pv):
        return add(add(paper_bottom_world, scale(u_dir, pu)), scale(v_dir, pv))

    square_size_m = (paper_side_m - 2.0 * outer_border_m) / cells_per_side
    marker_size_m = square_size_m * marker_fill_ratio
    marker_border_m = (square_size_m - marker_size_m) / 2.0
    origin_m = outer_border_m + marker_border_m

    markers = {}
    for index, marker_id in enumerate(marker_ids):
        # x-major order by (x, y): see module docstring.
        u_cell = index % cells_per_side
        v_cell = index // cells_per_side
        base_u = origin_m + u_cell * square_size_m
        base_v = origin_m + v_cell * square_size_m
        markers[str(marker_id)] = [
            list(paper_to_world(base_u, base_v + marker_size_m)),  # right
            list(paper_to_world(base_u + marker_size_m, base_v + marker_size_m)),  # top
            list(paper_to_world(base_u + marker_size_m, base_v)),  # left
            list(paper_to_world(base_u, base_v)),  # bottom
        ]

    return marker_ids, markers


def main():
    normal = normalize(NORMAL)
    up_dir = normalize(UP_DIAGONAL)
    # Toward the left corner: completes a right-handed (left, up, normal) triple --
    # the same rule the ported Rust test (`geometry_contract.rs`) and the retired
    # `hollow-board-config` golden test used.
    left_dir = normalize(cross(up_dir, normal))

    targets = {}
    for filename in MANIFESTS:
        manifest = load_manifest(filename)
        target_id = manifest["target_id"]
        marker_ids, markers = marker_corners_for_manifest(manifest, left_dir, up_dir)
        targets[target_id] = {
            "marker_ids": marker_ids,
            "markers": markers,
        }

    fixture = {
        "_comment": (
            "Target-keyed world marker geometry for canonical micrometre-normalized "
            "Target Definitions. Fixed pose maps local +X to world -Y, local +Y to "
            "world +Z, and local +Z to world -X. Do not re-baseline from "
            "implementation output -- regenerate with "
            "generate_marker_corners_world.py and compare by hand instead."
        ),
        "marker_corner_order": ["right", "top", "left", "bottom"],
        "mounting": {
            "plate_center": list(PLATE_CENTER),
            "local_x_toward_left": list(left_dir),
            "local_y_toward_top": list(up_dir),
            "local_z_normal": list(normal),
        },
        "targets": targets,
    }

    json.dump(fixture, sys.stdout, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
