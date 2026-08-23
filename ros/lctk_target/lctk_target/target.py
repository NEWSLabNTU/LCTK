"""Validated Target Definition parsing, identity, and marker-grid geometry.

This module intentionally imports neither rclpy nor ROS messages.  It is the Python
implementation of the target-definition contract shared with ``calibration-target``.
"""

from __future__ import annotations

import hashlib
import math
import struct
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Final

import json5

_SCHEMA_VERSION: Final = 1
_FRAME: Final = "corner_aligned_plate_center_v1"
_DICTIONARY: Final = "DICT_5X5_1000"
_DICTIONARY_CAPACITY: Final = 1000
_DIAMOND_TOLERANCE_UM: Final = 2


@dataclass(frozen=True)
class TargetIdentity:
    """Versioned binding of a target's semantic physical definition."""

    schema_version: int
    target_id: str
    revision: int
    semantic_sha256: str
    board_frame_convention: str


@dataclass(frozen=True)
class CircularCutout:
    x_um: int
    y_um: int
    radius_um: int


@dataclass(frozen=True)
class Surface:
    kind: str
    circular_cutouts: tuple[CircularCutout, ...] = ()


@dataclass(frozen=True)
class Plate:
    side_um: int
    surface: Surface


@dataclass(frozen=True)
class Fiducial:
    kind: str
    dictionary: str
    marker_ids: tuple[int, ...]
    paper_side_um: int
    paper_center_x_um: int
    paper_center_y_um: int
    outer_border_um: int
    cells_per_side: int
    marker_fill_ratio: float
    border_bits: int


@dataclass(frozen=True)
class LidarOrientationReference:
    kind: str
    local_axis: str | None = None


Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class ValidatedTarget:
    """Immutable physical target definition.  Construct with :func:`load_target`."""

    schema_version: int
    target_id: str
    revision: int
    board_frame_convention: str
    plate: Plate
    fiducial: Fiducial
    lidar_orientation_reference: LidarOrientationReference
    identity: TargetIdentity
    marker_corners_by_id: Mapping[int, tuple[Point3, Point3, Point3, Point3]]

    def canonical_bytes(self) -> bytes:
        """Return fixed grammar bytes used for ``identity.semantic_sha256``."""
        records: list[tuple[str, str]] = [
            ("schema_version", str(self.schema_version)),
            ("target_id", self.target_id),
            ("revision", str(self.revision)),
            ("board_frame_convention", self.board_frame_convention),
            ("plate.side_um", str(self.plate.side_um)),
            ("plate.surface.kind", self.plate.surface.kind),
        ]
        for index, cutout in enumerate(self.plate.surface.circular_cutouts):
            prefix = f"plate.surface.circular_cutouts[{index}]"
            records.extend(
                (
                    (f"{prefix}.x_um", str(cutout.x_um)),
                    (f"{prefix}.y_um", str(cutout.y_um)),
                    (f"{prefix}.radius_um", str(cutout.radius_um)),
                )
            )
        fiducial = self.fiducial
        records.extend(
            (
                ("fiducial.kind", fiducial.kind),
                ("fiducial.dictionary", fiducial.dictionary),
            )
        )
        records.extend(
            (f"fiducial.marker_ids[{index}]", str(marker_id))
            for index, marker_id in enumerate(fiducial.marker_ids)
        )
        records.extend(
            (
                ("fiducial.paper_side_um", str(fiducial.paper_side_um)),
                (
                    "fiducial.paper_center.toward_left_corner_um",
                    str(fiducial.paper_center_x_um),
                ),
                (
                    "fiducial.paper_center.toward_top_corner_um",
                    str(fiducial.paper_center_y_um),
                ),
                ("fiducial.outer_border_um", str(fiducial.outer_border_um)),
                ("fiducial.cells_per_side", str(fiducial.cells_per_side)),
                (
                    "fiducial.marker_fill_ratio_f64_bits",
                    str(_f64_bits(fiducial.marker_fill_ratio)),
                ),
                ("fiducial.border_bits", str(fiducial.border_bits)),
                (
                    "lidar_orientation_reference.kind",
                    self.lidar_orientation_reference.kind,
                ),
            )
        )
        if self.lidar_orientation_reference.local_axis is not None:
            records.append(
                (
                    "lidar_orientation_reference.local_axis",
                    self.lidar_orientation_reference.local_axis,
                )
            )
        return b"".join(
            f"{name}:{len(value.encode('utf-8'))}:{value}\n".encode()
            for name, value in records
        )


def load_target(path: str | Path) -> ValidatedTarget:
    """Load and strictly validate a JSON5 Target Definition from ``path``."""
    try:
        source = Path(path).read_text(encoding="utf-8")
    except OSError as error:
        raise ValueError(f"target_config: cannot read {path!s}: {error}") from error
    try:
        raw = json5.loads(source, allow_duplicate_keys=False)
    except Exception as error:
        raise ValueError(f"invalid Target Definition JSON5: {error}") from error
    return _validate_target(_mapping(raw, "target"))


def _validate_target(raw: Mapping[str, Any]) -> ValidatedTarget:
    _exact_keys(
        raw,
        {
            "schema_version",
            "target_id",
            "revision",
            "board_frame_convention",
            "plate",
            "fiducial",
            "lidar_orientation_reference",
        },
        "target",
    )
    schema_version = _uint(raw["schema_version"], "schema_version")
    if schema_version != _SCHEMA_VERSION:
        _fail("schema_version", f"expected 1, got {schema_version}")
    target_id = _string(raw["target_id"], "target_id")
    if not target_id.strip():
        _fail("target_id", "must not be empty")
    revision = _uint(raw["revision"], "revision")
    if revision == 0:
        _fail("revision", "must be greater than zero")
    frame = _string(raw["board_frame_convention"], "board_frame_convention")
    if frame != _FRAME:
        _fail("board_frame_convention", f"unsupported {frame!r}")

    plate = _validate_plate(_mapping(raw["plate"], "plate"))
    fiducial = _validate_fiducial(_mapping(raw["fiducial"], "fiducial"), plate)
    orientation = _validate_orientation(
        _mapping(raw["lidar_orientation_reference"], "lidar_orientation_reference"),
        plate.surface,
    )
    provisional = ValidatedTarget(
        schema_version=schema_version,
        target_id=target_id,
        revision=revision,
        board_frame_convention=frame,
        plate=plate,
        fiducial=fiducial,
        lidar_orientation_reference=orientation,
        identity=TargetIdentity(schema_version, target_id, revision, "", frame),
        marker_corners_by_id=_marker_corners(fiducial),
    )
    digest = hashlib.sha256(provisional.canonical_bytes()).hexdigest()
    return ValidatedTarget(
        schema_version=schema_version,
        target_id=target_id,
        revision=revision,
        board_frame_convention=frame,
        plate=plate,
        fiducial=fiducial,
        lidar_orientation_reference=orientation,
        identity=TargetIdentity(schema_version, target_id, revision, digest, frame),
        marker_corners_by_id=provisional.marker_corners_by_id,
    )


def _validate_plate(raw: Mapping[str, Any]) -> Plate:
    _exact_keys(raw, {"side", "surface"}, "plate")
    side_um = _positive_length(raw["side"], "plate.side")
    surface_raw = _mapping(raw["surface"], "plate.surface")
    kind = _string(surface_raw.get("kind"), "plate.surface.kind")
    if kind == "solid":
        _exact_keys(surface_raw, {"kind"}, "plate.surface")
        return Plate(side_um, Surface(kind))
    if kind != "perforated":
        _fail("plate.surface.kind", f"unsupported {kind!r}")
    _exact_keys(surface_raw, {"kind", "circular_cutouts"}, "plate.surface")
    entries = _list(surface_raw["circular_cutouts"], "plate.surface.circular_cutouts")
    if not entries:
        _fail(
            "plate.surface.circular_cutouts",
            "perforated surface needs at least one cutout",
        )
    cutouts = tuple(
        _validate_cutout(
            _mapping(value, f"plate.surface.circular_cutouts[{index}]"), index, side_um
        )
        for index, value in enumerate(entries)
    )
    cutouts = tuple(
        sorted(cutouts, key=lambda cutout: (cutout.x_um, cutout.y_um, cutout.radius_um))
    )
    for left, first in enumerate(cutouts):
        for second in cutouts[left + 1 :]:
            if (
                math.hypot(first.x_um - second.x_um, first.y_um - second.y_um)
                <= first.radius_um + second.radius_um
            ):
                _fail(
                    "plate.surface.circular_cutouts",
                    f"cutouts {left} and {left + 1} overlap",
                )
    if _quarter_turn_invariant(cutouts):
        _fail(
            "plate.surface.circular_cutouts",
            "geometry must break quarter-turn symmetry",
        )
    return Plate(side_um, Surface(kind, cutouts))


def _validate_cutout(
    raw: Mapping[str, Any], index: int, side_um: int
) -> CircularCutout:
    field = f"plate.surface.circular_cutouts[{index}]"
    _exact_keys(raw, {"center", "radius"}, field)
    center = _mapping(raw["center"], f"{field}.center")
    _exact_keys(center, {"x", "y"}, f"{field}.center")
    x_um = _length(center["x"], f"{field}.center.x")
    y_um = _length(center["y"], f"{field}.center.y")
    radius_um = _positive_length(raw["radius"], f"{field}.radius")
    if (
        abs(x_um) + abs(y_um) + radius_um * math.sqrt(2.0)
        > side_um / math.sqrt(2.0) + _DIAMOND_TOLERANCE_UM
    ):
        _fail(field, "cutout extends outside plate")
    return CircularCutout(x_um, y_um, radius_um)


def _validate_fiducial(raw: Mapping[str, Any], plate: Plate) -> Fiducial:
    _exact_keys(
        raw,
        {
            "kind",
            "dictionary",
            "marker_ids",
            "paper_side",
            "paper_center",
            "outer_border",
            "cells_per_side",
            "marker_fill_ratio",
            "border_bits",
        },
        "fiducial",
    )
    kind = _string(raw["kind"], "fiducial.kind")
    if kind != "square_aruco_grid":
        _fail("fiducial.kind", f"unsupported {kind!r}")
    dictionary = _string(raw["dictionary"], "fiducial.dictionary")
    if dictionary != _DICTIONARY:
        _fail("fiducial.dictionary", f"unsupported {dictionary!r}")
    cells = _uint(raw["cells_per_side"], "fiducial.cells_per_side")
    if cells == 0:
        _fail("fiducial.cells_per_side", "must be greater than zero")
    identifiers = tuple(
        _uint(value, f"fiducial.marker_ids[{index}]")
        for index, value in enumerate(_list(raw["marker_ids"], "fiducial.marker_ids"))
    )
    if len(identifiers) != cells * cells:
        _fail(
            "fiducial.marker_ids",
            f"expected {cells * cells} IDs for {cells}x{cells} grid, got {len(identifiers)}",
        )
    if len(set(identifiers)) != len(identifiers):
        _fail("fiducial.marker_ids", "duplicate ID")
    if any(marker_id >= _DICTIONARY_CAPACITY for marker_id in identifiers):
        _fail("fiducial.marker_ids", f"ID is outside {_DICTIONARY}")
    paper_side_um = _positive_length(raw["paper_side"], "fiducial.paper_side")
    center = _mapping(raw["paper_center"], "fiducial.paper_center")
    _exact_keys(
        center, {"toward_left_corner", "toward_top_corner"}, "fiducial.paper_center"
    )
    center_x_um = _length(
        center["toward_left_corner"], "fiducial.paper_center.toward_left_corner"
    )
    center_y_um = _length(
        center["toward_top_corner"], "fiducial.paper_center.toward_top_corner"
    )
    border_um = _length(raw["outer_border"], "fiducial.outer_border")
    if border_um < 0:
        _fail("fiducial.outer_border", "must not be negative")
    if 2 * border_um >= paper_side_um:
        _fail("fiducial.outer_border", "twice border must be less than paper_side")
    fill_ratio = _number(raw["marker_fill_ratio"], "fiducial.marker_fill_ratio")
    if not math.isfinite(fill_ratio) or not 0.0 < fill_ratio <= 1.0:
        _fail("fiducial.marker_fill_ratio", "must be finite and in (0, 1]")
    border_bits = _uint(raw["border_bits"], "fiducial.border_bits")
    if border_bits < 1:
        _fail("fiducial.border_bits", "must be at least 1")
    fiducial = Fiducial(
        kind,
        dictionary,
        identifiers,
        paper_side_um,
        center_x_um,
        center_y_um,
        border_um,
        cells,
        fill_ratio,
        border_bits,
    )
    _validate_paper(fiducial, plate)
    return fiducial


def _validate_paper(fiducial: Fiducial, plate: Plate) -> None:
    paper_radius = fiducial.paper_side_um / math.sqrt(2.0)
    plate_radius = plate.side_um / math.sqrt(2.0)
    for x, y in (
        (0.0, paper_radius),
        (0.0, -paper_radius),
        (paper_radius, 0.0),
        (-paper_radius, 0.0),
    ):
        if (
            abs(fiducial.paper_center_x_um + x) + abs(fiducial.paper_center_y_um + y)
            > plate_radius + _DIAMOND_TOLERANCE_UM
        ):
            _fail("fiducial", "paper corners extend outside plate")
    for index, cutout in enumerate(plate.surface.circular_cutouts):
        distance = _distance_to_diamond(
            cutout.x_um - fiducial.paper_center_x_um,
            cutout.y_um - fiducial.paper_center_y_um,
            round(fiducial.paper_side_um / math.sqrt(2.0)),
        )
        if distance < cutout.radius_um - _DIAMOND_TOLERANCE_UM:
            _fail("fiducial", f"paper intersects circular cutout {index}")


def _validate_orientation(
    raw: Mapping[str, Any], surface: Surface
) -> LidarOrientationReference:
    kind = _string(raw.get("kind"), "lidar_orientation_reference.kind")
    if kind == "mounting_up":
        _exact_keys(raw, {"kind", "local_axis"}, "lidar_orientation_reference")
        axis = _string(raw["local_axis"], "lidar_orientation_reference.local_axis")
        if axis != "+y":
            _fail(
                "lidar_orientation_reference.local_axis", f"expected +y, got {axis!r}"
            )
        return LidarOrientationReference(kind, axis)
    if kind == "asymmetric_cutouts":
        _exact_keys(raw, {"kind"}, "lidar_orientation_reference")
        if surface.kind != "perforated":
            _fail(
                "lidar_orientation_reference.kind",
                "asymmetric_cutouts requires a perforated surface",
            )
        return LidarOrientationReference(kind)
    _fail("lidar_orientation_reference.kind", f"unsupported {kind!r}")


def _marker_corners(
    fiducial: Fiducial,
) -> Mapping[int, tuple[Point3, Point3, Point3, Point3]]:
    square_um = (
        fiducial.paper_side_um - 2 * fiducial.outer_border_um
    ) / fiducial.cells_per_side
    marker_um = square_um * fiducial.marker_fill_ratio
    marker_border_um = (square_um - marker_um) / 2.0
    inv_sqrt2 = 1.0 / math.sqrt(2.0)

    def paper_point(u_um: float, v_um: float) -> Point3:
        half = fiducial.paper_side_um / 2.0
        x_um = fiducial.paper_center_x_um + (u_um - v_um) * inv_sqrt2
        y_um = fiducial.paper_center_y_um + (u_um + v_um - 2 * half) * inv_sqrt2
        return (x_um / 1_000_000.0, y_um / 1_000_000.0, 0.0)

    corners: dict[int, tuple[Point3, Point3, Point3, Point3]] = {}
    for index, marker_id in enumerate(fiducial.marker_ids):
        u_cell = index % fiducial.cells_per_side
        v_cell = index // fiducial.cells_per_side
        base_u = fiducial.outer_border_um + marker_border_um + u_cell * square_um
        base_v = fiducial.outer_border_um + marker_border_um + v_cell * square_um
        bottom = paper_point(base_u, base_v)
        left = paper_point(base_u + marker_um, base_v)
        right = paper_point(base_u, base_v + marker_um)
        top = paper_point(base_u + marker_um, base_v + marker_um)
        corners[marker_id] = (right, top, left, bottom)
    return MappingProxyType(corners)


def _length(value: Any, field: str) -> int:
    text = _string(value, field)
    if text.endswith("mm"):
        number, multiplier = text[:-2], 1_000.0
    elif text.endswith("m"):
        number, multiplier = text[:-1], 1_000_000.0
    else:
        _fail(field, "expected a length ending in mm or m")
    try:
        parsed = float(number.strip())
    except ValueError as error:
        raise ValueError(f"{field}: invalid length {text!r}") from error
    if not math.isfinite(parsed):
        _fail(field, "length must be finite")
    micrometres = parsed * multiplier
    if not math.isfinite(micrometres):
        _fail(field, "length is outside supported range")
    rounded = _round_half_away(micrometres)
    if abs(rounded) > 2**63 - 1:
        _fail(field, "length is outside supported range")
    return rounded


def _positive_length(value: Any, field: str) -> int:
    result = _length(value, field)
    if result <= 0:
        _fail(field, "must be positive")
    return result


def _round_half_away(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def _f64_bits(value: float) -> int:
    return struct.unpack(">Q", struct.pack(">d", value))[0]


def _distance_to_diamond(x_um: int, y_um: int, radius_um: int) -> float:
    excess = abs(x_um) + abs(y_um) - radius_um
    return 0.0 if excess <= 0 else excess / math.sqrt(2.0)


def _quarter_turn_invariant(cutouts: tuple[CircularCutout, ...]) -> bool:
    values = {(cutout.x_um, cutout.y_um, cutout.radius_um) for cutout in cutouts}
    return all(
        (-cutout.y_um, cutout.x_um, cutout.radius_um) in values for cutout in cutouts
    )


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        _fail(field, "must be an object")
    return value


def _list(value: Any, field: str) -> list[Any]:
    if not isinstance(value, list):
        _fail(field, "must be an array")
    return value


def _string(value: Any, field: str) -> str:
    if not isinstance(value, str):
        _fail(field, "must be a string")
    return value


def _uint(value: Any, field: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < 0
        or value > 2**32 - 1
    ):
        _fail(field, "must be a uint32")
    return value


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(field, "must be a number")
    return float(value)


def _exact_keys(value: Mapping[str, Any], expected: set[str], field: str) -> None:
    unknown = set(value) - expected
    missing = expected - set(value)
    if unknown:
        _fail(field, f"unknown field {min(unknown)!r}")
    if missing:
        _fail(field, f"missing field {min(missing)!r}")


def _fail(field: str, message: str) -> None:
    raise ValueError(f"{field}: {message}")
