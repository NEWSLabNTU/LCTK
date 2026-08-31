import builtins
import hashlib
import importlib
import json
import math
import sys
from pathlib import Path
from types import MappingProxyType

import pytest
from lctk_target import load_target

ROOT = Path(__file__).resolve().parents[3]
FIXTURES = ROOT / "fixtures" / "targets"
LAUNCH_TARGETS = ROOT / "ros" / "lctk_launch" / "config" / "targets"


def _goldens() -> dict[str, tuple[str, bytes]]:
    source = (FIXTURES / "canonical_identity.golden").read_text(encoding="utf-8")
    result = {}
    for block in source.split("---\n"):
        target_index = block.find("target_id=")
        if target_index < 0:
            continue
        lines = block[target_index:].splitlines(keepends=True)
        target_id = lines[0].removeprefix("target_id=").strip()
        digest = lines[1].removeprefix("semantic_sha256=").strip()
        start = lines.index("canonical_bytes:\n") + 1
        result[target_id] = (digest, "".join(lines[start:]).encode())
    return result


@pytest.mark.parametrize("name", ["solid_600_aruco_1", "hollow_1000_aruco_4"])
def test_target_manifests_and_goldens(name):
    fixture = load_target(FIXTURES / f"{name}_v1.json5")
    launch = load_target(LAUNCH_TARGETS / f"{name}_v1.json5")
    digest, canonical = _goldens()[name]

    assert fixture.identity == launch.identity
    assert fixture.canonical_bytes() == launch.canonical_bytes() == canonical
    assert fixture.identity.semantic_sha256 == digest
    assert hashlib.sha256(canonical).hexdigest() == digest


def test_shared_world_marker_golden_matches_both_targets():
    golden = json.loads(
        (FIXTURES / "marker_corners_world.golden.json").read_text(encoding="utf-8")
    )
    target_ids = ["solid_600_aruco_1", "hollow_1000_aruco_4"]
    assert list(golden["targets"]) == target_ids
    assert golden["marker_corner_order"] == ["right", "top", "left", "bottom"]

    mounting = golden["mounting"]
    center = mounting["plate_center"]
    axes = (
        mounting["local_x_toward_left"],
        mounting["local_y_toward_top"],
        mounting["local_z_normal"],
    )
    for target_id in target_ids:
        target = load_target(FIXTURES / f"{target_id}_v1.json5")
        expected = golden["targets"][target_id]
        assert list(target.marker_corners_by_id) == expected["marker_ids"]
        assert list(expected["markers"]) == [
            str(marker_id) for marker_id in expected["marker_ids"]
        ]
        for marker_id, local_corners in target.marker_corners_by_id.items():
            world_corners = [
                tuple(
                    center[world_axis]
                    + sum(
                        local[local_axis] * axes[local_axis][world_axis]
                        for local_axis in range(3)
                    )
                    for world_axis in range(3)
                )
                for local in local_corners
            ]
            actual_flat = [value for corner in world_corners for value in corner]
            expected_flat = [
                value
                for corner in expected["markers"][str(marker_id)]
                for value in corner
            ]
            assert actual_flat == pytest.approx(expected_flat, abs=1e-12)


def test_solid_marker_is_exactly_480_mm_and_has_documented_corners():
    target = load_target(FIXTURES / "solid_600_aruco_1_v1.json5")
    fiducial = target.fiducial
    assert fiducial.paper_side_um - 2 * fiducial.outer_border_um == 480_000
    expected = 0.480 / math.sqrt(2.0)
    actual = [
        coordinate
        for corner in target.marker_corners_by_id[24]
        for coordinate in corner
    ]
    wanted = [
        coordinate
        for corner in (
            (-expected, 0.0, 0.0),
            (0.0, expected, 0.0),
            (expected, 0.0, 0.0),
            (0.0, -expected, 0.0),
        )
        for coordinate in corner
    ]
    assert actual == pytest.approx(wanted)


def test_generic_grid_expands_2x2_in_declared_marker_id_order():
    target = load_target(FIXTURES / "hollow_1000_aruco_4_v1.json5")
    assert tuple(target.marker_corners_by_id) == (696, 64, 306, 195)
    assert all(len(corners) == 4 for corners in target.marker_corners_by_id.values())
    assert target.marker_corners_by_id[696][0][2] == 0.0


def test_hollow_marker_corners_match_canonical_micrometre_geometry():
    """Pin independently derived board-local corners after µm normalization.

    Canonical inputs are paper centre ``(0, -353553) µm``, paper side 500000 µm,
    outer border 10000 µm, two 240000 µm cells, and 0.8 fill. Thus each marker is
    192000 µm with 24000 µm cell padding. For paper coordinates ``(u, v)``, the
    accepted corner-aligned transform is ``x=(u-v)/sqrt(2)`` and
    ``y=-353553+(u+v-500000)/sqrt(2)`` µm. Values below are that calculation,
    independent of loader output and distinct from pre-normalization legacy values.
    """
    target = load_target(FIXTURES / "hollow_1000_aruco_4_v1.json5")
    expected = {
        696: (
            (-0.135764501987817, -0.523258627484771, 0.0),
            (0.0, -0.387494125496954, 0.0),
            (0.135764501987817, -0.523258627484771, 0.0),
            (0.0, -0.659023129472589, 0.0),
        ),
        64: (
            (0.033941125496954, -0.353553000000000, 0.0),
            (0.169705627484771, -0.217788498012183, 0.0),
            (0.305470129472589, -0.353553000000000, 0.0),
            (0.169705627484771, -0.489317501987817, 0.0),
        ),
        306: (
            (-0.305470129472589, -0.353553000000000, 0.0),
            (-0.169705627484771, -0.217788498012183, 0.0),
            (-0.033941125496954, -0.353553000000000, 0.0),
            (-0.169705627484771, -0.489317501987817, 0.0),
        ),
        195: (
            (-0.135764501987817, -0.183847372515229, 0.0),
            (0.0, -0.048082870527411, 0.0),
            (0.135764501987817, -0.183847372515229, 0.0),
            (0.0, -0.319611874503046, 0.0),
        ),
    }
    assert tuple(target.marker_corners_by_id) == tuple(expected)
    for marker_id, wanted in expected.items():
        actual_flat = [
            value
            for corner in target.marker_corners_by_id[marker_id]
            for value in corner
        ]
        wanted_flat = [value for corner in wanted for value in corner]
        assert actual_flat == pytest.approx(wanted_flat, abs=1e-12)


def test_marker_geometry_is_deeply_immutable():
    target = load_target(FIXTURES / "solid_600_aruco_1_v1.json5")
    assert isinstance(target.marker_corners_by_id, MappingProxyType)
    assert isinstance(target.marker_corners_by_id[24], tuple)
    assert all(isinstance(corner, tuple) for corner in target.marker_corners_by_id[24])
    with pytest.raises(TypeError):
        target.marker_corners_by_id[24] = ()


def test_loader_imports_without_rclpy(monkeypatch):
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if name == "rclpy" or name.startswith("rclpy."):
            raise AssertionError("target loader must not import rclpy")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    monkeypatch.delitem(sys.modules, "lctk_target", raising=False)
    monkeypatch.delitem(sys.modules, "lctk_target.target", raising=False)
    freshly_imported = importlib.import_module("lctk_target")
    assert (
        freshly_imported.load_target(FIXTURES / "solid_600_aruco_1_v1.json5").target_id
        == "solid_600_aruco_1"
    )


@pytest.mark.parametrize(
    "duplicate",
    [
        "revision: 1, revision: 2,",
        'plate: { surface: { kind: "solid", kind: "solid" }, side: "0.600m" },',
        'paper_side: "0.6m", marker_ids: [24], marker_ids: [2],',
    ],
)
def test_duplicate_keys_are_rejected_at_every_nesting_level(tmp_path, duplicate):
    source = (FIXTURES / "solid_600_aruco_1_v1.json5").read_text(encoding="utf-8")
    if duplicate.startswith("revision"):
        source = source.replace("revision: 1,", duplicate, 1)
    elif duplicate.startswith("plate"):
        source = source.replace(
            'plate: { surface: { kind: "solid" }, side: "0.600m" },', duplicate, 1
        )
    else:
        source = source.replace('paper_side: "0.6m", marker_ids: [24],', duplicate, 1)
    path = tmp_path / "duplicate.json5"
    path.write_text(source, encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate|Duplicate"):
        load_target(path)


@pytest.mark.parametrize(
    ("old", "new", "field"),
    [
        ('side: "0.6m"', 'side: "600mm"', None),
        ("marker_ids: [24]", "marker_ids: [24, 24]", "fiducial.marker_ids"),
        ("marker_ids: [24]", "marker_ids: [1000]", "fiducial.marker_ids"),
        ('outer_border: "0.060m"', 'outer_border: "300mm"', "fiducial.outer_border"),
        ('kind: "solid"', 'kind: "unknown"', "plate.surface.kind"),
    ],
)
def test_schema_validation_and_equivalent_lengths(tmp_path, old, new, field):
    source = (
        (FIXTURES / "solid_600_aruco_1_v1.json5")
        .read_text(encoding="utf-8")
        .replace(old, new, 1)
    )
    path = tmp_path / "target.json5"
    path.write_text(source, encoding="utf-8")
    if field is None:
        assert (
            load_target(path).identity.semantic_sha256
            == load_target(
                FIXTURES / "solid_600_aruco_1_v1.json5"
            ).identity.semantic_sha256
        )
    else:
        with pytest.raises(ValueError, match=field):
            load_target(path)


def test_unknown_fields_are_rejected(tmp_path):
    source = (
        (FIXTURES / "solid_600_aruco_1_v1.json5")
        .read_text(encoding="utf-8")
        .replace("revision: 1,", "revision: 1, extra: 0,", 1)
    )
    path = tmp_path / "target.json5"
    path.write_text(source, encoding="utf-8")
    with pytest.raises(ValueError, match="unknown field"):
        load_target(path)


def test_finite_length_whose_unit_product_overflows_returns_value_error(tmp_path):
    source = (
        (FIXTURES / "solid_600_aruco_1_v1.json5")
        .read_text(encoding="utf-8")
        .replace('side: "0.600m"', 'side: "1e308m"', 1)
    )
    path = tmp_path / "overflow.json5"
    path.write_text(source, encoding="utf-8")
    with pytest.raises(
        ValueError, match="plate.side: length is outside supported range"
    ):
        load_target(path)
