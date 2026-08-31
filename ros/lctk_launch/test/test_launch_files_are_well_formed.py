"""Every launch file in this repo must actually load.

A malformed launch file passes `just build`, `just test` and `just lint` without
a murmur: none of them parse XML, and Python launch files are only imported by
the tests that happen to load them. The failure surfaces at `ros2 launch` time as
a wall of nested exceptions, which is the worst place to learn about a typo.

This bit for real. A `--` inside an XML comment -- illegal, since it terminates
the comment -- shipped green through all four gates and broke `just demo`.
"""

from __future__ import annotations

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]


def _launch_files(suffix: str) -> list[Path]:
    """Launch files this repo owns. `ros/conflux` is a submodule: upstream's problem."""
    return sorted(
        path
        for path in (REPO_ROOT / "ros").rglob(f"*{suffix}")
        if "conflux" not in path.parts and "build" not in path.parts
    )


XML_LAUNCH = _launch_files(".launch.xml")
PY_LAUNCH = _launch_files(".launch.py")


def test_there_are_launch_files_to_check():
    """A glob that silently matches nothing would make every test below vacuous."""
    assert XML_LAUNCH, "no .launch.xml found -- the glob is wrong"
    assert PY_LAUNCH, "no .launch.py found -- the glob is wrong"


@pytest.mark.parametrize("path", XML_LAUNCH, ids=lambda p: p.name)
def test_xml_launch_file_is_well_formed(path: Path):
    try:
        ET.parse(path)
    except ET.ParseError as error:
        pytest.fail(f"{path.relative_to(REPO_ROOT)} is not well-formed XML: {error}")


@pytest.mark.parametrize("path", PY_LAUNCH, ids=lambda p: p.name)
def test_python_launch_file_parses(path: Path):
    """Syntax only -- importing would need a ROS context and the package on sys.path."""
    try:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except SyntaxError as error:
        pytest.fail(f"{path.relative_to(REPO_ROOT)} does not parse: {error}")
