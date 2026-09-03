"""Tests for the setup step engine.

The engine exists because the previous one reported success for steps whose software
was not on the machine. These tests pin the two properties that fix: a script cannot
mark itself done without its verifier passing, and editing a script invalidates the
marker that was skipping it.
"""

import re
import subprocess
import sys
from pathlib import Path

import pytest

SETUP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SETUP_DIR))

import steps as S


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    """Redirect the engine's script and marker directories into tmp_path."""
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    monkeypatch.setattr(S, "SCRIPTS_DIR", scripts)
    monkeypatch.setattr(S, "MARKER_DIR", tmp_path / ".markers")
    return scripts


def make_step(sandbox, body="exit 0", verify="true", **kwargs):
    script = sandbox / "install-thing.sh"
    script.write_text("#!/usr/bin/env bash\n" + body + "\n")
    return S.Step(
        id=kwargs.pop("id", "thing"),
        title="Thing",
        group="Test",
        script="install-thing.sh",
        verify=verify,
        why="test fixture",
        **kwargs,
    )


class Args:
    def __init__(self, step):
        self.step = step


# --- the property that matters most -------------------------------------------------


def test_script_exiting_zero_without_installing_is_a_failure(sandbox, monkeypatch):
    """A script that succeeds while its verifier fails must not be marked done.

    This is the exact shape of the old `dev-tools` step, which printed
    "cargo not found, skipping mdbook", exited 0, and was skipped forever after.
    """
    step = make_step(sandbox, body="echo pretending", verify="test -f /nope/nope")
    monkeypatch.setitem(S.BY_ID, step.id, step)

    assert S.cmd_run(Args(step.id)) != 0
    assert not step.marker.exists()


def test_verified_script_is_marked_done(sandbox, monkeypatch):
    step = make_step(sandbox, verify="true")
    monkeypatch.setitem(S.BY_ID, step.id, step)

    assert S.cmd_run(Args(step.id)) == 0
    assert step.marker.exists()
    assert step.is_done()


def test_failing_script_propagates_its_exit_code(sandbox, monkeypatch):
    step = make_step(sandbox, body="exit 3", verify="true")
    monkeypatch.setitem(S.BY_ID, step.id, step)

    assert S.cmd_run(Args(step.id)) == 3
    assert not step.marker.exists()


# --- content-addressed markers ------------------------------------------------------


def test_editing_the_script_invalidates_the_marker(sandbox):
    """Adding a package to a script must make the step run again.

    `setup/.markers/geometric-libs` existed on a machine with no libsfcgal-dev because
    the old marker recorded only the step's name.
    """
    step = make_step(sandbox)
    step.mark_done()
    assert step.is_done()

    (sandbox / "install-thing.sh").write_text(
        "#!/usr/bin/env bash\n# new package\nexit 0\n"
    )
    assert not step.is_done()
    assert step.is_stale()


def test_editing_the_verifier_invalidates_the_marker(sandbox):
    step = make_step(sandbox, verify="true")
    step.mark_done()
    assert step.is_done()

    step.verify = "test -f /somewhere/else"
    assert not step.is_done()


def test_missing_marker_is_not_stale(sandbox):
    """No marker means "installed some other way", not "installed by an old script"."""
    step = make_step(sandbox)
    assert not step.is_done()
    assert not step.is_stale()


def test_cache_never_steps_always_rerun(sandbox):
    """`ros-deps` reads every package.xml, so a durable marker on it is always wrong."""
    step = make_step(sandbox, cache=S.CACHE_NEVER)
    step.mark_done()
    assert not step.is_done()
    assert not step.is_stale()


# --- dependency resolution ----------------------------------------------------------


def test_resolve_includes_dependencies_in_order():
    order = [s.id for s in S.resolve(["colcon-rust"])]
    assert "colcon-rust" in order
    for dep in ("ros2", "rust", "system-base"):
        assert order.index(dep) < order.index("colcon-rust")


def test_resolve_rejects_unknown_step():
    with pytest.raises(SystemExit):
        S.resolve(["no-such-step"])


def test_every_step_declares_known_dependencies():
    for step in S.STEPS:
        for dep in step.needs:
            assert dep in S.BY_ID, f"{step.id} needs unknown step {dep}"


def test_every_step_has_an_existing_script():
    for step in S.STEPS:
        assert step.script_path.is_file(), f"{step.id}: missing {step.script_path}"


def test_every_step_has_a_verifier():
    for step in S.STEPS:
        assert step.verify.strip(), f"{step.id} has no verifier"


def test_verifiers_are_cheap_existence_checks():
    """Verifiers must not build anything -- they run on every status call.

    Matched on word boundaries: `cmake` in a `command -v cmake` check is fine, a bare
    `make` invocation is not.
    """
    forbidden = (r"colcon\s+build", r"cargo\s+build", r"just\s+build", r"\bmake\b")
    for step in S.STEPS:
        for pattern in forbidden:
            assert not re.search(pattern, step.verify), (
                f"{step.id}'s verifier runs a build: {step.verify}"
            )


def test_default_plan_is_orderable():
    plan = [s.id for s in S.resolve(S.default_selection())]
    assert plan
    seen = set()
    for step_id in plan:
        for dep in S.BY_ID[step_id].needs:
            if S.BY_ID[dep].applicable:
                assert dep in seen, f"{step_id} runs before its dependency {dep}"
        seen.add(step_id)


# --- the shared python-environment guard --------------------------------------------


def test_python_env_guard_is_executable_and_decisive():
    """The guard must actually run and return a boolean verdict, not always pass."""
    guard = SETUP_DIR / "scripts" / "check-python-env.sh"
    assert guard.is_file()
    result = subprocess.run(
        ["bash", str(guard)],
        capture_output=True,
        text=True,
        cwd=str(SETUP_DIR.parent),
        check=False,
    )
    assert result.returncode in (0, 1)
    if result.returncode == 1:
        assert "shadows" in result.stderr or "import cv2" in result.stderr
