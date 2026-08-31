"""End-to-end smoke check: does a session actually produce detections?

Every other suite in this repo is unit-level. They prove a manifest parses, names
a real crop box, and generates the right graph -- and a session can pass all of
that while producing nothing at all. That gap is not hypothetical: it is
[M-29](../../../docs/issues/M-29-sample-data-path-dead-shared-bbox-and-icp-gate.md),
where a shared crop box and an ICP gate below the sensor noise floor together
killed the shipped demo while every component reported success. It recurred on
2026-09-01 when four new sessions shipped with a bbox-free preset that silently
found zero clusters.

The thing that catches this class of failure is playing the recording and
asserting data comes out the other end. That is what this file does.

**Not part of `just test`.** It lives outside `test/` deliberately, because each
session takes tens of seconds of real playback -- too slow for the edit loop.
Run it with `just smoke`.

Design notes, since both matter for trusting the result:

- It **polls with a deadline** rather than sleeping a fixed time. A `sleep` long
  enough for this machine is a false pass on a slower one, and a false failure on
  a loaded one.
- Teardown kills the **process group**, in a `finally`. `ros2 launch` spawns a
  driver, a decoder, detectors and solvers; killing the parent alone orphans them
  and the next session then fails on topics that are still live. That is the trap
  `CLAUDE.md` Known Issue 6 describes.
"""

from __future__ import annotations

import contextlib
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SESSIONS = REPO_ROOT / "sessions"

# How long to wait for the first non-empty detection. Playback needs a moment to
# start, the detector needs a warmup, and the synchronizer needs a matched pair;
# 90 s is generous on purpose, and the poll exits as soon as data arrives.
DETECTION_TIMEOUT_S = 90.0
POLL_INTERVAL_S = 2.0


def _pcap_sessions() -> list[Path]:
    """Shipped sessions this check can drive: LCTK plays their data itself.

    `live` sessions need hardware and `bag` sessions need a gitignored recording,
    so neither can be asserted on in a check that must pass from a clean clone.
    """
    import yaml

    found = []
    for manifest in sorted(SESSIONS.glob("*/session.yaml")):
        data = (yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}).get("data")
        if (data or {}).get("kind") == "pcap_avi":
            found.append(manifest.parent)
    return found


PCAP_SESSIONS = _pcap_sessions()


def test_there_are_sessions_to_smoke():
    """A glob matching nothing would make every check below vacuously green."""
    assert PCAP_SESSIONS, f"no pcap_avi sessions found under {SESSIONS}"


def _detection_topic(session_dir: Path) -> str:
    sys.path.insert(0, str(REPO_ROOT / "ros" / "lctk_launch"))
    from lctk_launch.config_parser import parse_config

    pipeline = parse_config(str(session_dir / "session.yaml"))
    detectors = pipeline.lidar_board_detectors
    assert detectors, f"{session_dir.name} generates no board detector"
    return detectors[0].output_topic


def _first_nonempty_detection(topic: str, deadline: float) -> str | None:
    """Poll `topic` until a message carries a detection, or the deadline passes."""
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["ros2", "topic", "echo", "--once", topic],
            capture_output=True,
            text=True,
            timeout=20,
            check=False,  # a topic with no publisher yet is expected, not an error
        )
        if "class_id:" in result.stdout:
            return result.stdout
        time.sleep(POLL_INTERVAL_S)
    return None


@pytest.mark.parametrize("session_dir", PCAP_SESSIONS, ids=lambda p: p.name)
def test_session_produces_real_detections(session_dir: Path):
    """Play the recording and assert the detector publishes something.

    An empty `detections: []` array is a *well-formed* message, so the graph, the
    synchronizer statistics and the solver all look healthy while the pipeline is
    dead. Asserting on the array's contents is the only thing that separates the
    two.
    """
    topic = _detection_topic(session_dir)
    log = session_dir / "out" / "smoke.log"
    log.parent.mkdir(exist_ok=True)

    with log.open("w") as handle:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "lctk_launch",
                "session.launch.py",
                f"session:={session_dir}",
                "enable_rviz:=false",
                "enable_overlay:=false",
                "enable_judge:=false",
            ],
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,  # its own process group, so teardown is total
            cwd=REPO_ROOT,
        )
    try:
        message = _first_nonempty_detection(
            topic, deadline=time.monotonic() + DETECTION_TIMEOUT_S
        )
        text = log.read_text(encoding="utf-8", errors="replace")

        if message is None:
            # The detector explains itself; surfacing that beats "assert failed".
            reasons = sorted(
                {
                    line.strip()
                    for line in text.splitlines()
                    if "no board selected" in line or "target rejected" in line
                }
            )[:3]
            pytest.fail(
                f"{session_dir.name} published no detections within "
                f"{DETECTION_TIMEOUT_S:.0f}s on {topic}.\n"
                + (
                    "Detector said:\n  " + "\n  ".join(reasons)
                    if reasons
                    else "The detector logged no rejection either -- check that the "
                    "recording played and the topics match."
                )
            )

        rejected = text.count("target rejected")
        assert rejected == 0, (
            f"{session_dir.name} detected the board but rejected {rejected} frame(s); "
            "a gate is sitting too tight, which is the M-29 failure mode"
        )
    finally:
        # Kill the whole group: ros2 launch spawns a driver, a decoder, detectors
        # and solvers, and orphans break every session after this one.
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=15)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            with contextlib.suppress(ProcessLookupError):
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        time.sleep(3)  # let DDS discovery drop the dead endpoints
