#!/usr/bin/env python3
"""Step table and execution engine for LCTK development-environment setup.

Single source of truth for what setup installs. `setup/justfile` recipes delegate
here, and `setup/tui.py` renders this table, so the step list, its dependency edges,
its markers and its verifiers are all defined once.

Stdlib only, Python 3.8+. Jammy ships 3.10, which has no `tomllib`, so the table is a
Python data module rather than a config file.

Two properties matter, both of them reactions to setup steps that reported success on a
machine that did not have the software (see
`docs/superpowers/specs/2026-08-30-setup-rework-design.md`):

- **Markers are content-addressed.** A step's marker records a hash of its script and
  its verifier, so editing the script to add a package invalidates the marker instead of
  leaving the step unreachable.
- **Every step has a verifier.** It runs after the script and on `status`/`verify`
  without installing. A script that exits 0 while its verifier fails is an error, not a
  completed step. Verifiers are deliberately cheap existence checks -- `test -f`,
  `command -v`, `dpkg -s` -- never a build.
"""

import argparse
import hashlib
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

SETUP_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SETUP_DIR / "scripts"
MARKER_DIR = SETUP_DIR / ".markers"
PROJECT_ROOT = SETUP_DIR.parent

# Cache policies.
CACHE_HASH = "hash"  # skip while the script+verifier hash is unchanged (the default)
CACHE_NEVER = "never"  # always re-run: the step's real input is the working tree


class Step:
    """One installable unit.

    `verify` is a shell snippet that must exit 0 once the step's software is present.
    `needs` are step ids that must run first. `arches` restricts a step to specific
    `uname -m` values (None = every host).
    """

    def __init__(
        self,
        id,
        title,
        group,
        script,
        verify,
        why,
        needs=(),
        sudo=True,
        size_mb=0,
        optional=False,
        default_on=True,
        cache=CACHE_HASH,
        arches=None,
    ):
        self.id = id
        self.title = title
        self.group = group
        self.script = script
        self.verify = verify
        self.why = why
        self.needs = list(needs)
        self.sudo = sudo
        self.size_mb = size_mb
        self.optional = optional
        self.default_on = default_on
        self.cache = cache
        self.arches = arches

    @property
    def script_path(self):
        return SCRIPTS_DIR / self.script

    @property
    def applicable(self):
        return self.arches is None or platform.machine() in self.arches

    def fingerprint(self):
        """Hash of everything that decides whether a completed step is still valid."""
        h = hashlib.sha256()
        h.update(self.script_path.read_bytes())
        h.update(b"\0")
        h.update(self.verify.encode())
        return h.hexdigest()

    @property
    def marker(self):
        return MARKER_DIR / self.id

    def is_done(self):
        """True when a marker exists AND records the current fingerprint."""
        if self.cache == CACHE_NEVER:
            return False
        try:
            return self.marker.read_text().strip() == self.fingerprint()
        except OSError:
            return False

    def is_stale(self):
        """True when a marker exists but no longer matches the script and verifier.

        Distinct from "no marker at all": a stale marker means the step was installed
        by an older version of its script, which is worth saying out loud. A missing
        marker on verified software just means it was installed some other way.
        """
        if self.cache == CACHE_NEVER or not self.marker.exists():
            return False
        return not self.is_done()

    def mark_done(self):
        MARKER_DIR.mkdir(parents=True, exist_ok=True)
        self.marker.write_text(self.fingerprint() + "\n")

    def run_verify(self):
        r = subprocess.run(
            ["bash", "-c", self.verify],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            cwd=str(PROJECT_ROOT),
        )
        return r.returncode == 0


# Ordering within a group is display order; execution order comes from `needs`.
STEPS = [
    Step(
        id="system-base",
        title="Base system packages",
        group="Core toolchain",
        script="install-system-base.sh",
        verify="command -v cmake && command -v git && command -v curl",
        why="compilers, cmake, curl -- everything else assumes these",
        size_mb=50,
    ),
    Step(
        id="build-tools",
        title="C/C++ build and math libraries",
        group="Core toolchain",
        script="install-build-tools.sh",
        verify="dpkg -s libeigen3-dev libclang-dev libtbb-dev >/dev/null 2>&1",
        why="libclang for bindgen; Eigen/TBB for the geometry crates",
        needs=["system-base"],
        size_mb=400,
    ),
    Step(
        id="python",
        title="System Python and scientific stack",
        group="Core toolchain",
        script="install-python.sh",
        # Deliberately needs only system-base. It used to depend on ros2, which forced a
        # 2.5 GB ROS install before apt could put numpy on disk.
        verify="python3 -c 'import numpy, scipy, matplotlib'",
        why="apt numpy/scipy are the ABI the ROS and OpenCV packages were built against",
        needs=["system-base"],
        size_mb=200,
    ),
    Step(
        id="ros2",
        title="ROS 2 Humble and sensor drivers",
        group="ROS 2",
        script="install-ros2.sh",
        verify="test -d /opt/ros/humble && dpkg -s ros-humble-vision-msgs ros-humble-velodyne >/dev/null 2>&1",
        why="the runtime; vision_msgs is a hard dep of six packages in ros/",
        needs=["system-base"],
        size_mb=2500,
    ),
    Step(
        id="rosdep-init",
        title="rosdep database",
        group="ROS 2",
        script="rosdep-init.sh",
        verify="test -f /etc/ros/rosdep/sources.list.d/20-default.list",
        why="resolves package.xml keys to apt packages",
        needs=["ros2"],
        size_mb=0,
    ),
    Step(
        id="ros-deps",
        title="Workspace dependencies from package.xml",
        group="ROS 2",
        script="install-ros-deps.sh",
        verify=(
            "ROS_DISTRO=${ROS_DISTRO:-humble} rosdep check --from-paths ros --ignore-src"
            " --skip-keys 'rclrs ament_python calibration_evaluator ament_cargo'"
            " >/dev/null 2>&1"
        ),
        why="every <depend> in ros/*/package.xml; the set changes on most branches",
        needs=["rosdep-init"],
        # Its input is the working tree, not its own script. A durable marker on this
        # step is always wrong -- it is what hid four missing apt packages.
        cache=CACHE_NEVER,
        size_mb=0,
    ),
    Step(
        id="rust",
        title="Rust toolchain",
        group="Rust",
        script="install-rust.sh",
        verify="command -v cargo && command -v cargo-nextest && command -v cargo-ament-build",
        why="rustc/cargo plus the pinned cargo-ament-build and cargo-nextest",
        needs=["system-base"],
        sudo=False,
        size_mb=1000,
    ),
    Step(
        id="just",
        title="just command runner",
        group="Rust",
        script="install-just.sh",
        verify="command -v just",
        why="the build/test/lint interface; no jammy apt package, so a pinned binary",
        needs=[],
        sudo=False,
        size_mb=5,
    ),
    Step(
        id="colcon-rust",
        title="colcon-cargo-ros2",
        group="Rust",
        script="install-colcon-rust.sh",
        verify=(
            'python3 -c "import colcon_cargo_ros2" && '
            'python3 -c "import sys;from importlib.metadata import version;'
            "v=[int(x) for x in version('colcon-cargo-ros2').split('.')[:3]];"
            'sys.exit(0 if v>=[0,5,3] else 1)"'
        ),
        why="builds the Rust ROS 2 packages; >=0.5.3 required by this workspace",
        needs=["ros2", "rust"],
        sudo=False,
        size_mb=10,
    ),
    Step(
        id="python-guard",
        title="pip shadowing guard",
        group="Rust",
        script="check-python-env.sh",
        verify="bash setup/scripts/check-python-env.sh",
        why="a pip --user setuptools/numpy/scipy/anyio silently shadows the apt build",
        needs=["python", "colcon-rust"],
        sudo=False,
        size_mb=0,
        cache=CACHE_NEVER,
    ),
    Step(
        id="opencv",
        title="OpenCV",
        group="Media and sensors",
        script="install-opencv.sh",
        verify="pkg-config --exists opencv4",
        why="ArUco detection",
        needs=["system-base"],
        size_mb=500,
    ),
    Step(
        id="opencv-45-prefix",
        title="OpenCV 4.5 private prefix",
        group="Media and sensors",
        script="install-opencv-4.5-prefix.sh",
        verify='test -f "${LCTK_OPENCV_PREFIX:-$HOME/opt/opencv-4.5.4}/include/opencv4/opencv2/aruco.hpp"',
        why="JetPack's libopencv-dev 4.8 owns /usr/include/opencv4 and has no aruco",
        needs=["system-base"],
        optional=True,
        default_on=False,
        arches=("aarch64",),
        size_mb=200,
    ),
    Step(
        id="gstreamer",
        title="GStreamer",
        group="Media and sensors",
        script="install-gstreamer.sh",
        verify="pkg-config --exists gstreamer-1.0",
        why="sample-data video playback via gscam",
        needs=["system-base"],
        size_mb=300,
    ),
    Step(
        id="network-libs",
        title="Packet capture libraries",
        group="Media and sensors",
        script="install-network-libs.sh",
        verify="test -f /usr/include/pcap/pcap.h",
        why="the sample data is Velodyne pcap; lctk_sample_data replays it",
        needs=["system-base"],
        size_mb=20,
    ),
    Step(
        id="geometric-libs",
        title="SFCGAL geometry library",
        group="Media and sensors",
        script="install-geometric-libs.sh",
        verify="test -f /usr/include/SFCGAL/capi/sfcgal_c.h",
        why="sfcgal-sys backs newslab-geom-algo, a dep of aruco_locator_node",
        needs=["system-base"],
        size_mb=40,
    ),
    Step(
        id="lint-tools",
        title="ruff and uv",
        group="Test and lint tooling",
        script="install-lint-tools.sh",
        verify="command -v ruff && command -v uv",
        why="just lint needs ruff; regenerating parity fixtures needs uv (L-25)",
        needs=["system-base"],
        sudo=False,
        size_mb=60,
    ),
    Step(
        id="cuda",
        title="CUDA toolkit 11.8",
        group="Optional",
        script="install-cuda.sh",
        verify="test -d /usr/local/cuda",
        why="GPU acceleration; nothing in the default build path requires it",
        needs=["system-base"],
        optional=True,
        default_on=False,
        size_mb=3000,
    ),
    Step(
        id="dev-tools-debug",
        title="Debuggers and profilers",
        group="Optional",
        script="install-dev-tools-debug.sh",
        verify="command -v gdb && command -v valgrind && command -v ccache",
        why="gdb, valgrind, cppcheck, ccache",
        needs=["system-base"],
        optional=True,
        size_mb=500,
    ),
    Step(
        id="dev-tools-docs",
        title="Documentation tools (mdbook)",
        group="Optional",
        script="install-dev-tools-docs.sh",
        # Split out of dev-tools precisely so this dependency is declared. The combined
        # step needed only system-base, so on a machine without cargo it printed
        # "skipping mdbook", exited 0, and marked itself done forever.
        verify="command -v mdbook && command -v mdbook-mermaid",
        why="builds book/; needs cargo, which is why it is no longer bundled with gdb",
        needs=["rust"],
        optional=True,
        sudo=False,
        size_mb=100,
    ),
    Step(
        id="submodules",
        title="Git submodules",
        group="Repository",
        script="update-submodules.sh",
        verify="test -f ros/conflux/conflux_cpp/package.xml && test -f rust/multi-stream-synchronizer/Cargo.toml",
        why="conflux and multi-stream-synchronizer; just build needs them",
        needs=[],
        optional=True,
        default_on=False,
        sudo=False,
        cache=CACHE_NEVER,
        size_mb=0,
    ),
]

BY_ID = {s.id: s for s in STEPS}
GROUPS = []
for _s in STEPS:
    if _s.group not in GROUPS:
        GROUPS.append(_s.group)


def applicable_steps():
    return [s for s in STEPS if s.applicable]


def resolve(ids):
    """Expand `ids` with their dependencies and return them in execution order.

    Raises on an unknown id or a dependency cycle.
    """
    for i in ids:
        if i not in BY_ID:
            raise SystemExit(f"error: unknown step '{i}'")

    ordered, visiting, done = [], set(), set()

    def visit(sid):
        if sid in done:
            return
        if sid in visiting:
            raise SystemExit(f"error: dependency cycle at '{sid}'")
        visiting.add(sid)
        for dep in BY_ID[sid].needs:
            visit(dep)
        visiting.discard(sid)
        done.add(sid)
        ordered.append(sid)

    for i in ids:
        visit(i)
    return [BY_ID[i] for i in ordered if BY_ID[i].applicable]


def default_selection():
    return [s.id for s in applicable_steps() if s.default_on]


# --- output helpers -------------------------------------------------------------


def _tty():
    return sys.stdout.isatty()


C = {
    "green": "\033[0;32m",
    "yellow": "\033[0;33m",
    "red": "\033[0;31m",
    "blue": "\033[0;34m",
    "dim": "\033[2m",
    "off": "\033[0m",
}


def c(name, text):
    return f"{C[name]}{text}{C['off']}" if _tty() else text


# --- commands -------------------------------------------------------------------


def cmd_list(args):
    for s in applicable_steps():
        print(
            f"{s.id}\t{s.group}\t{s.title}\t{int(s.optional)}\t{int(s.default_on)}\t{s.size_mb}"
        )
    return 0


def cmd_plan(args):
    ids = args.steps or default_selection()
    for s in resolve(ids):
        print(s.id)
    return 0


def cmd_status(args):
    failed = 0
    for group in GROUPS:
        members = [s for s in applicable_steps() if s.group == group]
        if not members:
            continue
        print(f"\n{c('blue', group)}")
        for s in members:
            ok = s.run_verify()
            if not ok and not s.optional:
                failed += 1
            if ok:
                state = c("green", "installed")
            elif s.optional:
                state = c("dim", "not installed (optional)")
            else:
                state = c("red", "MISSING")
            stale = ""
            if ok and s.is_stale():
                stale = c("yellow", "  (installed by an older version of the script)")
            print(f"  {s.id:<20} {state}{stale}")
    print()
    return 1 if failed else 0


def cmd_verify(args):
    steps = resolve(args.steps) if args.steps else applicable_steps()
    bad = []
    for s in steps:
        if s.run_verify():
            print(f"  {c('green', 'ok')}    {s.id}")
        elif s.optional:
            print(f"  {c('dim', 'skip')}  {s.id:<20} {s.why}")
        else:
            bad.append(s)
            print(f"  {c('red', 'FAIL')}  {s.id:<20} {s.why}")
    if bad:
        print(f"\n{c('red', 'verification failed')}: {', '.join(s.id for s in bad)}")
        print("Run './setup.sh' to install what is missing.")
        return 1
    print(f"\n{c('green', 'all required steps verified')}")
    return 0


def cmd_run(args):
    s = BY_ID.get(args.step)
    if s is None:
        raise SystemExit(f"error: unknown step '{args.step}'")
    if not s.applicable:
        print(f"{c('dim', '[skip]')} {s.id} (not applicable to {platform.machine()})")
        return 0

    if s.is_done() and s.run_verify():
        print(f"{c('green', '[done]')} {s.id}")
        return 0

    print(f"{c('yellow', '->')} {s.title} ({s.id})")
    env = dict(os.environ, PROJECT_ROOT=str(PROJECT_ROOT), ARCH=platform.machine())
    r = subprocess.run(
        ["bash", str(s.script_path)], cwd=str(PROJECT_ROOT), env=env, check=False
    )
    if r.returncode != 0:
        print(f"{c('red', '[fail]')} {s.id}: script exited {r.returncode}")
        return r.returncode

    # A script that exits 0 without installing anything is the failure mode this
    # engine exists to catch, so the verifier decides, not the exit status.
    if not s.run_verify():
        print(f"{c('red', '[fail]')} {s.id}: script succeeded but verification failed")
        print(f"        check: {s.verify}")
        return 1

    if s.cache != CACHE_NEVER:
        s.mark_done()
    print(f"{c('green', '[done]')} {s.id}")
    return 0


def cmd_clean(args):
    if args.steps:
        for i in args.steps:
            if i not in BY_ID:
                raise SystemExit(f"error: unknown step '{i}'")
            BY_ID[i].marker.unlink(missing_ok=True)
            print(f"cleared marker: {i}")
    else:
        shutil.rmtree(MARKER_DIR, ignore_errors=True)
        print("cleared all markers")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(prog="steps.py", description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="tab-separated step table").set_defaults(fn=cmd_list)

    sp = sub.add_parser("plan", help="print execution order for the given steps")
    sp.add_argument("steps", nargs="*")
    sp.set_defaults(fn=cmd_plan)

    sub.add_parser("status", help="verify every step against the machine").set_defaults(
        fn=cmd_status
    )

    sv = sub.add_parser(
        "verify", help="run verifiers; non-zero if a required one fails"
    )
    sv.add_argument("steps", nargs="*")
    sv.set_defaults(fn=cmd_verify)

    sr = sub.add_parser("run", help="run one step (marker check, script, verify, mark)")
    sr.add_argument("step")
    sr.set_defaults(fn=cmd_run)

    sc = sub.add_parser("clean", help="clear markers")
    sc.add_argument("steps", nargs="*")
    sc.set_defaults(fn=cmd_clean)

    args = p.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
