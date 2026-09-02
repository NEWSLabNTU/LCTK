"""Play a rosbag2 recording, but not before anything is listening.

`session.launch.py` starts the playback and the calibration graph together, and
the playback wins: `ros2 bag play` is a Python entry point that begins publishing
in about a second, while the Rust detectors have to load, read a Target
Definition and build a background model. Every message published into that gap is
gone -- the bag topics replay BEST_EFFORT and VOLATILE, so a subscriber that
arrives late gets nothing that was sent before it -- and with RViz, the overlay
and the judge enabled the gap can swallow a short recording whole.

The obvious fix is a `sleep`, and it is the wrong one: long enough on a loaded
Jetson is wasted time everywhere else, and short enough to feel quick is a silent
data-loss bug on the next slower machine. The condition that actually matters is
observable -- *are the subscriptions up yet?* -- so this waits for that, then
execs the real player.

Timing out is deliberately **not** fatal. A session may legitimately name a topic
nothing in the graph consumes, and refusing to play the recording would be a
worse failure than playing it into a partly-assembled graph. The wait is a
safeguard, not a gate.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time


def missing_subscribers(node, topics: list[str]) -> list[str]:
    """Which of `topics` nothing has subscribed to yet.

    Split out from the wait loop so the decision is testable without a graph.
    """
    return [
        topic for topic in topics if not node.get_subscriptions_info_by_topic(topic)
    ]


def wait_for_subscribers(topics: list[str], timeout_s: float) -> list[str]:
    """Block until every topic has a subscriber. Returns those still missing."""
    if not topics:
        return []

    import rclpy
    from rclpy.node import Node

    rclpy.init(args=[])
    try:
        node = Node("lctk_bag_play_waiter")
        deadline = time.monotonic() + timeout_s
        try:
            while True:
                missing = missing_subscribers(node, topics)
                if not missing or time.monotonic() >= deadline:
                    return missing
                # Discovery is event-driven; spinning briefly lets it land.
                rclpy.spin_once(node, timeout_sec=0.2)
                time.sleep(0.1)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag2 directory to play")
    parser.add_argument(
        "--wait-for",
        action="append",
        default=[],
        metavar="TOPIC",
        help="wait until this topic has a subscriber before playing (repeatable)",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=60.0,
        help="give up waiting after this many seconds and play anyway",
    )
    parser.add_argument(
        "--play-arg",
        action="append",
        default=[],
        metavar="ARG",
        help="extra argument passed through to `ros2 bag play`",
    )
    # This runs as a launch `Node` (see session_data.launch.py for why it must),
    # and launch_ros appends `--ros-args -r __node:=... --params-file ...` to
    # every node it starts. None of it means anything to a bag player, so it is
    # dropped rather than allowed to fail the parse.
    raw = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in raw:
        raw = raw[: raw.index("--ros-args")]
    args = parser.parse_args(raw)

    if args.wait_for:
        print(
            f"[lctk_bag_play] waiting for subscribers on "
            f"{', '.join(args.wait_for)} (up to {args.wait_timeout:.0f}s)",
            flush=True,
        )
        started = time.monotonic()
        missing = wait_for_subscribers(args.wait_for, args.wait_timeout)
        waited = time.monotonic() - started
        if missing:
            print(
                f"[lctk_bag_play] WARNING: after {waited:.1f}s nothing subscribes to "
                f"{', '.join(missing)}; playing anyway. Whatever those topics feed "
                "will see no data -- check the manifest names the topics the graph "
                "actually uses.",
                flush=True,
            )
        else:
            print(
                f"[lctk_bag_play] graph is listening after {waited:.1f}s; playing",
                flush=True,
            )

    ros2 = shutil.which("ros2")
    if ros2 is None:
        print("[lctk_bag_play] `ros2` is not on PATH", file=sys.stderr)
        return 1
    command = [ros2, "bag", "play", args.bag, *args.play_arg]
    os.execv(ros2, command)  # replaces this process, so Ctrl-C reaches the player


if __name__ == "__main__":
    raise SystemExit(main())
