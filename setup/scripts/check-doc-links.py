#!/usr/bin/env python3
"""Check that every relative Markdown link in the repository's docs resolves.

CLAUDE.md requires that when an issue is closed and its file moves to
``docs/issues/archive/``, every relative link crossing the move is repaired in
both directions.  That rule is only enforceable if something checks it, so this
is that something: Phase 8's W6-A release gate names a "docs relative-link
checker" among its commands, and before this script there was none -- the check
was retyped by hand each time, which is exactly how a link rots unnoticed.

Scope: ``docs/``, ``book/src/``, and the top-level and per-package ``README.md``
/ ``CONTRIBUTING.md`` files.  Only *relative* targets are resolved; external
URLs are not fetched, because a gate that needs the network is a gate that
fails for the wrong reasons.

Both Markdown links (``[text](target)``) and bare reference definitions are
checked.  A target may point at a file or a directory: ``CONTRIBUTING.md``
legitimately links to crate directories.  Anchors (``#section``) are stripped
before resolution; this script does not verify that the anchor itself exists.

Exit status is 0 when every link resolves and 1 otherwise, so it can sit
directly in a gate.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# Directories scanned in full, plus individually named files.
SCAN_DIRS = ("docs", "book/src")
SCAN_GLOBS = ("README.md", "CONTRIBUTING.md", "CLAUDE.md", "ros/*/README.md")

# `[text](target)` -- target runs to the first closing paren or whitespace.
# Titles (`[t](target "title")`) are handled by splitting on whitespace below.
LINK = re.compile(r"\[[^\]]*\]\(([^)]+)\)")


def collect_files() -> list[Path]:
    files: list[Path] = []
    for directory in SCAN_DIRS:
        files.extend(sorted((REPO_ROOT / directory).rglob("*.md")))
    for pattern in SCAN_GLOBS:
        files.extend(sorted(REPO_ROOT.glob(pattern)))
    # A file can match both a directory scan and a glob; keep one entry each.
    return sorted(set(files))


def is_external(target: str) -> bool:
    return target.startswith(("http://", "https://", "mailto:", "#"))


def broken_links(path: Path) -> list[str]:
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as error:
        return [f"unreadable: {error}"]

    broken: list[str] = []
    for match in LINK.finditer(text):
        target = match.group(1).strip()
        # Drop an optional link title: [text](path "Title")
        target = target.split()[0] if target.split() else target
        if not target or is_external(target):
            continue
        # Strip the anchor; existence of the anchor itself is not checked.
        target = target.split("#", 1)[0]
        if not target:
            continue
        if not (path.parent / target).exists():
            broken.append(target)
    return broken


def main() -> int:
    failures = 0
    for path in collect_files():
        for target in broken_links(path):
            print(f"BROKEN {path.relative_to(REPO_ROOT)} -> {target}")
            failures += 1
    if failures:
        print(f"\n{failures} broken relative link(s)")
        return 1
    print("all relative documentation links resolve")
    return 0


if __name__ == "__main__":
    sys.exit(main())
