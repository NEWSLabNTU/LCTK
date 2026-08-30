#!/usr/bin/env python3
"""Curses selector for LCTK setup steps.

Stdlib `curses` only. The whole point of this screen is to run on a machine that has
nothing installed yet, so it must not need pip, and it must not need `just` -- both are
things setup is here to provide.

Writes the chosen step ids, one per line, to the file named by `--out`, and exits 0.
Exits 1 if the user quits. The result goes to a file rather than stdout because curses
owns stdout for the duration -- redirecting it into a pipe breaks the display.

Everything the user sees comes from `steps.py`, and the right-hand column is live
verifier output, so the screen reports what is on the machine rather than what a marker
claims.
"""

import argparse
import curses
import sys

import steps as S

HELP = "space toggle  ·  a/n all/none  ·  v re-verify  ·  ⏎ install  ·  q quit"


class Row:
    """One line on screen: either a group header or a step."""

    def __init__(self, kind, group, step=None):
        self.kind = kind  # "group" | "step"
        self.group = group
        self.step = step


def build_rows():
    rows = []
    for group in S.GROUPS:
        members = [s for s in S.applicable_steps() if s.group == group]
        if not members:
            continue
        rows.append(Row("group", group))
        for st in members:
            rows.append(Row("step", group, st))
    return rows


def verify_all():
    return {s.id: s.run_verify() for s in S.applicable_steps()}


def group_state(rows, group, selected):
    """Return 'all', 'none' or 'some' for a group's steps."""
    ids = [r.step.id for r in rows if r.kind == "step" and r.group == group]
    on = sum(1 for i in ids if i in selected)
    if on == 0:
        return "none"
    return "all" if on == len(ids) else "some"


def toggle_group(rows, group, selected):
    ids = [r.step.id for r in rows if r.kind == "step" and r.group == group]
    if group_state(rows, group, selected) == "all":
        for i in ids:
            selected.discard(i)
    else:
        selected.update(ids)


def draw(win, rows, selected, installed, cursor, top, message):
    win.erase()
    height, width = win.getmaxyx()
    body_height = max(1, height - 4)

    title = " LCTK setup "
    win.addnstr(0, 0, title.ljust(width - 1), width - 1, curses.A_REVERSE)
    win.addnstr(1, 1, HELP[: width - 2], width - 2, curses.A_DIM)

    for offset in range(body_height):
        index = top + offset
        if index >= len(rows):
            break
        row = rows[index]
        y = 2 + offset
        focused = index == cursor
        attr = curses.A_REVERSE if focused else curses.A_NORMAL

        if row.kind == "group":
            state = group_state(rows, row.group, selected)
            box = {"all": "[x]", "none": "[ ]", "some": "[~]"}[state]
            win.addnstr(
                y, 1, f"{box} {row.group}"[: width - 2], width - 2, attr | curses.A_BOLD
            )
            continue

        st = row.step
        box = "[x]" if st.id in selected else "[ ]"
        size = (
            f"~{st.size_mb / 1000:.1f} GB"
            if st.size_mb >= 1000
            else (f"~{st.size_mb} MB" if st.size_mb else "")
        )
        left = f"  {box} {st.title}"
        mid = f"{size}"
        if installed.get(st.id):
            right = "installed"
        elif st.optional:
            right = "not installed"
        else:
            right = "MISSING"
        if st.cache == S.CACHE_NEVER:
            right = "always re-runs" if installed.get(st.id) else right

        line = f"{left:<44}{mid:>10}   {right}"
        win.addnstr(y, 1, line[: width - 2], width - 2, attr)

    total = sum(S.BY_ID[i].size_mb for i in selected if not installed.get(i))
    need_sudo = any(S.BY_ID[i].sudo for i in selected if not installed.get(i))
    summary = (
        f" {len(selected)} selected · ~{total / 1000:.1f} GB to install"
        f"{' · sudo required' if need_sudo else ''} "
    )
    if message:
        summary = f" {message} "
    win.addnstr(
        height - 1,
        0,
        summary.ljust(width - 1)[: width - 1],
        width - 1,
        curses.A_REVERSE,
    )
    win.refresh()


def run(stdscr):
    curses.curs_set(0)
    stdscr.keypad(True)

    rows = build_rows()
    installed = verify_all()
    # Preselect the defaults, minus anything already on the machine, plus anything
    # required that is missing. A step whose real input is the tree always re-runs.
    selected = set()
    for st in S.applicable_steps():
        if not st.default_on:
            continue
        if st.cache == S.CACHE_NEVER or not installed.get(st.id):
            selected.add(st.id)

    cursor = next((i for i, r in enumerate(rows) if r.kind == "step"), 0)
    top = 0
    message = ""

    while True:
        height, _ = stdscr.getmaxyx()
        body_height = max(1, height - 4)
        if cursor < top:
            top = cursor
        elif cursor >= top + body_height:
            top = cursor - body_height + 1

        draw(stdscr, rows, selected, installed, cursor, top, message)
        message = ""
        key = stdscr.getch()

        if key in (curses.KEY_DOWN, ord("j")):
            cursor = min(cursor + 1, len(rows) - 1)
        elif key in (curses.KEY_UP, ord("k")):
            cursor = max(cursor - 1, 0)
        elif key == curses.KEY_NPAGE:
            cursor = min(cursor + body_height, len(rows) - 1)
        elif key == curses.KEY_PPAGE:
            cursor = max(cursor - body_height, 0)
        elif key == curses.KEY_HOME:
            cursor = 0
        elif key == curses.KEY_END:
            cursor = len(rows) - 1
        elif key == ord(" "):
            row = rows[cursor]
            if row.kind == "group":
                toggle_group(rows, row.group, selected)
            elif row.step.id in selected:
                selected.discard(row.step.id)
            else:
                selected.add(row.step.id)
        elif key == ord("a"):
            selected.update(r.step.id for r in rows if r.kind == "step")
        elif key == ord("n"):
            selected.clear()
        elif key == ord("v"):
            installed = verify_all()
            message = "re-verified against the machine"
        elif key in (curses.KEY_ENTER, 10, 13):
            return sorted(selected)
        elif key in (ord("q"), 27):
            return None


def main():
    parser = argparse.ArgumentParser(prog="tui.py")
    parser.add_argument(
        "--out", required=True, help="file to write the selected step ids into"
    )
    args = parser.parse_args()

    if not sys.stdin.isatty():
        print("error: tui.py needs a terminal", file=sys.stderr)
        return 2

    chosen = curses.wrapper(run)
    if chosen is None:
        return 1
    # Dependencies are resolved by the engine; record only what the user picked.
    with open(args.out, "w") as handle:
        handle.writelines(step_id + "\n" for step_id in chosen)
    return 0


if __name__ == "__main__":
    sys.exit(main())
