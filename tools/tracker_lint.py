#!/usr/bin/env python3
"""Lint BACKLOG.md's table, and EXIT NON-ZERO when it is broken.

Written because this check was retyped inline six times in one session and got
it wrong once -- printing the failure next to a shell chain that committed
anyway. A verifier that reports beside the commit instead of gating it is not a
verifier. Use it as `python3 tools/tracker_lint.py && git commit ...`.

What it catches, all of which happened:
  - An unescaped `|` in prose (|H|, |eps|, a pipe inside a code span) splits the
    row and silently eats a cell. Five rows lost cells to this in one session.
  - A row with the wrong number of cells for the header.
  - Duplicate ids.

It does NOT enforce a Status cell. Rows from AP-25 on carry status as bold lead
text inside the Item cell instead; both forms are in use and the header says so.
That is a documented convention drift, not a defect, and a linter that fought it
would be noise.
"""
import io
import re
import sys

ROW = re.compile(r"^\|\s*(P[0-9])\s*\|\s*(AP-\d+)\s*\|")


def cells(line):
    """Structural cell count: escaped pipes do not split a markdown cell."""
    return line.rstrip().count("|") - line.count("\\|")


def main(path="BACKLOG.md"):
    bad, seen = [], {}
    for n, line in enumerate(io.open(path, encoding="utf-8"), 1):
        m = ROW.match(line)
        if not m:
            continue
        ident = m.group(2)
        if ident in seen:
            bad.append("%s:%d  duplicate id %s (also line %d)"
                       % (path, n, ident, seen[ident]))
        seen[ident] = n
        c = cells(line)
        if c not in (4, 5):
            bad.append("%s:%d  %s has %d structural pipes, want 4 (3 cells) or "
                       "5 (4 cells) -- an unescaped | in prose splits the row"
                       % (path, n, ident, c))
    print("%s: %d rows" % (path, len(seen)))
    for b in bad:
        print("  BROKEN " + b)
    if bad:
        print("\n%d broken row(s). Escape a literal pipe as \\| -- including "
              "inside code spans and in |x| notation." % len(bad))
        return 1
    print("  table sound")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
