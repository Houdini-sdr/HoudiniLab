"""The sounder log lines that more than one run tool reads, parsed in one place.
Stdlib only. Each pattern is the sounder's own format, and the source that
prints it is named, so a change to a log line is one change here.

  link health   RadioHoudini.cc's health thread: "<UE|BS> <id> link health: ...".
                A counter that moved in the period prints as "<name> +<increment>"
                (tx<ch>.<counter> for a TX stream); link_health.h line() gives
                the other item forms alarm_kinds reads.
  CNS summary   recorder_worker.cc: "CNS score ... at frame F (D datagrams, L low)".
                D and L are cumulative, so the last summary is the run's total.
                The per-event warning's "low occurrence K of D" is throttled on
                powers of two (AP-58): it is not a count and is not read.
"""
import re
from collections import Counter

ANSI = re.compile(r"\x1b\[[0-9;]*m")
LINK_HEALTH = re.compile(r"\b(UE|BS) \S+ link health: .*")
TX_INCREMENT = re.compile(r"(tx\d\.\w+) \+(\d+)")
CNS_SUMMARY = re.compile(r"\((\d+) datagrams, (\d+) low\)")
ALARM_ITEM = re.compile(r"((?:tx|rx)\d\.\w+ \+\d+|(?:egress|host)\.\w+ \+\d+|preflight new FAIL [^|;]+"
                        r"|(?:rx|tx)_\w+ \+[1-9]\d*)")


def read_lines(path):
    """The log's lines with the colour codes removed."""
    with open(path, errors="replace") as f:
        return [ANSI.sub("", l) for l in f.read().splitlines()]


def tx_totals(lines):
    """Counter {(role, "tx<ch>.<counter>"): the increments summed over the
    role's link-health lines}. Each node prints the same keys for its own
    streams, so the role keeps the UE's apart from the BS's."""
    out = Counter()
    for l in lines:
        m = LINK_HEALTH.search(l)
        if m:
            for k, v in TX_INCREMENT.findall(m.group(0)):
                out[(m.group(1), k)] += int(v)
    return out


def cns_summaries(lines):
    """[(line index, datagrams, low)], one per CNS summary line, in log order."""
    return [(i, int(m.group(1)), int(m.group(2))) for i, l in enumerate(lines)
            for m in [CNS_SUMMARY.search(l)] if m]


def cns_total(lines):
    """(datagrams, low) from the last CNS summary, or None when there is none."""
    s = cns_summaries(lines)
    return s[-1][1:] if s else None


def alarm_kinds(lines):
    """Counter of the alarm kinds in link-health lines, one entry per item, in
    the forms link_health.h writes them: a counter rise '<name> +N', a new
    preflight FAIL, and the app counters."""
    out = Counter()
    for l in lines:
        for m in ALARM_ITEM.finditer(l):
            out[re.sub(r"\+\d+", "+N", m.group(1)).strip()] += 1  # an item before ' |' keeps no trailing space
    return out
