#!/usr/bin/env python3
"""Extract a sounder run's gate metrics from its log, reproducibly.

Written because the gate criteria (0 escalations, 0 off-grid, the CNS low
fraction, the sync residual spread) were previously read out of logs by eye,
and DEMO_VERIFICATION 8.51 records a residual-dispersion change that nobody
could explain partly because nobody had the per-run distribution in front of
them. Numbers a decision rests on should come from a script that can be re-run
on the same file and produce the same answer.

It also censuses WARN/ERROR lines, because a run can hit every numeric gate
while logging something new that nobody looked at.

  python3 tests/demo-verify/gate_summary.py run1.log run2.log ...
  python3 tests/demo-verify/gate_summary.py --json out.json run*.log
"""
import argparse
import json
import math
import re
import sys
from collections import Counter

RE_ALIVE = re.compile(r"beacon alive on the anchored grid \(resid ([+-]?\d+) "
                      r"within scatter, snr ([\d.]+) dB")
RE_OFFGRID = re.compile(r"off-grid detection ([+-]?\d+)")
# ONE ESCALATION LOGS FOUR DIFFERENT LINES (receiver.cc 1786 / 1837 / 1851 /
# 1855), so matching the common prefix counted a single event 2-3 times and the
# multiplier CHANGED silently when this branch added the period-disagreement
# WARN at 1837. Match only the line that fires exactly once per escalation.
RE_ESC = re.compile(r"Re-sync ESCALATION \(")
# CASE. receiver.cc:1855 logs "re-acquisition did not confirm" in lowercase,
# so this never matched and a run in which EVERY re-anchor failed scored
# reanchor_failed = 0 -- the instrument reporting the worst failure mode as
# clean. Anchored case-insensitively on the stable part of the sentence.
RE_REANCHOR_FAIL = re.compile(r"re-acquisition did not confirm", re.I)
RE_CNS_OK = re.compile(r"CNS score ([\d.]+) rot ([+-][\d.]+) deg at frame "
                       r"(\d+) \((\d+) datagrams, (\d+) low\)")
RE_ACQ = re.compile(r"lock CONFIRMED \(resid ([+-]?\d+) over (\d+)")
# Throttled too (1st and every 100th), same as RE_INNOV. Take the total the
# text carries, never the line count.
RE_STARVED = re.compile(r"tracker did NOT update -- (\d+) such accepts so far")
# THROTTLED, LIKE THE CNS WARNING. The line prints on the 1st rejection and
# every 50th, so COUNTING LINES under-reports by up to 50x -- measured
# 2026-09-02: 3 lines against "(50 so far)". This is the same defect this
# same file was written to fix for CNS, missed here because I fixed the one
# instance I had found instead of grepping the sounder for the pattern.
# The running total is in the text; take it.
RE_INNOV = re.compile(r"innovation [+-]?[\d.]+ sigma exceeds .*?\((\d+) so far")
RE_GEOM = re.compile(r"Beacon accept window (\d+) samples of a (\d+)-sample "
                     r"slot \(([\d.]+)%\), scatter tol (\d+) samples")
RE_TRACKER = re.compile(r"Grid tracker: (\S+)")
RE_CADENCE = re.compile(r"resync every ([\d.]+) ms")
RE_LEVEL = re.compile(r"\b(WARNG|WARNING|ERROR)\b[]:]?\s*(.*)")


def sd(xs):
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def analyse(path):
    r = {"log": path, "resid": [], "snr": [], "offgrid": [], "escalations": 0,
         "reanchor_failed": 0, "starved": 0, "innov_rejected": 0,
         "cns_total": 0, "cns_low": 0, "acq": None, "geometry": None,
         "tracker": None, "cadence_ms": None, "levels": Counter(),
         "level_samples": {}}
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = RE_ALIVE.search(line)
            if m:
                r["resid"].append(int(m.group(1)))
                r["snr"].append(float(m.group(2)))
                continue
            m = RE_OFFGRID.search(line)
            if m:
                r["offgrid"].append(int(m.group(1)))
                continue
            if RE_ESC.search(line):
                r["escalations"] += 1
            if RE_REANCHOR_FAIL.search(line):
                r["reanchor_failed"] += 1
            m = RE_STARVED.search(line)
            if m:
                r["starved"] = max(r["starved"], int(m.group(1)))
            m = RE_INNOV.search(line)
            if m:
                r["innov_rejected"] = max(r["innov_rejected"], int(m.group(1)))
            # ONLY THE PERIODIC SUMMARY LINE CARRIES A TOTAL. The sounder emits
            # two CNS forms and they are NOT interchangeable:
            #   summary  "CNS score X rot Y deg at frame N (D datagrams, L low)"
            #   warning  "CNS score X at frame N, r=Z (low occurrence K of D)"
            # The warning is THROTTLED ON A DOUBLING SCHEDULE, so K is only ever
            # 1, 2, 4, 8, 16, 32 -- it reports which occurrence tripped the log,
            # not how many there were. Reading K as a count produced a table
            # whose every entry was a power of two (measured 2026-09-02, 15 runs,
            # values 2/4/8/16/32 with no exceptions), which is the signature of
            # counting log lines rather than events. It nearly shipped as a CNS
            # regression against DEMO_VERIFICATION 8.51. Take the summary only.
            m = RE_CNS_OK.search(line)
            if m:
                # Cumulative, so the LAST one wins rather than summing.
                if int(m.group(4)) >= r["cns_total"]:
                    r["cns_total"] = int(m.group(4))
                    r["cns_low"] = int(m.group(5))
                continue
            m = RE_ACQ.search(line)
            if m and r["acq"] is None:
                r["acq"] = {"resid": int(m.group(1)), "frames": int(m.group(2))}
            m = RE_GEOM.search(line)
            if m:
                r["geometry"] = {"window": int(m.group(1)),
                                 "slot": int(m.group(2)),
                                 "pct": float(m.group(3)),
                                 "scatter_tol": int(m.group(4))}
            m = RE_TRACKER.search(line)
            if m:
                r["tracker"] = m.group(1)
            m = RE_CADENCE.search(line)
            if m:
                r["cadence_ms"] = float(m.group(1))
            m = RE_LEVEL.search(line)
            if m:
                # Collapse digits so "frame 4414" and "frame 11967" are one
                # category; the point is which KINDS of complaint appeared.
                kind = re.sub(r"\d+", "N", m.group(2))[:70].strip()
                r["levels"][(m.group(1), kind)] += 1
                r["level_samples"].setdefault(kind, m.group(2).strip()[:110])
    return r


def report(rs):
    print("%-22s %7s %6s %7s %7s %8s %8s %7s %9s"
          % ("run", "accept", "esc", "offgrid", "sd", "max|r|", "starved",
             "innovRej", "CNS low"))
    for r in rs:
        res = r["resid"]
        print("%-22s %7d %6d %7d %7.2f %8d %8d %7d  %d/%d"
              % (r["log"].split("/")[-1], len(res), r["escalations"],
                 len(r["offgrid"]), sd(res), max([abs(x) for x in res] or [0]),
                 r["starved"], r["innov_rejected"], r["cns_low"],
                 r["cns_total"]))
    print()
    for r in rs:
        g, name = r["geometry"], r["log"].split("/")[-1]
        print("%-22s tracker=%s cadence=%s ms geometry=%s"
              % (name, r["tracker"], r["cadence_ms"],
                 ("%d samp / %d (%.1f%%), tol %d" %
                  (g["window"], g["slot"], g["pct"], g["scatter_tol"]))
                 if g else "<absent>"))
        if r["acq"]:
            print("%-22s   acquisition confirmed resid %+d over %d frames"
                  % ("", r["acq"]["resid"], r["acq"]["frames"]))
    print()
    # Consistency across runs is the point of running more than once.
    if len(rs) > 1:
        for key, fn in (("accept", lambda r: len(r["resid"])),
                        ("resid sd", lambda r: sd(r["resid"])),
                        ("max |resid|", lambda r: max([abs(x) for x in r["resid"]] or [0])),
                        ("CNS low frac", lambda r: (r["cns_low"] / r["cns_total"]
                                                    if r["cns_total"] else 0.0))):
            vals = [fn(r) for r in rs]
            lo, hi = min(vals), max(vals)
            print("  %-14s %s   spread %.4g" % (key,
                  " ".join("%.4g" % v for v in vals), hi - lo))
    print()
    print("=== WARN/ERROR census (kinds, digits collapsed) ===")
    agg = Counter()
    samples = {}
    for r in rs:
        for k, n in r["levels"].items():
            agg[k] += n
        samples.update(r["level_samples"])
    if not agg:
        print("  none")
    for (lvl, kind), n in agg.most_common(20):
        print("  %-6s %5d  %s" % (lvl, n, samples.get(kind, kind)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--json")
    a = ap.parse_args()
    rs = [analyse(p) for p in a.logs]
    report(rs)
    if a.json:
        out = []
        for r in rs:
            d = dict(r)
            d["levels"] = {"%s|%s" % k: v for k, v in r["levels"].items()}
            d["resid_sd"] = sd(r["resid"])
            d["resid_max_abs"] = max([abs(x) for x in r["resid"]] or [0])
            d["accepted"] = len(r["resid"])
            out.append(d)
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1)
        print("\nwrote", a.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
