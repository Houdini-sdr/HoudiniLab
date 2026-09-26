#!/usr/bin/env python3
"""Per-shape, per-round table for a run_shape_campaign.sh directory.

Built on gate_summary.analyse so every number that gate_summary already defines
(accepts, escalations, off-grid, acquisition residual, residual sd) is the SAME
number here; this adds only what the shape comparison needs on top:

  jitter   adjacent-difference residual jitter, sd(resid[k+1] - resid[k]) /
           sqrt(2), which cancels the clock wander a raw sd is dominated by
           (DEMO_VERIFICATION 8.111); reported per run, not gated.
  lowsnr   detections the SNR floor rejected. The log line is THROTTLED (1st
           and every 16th), so the count is the largest "count N" seen, never
           the number of lines.
  bcfo sd  scatter of the beacon's own CFO reading (the TRS-pair estimator)
           over the logged "Beacon CFO frame" lines, in Hz.
  forced   1 if the run logged the single-copy replica line, i.e. the plain
           matched filter was in effect (nr_pss).

  python3 tests/demo-verify/shape_campaign_summary.py logs/shape_A
"""
import glob
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gate_summary import analyse, sd  # noqa: E402

RE_LOW = re.compile(r"rejected low-SNR detection .*count (\d+)")
RE_BCFO = re.compile(r"Beacon CFO frame \d+: tracked ([+-]?[\d.]+) Hz .*\| beacon "
                     r"([+-]?[\d.]+|[+-]?nan) Hz")
RE_FORCED = re.compile(r"threshold form forced to (nolag|coherence)")
RE_NAME = re.compile(r"([a-z0-9_]+)_r(\d+)\.log$")


def extra(path):
    low, bcfo, forced = 0, [], 0
    with open(path, errors="replace") as f:
        for line in f:
            m = RE_LOW.search(line)
            if m:
                low = max(low, int(m.group(1)))
                continue
            m = RE_BCFO.search(line)
            if m:
                try:
                    v = float(m.group(2))
                    if not math.isnan(v):
                        bcfo.append(v - float(m.group(1)))
                except ValueError:
                    pass
                continue
            if RE_FORCED.search(line):
                forced = 1
    return low, bcfo, forced


def jitter(resid):
    d = [b - a for a, b in zip(resid, resid[1:])]
    return sd(d) / math.sqrt(2.0) if len(d) > 1 else 0.0


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    d = sys.argv[1]
    rows = []
    for p in sorted(glob.glob(os.path.join(d, "*_r*.log"))):
        m = RE_NAME.search(p)
        if not m or os.path.basename(p).startswith("teardown"):
            continue
        r = analyse(p)
        low, bcfo, forced = extra(p)
        rows.append({
            "shape": m.group(1), "round": int(m.group(2)),
            "accepts": len(r["resid"]), "esc": r["escalations"],
            "offgrid": len(r["offgrid"]),
            "acq": r["acq"]["resid"] if r["acq"] else None,
            "resid_sd": sd(r["resid"]), "jitter": jitter(r["resid"]),
            "max_resid": max([abs(x) for x in r["resid"]] or [0]),
            "lowsnr": low, "bcfo_sd": sd(bcfo), "bcfo_n": len(bcfo),
            "forced": forced,
        })
    rows.sort(key=lambda x: (x["round"], x["shape"]))
    print("%-13s %2s %7s %3s %4s %5s %8s %7s %5s %6s %8s %6s" %
          ("shape", "rd", "accepts", "esc", "offg", "acq", "resid sd",
           "jitter", "max", "lowsnr", "bcfo sd", "forced"))
    for x in rows:
        print("%-13s %2d %7d %3d %4d %5s %8.2f %7.2f %5d %6d %8.0f %6d" %
              (x["shape"], x["round"], x["accepts"], x["esc"], x["offgrid"],
               "none" if x["acq"] is None else "%+d" % x["acq"], x["resid_sd"],
               x["jitter"], x["max_resid"], x["lowsnr"], x["bcfo_sd"],
               x["forced"]))
    print()
    shapes = sorted(set(x["shape"] for x in rows))
    print("%-13s %6s %8s %8s %7s %7s %8s" %
          ("shape", "rounds", "accepts", "esc+offg", "sd mean", "jitter",
           "lowsnr"))
    for s in shapes:
        xs = [x for x in rows if x["shape"] == s]
        n = len(xs)
        print("%-13s %6d %8.1f %8d %7.2f %7.2f %8d" %
              (s, n, sum(x["accepts"] for x in xs) / n,
               sum(x["esc"] + x["offgrid"] for x in xs),
               sum(x["resid_sd"] for x in xs) / n,
               sum(x["jitter"] for x in xs) / n,
               sum(x["lowsnr"] for x in xs)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
