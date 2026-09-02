#!/usr/bin/env python3
"""Size the host plugin's `rx_rearm_drain_ms` from a measurement, not a guess.

The software lane made the END_BURST re-arm drain a settable setupStream arg
(0-50 ms, default 5, a conservative bound on the device's rounded whole-frame
excess) and asked for the smallest value that never produces an empty or short
read. That number, with margin, becomes the default.

TWO THINGS THIS SCRIPT INSISTS ON, because a sweep is easy to get wrong:

1. IT SWEEPS A VALUE THAT MUST FAIL. If every value passes, "safe" and "my test
   cannot see failure" look identical, and only the second one is likely. 0 ms
   is included by default and a run where 0 ms passes cleanly is reported as
   INCONCLUSIVE rather than as a result.

2. IT REPEATS EVERY VALUE. A marginal drain passes some runs by luck. The
   answer is the smallest value that passes EVERY repeat, not the smallest that
   passed once -- one run is not a behaviour.

  python3 tests/demo-verify/rearm_drain_sweep.py --ue <ip> --values 0,1,2,3,5 \
      --reps 60 --repeats 3
"""
import argparse
import statistics
import subprocess
import sys
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
PROBE = os.path.join(HERE, "burst_rx_cost_probe.py")


def run_one(ue, drain, reps, lead):
    """A FAILED SUBPROCESS MUST NOT LOOK LIKE A CLEAN RESULT.

    Measured 2026-09-02: the probe rejected an unrecognised --rearm-drain-ms,
    printed a usage message to stderr, and produced no stdout. Every regex below
    then found nothing, which this function reported as empty=0 and short=0 --
    a perfectly clean sweep across fifteen runs that never started. The
    INCONCLUSIVE guard on 0 ms caught it, but only by luck of the design; this
    check catches it directly.
    """
    r = subprocess.run(
        [sys.executable, PROBE, "--ue", ue, "--reps", str(reps),
         "--lead-frames", str(lead), "--rearm-drain-ms", str(drain),
         # The drain only applies to an arm that follows a delivered burst with
         # NO deactivate in between, so a sweep that deactivates measures
         # nothing about it.
         "--no-deactivate"],
        capture_output=True, text=True, timeout=900)
    out = r.stdout
    if r.returncode != 0 or "leg B" not in out:
        raise SystemExit(
            "probe FAILED at drain=%d (rc=%d). This is a broken measurement,\n"
            "not a clean one. stderr:\n%s\nstdout tail:\n%s"
            % (drain, r.returncode, (r.stderr or "")[-600:], out[-600:]))
    empty = 0
    short = 0
    cycle = None
    m = re.search(r"(\d+) burst\(s\) returned NOTHING", out)
    if m:
        empty = int(m.group(1))
    m = re.search(r"(\d+) burst\(s\) truncated", out)
    if m:
        short = int(m.group(1))
    m = re.search(r"burst FULL CYCLE\s+([\d.]+) us", out)
    if m:
        cycle = float(m.group(1))
    m = re.search(r"arm \(activateStream\)\s+n=\s*(\d+)", out)
    n = int(m.group(1)) if m else 0
    return {"drain": drain, "empty": empty, "short": short, "cycle": cycle, "n": n}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ue", required=True)
    ap.add_argument("--values", default="0,1,2,3,5")
    ap.add_argument("--reps", type=int, default=60)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--lead-frames", type=int, default=8)
    a = ap.parse_args()

    vals = [int(v) for v in a.values.split(",") if v.strip() != ""]
    if 0 not in vals:
        print("REFUSING: 0 ms is not in the sweep. Without a value that is")
        print("expected to fail, a clean sweep cannot be told apart from a test")
        print("that cannot detect failure. Add 0 to --values.")
        return 2

    print("%-8s %-9s %-8s %-8s %-12s" % ("drain", "repeat", "empty", "short", "cycle us"))
    results = {}
    for d in sorted(vals):
        results[d] = []
        for r in range(a.repeats):
            res = run_one(a.ue, d, a.reps, a.lead_frames)
            results[d].append(res)
            print("%-8d %-9d %-8d %-8d %-12s"
                  % (d, r + 1, res["empty"], res["short"],
                     ("%.0f" % res["cycle"]) if res["cycle"] else "-"))

    print()
    zero_clean = all(x["empty"] == 0 and x["short"] == 0 for x in results[0])
    if zero_clean:
        print("INCONCLUSIVE. A 0 ms drain produced no empty and no short reads in")
        print("any repeat. Either the drain is not needed at all on this build --")
        print("in which case say so explicitly and let the software lane confirm")
        print("from their side -- or this probe cannot see the failure it is")
        print("looking for. Do NOT report a minimum from this run.")
        return 1

    print("0 ms DOES fail (empty %s, short %s), so the sweep can see failure."
          % ([x["empty"] for x in results[0]], [x["short"] for x in results[0]]))
    best = None
    for d in sorted(vals):
        if all(x["empty"] == 0 and x["short"] == 0 for x in results[d]):
            best = d
            break
    if best is None:
        print("No swept value was clean in every repeat. Sweep higher.")
        return 1
    cyc = [x["cycle"] for x in results[best] if x["cycle"]]
    print("SMALLEST CLEAN VALUE: %d ms, clean in %d of %d repeats, cycle %.0f us"
          % (best, a.repeats, a.repeats,
             statistics.fmean(cyc) if cyc else float("nan")))
    print()
    print("Recommend the DEFAULT be above this, not at it: the sweep bounds")
    print("where it breaks on THIS bench on THIS day, and the quantity being")
    print("drained is a device-side excess that other conditions may lengthen.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
