#!/usr/bin/env python3
"""Are short reads dropped packets? Recover each sample's time and measure it.

A short read is either (a) the driver handing back a partial window early with
NO data missing, or (b) rx_gap_break truncating at a real hole. Those have
opposite consequences and look identical from the return length alone.

THE TEST. Every readStream reports the time of its FIRST sample. So for reads
k and k+1:

    expected_start(k+1) = t[k] + ret[k]        (contiguous)
    gap                 = t[k+1] - expected_start(k+1)

gap == 0 means nothing was lost, whatever the length. gap > 0 is exactly the
number of samples that never arrived. That distinguishes (a) from (b) and
sizes (b), which a count of short reads cannot.

WHY IT MATTERS HERE. Runs with many short reads showed MORE detections per
second (93.9 vs 61.5) and LESS arrival jitter (7.5 vs 11.2 samples) than runs
with none -- backwards from what loss would do. That is correlational across
runs that differ in several ways, so it argues for measuring rather than
concluding. If gap is ~0 on short reads, they are early returns and the
"1-in-6 short reads" observation is not loss at all.

  python3 tests/demo-verify/rx_gap_census.py --ue <ip> --reads 4000
"""
import argparse
import sys

import numpy as np

import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_HAS_TIME

RPC_TIMEOUT_US = "30000000"
RATE = 122.88e6



def census(a, nsamps, reads):
    """One (node, request size) point: short-read rate and minimum length."""
    dev = SoapySDR.Device(dict(driver="houdinisdr",
                               remote="tcp://%s:55132" % a.ue,
                               timeout=RPC_TIMEOUT_US))
    dev.setSampleRate(SOAPY_SDR_RX, a.ch, RATE)
    dev.setFrequency(SOAPY_SDR_RX, a.ch, a.freq)
    rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [a.ch],
                          dict(local_port=str(10001 + a.ch),
                               rx_gap_break=a.gap_break))
    dev.activateStream(rxs)
    buf = np.zeros(2 * nsamps, dtype=np.int16)
    rets, times = [], []
    for _ in range(reads):
        sr = dev.readStream(rxs, [buf], nsamps, timeoutUs=1000000)
        if sr.ret > 0:
            rets.append(int(sr.ret))
            times.append(int(round(sr.timeNs * RATE / 1e9))
                         if (sr.flags & SOAPY_SDR_HAS_TIME) else None)
    dev.deactivateStream(rxs)
    dev.closeStream(rxs)
    if not rets:
        return None
    short = sum(1 for r in rets if r != nsamps)
    # THE SWEEP MUST CARRY THE GAP COLUMN OR IT CANNOT SETTLE THE QUESTION IT
    # EXISTS FOR. The first version reported only the short-read RATE, and the
    # whole point of AP-59 is that a short read is an early return with nothing
    # missing. A rate that climbs to 96.8 % at 32768 samples means nothing until
    # you know whether those reads lost anything, and reporting the rate without
    # the gap invites exactly the reading the row was written to retire.
    # gap = t[k+1] - (t[k] + ret[k]); zero means nothing was lost, whatever the
    # read length. [user proposed this test.]
    gaps, pairs, worst = 0, 0, 0
    for k in range(len(rets) - 1):
        if times[k] is None or times[k + 1] is None:
            continue
        pairs += 1
        g = times[k + 1] - (times[k] + rets[k])
        if g != 0:
            gaps += 1
            worst = max(worst, abs(g))
    return {"n": len(rets), "short_pct": 100.0 * short / len(rets),
            "min": min(rets), "gaps": gaps, "pairs": pairs, "worst_gap": worst}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ue", required=True)
    ap.add_argument("--ch", type=int, default=1)
    ap.add_argument("--freq", type=float, default=500e6)
    ap.add_argument("--nsamps", type=int, default=12288)
    ap.add_argument("--reads", type=int, default=4000)
    ap.add_argument("--sweep-nsamps", default="",
                    help="comma-separated request sizes to sweep, e.g. "
                         "2048,4096,12288,32768,65536. Discriminates a "
                         "BUFFERING BOUNDARY (rate varies with request size "
                         "against some quantum) from a timing property (rate "
                         "flat in request size). Either answer narrows the "
                         "node asymmetry; another hypothesis does not.")
    ap.add_argument("--gap-break", default="1",
                    help="rx_gap_break setting. Run it BOTH ways: with 1 the "
                         "driver truncates at a hole, with 0 it does not, and "
                         "the difference between the two is the answer.")
    a = ap.parse_args()

    if a.sweep_nsamps:
        # Sweep mode: same node, same stream settings, request size varied.
        # Reports the rate AND the minimum returned length, because those
        # discriminate differently -- a rate difference at equal minima is a
        # timing property, while different minima say the batch SIZE
        # distribution differs, not only its cadence.
        print("%-8s %-9s %-9s %-7s %-9s %-9s %-10s"
              % ("nsamps", "short %", "min ret", "reads", "gaps", "pairs",
                 "worst gap"))
        bad, unmeasured, total_pairs = 0, 0, 0
        for ns in [int(v) for v in a.sweep_nsamps.split(",") if v.strip()]:
            r = census(a, ns, a.reads)
            if r is None:
                print("%-8d %s" % (ns, "no reads"))
                unmeasured += 1
                continue
            if r["gaps"]:
                bad += 1
            if r["pairs"] == 0:
                unmeasured += 1
            total_pairs += r["pairs"]
            print("%-8d %-9.1f %-9d %-7d %-9d %-9d %-10d"
                  % (ns, r["short_pct"], r["min"], r["n"], r["gaps"],
                     r["pairs"], r["worst_gap"]))
        print()
        if unmeasured:
            # ZERO PAIRS IS NOT ZERO LOSS. `gaps` only counts pairs where both
            # reads carried SOAPY_SDR_HAS_TIME; if the device never sets it,
            # pairs == 0 and gaps == 0, and the old verdict printed "zero gaps at
            # every request size" from a run that measured nothing. That is the
            # exact "a guard returns something the caller cannot tell from a
            # measurement" pattern -- in the tool that settled AP-59.
            print("NO LOSS MEASUREMENT at %d request size(s): no read pair "
                  "carried a timestamp," % unmeasured)
            print("so the gap test could not run. This is NOT a zero-loss "
                  "result. Check that the")
            print("stream reports SOAPY_SDR_HAS_TIME before reading anything "
                  "into the rates above.")
            return 2
        if bad:
            print("SAMPLES WERE LOST at %d request size(s): a short read at "
                  "those sizes is NOT" % bad)
            print("just an early return. That changes AP-59's answer -- report it.")
            return 1
        print("Zero gaps at every request size, over %d measured pairs: every "
              "short read is an" % total_pairs)
        print("early return with nothing missing, so the short-read RATE is not "
              "a loss measurement.")
        return 0

    dev = SoapySDR.Device(dict(driver="houdinisdr",
                               remote="tcp://%s:55132" % a.ue,
                               timeout=RPC_TIMEOUT_US))
    dev.setSampleRate(SOAPY_SDR_RX, a.ch, RATE)
    dev.setFrequency(SOAPY_SDR_RX, a.ch, a.freq)
    rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [a.ch],
                          dict(local_port=str(10001 + a.ch),
                               rx_gap_break=a.gap_break))
    dev.activateStream(rxs)
    buf = np.zeros(2 * a.nsamps, dtype=np.int16)

    rets, times, hastime = [], [], 0
    for _ in range(a.reads):
        sr = dev.readStream(rxs, [buf], a.nsamps, timeoutUs=1000000)
        if sr.ret <= 0:
            continue
        rets.append(int(sr.ret))
        if sr.flags & SOAPY_SDR_HAS_TIME:
            hastime += 1
            times.append(int(round(sr.timeNs * RATE / 1e9)))
        else:
            times.append(None)
    dev.deactivateStream(rxs)
    dev.closeStream(rxs)

    n = len(rets)
    if n < 10:
        print("only %d reads; nothing to say." % n)
        return 1
    short = [i for i, r in enumerate(rets) if r != a.nsamps]
    print("rx_gap_break=%s, %d reads of %d samples requested" % (a.gap_break, n, a.nsamps))
    print("  short reads      : %d of %d (%.1f%%), min %d"
          % (len(short), n, 100.0 * len(short) / n, min(rets)))
    print("  reads with a time: %d of %d" % (hastime, n))
    if hastime < n:
        print("  NOTE: not every read carried a timestamp, so the gaps below are")
        print("  computed only across consecutive pairs that both did.")
    print()

    gaps, gaps_after_short, gaps_after_full = [], [], []
    for k in range(n - 1):
        if times[k] is None or times[k + 1] is None:
            continue
        g = times[k + 1] - (times[k] + rets[k])
        gaps.append(g)
        (gaps_after_short if rets[k] != a.nsamps else gaps_after_full).append(g)

    if not gaps:
        print("no consecutive timestamped pairs; cannot measure gaps.")
        return 1

    def show(label, xs):
        if not xs:
            print("  %-26s (none)" % label)
            return
        arr = np.asarray(xs, dtype=np.int64)
        nz = arr[arr != 0]
        print("  %-26s n=%5d  zero %5d (%.1f%%)  nonzero %4d  "
              "median nz %s  max %d"
              % (label, len(arr), int((arr == 0).sum()),
                 100.0 * (arr == 0).sum() / len(arr), len(nz),
                 ("%d" % int(np.median(nz))) if len(nz) else "-",
                 int(arr.max()) if len(arr) else 0))

    print("SAMPLE GAP between consecutive reads (0 = contiguous, >0 = lost):")
    show("all pairs", gaps)
    show("after a SHORT read", gaps_after_short)
    show("after a FULL read", gaps_after_full)
    print()
    arr = np.asarray(gaps_after_short, dtype=np.int64) if gaps_after_short else np.zeros(0, dtype=np.int64)
    if len(arr) == 0:
        print("  VERDICT: no short reads in this run, so nothing to attribute.")
    elif (arr == 0).all():
        print("  VERDICT: EVERY short read was followed by a contiguous next read.")
        print("  Nothing was lost. Short reads here are EARLY RETURNS, not gaps,")
        print("  and a count of them is not a loss measurement.")
    else:
        frac = 100.0 * (arr != 0).sum() / len(arr)
        tot = int(arr[arr != 0].sum())
        print("  VERDICT: %.1f%% of short reads were followed by a real gap." % frac)
        print("  Total samples lost: %d over %d reads = %.1f ppm of the stream."
              % (tot, n, 1e6 * tot / float(n * a.nsamps)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
