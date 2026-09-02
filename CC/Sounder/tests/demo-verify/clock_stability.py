#!/usr/bin/env python3
"""Allan deviation of the two-node clock offset, from a clock_drift_probe run.

Sets the tolerance budget for the sync cadence (AP-31). The question the demo
actually asks is not "what is eps" -- that is measured and stable to 4 decimal
places over a single run -- but "how far can the UE coast between beacon
observations before its prediction leaves the guard band". That is the Allan
deviation of eps at tau = the observation gap, and nothing else.

Instrument choice matters. hwtime_rate_probe's RPC jitter is ~106 us per
sample, which only resolves ~0.06 ppm over a 120 s fit and is useless below
tau ~ 60 s. The beacon ARRIVAL tick is sample-accurate (jitter measured 8-23
samples = 0.07-0.19 us), roughly 3000x finer, so the arrival phase series is
the right input.

Reads the probe's json (needs the `ticks` array, i.e. NOT the archived summary
with the arrays stripped).

    python3 clock_stability.py clock_stability.json
"""
import argparse
import json
import math
import sys

import numpy as np

FRAME = 122880
RATE = 122.88e6


def phase_seconds(ticks):
    """Beacon arrival phase against a fixed nominal grid, in seconds.

    Same unwrap as the probe: the residual against k*FRAME IS the accumulated
    time difference between the BS and UE clocks.
    """
    t = np.asarray(ticks, dtype=np.int64)
    k = np.round((t - t[0]) / FRAME)
    r = (t - t[0]) - k * FRAME
    out = r.astype(np.float64).copy()
    add = 0.0
    for i in range(1, len(out)):
        d = (out[i] + add) - out[i - 1]
        if d > FRAME / 2:
            add -= FRAME * round(d / FRAME)
        elif d < -FRAME / 2:
            add += FRAME * round(-d / FRAME)
        out[i] += add
    return (t - t[0]) / RATE, out / RATE, out


def bin_phase(t, x, tau0):
    """Average the irregular detections into even tau0 bins (reduces the white
    phase noise by sqrt(n) before the ADEV, which is what makes short tau
    readable at all)."""
    nb = int(np.floor((t[-1] - t[0]) / tau0))
    tb, xb, cnt = [], [], []
    for i in range(nb):
        lo, hi = t[0] + i * tau0, t[0] + (i + 1) * tau0
        m = (t >= lo) & (t < hi)
        if m.sum() == 0:
            tb.append(np.nan); xb.append(np.nan); cnt.append(0)
        else:
            # Timestamp the bin at the MEAN DETECTION TIME, not the bin
            # centre. Detections arrive irregularly, so their mean time wanders
            # from the centre by ~tau0/sqrt(12n); against a real ppm ramp that
            # wander converts straight into per-bin phase error -- white in
            # phase, so it yields an ADEV falling as 1/tau and a FLAT
            # drift-in-samples column. That is exactly the "measurement floor"
            # plateau this tool reported, and it scales with eps, so it is
            # absent at eps=0 and largest on the fastest leg: it masquerades as
            # a clock property while being the binning's own artifact.
            tb.append(float(t[m].mean())); xb.append(x[m].mean()); cnt.append(int(m.sum()))
    return np.array(tb), np.array(xb), np.array(cnt)


def adev(x, tau0, m):
    """Overlapping Allan deviation of fractional frequency, from phase x[s]."""
    n = len(x)
    if n < 2 * m + 1:
        return None, 0
    d = x[2 * m:] - 2 * x[m:n - m] + x[:n - 2 * m]
    d = d[np.isfinite(d)]
    if len(d) < 2:
        return None, 0
    tau = m * tau0
    return math.sqrt(np.sum(d ** 2) / (2 * len(d) * tau ** 2)), len(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json")
    ap.add_argument("--tau0", type=float, default=0.5, help="bin width (s)")
    ap.add_argument("--guard", type=float, default=128.0,
                    help="ofdm_tx_zero_prefix, the timing slack in samples")
    args = ap.parse_args()

    d = json.load(open(args.json))
    if "ticks" not in d:
        print("this json has no `ticks` array (archived summary?) -- need the "
              "full probe output")
        return 2
    t, x, x_samp = phase_seconds(d["ticks"])
    print("run %s: %d detections over %.1f s" % (d.get("label", "?"), len(t), t[-1]))
    print("  eps from the whole-run slope : %+.4f ppm"
          % (-np.polyfit(t, x_samp, 1)[0] / RATE * 1e6))

    tb, xb, cnt = bin_phase(t, x, args.tau0)
    ok = np.isfinite(xb)
    print("  binned at %.2f s: %d/%d bins populated, median %d detections/bin"
          % (args.tau0, ok.sum(), len(xb), int(np.median(cnt[cnt > 0]))))
    # de-trend: ADEV is about the CHANGE in rate, not the rate itself
    sl, ic = np.polyfit(tb[ok], xb[ok], 1)
    xd = xb - (sl * tb + ic)
    print("  residual after removing the mean rate: %.3f us rms"
          % (np.nanstd(xd) * 1e6))
    print()
    print("%8s %14s %14s %10s" % ("tau (s)", "ADEV (ppm)", "drift (samples)", "pairs"))
    for m in (1, 2, 4, 10, 20, 40, 100, 200, 400):
        tau = m * args.tau0
        s, npair = adev(xd, args.tau0, m)
        if s is None:
            continue
        # what it costs operationally: coasting for tau accumulates this much
        # timing error, against a `guard`-sample budget.
        samples = s * tau * RATE
        flag = ""
        if samples > args.guard:
            flag = "  <- past the guard"
        elif samples > args.guard / 4:
            flag = "  <- past 1/4 guard"
        print("%8.1f %14.5f %14.2f %10d%s"
              % (tau, s * 1e6, samples, npair, flag))
    print()
    print("white phase noise (measurement floor) falls as 1/tau; oscillator")
    print("wander rises as sqrt(tau). The minimum is where coasting stops")
    print("getting cheaper -- that is the X to pick.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
