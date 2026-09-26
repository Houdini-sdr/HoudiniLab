#!/usr/bin/env python3
"""Board sample-clock rate probe -- NO RF, no beacon, no correlator.

Each board's getHardwareTime() is its own sample counter rendered as ns, so it
advances at that board's sample clock. Reading both boards alternately against
the host's monotonic clock and fitting each gives a rate per board; the RATIO of
the two fits is eps = (f_BS - f_UE)/f_UE with the host's own clock error
cancelled to first order, because both fits share it.

This exists because the RF instrument goes blind exactly when it is most needed:
a large enough offset decoheres the 128-tap gold matched filter, so a node whose
reference is far off produces no detections at all and the beacon channel cannot
say whether the cause is frequency, level, or a dead path. This probe answers
"how far off is each board" regardless, at roughly ppm precision -- coarse next
to the beacon arrival ramp, but it works in the regime where that ramp does not
exist yet, and it needs nothing on air.

    python3 hwtime_rate_probe.py --duration 120
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import SoapySDR


# SoapyRemote's `timeout` device arg is MICROSECONDS, and it bounds the
# make() RPC. Measured 2026-09-01 on this bench: a COLD make (the server
# holds no live device instance, so construction runs the full RFDC
# bring-up) takes 3.34 s; a WARM one 0.34 s. The long-standing 1000000
# (= 1 s) therefore sits INSIDE the normal spread, and make() failed with
# "SoapyRPCUnpacker::recv() TIMEOUT" three times in one session depending
# only on whether a previous run still held the instance. That is a slow
# call against a short deadline, NOT an unresponsive server -- do not
# read it as one.
RPC_TIMEOUT_US = "30000000"


def open_dev(ip):
    return SoapySDR.Device(dict(driver="houdinisdr",
                                remote="tcp://%s:55132" % ip,
                                timeout=RPC_TIMEOUT_US))


def fit(x, y):
    """Slope + standard error of y on x (both 1-D, len >= 3)."""
    a, b = np.polyfit(x, y, 1)
    r = y - (a * x + b)
    sy = float(np.sqrt(np.sum(r ** 2) / (len(x) - 2)))
    sxx = float(np.sum((x - x.mean()) ** 2))
    return float(a), (sy / math.sqrt(sxx) if sxx > 0 else None), sy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bs-ip", default="168.6.244.21")
    ap.add_argument("--ue-ip", default="168.6.244.22")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--period", type=float, default=0.05,
                    help="seconds between sample pairs")
    ap.add_argument("--label", default="run")
    ap.add_argument("--out", default="hwtime_rate_probe.json")
    args = ap.parse_args()

    devs = {"bs": open_dev(args.bs_ip), "ue": open_dev(args.ue_ip)}
    for k, d in devs.items():
        hi = dict(d.getHardwareInfo())
        print("  %s = %s (%s) clock_ref=%s" % (k.upper(), hi.get("label", "?"),
                                               hi.get("ip_address", ""),
                                               hi.get("clock_ref", "?")))
    rows = {"bs": [], "ue": []}
    t_end = time.time() + args.duration
    try:
        while time.time() < t_end:
            for k, d in devs.items():
                # Bracket the RPC so the host timestamp carries its own
                # uncertainty: use the midpoint, and keep the width so a
                # stalled call can be rejected rather than silently skew a fit.
                h0 = time.monotonic()
                ns = d.getHardwareTime()
                h1 = time.monotonic()
                rows[k].append((0.5 * (h0 + h1), ns, h1 - h0))
            time.sleep(args.period)
    finally:
        pass

    out = {"label": args.label, "duration_req": args.duration}
    rate = {}
    for k in ("bs", "ue"):
        # Never lose a completed run to an analysis edge case: too few samples,
        # or an MAD filter that rejects everything, is a NULL RESULT to report
        # and write out -- not an exception after the hardware time is spent.
        if len(rows[k]) < 3:
            print("%s: only %d samples, cannot fit" % (k.upper(), len(rows[k])))
            out[k] = {"n": len(rows[k]), "error": "too_few_samples"}
            rate[k] = float("nan")
            continue
        a = np.array(rows[k], dtype=np.float64)
        host, ns, width = a[:, 0], a[:, 1], a[:, 2]
        # Drop calls whose RPC bracket is an outlier: those carry a host
        # timestamp uncertainty comparable to the effect being measured.
        keep = width <= np.median(width) + 5 * (np.median(np.abs(
            width - np.median(width))) + 1e-9)
        if keep.sum() < 3:
            print("%s: RPC-width filter kept only %d of %d samples"
                  % (k.upper(), keep.sum(), len(keep)))
            out[k] = {"n": int(keep.sum()), "n_dropped": int((~keep).sum()),
                      "error": "all_samples_rejected"}
            rate[k] = float("nan")
            continue
        host, ns = host[keep] - host[keep][0], ns[keep] - ns[keep][0]
        slope, se, sy = fit(host, ns * 1e-9)   # board seconds per host second
        rate[k] = slope
        out[k] = dict(n=int(keep.sum()), n_dropped=int((~keep).sum()),
                      span_s=float(host[-1]), rate_vs_host=slope,
                      rate_se=se, fit_rms_s=sy,
                      rpc_width_ms=float(np.median(width) * 1e3))
        print("%s: %d pts over %.1f s, rate %+.6f ppm vs host "
              "(se %.3f ppm), fit rms %.3f ms, rpc %.1f ms"
              % (k.upper(), keep.sum(), host[-1], (slope - 1) * 1e6,
                 (se or 0) * 1e6, sy * 1e3, np.median(width) * 1e3))
    if not all(np.isfinite(v) for v in rate.values()):
        print("eps NOT COMPUTED -- see the per-board errors above")
        out["eps_ppm"] = None
        with open(args.out, "w") as f:
            json.dump(out, f, indent=1, sort_keys=True)
        print("wrote %s" % args.out)
        return 2
    eps = rate["bs"] / rate["ue"] - 1.0
    out["eps_ppm"] = eps * 1e6
    print("eps = (f_BS - f_UE)/f_UE = %+.3f ppm  "
          "(host clock error cancels in the ratio)" % (eps * 1e6))
    print("  => carrier offset at 500 MHz: %+.1f kHz" % (eps * 500e6 / 1e3))
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1, sort_keys=True)
    print("wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
