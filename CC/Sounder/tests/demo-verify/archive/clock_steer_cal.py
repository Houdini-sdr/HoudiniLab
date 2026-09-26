#!/usr/bin/env python3
"""AP-48: measure the CLOCK_ADJ actuator gain with OUR estimator.

The two source documents disagree on the actuator's scale -- HOUDINI_PROTOCOL 2.1
says ~0.129 ppm per DAC count, CLOCK_STEER_BENCH says ~0.087 and calls it a lower
bound. That 1.5x spread IS the loop gain of the steering loop (AP-47), so it gets
measured rather than inherited.

We are well placed to measure it: the beacon arrival ramp resolves eps to ~0.00002
ppm over 120 s, four orders of magnitude finer than the 0.129 ppm quantum being
calibrated. Sweep the UE's hold code, measure eps at each, fit a line.

The sweep is on ONE node only (the UE): the BS is the reference and the UE
conforms, so steering the UE is both the architecturally correct direction and the
one that leaves the reference untouched.

ALWAYS releases the clock back to its calibrated hold on the way out, including on
exception, because the hold is bench state that outlives this process.

    python3 clock_steer_cal.py --codes -20,-10,0,10,20 --dwell 60
"""
import argparse
import json
import os
import subprocess
import sys
import time

import SoapySDR

RPC_TIMEOUT_US = "30000000"


def open_dev(ip):
    return SoapySDR.Device(dict(driver="houdinisdr",
                                remote="tcp://%s:55132" % ip,
                                timeout=RPC_TIMEOUT_US))


def read_state(dev):
    """readSetting -> dict. 'holdover=1 man_dac=404 rb_dac=404 ... offset=0'."""
    txt = dev.readSetting("CLOCK_ADJ")
    out = {}
    for tok in str(txt).split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    out["_raw"] = str(txt)
    return out


def set_offset(dev, cal_dac, off):
    """Set the ABSOLUTE code, not a relative push: a sweep that pushes
    relatively accumulates any refused or clamped step silently. Verify by
    readback -- writeSetting returning is not evidence the DAC moved."""
    want = int(cal_dac) + int(off)
    dev.writeSetting("CLOCK_ADJ", str(want))
    time.sleep(0.4)                      # settles <100 ms per the protocol doc
    st = read_state(dev)
    got = int(st.get("rb_dac", -1))
    if got != want:
        raise RuntimeError("CLOCK_ADJ readback %d != wanted %d (%s)"
                           % (got, want, st["_raw"]))
    return st


def measure_eps(probe, gold, core, dwell, tag, outdir):
    """One clock_drift_probe run -> eps in ppm from the beacon arrival ramp."""
    out = "%s/steercal_%s.json" % (outdir, tag)
    # NEVER read a stale result. The path is deterministic, so a probe that
    # dies leaves the PREVIOUS sweep point's json in place, json.load succeeds
    # on it, and that stale eps enters the least-squares fit -- corrupting the
    # very ppm-per-count gain the steering loop then hardcodes. Remove first,
    # and check the exit status.
    try:
        os.unlink(out)
    except FileNotFoundError:
        pass
    cmd = ["python3", probe, "--duration", str(dwell), "--label", tag,
           "--gold", gold, "--core", core, "--out", out]
    rc = subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL)
    if rc.returncode != 0:
        print("  probe exited %d for %s -- treating as no measurement"
              % (rc.returncode, tag))
        return None, None, None
    try:
        d = json.load(open(out))
    except Exception:
        return None, None, None
    return (d.get("eps_ppm_sco"), d.get("resid_slope_se"),
            d.get("detections", 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ue-ip", default="168.6.244.22")
    ap.add_argument("--codes", default="-20,-10,0,10,20",
                    help="DAC offsets from cal_dac to visit, in order")
    ap.add_argument("--dwell", type=float, default=60.0,
                    help="seconds of arrival-ramp measurement per point")
    ap.add_argument("--probe", required=True)
    ap.add_argument("--gold", required=True)
    ap.add_argument("--core", required=True)
    ap.add_argument("--outdir", default=".")
    ap.add_argument("--out", default="clock_steer_cal.json")
    args = ap.parse_args()

    codes = [int(x) for x in args.codes.split(",")]
    dev = open_dev(args.ue_ip)
    st0 = read_state(dev)
    print("UE clock state at entry: %s" % st0["_raw"])
    if st0.get("ref") != "calibrated":
        print("ref=%s -- CLOCK_ADJ is an actuator only under 'calibrated'; stopping"
              % st0.get("ref"))
        return 2
    cal = int(st0["cal_dac"])
    rows = []
    try:
        for off in codes:
            st = set_offset(dev, cal, off)
            eps, se, n = measure_eps(args.probe, args.gold, args.core,
                                     args.dwell, "off%+d" % off, args.outdir)
            st2 = read_state(dev)          # still where we put it afterwards?
            drift = (st2.get("rb_dac") != st.get("rb_dac"))
            rows.append(dict(offset=off, dac=int(st["rb_dac"]), eps_ppm=eps,
                             se=se, detections=n, moved_during_run=drift))
            print("  offset %+4d (dac %d): eps %s ppm  se %s  n=%s%s"
                  % (off, int(st["rb_dac"]),
                     ("%+.5f" % eps) if eps is not None else "  FAILED",
                     ("%.5f" % se) if se else "n/a", n,
                     "   DAC MOVED MID-RUN" if drift else ""))
    finally:
        # The hold is bench state that outlives this process.
        try:
            dev.writeSetting("CLOCK_ADJ", "release")
            time.sleep(0.4)
            print("released -> %s" % read_state(dev)["_raw"])
        except Exception as e:  # noqa: BLE001
            print("RELEASE FAILED (%s) -- the node is left steered, fix by hand" % e)

    good = [r for r in rows if r["eps_ppm"] is not None]
    res = dict(cal_dac=cal, points=rows)
    if len(good) >= 3:
        n = len(good)
        mx = sum(r["offset"] for r in good) / n
        my = sum(r["eps_ppm"] for r in good) / n
        sxy = sum((r["offset"] - mx) * (r["eps_ppm"] - my) for r in good)
        sxx = sum((r["offset"] - mx) ** 2 for r in good)
        slope = sxy / sxx
        resid = [r["eps_ppm"] - (my + slope * (r["offset"] - mx)) for r in good]
        rms = (sum(x * x for x in resid) / n) ** 0.5
        res.update(ppm_per_count=slope, fit_rms_ppm=rms, n_points=n)
        print()
        print("ppm per DAC count = %+.4f   (protocol says 0.129, bench doc 0.087)"
              % slope)
        print("linearity: fit residual %.5f ppm rms over %d points" % (rms, n))
        # eps = (f_BS - f_UE)/f_UE and we steer the UE, so the UE sits in the
        # numerator AND the denominator: raising f_UE LOWERS eps. A NEGATIVE
        # slope therefore means higher code = faster UE. The first cut of this
        # line read the slope as if eps were the UE's own frequency and printed
        # the sign backwards, which would have inverted the steering loop.
        print("sign: higher code = %s UE clock  (d(eps)/d(count) = %+.4f; eps "
              "has f_UE in its denominator, so the sign inverts)"
              % ("FASTER" if slope < 0 else "SLOWER", slope))
        print("loop: to null eps, push %+d counts per ppm of eps"
              % round(1.0 / abs(slope)))
    else:
        print("not enough good points to fit")
    with open(args.out, "w") as f:
        json.dump(res, f, indent=1, sort_keys=True)
    print("wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
