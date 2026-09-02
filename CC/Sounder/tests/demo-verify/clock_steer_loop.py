#!/usr/bin/env python3
"""AP-47: close the clock-steering loop with our own rate tracker as the sensor.

This is the point the whole free-running campaign was walking toward. Timing
drift and carrier offset were never two problems: the sample clock and the RFDC
NCO come off one LMK reference, so they are one reference error seen two ways.
An actuator at that reference nulls both at once, instead of two software
corrections chasing two symptoms.

Sensor  : the beacon arrival ramp, eps to ~0.00002 ppm over 120 s -- four orders
          finer than the actuator quantum, so the sensor is never the limit.
Actuator: CLOCK_ADJ, MEASURED at -0.1251 ppm per count by clock_steer_cal.py
          (AP-48), which confirmed the protocol's 0.129 and ruled out the bench
          doc's 0.087; using the latter would have made the gain 1.44x too large.

SIGN, the thing most likely to be got backwards: eps = (f_BS - f_UE)/f_UE and we
steer the UE, so the UE is in the numerator AND the denominator and raising f_UE
LOWERS eps. d(eps)/d(count) is NEGATIVE, so to reduce a positive eps we RAISE the
code. push = +round(gain * eps / |ppm_per_count|).

Steers the UE only: the BS is the reference and the UE conforms.

DELIBERATELY SLOW. A held VCXO wanders about one count in ten minutes, so the
loop has no need of speed -- and a slow loop is also what Doppler invariance
wants (AP-51): on a one-way link a range rate is indistinguishable from a clock
offset, so a fast steer would integrate motion into the oscillator. Long time
constant and bounded authority separate quasi-static clock error from transient
motion in the frequency domain.
"""
import argparse
import json
import subprocess
import sys
import time

import SoapySDR

RPC_TIMEOUT_US = "30000000"
PPM_PER_COUNT = 0.1251        # measured, AP-48. Magnitude; the sign is handled below.


def open_dev(ip):
    return SoapySDR.Device(dict(driver="houdinisdr",
                                remote="tcp://%s:55132" % ip,
                                timeout=RPC_TIMEOUT_US))


def read_state(dev):
    out = {}
    txt = str(dev.readSetting("CLOCK_ADJ"))
    for tok in txt.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    out["_raw"] = txt
    return out


def measure_eps(probe, gold, core, dwell, tag, outdir):
    out = "%s/steerloop_%s.json" % (outdir, tag)
    subprocess.run(["python3", probe, "--duration", str(dwell), "--label", tag,
                    "--gold", gold, "--core", core, "--out", out],
                   check=False, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    try:
        d = json.load(open(out))
    except Exception:
        return None, None
    return d.get("eps_ppm_sco"), d.get("detections", 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ue-ip", default="168.6.244.22")
    ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--dwell", type=float, default=60.0)
    ap.add_argument("--gain", type=float, default=0.7)
    ap.add_argument("--max-offset", type=int, default=30,
                    help="bounded authority: never wander further than this "
                         "many counts from the calibration point")
    ap.add_argument("--deadband-ppm", type=float, default=0.06,
                    help="do not push below half the actuator quantum; there is "
                         "nothing there to correct with")
    ap.add_argument("--keep", action="store_true",
                    help="leave the converged code in place instead of "
                         "releasing to the calibrated hold")
    ap.add_argument("--probe", required=True)
    ap.add_argument("--gold", required=True)
    ap.add_argument("--core", required=True)
    ap.add_argument("--outdir", default=".")
    ap.add_argument("--out", default="clock_steer_loop.json")
    args = ap.parse_args()

    dev = open_dev(args.ue_ip)
    st = read_state(dev)
    print("UE at entry: %s" % st["_raw"])
    if st.get("ref") != "calibrated":
        print("ref=%s -- CLOCK_ADJ is an actuator only under 'calibrated'" % st.get("ref"))
        return 2
    cal = int(st["cal_dac"])
    rows = []
    try:
        for i in range(args.iters):
            eps, n = measure_eps(args.probe, args.gold, args.core, args.dwell,
                                 "it%02d" % i, args.outdir)
            st = read_state(dev)
            off = int(st["rb_dac"]) - cal
            if eps is None:
                print("  it%02d: measurement FAILED (n=%s), holding at %+d"
                      % (i, n, off))
                rows.append(dict(i=i, eps_ppm=None, offset=off, push=0))
                continue
            push = int(round(args.gain * eps / PPM_PER_COUNT))
            if abs(eps) < args.deadband_ppm:
                push = 0
            new_off = max(-args.max_offset, min(args.max_offset, off + push))
            push = new_off - off
            print("  it%02d: eps %+.4f ppm (n=%d) at offset %+d -> push %+d"
                  % (i, eps, n, off, push))
            rows.append(dict(i=i, eps_ppm=eps, offset=off, push=push,
                             detections=n))
            if push != 0:
                dev.writeSetting("CLOCK_ADJ", str(cal + new_off))
                time.sleep(0.5)
                chk = read_state(dev)
                if int(chk["rb_dac"]) != cal + new_off:
                    raise RuntimeError("readback %s != %d"
                                       % (chk["rb_dac"], cal + new_off))
    finally:
        if args.keep:
            print("keeping the steered code: %s" % read_state(dev)["_raw"])
        else:
            try:
                dev.writeSetting("CLOCK_ADJ", "release")
                time.sleep(0.5)
                print("released -> %s" % read_state(dev)["_raw"])
            except Exception as e:  # noqa: BLE001
                print("RELEASE FAILED (%s) -- node left steered, fix by hand" % e)

    good = [r["eps_ppm"] for r in rows if r["eps_ppm"] is not None]
    if len(good) >= 2:
        print()
        print("eps trajectory: %s" % " -> ".join("%+.3f" % e for e in good))
        print("|eps| first %.4f ppm, last %.4f ppm, best %.4f"
              % (abs(good[0]), abs(good[-1]), min(abs(e) for e in good)))
    with open(args.out, "w") as f:
        json.dump(dict(cal_dac=cal, ppm_per_count=PPM_PER_COUNT,
                       gain=args.gain, points=rows), f, indent=1,
                  sort_keys=True)
    print("wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
