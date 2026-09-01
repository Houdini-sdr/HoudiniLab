#!/usr/bin/env python3
"""Free-running clock drift probe -- the AP-33 measurement.

Measures the BS->UE clock offset epsilon = (f_BS - f_UE)/f_UE on real silicon,
with NO dependence on the sounder's acquisition or control law. The BS is armed
with the real beacon exactly as two_node_beacon_arrival.py does; the UE reads
windows and searches EACH ONE whole (no anchored grid, so nothing stops finding
the beacon when it drifts).

Two INDEPENDENT observables, both from the same detections:

  SCO  the arrival time of the beacon core in UE sample ticks. The BS emits one
       beacon per 122880 BS ticks; in UE ticks that period is 122880/(1+eps), so
       the residual against a fixed 122880 grid RAMPS at -RATE*eps samples per
       second. Slope of that ramp = the sample-clock offset.
  CFO  the two-stage Schmidl-Cox estimate off the beacon core itself (receiver.cc
       estimateCFO arithmetic, replicated exactly), = FREQ*eps Hz.

They must agree, because both derive from the one reference: the LMK PLL1 input
sets the sample clock AND the RFDC NCO, so eps is the same fraction read two
ways. That agreement is the instrument's own validation -- and it checks the
SIGN of estimateCFO()'s houdini flip against a physical truth the estimator
cannot infer, which deliberate injection (AP-30) could only check against a
model. Run it first with both boards on the shared external 10 MHz: the
known-good case, where both readings must be ~0.

    python3 clock_drift_probe.py --duration 60 --label A-both-external
"""
import argparse
import cmath
import json
import math
import sys
import time

import numpy as np

from two_node_beacon_arrival import (CORE_OFF_2NDREP, FRAME, RATE, Bs, Ue,
                                     find_beacon)

FREQ = 500e6          # the RFDC NCO both boards are tuned to
STS_LEN, STS_REPS = 16, 15
GOLD_LEN, GOLD_REPS = 128, 2
CORE = STS_LEN * STS_REPS + GOLD_LEN * GOLD_REPS   # 496


def estimate_cfo(core):
    """receiver.cc estimateCFO(), same arithmetic, on the 496-sample core.

    Returns the PHYSICAL offset in Hz: the final negation undoes the matched-NCO
    R2C mixer's conjugation, exactly as the C++ does for is_houdini().
    """
    if len(core) != CORE:
        return None
    g1 = STS_LEN * STS_REPS
    g2 = g1 + GOLD_LEN
    r_fine = np.vdot(core[g1:g1 + GOLD_LEN], core[g2:g2 + GOLD_LEN])
    r_coarse = np.vdot(core[0:(STS_REPS - 1) * STS_LEN],
                       core[STS_LEN:STS_REPS * STS_LEN])
    if r_fine == 0 or r_coarse == 0:
        return None
    f_fine = cmath.phase(r_fine) / (2 * math.pi * GOLD_LEN)
    f_coarse = cmath.phase(r_coarse) / (2 * math.pi * STS_LEN)
    amb = 1.0 / GOLD_LEN
    f = f_fine + round((f_coarse - f_fine) / amb) * amb
    return -f * RATE


def unwrap_resid(t):
    """Residual of arrival ticks against a fixed FRAME grid, unwrapped.

    np.unwrap's period= kwarg needs numpy >= 1.21; do it by hand so the probe
    runs on whatever the rig venv has.
    """
    k = np.round((t - t[0]) / FRAME)
    r = (t - t[0]) - k * FRAME          # wrapped into +-FRAME/2 by construction
    out = r.astype(np.float64).copy()
    add = 0.0
    for i in range(1, len(out)):
        d = (out[i] + add) - out[i - 1]
        if d > FRAME / 2:
            add -= FRAME * round(d / FRAME)
        elif d < -FRAME / 2:
            add += FRAME * round(-d / FRAME)
        out[i] += add
    return out


def fit(x, y, nsig=3.0, iters=4):
    """Least-squares slope + standard error, with a 3-sigma iterative reject.

    A single spurious detection (noise peak beating the beacon in one window)
    would otherwise lever the slope, and the slope IS the measurement. The drop
    count is returned and reported, never applied silently.
    """
    keep = np.ones(len(x), dtype=bool)
    a = b = sy = None
    for _ in range(iters):
        if keep.sum() < 3:
            return None, None, None, int((~keep).sum())
        a, b = np.polyfit(x[keep], y[keep], 1)
        r = y - (a * x + b)
        sy = float(np.sqrt(np.sum(r[keep] ** 2) / max(1, keep.sum() - 2)))
        if sy == 0.0:
            break
        nk = np.abs(r) <= nsig * sy
        if (nk == keep).all():
            break
        keep = nk
    sxx = float(np.sum((x[keep] - x[keep].mean()) ** 2))
    return (float(a), (sy / math.sqrt(sxx) if sxx > 0 else None), sy,
            int((~keep).sum()))


def self_test(gold_path, core_path):
    """Known-good case for the ANALYSIS, no hardware (measurement discipline).

    Synthesizes windows carrying a beacon at a KNOWN eps and checks the probe
    recovers it on both channels with the right sign. It validates the fit, the
    unwrap, and the slope<->eps relation, plus that estimate_cfo() inverts the
    modelled mixer. It CANNOT validate the mixer model itself -- that is what
    the hardware agreement check (CFO vs SCO on a real link) is for.
    """
    core = np.fromfile(core_path, dtype=np.int16)
    cc = (core[0::2].astype(np.float64) - 1j * core[1::2])
    cc = cc / np.abs(cc).max()
    gold = np.fromfile(gold_path, dtype=np.complex64).astype(np.complex128)
    rng = np.random.default_rng(7)
    ok = True
    for eps_ppm in (0.0, +2.5, -2.5, +25.0):
        eps = eps_ppm * 1e-6
        ticks, cfos = [], []
        # One detection every ~0.25 s over 60 s, at random window phases.
        for j in range(240):
            k = int(round(j * 0.25 * RATE / FRAME))
            # Arrival in UE ticks: the BS frame period is FRAME/(1+eps) here.
            t = int(round(k * FRAME / (1.0 + eps))) + int(rng.normal(0, 6))
            w = (rng.normal(0, 2e-3, 12288) +
                 1j * rng.normal(0, 2e-3, 12288)).astype(np.complex128)
            off = 2000
            n = np.arange(CORE)
            # Physical +CFO on the carrier, then the matched-NCO R2C mixer,
            # which delivers baseband CONJUGATED (cfo_model.py, AP-30).
            sig = cc * np.exp(2j * np.pi * (FREQ * eps / RATE) * n)
            w[off:off + CORE] += np.conj(sig)
            idx, ratio = find_beacon(w, gold, 10.0)
            if idx < 0:
                continue
            start = idx - CORE_OFF_2NDREP
            if start < 0 or start + CORE > len(w):
                continue
            f = estimate_cfo(w[start:start + CORE])
            if f is None:
                continue
            ticks.append(t - off + start)
            cfos.append(f)
        t = np.array(ticks, dtype=np.int64)
        secs = (t - t[0]).astype(np.float64) / RATE
        slope, se, jit, _ = fit(secs, unwrap_resid(t))
        got_sco = -slope / RATE * 1e6
        got_cfo = float(np.mean(cfos)) / FREQ * 1e6
        good = (abs(got_sco - eps_ppm) < 0.02 and abs(got_cfo - eps_ppm) < 0.05)
        ok = ok and good
        print("  eps %+7.3f ppm -> SCO %+7.3f  CFO %+7.3f  (n=%d jit %.1f) %s"
              % (eps_ppm, got_sco, got_cfo, len(t), jit,
                 "OK" if good else "FAIL"))
    print("SELF-TEST %s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bs-ip", default="168.6.244.21")
    ap.add_argument("--ue-ip", default="168.6.244.22")
    ap.add_argument("--core", default="beacon_core.bin")
    ap.add_argument("--gold", default="gold.bin")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="detection-collection wall-clock budget (s)")
    ap.add_argument("--window", type=int, default=12288,
                    help="UE read window in samples; the beacon lands in "
                         "~window/FRAME of them")
    ap.add_argument("--corr-scale", type=float, default=10.0)
    ap.add_argument("--min-ratio", type=float, default=1.0,
                    help="detector-ratio floor. find_beacon's ratio is "
                         "peak/thresh and its own crossing rule is "
                         "ratio > 1/corr_scale = 0.1, so the scale here is "
                         "ORDER ONE: the live beacon on this bench measured "
                         "5.76 (two_node_beacon_arrival, 2026-09-01). "
                         "two_node's '>=1e7' help text is from another "
                         "scaling and cost this probe one null run.")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--label", default="run",
                    help="clock-configuration label, recorded in the json")
    ap.add_argument("--out", default="clock_drift_probe.json")
    ap.add_argument("--self-test", action="store_true",
                    help="validate the analysis on synthetic "
                         "windows at a known eps; no hardware")
    args = ap.parse_args()

    if args.self_test:
        return self_test(args.gold, args.core)

    if not (496 < args.window <= 16384):
        print("--window must be in (496, 16384]: Ue.buf is 16384 "
              "samples deep")
        return 2

    core = np.fromfile(args.core, dtype=np.int16)
    cc = core[0::2].astype(np.float64) - 1j * core[1::2]   # conjugate
    pk = np.abs(cc).max()
    ram = np.zeros(2 * 4096, dtype=np.int16)
    ram[0:2 * len(cc):2] = np.round(cc.real / pk * 0.6 * 32767).astype(np.int16)
    ram[1:2 * len(cc):2] = np.round(cc.imag / pk * 0.6 * 32767).astype(np.int16)
    gold = np.fromfile(args.gold, dtype=np.complex64).astype(np.complex128)
    print("label=%s  beacon core %d samp, gold %d taps, window %d"
          % (args.label, len(cc), len(gold), args.window))

    out = {"label": args.label, "rate": RATE, "freq": FREQ, "frame": FRAME,
           "window": args.window, "duration_req": args.duration}
    bs = Bs(args.bs_ip, ram, args.tx_ch)
    ue = None
    ticks, cfos, ratios = [], [], []
    windows = 0
    best_ratio = 0.0   # so a zero-detection run is diagnosable rather than mute
    peak_rms = 0.0
    try:
        bs.open_and_arm()
        if not bs.liveness():
            print("  BS beacon NOT alive -- collecting nothing, see the json")
            out["error"] = "bs_not_alive"
        else:
            ue = Ue(args.ue_ip, args.rx_ch)
            senses = [("gold", gold), ("conj", np.conj(gold))]
            sense = None
            # Wall-clock budget INSIDE the loop: this script owns armed hardware,
            # so it must always exit through its own teardown (never be timeout-
            # killed, which skips the ladder and leaves the framer armed).
            deadline = time.time() + args.duration
            while time.time() < deadline:
                windows += 1
                tk, c = ue.window(args.window)
                if tk is None:
                    continue
                peak_rms = max(peak_rms, float(np.sqrt(np.mean(
                    np.abs(c) ** 2))) * 32767.0)
                for name, g in (senses if sense is None else [sense]):
                    idx, ratio = find_beacon(c, g, args.corr_scale)
                    if idx >= 0:
                        best_ratio = max(best_ratio, ratio)
                    if idx >= 0 and ratio >= args.min_ratio:
                        start = idx - CORE_OFF_2NDREP
                        if start < 0 or start + CORE > len(c):
                            break        # core straddles the window edge, skip
                        if sense is None:
                            sense = (name, g)
                            print("  first match: sense=%s idx=%d ratio=%.3g"
                                  % (name, idx, ratio))
                        f = estimate_cfo(c[start:start + CORE])
                        if f is None:
                            break
                        ticks.append(tk + start)
                        cfos.append(f)
                        ratios.append(ratio)
                        break
    finally:
        if ue is not None:
            ue.close()
        bs.close()

    n = len(ticks)
    print("detections %d over %d windows (%.1f%% hit rate)"
          % (n, windows, 100.0 * n / max(1, windows)))
    print("best detector ratio seen %.3g, loudest window rms %.1f"
          % (best_ratio, peak_rms))
    out.update(windows=windows, detections=n, best_ratio=best_ratio,
               peak_rms=peak_rms,
               ue_reads=(ue.reads if ue else 0),
               ue_fails=(ue.fails if ue else 0))
    if n >= 3:
        t = np.array(ticks, dtype=np.int64)
        order = np.argsort(t)
        t = t[order]
        cf = np.array(cfos, dtype=np.float64)[order]
        secs = (t - t[0]).astype(np.float64) / RATE
        r = unwrap_resid(t)
        slope, slope_se, jitter, dropped = fit(secs, r)
        # eps from timing: resid ramps at -RATE*eps samples per second.
        eps_sco = -slope / RATE
        eps_cfo = float(np.mean(cf)) / FREQ
        out.update(
            span_s=float(secs[-1]),
            resid_slope_samp_per_s=slope,
            resid_slope_se=slope_se,
            resid_jitter_samp=jitter, outliers_dropped=dropped,
            cfo_hz_mean=float(np.mean(cf)),
            cfo_hz_sd=float(np.std(cf)),
            eps_ppm_sco=eps_sco * 1e6,
            eps_ppm_cfo=eps_cfo * 1e6,
            ratio_min=float(np.min(ratios)), ratio_max=float(np.max(ratios)),
            ticks=[int(v) for v in t], resid=[float(v) for v in r],
            cfo_hz=[float(v) for v in cf])
        print("span %.1f s, arrival jitter %.1f samp rms, %d outlier(s) dropped"
              % (secs[-1], jitter, dropped))
        print("SCO: resid slope %+.4f samp/s (se %.4f)  -> eps %+.4f ppm"
              % (slope, slope_se if slope_se else float("nan"),
                 eps_sco * 1e6))
        print("CFO: %+.1f Hz (sd %.1f over %d)          -> eps %+.4f ppm"
              % (np.mean(cf), np.std(cf), n, eps_cfo * 1e6))
        d = eps_cfo - eps_sco
        out["eps_ppm_disagree"] = d * 1e6
        print("AGREEMENT: CFO - SCO = %+.4f ppm (%+.1f Hz at %.0f MHz)"
              % (d * 1e6, d * FREQ, FREQ / 1e6))
    else:
        print("NOT ENOUGH DETECTIONS to fit -- report as a null result, not a "
              "zero drift")
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1, sort_keys=True)
    print("wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
