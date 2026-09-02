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
    else:
        # Loop exhausted `iters` rather than converging. `keep` was reassigned
        # on the last pass but a, b and sy still belong to the PREVIOUS mask, so
        # returning them alongside an sxx computed from the new one reports a
        # slope from one subset with a standard error describing another. That
        # SE is what clock_steer_cal.py uses as the per-point weight for the
        # ppm-per-count calibration, so the mismatch propagates into a shipped
        # number. Refit once on the final mask.
        if keep.sum() >= 3:
            a, b = np.polyfit(x[keep], y[keep], 1)
            r = y - (a * x + b)
            sy = float(np.sqrt(np.sum(r[keep] ** 2) / max(1, keep.sum() - 2)))
        else:
            return None, None, None, int((~keep).sum())
    sxx = float(np.sum((x[keep] - x[keep].mean()) ** 2))
    return (float(a), (sy / math.sqrt(sxx) if sxx > 0 else None), sy,
            int((~keep).sum()))



def derotate(c, f_try, cache={}):
    """c * exp(-j2*pi*f_try*n/RATE). The phasor is cached per (len, f_try):
    the coarse search reuses one grid over thousands of windows.
    """
    if f_try == 0.0:
        return c
    key = (len(c), f_try)
    ph = cache.get(key)
    if ph is None:
        ph = np.exp(-2j * np.pi * (f_try / RATE) * np.arange(len(c)))
        if len(cache) < 256:
            cache[key] = ph
    return c * ph


def coarse_search(ue, gold, corr_scale, min_ratio, fmax, fstep, windows,
                  deadline):
    """Find the de-rotation that restores the matched filter's coherence.

    find_beacon correlates a 128-tap gold sequence, so a carrier offset f costs
    it sinc(f*128/RATE): coherence is GONE by f = RATE/128 = 960 kHz and already
    halved well before that. Past a few hundred kHz the beacon is simply not
    detectable, so there is no acquisition to ride and no residual ramp to fit
    -- which is the state a free-running node can land in. Sweeping f restores
    detection WITHOUT requiring acquisition (AP-33's 'way in').

    Returns (f_try, ratio, scan) -- scan is the best ratio per candidate over
    every window tried, so a failed search is readable rather than mute.
    """
    grid = np.arange(-fmax, fmax + fstep / 2, fstep)
    scan = np.zeros(len(grid))
    senses = [("gold", gold), ("conj", np.conj(gold))]
    best = (0.0, None, None)
    for w in range(windows):
        if time.time() > deadline:
            break
        tk, c = ue.window()
        if tk is None:
            continue
        for j, f in enumerate(grid):
            d = derotate(c, float(f))
            for name, g in senses:
                idx, ratio = find_beacon(d, g, corr_scale)
                if idx < 0:
                    continue
                scan[j] = max(scan[j], ratio)
                if ratio > best[0]:
                    best = (ratio, float(f), (name, g))
        if best[0] >= min_ratio:
            break
    return best[1], best[0], (grid, scan), best[2]


class _FakeUe:
    """Synthetic window source for the self-test: same .window() contract as
    Ue, so coarse_search and the collection loop run unmodified."""

    def __init__(self, cc, eps, seed=7, noise=2e-3, window=12288):
        self.cc, self.eps, self.n = cc, eps, window
        self.rng = np.random.default_rng(seed)
        self.j = 0
        self.noise = noise
        self.reads = self.fails = 0

    def window(self, n=None):
        n = n or self.n
        self.j += 1
        self.reads += 1
        k = int(round(self.j * 0.25 * RATE / FRAME))
        # Arrival in UE ticks: the BS frame period is FRAME/(1+eps) here.
        t = int(round(k * FRAME / (1.0 + self.eps))) + int(self.rng.normal(0, 6))
        off = 2000
        w = (self.rng.normal(0, self.noise, n) +
             1j * self.rng.normal(0, self.noise, n)).astype(np.complex128)
        m = np.arange(CORE)
        # Physical +CFO on the carrier, then the matched-NCO R2C mixer, which
        # delivers baseband CONJUGATED (cfo_model.py, AP-30).
        sig = self.cc * np.exp(2j * np.pi * (FREQ * self.eps / RATE) * m)
        w[off:off + CORE] += np.conj(sig)
        return t - off, w


def self_test(gold_path, core_path):
    """Known-good case for the ANALYSIS, no hardware (measurement discipline).

    Synthesizes windows carrying a beacon at a KNOWN eps and checks the probe
    recovers it on both channels with the right sign, INCLUDING the coarse
    frequency search at an offset where the matched filter has decohered. It
    validates the fit, the unwrap, the slope<->eps relation and the
    search/de-rotation bookkeeping, plus that estimate_cfo() inverts the
    modelled mixer. It CANNOT validate the mixer model itself -- that is what
    the hardware agreement check (CFO vs SCO on a real link) is for.
    """
    core = np.fromfile(core_path, dtype=np.int16)
    cc = (core[0::2].astype(np.float64) - 1j * core[1::2])
    cc = cc / np.abs(cc).max()
    gold = np.fromfile(gold_path, dtype=np.complex64).astype(np.complex128)
    ok = True
    # The last case is past find_beacon's coherence: it MUST need the search.
    for eps_ppm, search in ((0.0, 0.0), (+2.5, 0.0), (-2.5, 0.0),
                            (+25.0, 0.0), (+800.0, 2e6), (-800.0, 2e6)):
        eps = eps_ppm * 1e-6
        ue = _FakeUe(cc, eps)
        f_lock, sense = 0.0, None
        if search > 0:
            f_lock, r_lock, _, sense = coarse_search(
                ue, gold, 10.0, 1.0, search, 100e3, 200,
                time.time() + 120)
            if f_lock is None:
                print("  eps %+8.3f ppm -> coarse search FOUND NOTHING  FAIL"
                      % eps_ppm)
                ok = False
                continue
        ticks, cfos = [], []
        senses = [("gold", gold), ("conj", np.conj(gold))]
        for _ in range(240):
            tk, c = ue.window()
            c = derotate(c, f_lock)
            for name, g in (senses if sense is None else [sense]):
                idx, ratio = find_beacon(c, g, 10.0)
                if idx < 0 or ratio < 1.0:
                    continue
                start = idx - CORE_OFF_2NDREP
                if start < 0 or start + CORE > len(c):
                    break
                f = estimate_cfo(c[start:start + CORE])
                if f is None:
                    break
                ticks.append(tk + start)
                cfos.append(f - f_lock)
                break
        if len(ticks) < 3:
            print("  eps %+8.3f ppm -> only %d detections  FAIL"
                  % (eps_ppm, len(ticks)))
            ok = False
            continue
        t = np.array(ticks, dtype=np.int64)
        secs = (t - t[0]).astype(np.float64) / RATE
        slope, se, jit, drop = fit(secs, unwrap_resid(t))
        got_sco = -slope / RATE * 1e6
        got_cfo = float(np.mean(cfos)) / FREQ * 1e6
        good = (abs(got_sco - eps_ppm) < 0.02 and abs(got_cfo - eps_ppm) < 0.1)
        ok = ok and good
        print("  eps %+8.3f ppm -> SCO %+8.3f  CFO %+8.3f  "
              "(n=%d jit %.1f f_lock %+.0f kHz) %s"
              % (eps_ppm, got_sco, got_cfo, len(t), jit, f_lock / 1e3,
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
    ap.add_argument("--min-ratio", type=float, default=0.2,
                    help="detector-ratio floor, as a MARGIN over find_beacon's "
                         "own crossing rule (ratio > 1/corr_scale = 0.1). Do "
                         "not raise this: the live beacon ratio is NOT a stable "
                         "absolute quantity -- measured 0.36 to 5.8 across one "
                         "session on this bench, tracking link level over a 16x "
                         "range. A 1.0 floor silently produced three "
                         "zero-detection runs on links the known-good "
                         "instrument locked. Robustness comes from the "
                         "3-sigma reject in the fit, not from this.")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--label", default="run",
                    help="clock-configuration label, recorded in the json")
    ap.add_argument("--out", default="clock_drift_probe.json")
    ap.add_argument("--search-hz", type=float, default=0.0,
                    help="half-width of the coarse CFO search "
                         "(Hz). 0 = no search, which only works "
                         "while the offset is small enough for "
                         "find_beacon to cohere (< ~200 kHz).")
    ap.add_argument("--search-step", type=float, default=100e3,
                    help="coarse search step; the gold matched "
                         "filter nulls at RATE/128 = 960 kHz, so "
                         "100 kHz costs under 1 dB")
    ap.add_argument("--search-windows", type=int, default=200)
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
    f_lock = 0.0
    best_ratio = 0.0   # so a zero-detection run is diagnosable rather than mute
    peak_rms = 0.0
    short = 0          # readStream truncates (rx_gap_break): ret=2032 seen live
    wmin = 1 << 30
    try:
        bs.open_and_arm()
        if not bs.liveness():
            print("  BS beacon NOT alive -- collecting nothing, see the json")
            out["error"] = "bs_not_alive"
        else:
            ue = Ue(args.ue_ip, args.rx_ch)
            senses = [("gold", gold), ("conj", np.conj(gold))]
            sense = None
            f_lock = 0.0
            if args.search_hz > 0:
                # Coarse lock ONCE, then de-rotate every window by it: the
                # per-window sweep costs ~40 find_beacons and would starve the
                # detection rate the slope fit lives on.
                f_lock, r_lock, scan, sense = coarse_search(
                    ue, gold, args.corr_scale, args.min_ratio, args.search_hz,
                    args.search_step, args.search_windows,
                    time.time() + args.duration / 2)
                grid, sc = scan
                out["search_grid_hz"] = [float(v) for v in grid]
                out["search_ratio"] = [float(v) for v in sc]
                if f_lock is None:
                    print("  coarse search FOUND NOTHING over +-%.0f kHz "
                          "(best ratio %.3g) -- widen --search-hz or the "
                          "beacon is absent" % (args.search_hz / 1e3, r_lock))
                    out["error"] = "coarse_search_failed"
                    f_lock = 0.0
                    sense = None
                else:
                    print("  coarse lock: f_try %+.1f kHz ratio %.3g "
                          "sense=%s  => CFO ~ %+.1f kHz"
                          % (f_lock / 1e3, r_lock, sense[0], -f_lock / 1e3))
                    out["f_lock_hz"] = f_lock
            # Wall-clock budget INSIDE the loop: this script owns armed hardware,
            # so it must always exit through its own teardown (never be timeout-
            # killed, which skips the ladder and leaves the framer armed).
            deadline = time.time() + args.duration
            while time.time() < deadline:
                windows += 1
                tk, c = ue.window(args.window)
                if tk is None:
                    continue
                wmin = min(wmin, len(c))
                if len(c) < args.window:
                    short += 1
                peak_rms = max(peak_rms, float(np.sqrt(np.mean(
                    np.abs(c) ** 2))) * 32767.0)
                c = derotate(c, f_lock)
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
                        # The de-rotation shifted the raw-domain frequency by
                        # -f_lock; estimate_cfo already undoes the mixer
                        # conjugation, so the physical offset is est - f_lock.
                        f -= f_lock
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
    print("best detector ratio seen %.3g (floor %.3g), loudest window rms %.1f, "
          "%d/%d short reads (min %d of %d)"
          % (best_ratio, args.min_ratio, peak_rms, short, windows,
             wmin if windows else 0, args.window))
    if n:
        q = np.percentile(ratios, [5, 50, 95])
        print("detector ratio p5/p50/p95 = %.2f / %.2f / %.2f" % tuple(q))
    out.update(windows=windows, detections=n, best_ratio=best_ratio,
               peak_rms=peak_rms, short_reads=short,
               window_min=(wmin if windows else 0),
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
