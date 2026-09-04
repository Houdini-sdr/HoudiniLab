#!/usr/bin/env python3
"""AP-51: separate the clock offset from the range rate, two-way.

THE PROBLEM. On a ONE-WAY link a clock rate offset and a Doppler shift are
physically degenerate. A range rate rdot/c changes the propagation delay
continuously, which is an apparent time scaling indistinguishable from a clock
rate offset: timing sees (eps + rdot/c), carrier sees (eps + rdot/c)*f_c, the
same fraction both times. No single-link estimator at any lag can separate them.

THE SEPARATION IS TWO-WAY. The clock term flips sign between directions and the
range-rate term does not:

    downlink, measured by the UE   d = +eps + rdot/c
    uplink,   measured by the BS   u = -eps + rdot/c

    d - u = 2*eps        the clock
    d + u = 2*rdot/c     the Doppler

We already produce both observables and need no new signal and no protocol: the
BS RX thread and the UE client thread run in the SAME sounder process, so the
reporting path on this bench is a shared log rather than an over-the-air
message. (A real deployment needs the message. The bench can prove the math
first.)

THE SUBTLETY THAT WOULD BITE AN IMPLEMENTER, AND IT IS WHY THIS SCRIPT EXISTS.
Two-way transfer assumes each end transmits on its RAW local clock. Ours does
not: the UE transmits on its TRACKED grid, already corrected by the downlink
measurement. So the BS-measured slope is the tracker's RESIDUAL, not the raw
uplink rate, and with a converged tracker it goes to zero and the sum and
difference DEGENERATE -- you get 2*eps = d and 2*rdot/c = d, i.e. the one-way
answer wearing a two-way costume.

The fix is bookkeeping, not a new measurement. The UE knows exactly what grid
rate it applied (`houdini_frame_period`), so removing it from the BS-measured
slope reconstructs the independent uplink rate:

    a = houdini_frame_period / samps_per_frame - 1     (applied, known exactly)
    s = BS pilot_grid_off slope per frame / samps_per_frame   (measured)
    u = s - a                                          (the de-embed)

This script does that, and SELF-TESTS it: it injects known (eps, rdot/c) into a
synthetic pair, recovers them, and demonstrates the degeneracy that appears if
you skip the de-embed. Run the self-test before believing a log.

  python3 tests/demo-verify/two_way_transfer.py --selftest
  python3 tests/demo-verify/two_way_transfer.py --log run.log
"""
import argparse
import math
import re
import sys

FRAME = 122880
RATE = 122.88e6
C = 299792458.0

# BS line: HOUDINI_BS_RX: frame=N stamp_ticks=T ... pilot_grid_off=G ...
BS_RE = re.compile(r"HOUDINI_BS_RX:\s+frame=(-?\d+)\s+stamp_ticks=(-?\d+).*?"
                   r"pilot_grid_off=(-?\d+)")
# UE line: Beacon CFO frame N: tracked +X Hz (+Y ppm) | ...
UE_RE = re.compile(r"Beacon CFO frame (\d+):\s+tracked\s+([-+][\d.]+)\s+Hz\s+"
                   r"\(([-+][\d.]+)\s+ppm\)")


def unwrap(vals, period):
    """pilot_grid_off is FOLDED into +-period/2, so a drifting offset wraps.
    Fitting a slope through wrapped data gives a slope of roughly zero with a
    beautiful correlation coefficient, which is the most convincing way to be
    wrong available here."""
    out, acc = [], 0.0
    for i, v in enumerate(vals):
        if i:
            d = v + acc - out[-1]
            while d > period / 2:
                acc -= period
                d -= period
            while d < -period / 2:
                acc += period
                d += period
        out.append(v + acc)
    return out


def slope(xs, ys):
    """Least squares slope, dy/dx."""
    n = len(xs)
    if n < 3:
        return float("nan")
    mx, my = sum(xs) / n, sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return float("nan")
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den


def separate(d, u):
    """d, u fractional. Returns (eps, rdot_over_c)."""
    return (d - u) / 2.0, (d + u) / 2.0


def report(d, a, s, label=""):
    u = s - a
    eps, rc = separate(d, u)
    eps_naive, rc_naive = separate(d, s)   # what you get WITHOUT the de-embed
    print("  %s" % label if label else "")
    print("    downlink   d = %+.4f ppm   (UE measures the beacon arrival rate)" % (d * 1e6))
    print("    applied    a = %+.4f ppm   (what the UE's TX grid already carries)" % (a * 1e6))
    print("    BS slope   s = %+.4f ppm   (pilot_grid_off, the tracker RESIDUAL)" % (s * 1e6))
    print("    de-embed   u = s - a = %+.4f ppm   (the raw uplink rate)" % (u * 1e6))
    print("    -> clock    eps      = %+.4f ppm" % (eps * 1e6))
    print("    -> Doppler  rdot/c   = %+.4f ppm  (%+.3f m/s)" % (rc * 1e6, rc * C))
    print("       without the de-embed you would read eps %+.4f ppm, "
          "rdot/c %+.4f ppm" % (eps_naive * 1e6, rc_naive * 1e6))
    return eps, rc


def selftest():
    fails = []

    def check(ok, what):
        print(("PASS  " if ok else "FAIL  ") + what)
        if not ok:
            fails.append(what)

    print("=== AP-51 self-test: inject known (eps, rdot/c), recover them ===\n")
    for eps_ppm, v_ms, conv in ((8.52, 0.0, 1.0),      # our wired bench, static
                                (8.52, 3.0, 1.0),      # walking speed
                                (0.06, 30.0, 1.0),     # steered, vehicle speed
                                (8.52, 3.0, 0.7)):     # tracker NOT converged
        eps = eps_ppm * 1e-6
        rc = v_ms / C
        # The UE measures the downlink arrival rate.
        d = eps + rc
        # It applies a fraction `conv` of that to its TX grid (conv = 1 is a
        # fully converged tracker; anything less is mid-convergence).
        a = conv * d
        # The BS then measures, as the pilot's drift per frame, the applied
        # grid rate carried through the link the other way.
        s = a + rc - eps
        got_eps, got_rc = report(d, a, s,
                                 "eps %.2f ppm, v %.1f m/s, tracker %.0f%% converged"
                                 % (eps_ppm, v_ms, conv * 100))
        check(abs(got_eps - eps) < 1e-12, "  clock recovered")
        check(abs(got_rc - rc) < 1e-12, "  range rate recovered")
        print()

    # The degeneracy this script exists to prevent: with a CONVERGED tracker and
    # no de-embed, s is ~0 and both answers collapse onto d/2.
    print("=== the degeneracy, shown rather than asserted ===")
    eps, rc = 8.52e-6, 3.0 / C
    d = eps + rc
    a = d           # fully converged
    s = a + rc - eps
    naive_eps, naive_rc = separate(d, s)
    print("  converged tracker: BS slope s = %+.6f ppm (near zero by construction)"
          % (s * 1e6))
    print("  naive:  eps %+.4f ppm, rdot/c %+.4f ppm" % (naive_eps * 1e6, naive_rc * 1e6))
    print("  truth:  eps %+.4f ppm, rdot/c %+.4f ppm" % (eps * 1e6, rc * 1e6))
    check(abs(naive_eps - eps) > 1e-6 * abs(eps) * 1e5 or abs(naive_eps - eps) > 1e-9,
          "skipping the de-embed gives the WRONG clock term")
    # And the specific failure the row predicts: without the de-embed the clock
    # and Doppler answers are contaminated by each other.
    check(abs(naive_rc - rc) > 1e-9,
          "skipping the de-embed gives the WRONG range rate")
    print()

    print("=== what this bench can and cannot show ===")
    print("  The rig is WIRED, so rdot/c is 0 by construction and the honest")
    print("  result is 'the sum reads zero to within the fit noise'. That")
    print("  validates the MACHINERY and the de-embed; it does not validate")
    print("  Doppler separation, which needs motion. Do not report the wired")
    print("  run as evidence of Doppler invariance.")
    print("\nRESULT: %s (%d failure(s))" % ("FAIL" if fails else "PASS", len(fails)))
    return 1 if fails else 0


def from_log(path, spf):
    bs_t, bs_g, ue_ppm = [], [], []
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = BS_RE.search(line)
            if m:
                bs_t.append(int(m.group(2)))
                bs_g.append(int(m.group(3)))
                continue
            m = UE_RE.search(line)
            if m:
                ue_ppm.append(float(m.group(3)))
    print("parsed %d BS pilot_grid_off point(s), %d UE tracked-CFO line(s)"
          % (len(bs_g), len(ue_ppm)))
    if len(bs_g) < 10 or not ue_ppm:
        print("NOT ENOUGH DATA. The BS needs HOUDINI_BS_RX_DEBUG=1 with")
        print("HOUDINI_BS_RX_EVERY=1, and the UE needs HOUDINI_CFO_LOG_EVERY=1.")
        return 1
    g = unwrap([float(v) for v in bs_g], FRAME)
    # The fit is pilot_grid_off (SAMPLES) against stamp_ticks (TICKS), so the
    # slope is samples per tick. On this platform the tick rate and the sample
    # rate are the same clock, so samples-per-tick IS the fractional rate
    # already: multiplying by ticks-per-frame and dividing by samples-per-frame
    # would just cancel. Stated rather than performed, so nobody re-derives it.
    s_frac = slope([float(t) for t in bs_t], g)
    print("  pilot_grid_off slope: %+.6f samples/tick = %+.4f samples per frame"
          % (s_frac, s_frac * spf))
    # The UE's tracked eps IS the applied grid rate: eps_tracked = spf/period - 1,
    # so the applied rate a = period/spf - 1 = 1/(1+eps_tracked) - 1.
    eps_tracked = (sum(ue_ppm) / len(ue_ppm)) * 1e-6
    a = 1.0 / (1.0 + eps_tracked) - 1.0
    d = a   # a converged tracker's state IS the downlink measurement
    print()
    report(d, a, s_frac, "from %s" % path)
    print()
    print("  Caveats. `d` here is taken as the tracker's converged state, which")
    print("  is only the downlink measurement once it HAS converged; a run that")
    print("  is still pulling in reports a clock term biased by the shortfall.")
    print("  And on a wired bench rdot/c is 0 by construction, so a nonzero")
    print("  Doppler term is fit noise or an unmodelled bias, not motion.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--log")
    ap.add_argument("--spf", type=int, default=FRAME)
    a = ap.parse_args()
    if a.selftest or not a.log:
        return selftest()
    return from_log(a.log, a.spf)


if __name__ == "__main__":
    sys.exit(main())
