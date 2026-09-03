#!/usr/bin/env python3
"""Null hypothesis and calibration for the phase probe's two chain statistics.

Written after 2026-09-03's ambiguous phase probe, when a "conjugate image"
was inferred from |wrong-sense peak| / |right-sense peak| against a rule of
thumb (1/sqrt(L) = 0.09 for a random 128-tap sequence). The rule was wrong for
this replica: the Gold IFFT sequence is structured and its conjugate-free
self-product is 0.16 to 0.28 depending on the fractional delay. The real
windows read 0.18 to 0.34, i.e. AT the null. [user: "any test should be
validated again even if it matches the hypothesis"]. This script IS that
validation, kept runnable:

  1. NULL: a pure delayed beacon (no image, 45 dB) through the identical
     analysis, over fractional delays 0..0.9: the image ratio and the lobe
     phase step a clean chain would produce.
  2. CALIBRATION: the same beacon plus a known conjugate image b*conj(rx):
     how the two statistics respond (they barely do below b = 0.3, so the
     image ratio is a weak detector and must be read against the null).
  3. REAL: the identical analysis on the sounder's golden windows
     (tests/comms-func/fixtures/golden/legacy) and, if given, a probe raw
     window (--raw FILE.c64 --csv records.csv).

Needs the dumper's legacy_core.bin and legacy_replica.bin (--shapes DIR).
Pure Python; numpy optional.

  python3 tests/demo-verify/phase_probe_null.py --shapes /tmp/shapes
"""
import argparse
import cmath
import csv
import math
import os
import random
import struct


def load_replica(path):
    raw = open(path, "rb").read()
    n = len(raw) // 8
    return [complex(*struct.unpack_from("ff", raw, 8 * i)) for i in range(n)]


def load_core_ci16(path):
    raw = open(path, "rb").read()
    n = len(raw) // 4
    return [complex(*struct.unpack_from("hh", raw, 4 * i)) for i in range(n)]


def fracdelay(x, tau, R=16):
    y = []
    for k in range(len(x) + 1):
        acc = 0
        for m in range(max(0, k - R), min(len(x), k + R + 1)):
            t = k - m - tau
            s = 1.0 if abs(t) < 1e-9 else math.sin(math.pi * t) / (math.pi * t)
            acc += x[m] * s * (0.5 + 0.5 * math.cos(math.pi * t / (R + 1)))
        y.append(acc)
    return y


def analyse(w, rep, start_guess, span=4):
    """right-sense = correlate with rep (conj(rep).w), wrong-sense = with
    conj(rep). Returns (right peak lag, |wrong peak|/|right peak|,
    right-sense phase step to the stronger neighbour, neighbour/peak)."""
    rc = [x.conjugate() for x in rep]

    def corr(r, j):
        return sum(r[i].conjugate() * w[j + i] for i in range(len(r)))

    lags = range(start_guess - span, start_guess + span + 1)
    vr = {j: corr(rep, j) for j in lags}
    vw = {j: corr(rc, j) for j in lags}
    jb = max(vr, key=lambda j: abs(vr[j]))
    jw = max(vw, key=lambda j: abs(vw[j]))
    nb = jb - 1 if abs(vr[jb - 1]) > abs(vr[jb + 1]) else jb + 1
    return jb, abs(vw[jw]) / abs(vr[jb]), cmath.phase(vr[nb] / vr[jb]), abs(vr[nb]) / abs(vr[jb])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shapes", required=True, help="beacon_shape_dump --out DIR")
    ap.add_argument("--golden", default="tests/comms-func/fixtures/golden/legacy")
    ap.add_argument("--raw", default=None, help="probe --dump-raw window (complex64)")
    ap.add_argument("--csv", default=None, help="probe --dump records for that window")
    ap.add_argument("--snr-db", type=float, default=45.0)
    a = ap.parse_args()
    g = load_replica(os.path.join(a.shapes, "legacy_replica.bin"))
    core = load_core_ci16(os.path.join(a.shapes, "legacy_core.bin"))
    # The receive mixer conjugates; the sounder pre-conjugates its transmit so
    # its windows correlate with g, the probe transmits the core as dumped so
    # its windows correlate with conj(g). Model the probe's case here.
    rx_core = [x.conjugate() for x in core]
    rep = [x.conjugate() for x in g]
    fine2 = 368  # legacy: 15 x STS(16) = 240, rep1 240..367, rep2 368..495
    random.seed(1)

    def synth(tau, bimg):
        y = fracdelay(rx_core, tau)
        pk = max(abs(v) for v in y)
        ns = pk / 10 ** (a.snr_db / 20) / math.sqrt(2)
        img = [bimg * v.conjugate() for v in y]
        return ([complex(0, 0)] * 300 +
                [p + q + complex(random.gauss(0, ns), random.gauss(0, ns)) for p, q in zip(y, img)] +
                [complex(random.gauss(0, ns), random.gauss(0, ns)) for _ in range(300)])

    print("=== NULL: pure delayed beacon, no image, %.0f dB ===" % a.snr_db)
    print("tau   lag  |wrong|/|right|  lobe step rad  nb/peak")
    for t in range(10):
        tau = 0.1 * t
        jb, ratio, step, nbm = analyse(synth(tau, 0.0), rep, 300 + fine2)
        print("%.1f  %+d   %.3f            %+.3f         %.2f" % (tau, jb - 300 - fine2, ratio, step, nbm))
    print("=== CALIBRATION: plus a conjugate image b at tau 0.3 ===")
    for b in (0.0, 0.05, 0.1, 0.2, 0.3, 0.5):
        jb, ratio, step, nbm = analyse(synth(0.3, b), rep, 300 + fine2)
        print("b=%.2f  |wrong|/|right| %.3f  lobe step %+.3f" % (b, ratio, step))
    print("=== REAL: sounder golden windows (correlate with g: the sounder pre-conjugates its TX) ===")
    k = 0
    while True:
        base = os.path.join(a.golden, "resyncwin_%02d" % k)
        if not os.path.exists(base + ".bin"):
            break
        meta = dict(l.split() for l in open(base + ".txt"))
        raw = open(base + ".bin", "rb").read()
        n = len(raw) // 4
        w = [complex(*struct.unpack_from("hh", raw, 4 * i)) for i in range(n)]
        j2 = int(meta["sync_index"]) - 127
        jb, ratio, step, nbm = analyse(w, g, j2)
        print("  win%d  lag %+d  |wrong|/|right| %.3f  lobe step %+.3f  nb/peak %.2f" % (k, jb - j2, ratio, step, nbm))
        k += 1
    if a.raw and a.csv:
        print("=== REAL: probe raw window (correlate with conj(g)) ===")
        raw = open(a.raw, "rb").read()
        n = len(raw) // 8
        w = [complex(*struct.unpack_from("ff", raw, 8 * i)) for i in range(n)]
        rows = [r for r in csv.DictReader(open(a.csv)) if int(r["window"]) == 0]
        for r in rows:
            pos = int(float(r["pos"]))
            jb, ratio, step, nbm = analyse(w, rep, pos)
            print("  frame%d  lag %+d  |wrong|/|right| %.3f  lobe step %+.3f  nb/peak %.2f" % (int(float(r["frame"])), jb - pos, ratio, step, nbm))
    print("\nRead the real rows against the NULL rows at the same nb/peak: a ratio at the null is NOT an image.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
