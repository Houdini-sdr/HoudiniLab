#!/usr/bin/env python3
"""The received beacon's spectrum, replica-free, from recorded windows.

A 512-point DFT of the 496-sample legacy core at the recorded position, in 12
bands across the 122.88 MHz baseband: band powers in dB relative to the
strongest, the spectral centroid in MHz, and the low-edge minus high-edge tilt
in dB. The same on a beacon-free stretch when the window is long enough, so a
receive-side tilt (which the noise floor inherits) can be told from a
transmit-side one (which it does not).

Inputs: sounder dumps (HOUDINI_DUMP_RESYNC_WIN: resyncwin_NN.bin + .txt, the
txt carrying sync_index = the beacon END) or a probe raw window with its
records (beacon_phase_coherence.py --dump-raw / --dump, pos = start of rep2).

  python3 tests/demo-verify/beacon_spectrum.py --windows /tmp/spec_1
  python3 tests/demo-verify/beacon_spectrum.py --raw run.c64 --csv run.csv

Written 2026-09-03 for DEMO_VERIFICATION 8.169; pure Python.
"""
import argparse
import cmath
import csv
import glob
import math
import os
import struct

N = 512
RATE = 122.88e6
CORE = 496
REP2 = 368


def bands(x, nb=12):
    n = len(x)
    X = [sum(x[t] * cmath.exp(-2j * math.pi * k * t / N) for t in range(n)) for k in range(N)]
    P = [abs(X[(k + N // 2) % N]) ** 2 for k in range(N)]
    per = N // nb
    b = [sum(P[i * per:(i + 1) * per]) for i in range(nb)]
    m = max(b) or 1.0
    tot = sum(P) or 1.0
    cen = sum(P[k] * (k - N / 2) for k in range(N)) / tot * (RATE / N) / 1e6
    lo = sum(P[16:96]); hi = sum(P[416:496])
    tilt = 10 * math.log10(lo / hi) if lo > 0 and hi > 0 else 0.0
    return ["%5.1f" % (10 * math.log10(v / m + 1e-12)) for v in b], cen, tilt


def report(label, seg, noise=None):
    b, cen, tilt = bands(seg)
    print("%-22s beacon  %s  centroid %+6.1f MHz  tilt %+5.1f dB" % (label, " ".join(b), cen, tilt))
    if noise is not None and len(noise) >= N:
        nb_, ncen, ntilt = bands(noise[:N])
        print("%-22s noise   %s  centroid %+6.1f MHz  tilt %+5.1f dB" % ("", " ".join(nb_), ncen, ntilt))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--windows", help="directory of resyncwin_NN.bin/.txt")
    ap.add_argument("--raw", help="probe raw window, complex64")
    ap.add_argument("--csv", help="probe records for that window")
    ap.add_argument("--max", type=int, default=6)
    a = ap.parse_args()
    print("bands: 12 x 10.24 MHz from -61.4 to +61.4 MHz, dB relative to the strongest band")
    if a.windows:
        for p in sorted(glob.glob(os.path.join(a.windows, "resyncwin_*.bin")))[:a.max]:
            meta = dict(l.split() for l in open(p[:-4] + ".txt"))
            raw = open(p, "rb").read()
            n = len(raw) // 4
            w = [complex(*struct.unpack_from("hh", raw, 4 * i)) for i in range(n)]
            end = int(meta["sync_index"])
            seg = w[end - CORE + 1:end + 1]
            noise = w[end + 300:end + 300 + N] if end + 300 + N <= n else None
            report(os.path.basename(p), seg, noise)
    if a.raw and a.csv:
        raw = open(a.raw, "rb").read()
        n = len(raw) // 8
        w = [complex(*struct.unpack_from("ff", raw, 8 * i)) for i in range(n)]
        rows = [r for r in csv.DictReader(open(a.csv)) if int(r["window"]) == 0][:a.max]
        for r in rows:
            j0 = int(float(r["pos"])) - REP2
            report("frame %d" % int(float(r["frame"])), w[j0:j0 + CORE], w[j0 + 20000:j0 + 20000 + N])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
