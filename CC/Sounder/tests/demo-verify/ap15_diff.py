#!/usr/bin/env python3
"""AP-15 offline diff: compare a CLEAN run's dump against a RING run's dump.

Inputs per run (from ap15_correlate.py): logs/ap15_runN_dump.bin
  [N cp es nsym ndata]i32  [H re,im f32]*N  [data_ind i32]*ndata
  [U-slot re,im i16]*4096
plus logs/ap15_runN_cns.npy (the live equalized constellation points) and the
fixed-seed frequency-domain reference logs/ul_data_f_*.bin
([sym][fft] cf32, one frame/slot/ch).

For each run: re-derive the equalized per-symbol tones from the RAW U slot,
then report
  R1  per-data-tone across-symbol coherence after removing the known
      modulation (the AP-15 discriminator: ~1 clean, ~0.6 ring);
  R2  the same with the reference rotated by k symbols (k = -4..4): a ring
      run peaking at k != 0 means the demod reference is misaligned to the
      transmitted symbol sequence -- the reference-rotation hypothesis;
  R3  pilot-tone (bins DC+-7, DC+-21) per-symbol phase trajectory stats;
  R4  raw-slot cross-correlation lag between the two runs' U slots (identical
      TX content -> a lag shows extraction misalignment).
"""
import argparse
import glob
import struct
import sys

import numpy as np

FFT = 64
CP = 16
SYM = 80
NSYM = 48
PILOT_BINS = [7, 21, 43, 57]  # DC+-7, DC+-21 in fft-shifted-to-0.. indexing?


def load_dump(path):
    b = open(path, "rb").read()
    n, cp, es, nsym, ndata = struct.unpack_from("<5i", b, 0)
    off = 20
    h = np.frombuffer(b, dtype=np.float32, count=2 * n, offset=off)
    H = (h[0::2] + 1j * h[1::2]).astype(np.complex128)
    off += 8 * n
    data_ind = np.frombuffer(b, dtype=np.int32, count=ndata, offset=off).copy()
    off += 4 * ndata
    u = np.frombuffer(b, dtype=np.int16, offset=off)
    slot = (u[0::2].astype(np.float64) + 1j * u[1::2]).astype(np.complex128)
    return dict(N=n, cp=cp, es=es, nsym=nsym, data_ind=data_ind, H=H,
                slot=slot)


def load_ref(logdir):
    fs = sorted(glob.glob(logdir + "/ul_data_f_*.bin"))
    if not fs:
        raise SystemExit("no ul_data_f_*.bin reference in " + logdir)
    r = np.fromfile(fs[0], dtype=np.float32)
    z = (r[0::2] + 1j * r[1::2]).astype(np.complex128)
    return z.reshape(-1, FFT)  # [sym][fft]


def tones(d, conj_rx=True):
    """Equalized per-symbol tones from the raw slot: [nsym][fft]."""
    s = d["slot"].conj() if conj_rx else d["slot"]
    es = d["es"]
    out = np.zeros((d["nsym"], FFT), dtype=np.complex128)
    Hs = d["H"].copy()
    Hs[np.abs(Hs) < 1e-9] = 1.0
    for k in range(d["nsym"]):
        w = s[es + k * SYM: es + k * SYM + FFT]
        if len(w) < FFT:
            break
        out[k] = np.fft.fft(w, FFT) / Hs
    return out


def coherence(t, ref, data_ind, rot=0):
    """Mean per-data-tone across-symbol coherence with the reference rotated
    by `rot` symbols."""
    nsym = t.shape[0]
    r = np.roll(ref[:nsym], rot, axis=0)
    cohs = []
    for j in data_ind:
        zj = t[:, j]
        rj = r[:, j]
        good = np.abs(rj) > 1e-9
        if good.sum() < 4:
            continue
        d = zj[good] * np.conj(rj[good])
        cohs.append(np.abs(d.mean()) / (np.abs(d).mean() + 1e-30))
    return float(np.mean(cohs)) if cohs else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=int,
                    help="run numbers to analyze (e.g. a clean and a ring)")
    ap.add_argument("--logdir", default="logs")
    args = ap.parse_args()
    ref = load_ref(args.logdir)
    slots = {}
    for n in args.runs:
        d = load_dump("%s/ap15_run%d_dump.bin" % (args.logdir, n))
        t = tones(d)
        base = coherence(t, ref, d["data_ind"], 0)
        rots = {k: coherence(t, ref, d["data_ind"], k) for k in range(-4, 5)}
        best = max(rots, key=rots.get)
        # pilot-tone per-symbol phase (raw tones, not modulation-removed)
        pil = []
        for j in PILOT_BINS:
            ph = np.unwrap(np.angle(t[:, j]))
            pil.append(float(np.std(np.diff(ph))))
        try:
            z = np.load("%s/ap15_run%d_cns.npy" % (args.logdir, n))
            score = float(np.abs(np.mean(z ** 4)) /
                          (np.mean(np.abs(z) ** 4) + 1e-30))
        except Exception:  # noqa: BLE001
            score = -1.0
        slots[n] = d["slot"]
        print("run %2d: live_score=%.3f  R1 coh(rot0)=%.3f  "
              "R2 best_rot=%+d (%.3f)  R3 pilot dphase-std=%s"
              % (n, score, base, best, rots[best],
                 ["%.2f" % v for v in pil]))
        print("        rots: %s" % {k: round(v, 3) for k, v in rots.items()})
    if len(args.runs) >= 2:
        a, b = args.runs[0], args.runs[1]
        xa, xb = slots[a], slots[b]
        n = min(len(xa), len(xb))
        c = np.abs(np.correlate(xa[:n], xb[:n], "full"))
        lag = int(np.argmax(c)) - (n - 1)
        print("R4 raw-slot xcorr lag run%d vs run%d: %+d samples "
              "(peak/next %.2f)" % (a, b, lag,
                                    float(np.sort(c)[-1] /
                                          (np.sort(c)[-2] + 1e-30))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
