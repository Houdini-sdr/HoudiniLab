#!/usr/bin/env python3
"""
find_beacon_py.py -- exact python replica of CommsLib::find_beacon_avx, to run the
REAL detector (not a peak/median proxy) on captured windows.

  gold_corr[i]      = sum_k raw[i+k] * conj(gold[k])          (matched filter, complex)
  gold_auto_corr[i] = gold_corr[i] * conj(gold_corr[i-L])     (L = len(gold); 2-rep peak)
  peak_metric[i]    = |gold_auto_corr[i]|^2
  thresh[i]         = sum(|gold_corr[i-L .. i-1]|^2)           (trailing energy)
  detect: first i with corr_scale * peak_metric[i] > thresh[i]  else -1

Usage: python3 find_beacon_py.py <cs16_file.bin> [gold.bin] [corr_scale]
"""
import sys
import numpy as np


def load_cs16(p):
    w = np.fromfile(p, dtype=np.int16)
    return (w[0::2].astype(np.float64) + 1j * w[1::2]).astype(np.complex128)


def find_beacon(raw, gold, corr_scale=1.0):
    L = len(gold)
    n = 1 << int(np.ceil(np.log2(len(raw) + L)))
    gc = np.fft.ifft(np.fft.fft(raw, n) * np.conj(np.fft.fft(gold, n)))
    gc = gc[:len(raw) - L + 1]
    ac = np.zeros(len(gc), dtype=np.complex128)
    ac[L:] = gc[L:] * np.conj(gc[:-L])
    peak = np.abs(ac) ** 2
    ca = np.abs(gc) ** 2
    # trailing window sum thresh[i] = sum ca[i-L .. i-1]
    csum = np.concatenate(([0.0], np.cumsum(ca)))
    thresh = np.zeros(len(gc))
    for i in range(len(gc)):
        lo = max(0, i - L)
        thresh[i] = csum[i] - csum[lo]
    valid = np.where(corr_scale * peak > thresh)[0]
    ratio = peak / (thresh + 1e-30)
    return (int(valid[0]) if len(valid) else -1), valid, ratio


def main():
    f = sys.argv[1]
    goldf = sys.argv[2] if len(sys.argv) > 2 else "/tmp/gold.bin"
    cs = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    raw = load_cs16(f)
    gold = np.fromfile(goldf, dtype=np.complex64).astype(np.complex128)
    print(f"{f}: {len(raw)} samp  rms {np.sqrt(np.mean(np.abs(raw)**2)):.1f}  "
          f"gold {len(gold)} taps  corr_scale {cs}")
    for sense, g in (("gold", gold), ("conj(gold)", np.conj(gold))):
        idx, valid, ratio = find_beacon(raw, g, cs)
        print(f"  {sense:11s}: find_beacon -> {idx:>7d}   "
              f"#valid_peaks {len(valid):5d}   max(peak/thresh) {ratio.max():.2f}")


if __name__ == "__main__":
    sys.exit(main())
