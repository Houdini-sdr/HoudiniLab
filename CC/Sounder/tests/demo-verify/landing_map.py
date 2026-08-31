#!/usr/bin/env python3
"""Signal-landing map from HOUDINI_BS_DUMP_FRAME dumps (phase 5-7 walk).

Mode A (default): place every burst of a raw BS continuous read on the
ABSOLUTE slot grid (stamp + in-buffer index - epoch, folded into the frame)
and report where the pilot and data slots actually sit versus the transmitted
[zero_prefix | 48x80 symbols | zero_postfix] layout. This is the evidence for
sizing ofdm_tx_zero_prefix/postfix (DEMO_VERIFICATION.md, ledger 4.43+).

Mode B (--resync DIR --core FILE): replicate beaconSnrDb on dumped resync
windows, place the TRUE beacon core by exact-waveform correlation, and
recompute the SNR at the true index and swept around it (ledger 4.42).

NB: SYM/FFT/CP and the [128 | 48x80 | 128] nominal layout are hardcoded to
the shipped houdini configs; re-derive before citing against another config.

Usage:
  landing_map.py DIR                      # BS frame dumps
  landing_map.py --resync DIR --core beacon_core.bin
"""
import argparse
import glob
import os
import sys

import numpy as np

SYM = 80  # cp+fft, the pilot LTS repetition lag


def read_meta(path):
    meta = {}
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "rx_slots":
                meta["rx_slots"] = [int(x) for x in parts[1:]]
            else:
                # Preserve fractions: int(float(x)) truncated snr 31.8 -> 31
                # and the loss was then attributed to the replica (Opus
                # review H4).
                v = float(parts[1])
                meta[parts[0]] = int(v) if v.is_integer() else v
    return meta


def read_iq(path):
    raw = np.fromfile(path, dtype=np.int16).astype(np.float64)
    if raw.size < 2 or raw.size % 2:
        raise SystemExit("%s: empty or odd-length IQ file (%d int16)"
                         % (path, raw.size))
    return raw[0::2] + 1j * raw[1::2]


def selfsim(w, lag=SYM):
    if len(w) <= lag:
        return 0.0
    a, b = w[:-lag], w[lag:]
    den = np.sum(np.abs(a) ** 2)
    return float(np.abs(np.sum(a * np.conj(b))) / den) if den > 0 else 0.0


def refine_edge(x, approx, rising, thr, span=96):
    """Sample-accurate edge from RAW |x|^2 around the smoothed-envelope edge
    (the 64-tap centered envelope biases onsets ~29 samples early and ends
    ~28 late, Opus review H5)."""
    p = np.abs(x) ** 2
    lo, hi = max(0, approx - span), min(len(x), approx + span)
    seg = p[lo:hi]
    above = seg > thr
    idx = np.nonzero(above)[0] if rising else np.nonzero(above)[0]
    if idx.size == 0:
        return approx
    return lo + (int(idx[0]) if rising else int(idx[-1]) + 1)


def find_bursts(x, n):
    """Contiguous above-threshold regions of the smoothed power envelope."""
    p = np.abs(x) ** 2
    k = 64
    env = np.convolve(p, np.ones(k) / k, mode="same")
    # noise floor from the quietest decile, threshold well above it but well
    # below the signal
    floor = np.percentile(env, 10)
    peak = np.percentile(env, 99)
    thr = max(floor * 10.0, peak * 0.05)
    above = env > thr
    bursts = []
    i = 0
    while i < len(above):
        if above[i]:
            j = i
            while j < len(above) and (above[j] or (j + 96 < len(above) and above[j:j + 96].any())):
                j += 1
            if j - i > n // 16:  # ignore blips
                bursts.append((i, j))
            i = j
        else:
            i += 1
    return bursts, env, thr


def mode_a(dump_dir):
    metas = sorted(glob.glob(os.path.join(dump_dir, "bsframe_*.txt")))
    if not metas:
        print(f"no bsframe_*.txt in {dump_dir}")
        return 1
    onsets = {"P": [], "U": []}
    tails = {"P": [], "U": []}
    for mp in metas:
        meta = read_meta(mp)
        x = read_iq(mp.replace(".txt", ".bin"))
        n = meta["n"]
        fr = meta["frame_ticks"]
        stamp_ticks = round(meta["ft_ns"] * meta["tick_rate"] / 1e9)
        base = (stamp_ticks - meta["epoch"]) % fr
        bursts, env, thr = find_bursts(x, n)
        print(f"\n== {os.path.basename(mp)}  frame {meta['frame']} cg={meta['cg']} "
              f"pad={meta['pad']}  p_start={meta['p_start']} u_start={meta['u_start']}")
        for (e0, e1) in bursts:
            w = x[e0:min(e1, e0 + n)]
            burst_med = float(np.median(np.abs(w) ** 2))
            # Sample-accurate edges from the RAW power (H5): the smoothed
            # envelope biases ~29 early / ~28 late.
            b0 = refine_edge(x, e0, True, 0.25 * burst_med)
            b1 = refine_edge(x, e1, False, 0.25 * burst_med)
            g0 = (base + b0) % fr
            g1 = (base + b1) % fr
            s0, o0 = divmod(g0, n)
            s1, o1 = divmod(g1, n)
            sim = selfsim(w)
            kind = "P(pilot)" if sim >= 0.5 else "U(data)"
            key = "P" if sim >= 0.5 else "U"
            # Fold the onset to a signed in-slot offset so a burst starting
            # just before its boundary reads -6, not +4090 (Opus review LOW).
            o0s = int(o0) - n if o0 > n // 2 else int(o0)
            if o0 > n // 2:
                s0 = (s0 + 1) % (fr // n)
            onsets[key].append(o0s)
            tails[key].append((int(s1 - s0), int(o1)))
            rms = float(np.sqrt(np.mean(np.abs(w) ** 2)))
            cross = ""
            ref = meta["p_start"] if key == "P" else meta["u_start"]
            if isinstance(ref, (int, float)) and ref >= 0:
                cross = f" (C++ start {int(ref)}, edge-vs-start {b0 - int(ref):+d})"
            print(f"  burst [{b0:7d},{b1:7d}) len={b1-b0:5d} rms={rms:7.1f} "
                  f"selfsim={sim:.2f} {kind}: onset slot {int(s0)} {o0s:+5d} "
                  f"end slot {int(s1)} +{int(o1):4d}{cross}")
    print("\n== SUMMARY across dumps (48x80=3840 samples expected per burst;")
    print("   transmitted layout [prefix 128 | signal | postfix 128], so nominal")
    print("   onset +128, nominal end +3968 in the SAME slot) ==")
    from collections import Counter
    for key, name in (("P", "pilot"), ("U", "data")):
        if onsets[key]:
            o = np.array(onsets[key])
            print(f"  {name}: onset offset min/med/max = {o.min()}/{int(np.median(o))}/{o.max()}"
                  f"   (early margin left = onset; late spill if end crosses slot)")
            for (ds, o1), cnt in sorted(Counter(tails[key]).items()):
                spill = "SPILLS into next slot" if ds > 0 else "contained"
                print(f"    end at +{o1} {ds} slot(s) past onset slot -> {spill}"
                      f"  x{cnt}")
    return 0


def snr_db(w, end_idx, core_len, guard=8):
    """Replicates receiver.cc beaconSnrDb INCLUDING its 8-sample guard band
    (Opus review H3: the guardless replica re-measured the very cliff the
    guard removed)."""
    lo = end_idx - core_len
    if lo < 0 or end_idx > len(w):
        return -99.0
    e = np.abs(w) ** 2
    core = e[lo:end_idx].sum()
    lo_g = max(0, lo - guard)
    hi_g = min(len(w), end_idx + guard)
    rest = e.sum() - e[lo_g:hi_g].sum()
    nrest = len(w) - (hi_g - lo_g)
    if nrest <= 0 or rest <= 0:
        return 99.0
    return 10 * np.log10((core / core_len) / (rest / nrest) + 1e-30)


def mode_b(dump_dir, core_path):
    core_raw = np.fromfile(core_path, dtype=np.int16).astype(np.float64)
    if len(core_raw) % 2:
        print("odd core file length?")
        return 1
    core = core_raw[0::2] + 1j * core_raw[1::2]
    print(f"core: {len(core)} samples from {core_path}")
    metas = sorted(glob.glob(os.path.join(dump_dir, "resyncwin_*.txt")))
    if not metas:
        print(f"no resyncwin_*.txt in {dump_dir}")
        return 1
    cl = len(core)
    for mp in metas:
        meta = read_meta(mp)
        w = read_iq(mp.replace(".txt", ".bin"))
        det = meta["sync_index"]
        # place the true core: correlate the exact waveform
        sc = np.abs(np.correlate(w, core, mode="valid"))
        t0 = int(np.argmax(sc))
        # Peak-to-sidelobe gate (Opus review): without it a beacon-free
        # window confidently confirms a noise peak as the real beacon.
        # The beacon core's 15x STS16 block self-correlates every 16 samples
        # out to +-240, so the sidelobe exclusion must span the STS block or
        # the gate cries wolf on every genuine beacon (measured 2.16 on a
        # 48 dB window with a +-16 mask).
        mask = np.ones(sc.size, bool)
        mask[max(0, t0 - 256):t0 + 257] = False
        psl = float(sc[t0] / (sc[mask].max() + 1e-30)) if mask.any() else 99.0
        true_end = t0 + cl
        print(f"\n== {os.path.basename(mp)} n={len(w)} live snr={meta['snr']:.1f} dB "
              f"det_idx={det}")
        if psl < 3.0:
            print(f"  NO CONFIDENT CORE (peak/sidelobe {psl:.2f} < 3): "
                  f"window may not contain the beacon; numbers below suspect")
        print(f"  true core start {t0}, end {true_end}  (detector bias "
              f"{det - true_end:+d} samples, peak/sidelobe {psl:.1f})")
        print(f"  SNR replica (guarded, as the live code): at detector idx "
              f"{snr_db(w, det, cl):6.1f} dB | at TRUE end "
              f"{snr_db(w, true_end, cl):6.1f} dB")
        curve = ", ".join(f"{d:+d}:{snr_db(w, true_end + d, cl):.1f}"
                          for d in (-8, -4, -2, -1, 0, 1, 2, 4, 8))
        print(f"  SNR vs end-offset sweep: {curve}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", nargs="?", help="bsframe dump dir (mode A)")
    ap.add_argument("--resync", help="resync window dump dir (mode B)")
    ap.add_argument("--core", help="exact beacon core waveform (ci16)")
    args = ap.parse_args()
    if args.resync:
        if not args.core:
            print("--resync needs --core")
            return 1
        return mode_b(args.resync, args.core)
    if not args.dir:
        print(__doc__)
        return 1
    return mode_a(args.dir)


if __name__ == "__main__":
    sys.exit(main())
