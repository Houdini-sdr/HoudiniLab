#!/usr/bin/env python3
"""Pilot vs UL-data constellation SNR from HOUDINI_BS_DUMP_FRAME dumps.

Ground-truth extraction (raw continuous read, not the live centroid path):
locate the P and U bursts, fine-align by across-symbol pilot coherence, then

  pilot SNR: 48 identical LTS symbols -> per occupied tone, power of the
             across-symbol MEAN over the across-symbol VARIANCE (reference-
             free EVM sense);
  data SNR:  equalize with the pilot-derived H; score against the known
             fixed-seed reference (ul_data_f_*.bin) if present, else
             decision-directed QPSK after 4th-power de-rotation.

Per-symbol EVM is printed for both slots: degradation confined to the first/
last symbols is ISI from guard-margin exhaustion and is the direct evidence
for sizing ofdm_tx_zero_prefix/postfix (phase 5-7 walk).

Usage: evm_compare.py DIR [--ref LOGDIR]
"""
import argparse
import glob
import os
import sys

import numpy as np

from landing_map import read_meta, read_iq, find_bursts, selfsim

FFT = 64
CP = 16
SYM = 80
NSYM = 48

# Consts::lts_seq (constants.h), DC-centered: the +-1 freq-domain LTS. The
# averaged received pilot tones are H*LTS, so the equalizer must divide the
# LTS back out -- skipping it leaves per-tone pi flips that destroy the data
# correlation while every pilot-side metric stays perfect (conjugation- and
# sign-insensitive), which is exactly how the bug hid.
LTS_F = np.zeros(FFT)
LTS_F[6:32] = [1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1,
               1, 1, -1, 1, -1, 1, 1, 1, 1]
LTS_F[33:59] = [1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1,
                -1, -1, 1, -1, 1, -1, 1, 1, 1, 1]


def demod(slot, start, conj_rx=True):
    """DC-centered per-symbol tones, live-pipeline convention (rx_conj +
    fftshift), window base = start + k*80 + cp."""
    s = slot.conj() if conj_rx else slot
    out = np.zeros((NSYM, FFT), dtype=np.complex128)
    for k in range(NSYM):
        base = start + k * SYM + CP
        w = s[base: base + FFT]
        if len(w) < FFT:
            return out[:k]
        out[k] = np.fft.fftshift(np.fft.fft(w, FFT))
    return out


def pilot_coh(t, occ):
    d = t[:, occ]
    return float(np.mean(np.abs(d.mean(0)) / (np.abs(d).mean(0) + 1e-30)))


def align(x, onset):
    """Sweep the demod start around the envelope onset, keep the start that
    maximizes across-symbol pilot-tone coherence (LTS symbols identical)."""
    best, bs = -1.0, onset
    for s0 in range(max(0, onset - 48), onset + 49, 2):
        t = demod(x, s0)
        if len(t) < NSYM:
            continue
        p = np.abs(t).mean(0)
        occ = np.where(p > 0.4 * np.median(p[p > 0]))[0]
        occ = occ[(occ != FFT // 2)]
        if len(occ) < 8:
            continue
        c = pilot_coh(t, occ)
        if c > best:
            best, bs = c, s0
    return bs, best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--ref", default=None, help="logdir with ul_data_f_*.bin")
    args = ap.parse_args()

    ref = None
    refdir = args.ref or args.dir
    fs = sorted(glob.glob(os.path.join(refdir, "../ul_data_f_*.bin"))) or \
        sorted(glob.glob(os.path.join(refdir, "ul_data_f_*.bin")))
    if fs:
        if "QPSK" not in os.path.basename(fs[0]):
            print(f"WARNING: reference {fs[0]} is not tagged QPSK -- a stale "
                  f"reference for another modulation scores as garbage")
        r = np.fromfile(fs[0], dtype=np.float32)
        ref = (r[0::2] + 1j * r[1::2]).astype(np.complex128).reshape(-1, FFT)
        print(f"data reference: {fs[0]}")
    else:
        print("no ul_data_f reference found -> decision-directed QPSK")

    psnrs, dsnrs = [], []
    for mp in sorted(glob.glob(os.path.join(args.dir, "bsframe_*.txt"))):
        meta = read_meta(mp)
        x = read_iq(mp.replace(".txt", ".bin"))
        bursts, _, _ = find_bursts(x, meta["n"])
        p_b = u_b = None
        n_p = n_u = 0
        for (b0, b1) in bursts:
            w = x[b0:min(b1, b0 + meta["n"])]
            if selfsim(w) >= 0.5:
                p_b = b0
                n_p += 1
            else:
                u_b = b0
                n_u += 1
        if p_b is None or u_b is None:
            print(f"{os.path.basename(mp)}: missing burst (P={p_b} U={u_b})")
            continue
        if n_p != 1 or n_u != 1:
            print(f"  WARNING: {n_p} pilot / {n_u} data bursts in this read "
                  f"-- last of each used; the P-U differential may span "
                  f"frames (the 1.17-frame read can carry two copies)")

        ps, pcoh = align(x, p_b)
        tp = demod(x, ps)
        pw = np.abs(tp).mean(0)
        occ = np.where(pw > 0.4 * np.median(pw[pw > 0]))[0]
        occ = occ[occ != FFT // 2]
        # pilot SNR: mean power over across-symbol variance, per occupied tone
        mean_t = tp[:, occ].mean(0)
        var_t = tp[:, occ].var(0)
        # Aggregate SNR (total signal power over total error power): the mean
        # of per-tone ratios lets one quiet-variance tone dominate (Opus
        # review).
        psnr = 10 * np.log10(np.sum(np.abs(mean_t) ** 2)
                             / (np.sum(var_t) + 1e-30))
        # per-symbol pilot EVM vs the across-symbol mean
        pevm = 10 * np.log10(
            np.mean(np.abs(tp[:, occ] - mean_t) ** 2, axis=1)
            / np.mean(np.abs(mean_t) ** 2) + 1e-30)

        # data slot: same in-slot alignment as the pilot (both bursts carry the
        # transmitted [128 | 48x80 | 128] layout), H from the pilot mean
        us = u_b + (ps - p_b)
        H = np.ones(FFT, dtype=np.complex128)
        H[occ] = mean_t * np.where(LTS_F[occ] != 0, LTS_F[occ], 1.0)
        td = demod(x, us) / H
        best_r = 0.0
        best_du = 0
        best_conj = True
        best_sc = float("nan")
        if ref is not None:
            rr = ref[:NSYM]
            good = np.abs(rr[:, occ]) > 1e-9
            # The pilot metrics are conjugation- and (within-CP) placement-
            # insensitive, so neither is validated yet at this point; the data
            # correlation is sensitive to both. Search them empirically: data
            # window placement du (coarse, the envelope onset carries ~30
            # samples of smoothing bias), fractional ramp r (the pilot<->data
            # timing differential, the AP-15 mechanism), and both conjugation
            # conventions. The winning (du, r) is a per-frame readout of the
            # data landing relative to the pilot.
            kk = np.arange(FFT) - FFT // 2

            # Ramp sign matches the sounder (it rotates H by +ang before
            # dividing, so the data acquires e^{-j ang}): this tool's r now
            # reads on the SAME convention as the sounder's logged r (Opus
            # review: the old +j form reported the opposite sign).
            def score(td_c, r):
                z = td_c[:, occ] * np.exp(-1j * 2 * np.pi * r * kk[occ] / FFT)
                num = np.abs(np.sum(z[good] * np.conj(rr[:, occ][good])))
                den = np.sqrt(np.sum(np.abs(z[good]) ** 2) *
                              np.sum(np.abs(rr[:, occ][good]) ** 2))
                return num / (den + 1e-30)

            # r covers the sounder's own +-8 blind range (Opus review H7:
            # +-2 saturated on the measured +3-sample draws and reported the
            # grid edge as the answer).
            best_sc = -1.0
            for cj in (True, False):
                for du in range(-48, 49, 2):
                    td_c = demod(x, us + du, conj_rx=cj) / (H if cj else H.conj())
                    if len(td_c) < NSYM:
                        continue
                    for r in np.arange(-8, 8.01, 0.5):
                        sc = score(td_c, r)
                        if sc > best_sc:
                            best_sc, best_du, best_r, best_conj = sc, du, r, cj
            if best_sc < 0.5:
                print(f"== {os.path.basename(mp)}: NO LOCK (best coherence "
                      f"{best_sc:.3f}) -- skipping frame")
                continue
            td = demod(x, us + best_du, conj_rx=best_conj) / \
                (H if best_conj else H.conj())
            rs = np.arange(best_r - 0.5, best_r + 0.5, 0.02)
            best_r = float(rs[int(np.argmax([score(td, r) for r in rs]))])
            if abs(best_r) >= 7.5:
                print(f"  WARNING: r={best_r:+.2f} at the search edge -- "
                      f"treat as unrecovered, not measured")
            z = td[:, occ] * np.exp(-1j * 2 * np.pi * best_r * kk[occ] / FFT)
            # one global complex gain (the reference has its own scale)
            g = np.sum(z[good] * np.conj(rr[:, occ][good])) / \
                np.sum(np.abs(rr[:, occ][good]) ** 2)
            # Score only reference-valid cells: one leaked null tone in occ
            # otherwise collapses the reported SNR (Opus review).
            err_all = z - g * rr[:, occ]
            sig_all = np.abs(g * rr[:, occ]) ** 2
            dsnr = 10 * np.log10(np.mean(sig_all[good])
                                 / (np.mean(np.abs(err_all[good]) ** 2) + 1e-30))
            gm = good.astype(float)
            devm = 10 * np.log10(
                (np.sum(np.abs(err_all) ** 2 * gm, axis=1)
                 / np.maximum(gm.sum(axis=1), 1))
                / np.mean(sig_all[good]) + 1e-30)
        else:
            z = td[:, occ].flatten()
            z = z[np.abs(z) > 1e-12]
            # (angle - pi)/4, matching recorder_worker's derotation: without
            # the -pi the decisions sit 45 deg off and the reported SNR pins
            # at 2-4 dB anti-correlated with the truth (Opus review H6).
            rot = (np.angle(np.mean((z / np.abs(z)) ** 4)) - np.pi) / 4
            z = z * np.exp(-1j * rot)
            dec = (np.sign(z.real) + 1j * np.sign(z.imag)) / np.sqrt(2)
            scale = np.mean(np.abs(z))
            err = z - dec * scale
            dsnr = 10 * np.log10(scale ** 2 / (np.mean(np.abs(err) ** 2) + 1e-30))
            devm = None

        psnrs.append(psnr)
        dsnrs.append(dsnr)
        print(f"\n== {os.path.basename(mp)} pilot~{ps} (CP-plateau argmax, "
              f"coh {pcoh:.3f}) data~{us}  occ tones {len(occ)}  "
              f"du={best_du:+d} r={best_r:+.2f} (sounder convention) "
              f"conj={best_conj} lock={best_sc:.3f}")
        print(f"  PILOT SNR {psnr:6.1f} dB   DATA SNR {dsnr:6.1f} dB   "
              f"delta {psnr - dsnr:+.1f} dB")
        print("  pilot per-sym EVM dB: " +
              " ".join(f"{v:.0f}" for v in pevm))
        if devm is not None:
            print("  data  per-sym EVM dB: " +
                  " ".join(f"{v:.0f}" for v in devm))
    if psnrs:
        print(f"\n== SUMMARY: pilot SNR {np.mean(psnrs):.1f} dB "
              f"(min {np.min(psnrs):.1f}) | data SNR {np.mean(dsnrs):.1f} dB "
              f"(min {np.min(dsnrs):.1f}) over {len(psnrs)} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
