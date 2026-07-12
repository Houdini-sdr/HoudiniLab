#!/usr/bin/python3
"""
 inspect_rx_record.py

 Inspect an rx-recorder capture (CC/Sounder/rx-recorder, see RX_RECORDER.md):
 attributes, the /Data/Gaps untrusted-extent table, payload sanity, and with
 --deep a per-slot RMS timeline + spectrum. With --tone (AP-5 loopback
 verification) it checks the tone's slot-boundary PHASE CONTINUITY against
 the gap table: a phase jump at a boundary no extent explains means the gap
 accounting missed samples.

 The standalone analog of PYTHON/IrisUtils/plot_hdf5.py --deep-inspect for
 the rx-recorder schema. Requires numpy + h5py; matplotlib only with --plot.

 Usage:
    ./inspect_rx_record.py capture.h5 [--deep] [--tone auto|HZ] [--plot out.png]

---------------------------------------------------------------------
 Copyright (c) 2018-2026. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import argparse
import sys

import h5py
import numpy as np

GAP_CAUSES = {0: "time_jump", 1: "host_ring", 2: "write_error",
              3: "backward", 4: "resync"}
FULL_SCALE = 32767.0


def load(path):
    f = h5py.File(path, "r")
    grp = f["Data"]
    attrs = {k: grp.attrs[k] for k in grp.attrs}
    samples = f["Data/Samples"]
    gaps = np.zeros((0, 4), dtype=np.int64)
    if "Gaps" in f["Data"]:
        g = f["Data/Gaps"]
        if g.shape[0] > 0:
            gaps = g[...]
    return f, attrs, samples, gaps


def scalar(v):
    """HDF5 scalar attributes arrive as 1-element arrays; unwrap them."""
    a = np.asarray(v)
    return a.reshape(-1)[0] if a.size else v


def attr_num(attrs, key, default=0):
    v = scalar(attrs.get(key, default))
    try:
        return int(v)
    except (TypeError, ValueError):
        return int(float(v))


def read_slot(samples, slot):
    row = samples[slot, 0, 0, 0, :]
    return row[0::2].astype(np.float32) + 1j * row[1::2].astype(np.float32)


def print_attrs(attrs):
    print("==== attributes ====")
    for k in sorted(attrs):
        print(f"  {k:24s} = {scalar(attrs[k])}")


def print_trust(attrs, gaps, n_slots, samps_per_slot, rate):
    print("==== trust report ====")
    total = n_slots * samps_per_slot
    untrusted = float(scalar(attrs.get("TOTAL_UNTRUSTED_SAMPLES", 0)))
    print(f"  slots recorded        : {n_slots} ({total} samples, "
          f"{total / rate:.3f} s)")
    print(f"  write errors          : {attr_num(attrs, 'WRITE_ERRORS')}")
    if "READ_PATH" in attrs:
        exact = attr_num(attrs, "GAPS_EXACT", -1)
        exactness = {1: "sample-exact", 0: "read-widened"}.get(exact, "?")
        print(f"  read path             : {scalar(attrs['READ_PATH'])} "
              f"({exactness} extents)")
    print(f"  untrusted samples     : {untrusted:.0f} "
          f"({100.0 * untrusted / max(total, 1):.2f} %)")
    if gaps.shape[0] == 0:
        print("  gap extents           : none — capture fully trusted")
        return
    print(f"  gap extents           : {gaps.shape[0]}")
    for cause in np.unique(gaps[:, 3]):
        sel = gaps[gaps[:, 3] == cause]
        name = GAP_CAUSES.get(int(cause), f"cause{cause}")
        print(f"    {name:12s}: {sel.shape[0]:8d} extents, "
              f"{int(sel[:, 1].sum()):12d} samples")
    resyncs = gaps[gaps[:, 3] == 4]
    if resyncs.shape[0] > 0:
        first = int(resyncs[0, 0])
        print(f"  !! time-base resync at sample {first}: absolute times from "
              f"FIRST_SAMPLE_TIME_NS are INVALID past that point")
    # Largest loss extents.
    loss = gaps[gaps[:, 1] > 0]
    if loss.shape[0] > 0:
        top = loss[np.argsort(loss[:, 1])[::-1][:5]]
        print("  largest extents (start_sample, n_samples, cause):")
        for row in top:
            print(f"    {int(row[0]):14d}  {int(row[1]):12d}  "
                  f"{GAP_CAUSES.get(int(row[3]), '?')}")
        # Per-second loss timeline.
        n_sec = int(np.ceil(total / rate))
        timeline = np.zeros(min(n_sec, 3600))
        for row in loss:
            sec = int(row[0] / rate)
            if 0 <= sec < timeline.size:
                timeline[sec] += row[1]
        print("  loss by second (untrusted fraction):")
        for s, v in enumerate(timeline):
            if v > 0:
                print(f"    s{s}: {100.0 * v / rate:.1f} %")


def payload_stats(samples, n_slots, samps_per_slot, n_probe=8):
    print("==== payload sanity ====")
    idx = np.unique(np.linspace(0, n_slots - 1, n_probe, dtype=int))
    mx = 0.0
    zero = 0
    count = 0
    acc = 0.0
    clipped = 0
    for slot in idx:
        iq = read_slot(samples, slot)
        m = np.abs(np.concatenate([iq.real, iq.imag]))
        mx = max(mx, float(m.max()))
        clipped += int((m >= 0.98 * FULL_SCALE).sum())
        zero += int((m == 0).sum())
        acc += float(m.mean()) * m.size
        count += m.size
    print(f"  probed slots          : {[int(i) for i in idx]}")
    print(f"  peak |int16|          : {mx:.0f} ({20 * np.log10(max(mx, 1) / FULL_SCALE):.1f} dBFS)")
    print(f"  mean |int16|          : {acc / max(count, 1):.1f}")
    print(f"  zero fraction         : {100.0 * zero / max(count, 1):.2f} %")
    if clipped > 0:
        print(f"  !! clipping           : {clipped} samples >= 98% full scale")


def rms_timeline(samples, n_slots, probe_len=2048, max_points=512):
    step = max(1, n_slots // max_points)
    slots = np.arange(0, n_slots, step)
    rms = np.zeros(slots.size)
    for i, slot in enumerate(slots):
        row = samples[slot, 0, 0, 0, : 2 * probe_len]
        x = row.astype(np.float32)
        rms[i] = np.sqrt(np.mean(x * x))
    return slots, rms


def spectrum(samples, n_slots, samps_per_slot, rate, nfft=65536, n_avg=8):
    idx = np.unique(np.linspace(0, n_slots - 1, n_avg, dtype=int))
    acc = np.zeros(nfft)
    win = np.hanning(nfft)
    for slot in idx:
        iq = read_slot(samples, slot)[:nfft]
        if iq.size < nfft:
            continue
        iq = (iq - iq.mean()) * win
        acc += np.abs(np.fft.fftshift(np.fft.fft(iq))) ** 2
    acc /= max(len(idx), 1)
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, 1.0 / rate))
    psd_db = 10 * np.log10(acc / (FULL_SCALE * nfft / 2) ** 2 + 1e-20)
    return freqs, psd_db


def deep_inspect(samples, n_slots, samps_per_slot, rate):
    print("==== deep inspect ====")
    slots, rms = rms_timeline(samples, n_slots)
    lo, hi = float(rms.min()), float(rms.max())
    print(f"  slot RMS (probe)      : min {lo:.1f}  max {hi:.1f}  "
          f"median {float(np.median(rms)):.1f}")
    dead = slots[rms < 0.01 * max(hi, 1)]
    if dead.size > 0:
        print(f"  !! near-silent slots  : {dead.size} of {slots.size} probed "
              f"(first: {dead[:8].tolist()})")
    iq = read_slot(samples, 0)
    dc = iq.mean()
    print(f"  DC offset (slot 0)    : {dc.real:.1f} + {dc.imag:.1f}j")
    freqs, psd_db = spectrum(samples, n_slots, samps_per_slot, rate)
    peak = int(np.argmax(psd_db))
    print(f"  spectrum peak         : {freqs[peak] / 1e6:+.3f} MHz at "
          f"{psd_db[peak]:.1f} dBFS")
    return (slots, rms), (freqs, psd_db)


def tone_phase_check(samples, gaps, n_slots, samps_per_slot, rate, tone_hz,
                     probe_len=4096):
    """AP-5 verification: a looped TX-replay tone is phase-continuous by
    construction, so its phase at each slot boundary is predictable. A jump
    that no /Data/Gaps extent explains = unaccounted sample loss."""
    print("==== tone phase continuity ====")
    freqs, psd_db = spectrum(samples, n_slots, samps_per_slot, rate)
    peak = int(np.argmax(psd_db))
    prominence = float(psd_db[peak] - np.median(psd_db))
    if tone_hz == "auto":
        tone_hz = float(freqs[peak])
        print(f"  tone (auto-detected)  : {tone_hz / 1e6:+.4f} MHz "
              f"({prominence:.1f} dB above median)")
    else:
        tone_hz = float(tone_hz)
        print(f"  tone (given)          : {tone_hz / 1e6:+.4f} MHz")
    if prominence < 20.0:
        print(f"  !! no dominant tone (prominence {prominence:.1f} dB < 20) — "
              "phase continuity needs the AP-5 loopback tone; skipping")
        return 0

    # Refine the tone to a per-slot-coherent estimate, then measure each
    # slot's phase at its own start using the GLOBAL sample index, so the
    # expected phase is identical for every slot; deviations localize loss.
    n = np.arange(probe_len)
    # Suspect slot set: any slot overlapping a gap extent (or adjacent).
    suspect = np.zeros(n_slots, dtype=bool)
    for row in gaps:
        first = int(row[0] // samps_per_slot)
        last = int((row[0] + max(row[1], 1) - 1) // samps_per_slot)
        suspect[max(first, 0): min(last + 2, n_slots)] = True

    phases = np.zeros(n_slots)
    for slot in range(n_slots):
        iq = read_slot(samples, slot)[:probe_len]
        base = slot * samps_per_slot
        lo_ref = np.exp(-2j * np.pi * tone_hz * (base + n) / rate)
        phases[slot] = np.angle(np.sum(iq * lo_ref))

    dphi = np.angle(np.exp(1j * (phases[1:] - phases[:-1])))
    # Threshold: generous 30 deg — noise on a real tone is far below this.
    jump = np.abs(dphi) > np.deg2rad(30)
    n_jump = int(jump.sum())
    explained = int((jump & suspect[1:]).sum())
    unexplained = n_jump - explained
    print(f"  boundaries checked    : {n_slots - 1}")
    print(f"  phase jumps (>30 deg) : {n_jump} "
          f"({explained} explained by /Data/Gaps)")
    if unexplained > 0:
        bad = np.nonzero(jump & ~suspect[1:])[0] + 1
        print(f"  !! UNEXPLAINED jumps  : {unexplained} at slots "
              f"{bad[:10].tolist()} — gap accounting missed samples")
    else:
        print("  verdict               : gap accounting complete "
              "(no unexplained discontinuity)")
    return unexplained


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[2].strip())
    p.add_argument("file")
    p.add_argument("--deep", action="store_true",
                   help="RMS timeline + DC + spectrum")
    p.add_argument("--tone", default=None, metavar="auto|HZ",
                   help="verify tone phase continuity vs the gap table (AP-5)")
    p.add_argument("--plot", default=None, metavar="OUT.png",
                   help="write RMS/spectrum/gap plots (needs matplotlib)")
    args = p.parse_args()

    f, attrs, samples, gaps = load(args.file)
    rate = float(scalar(attrs["RATE"]))
    samps_per_slot = attr_num(attrs, "SLOT_SAMP_LEN")
    n_slots = attr_num(attrs, "SLOTS_RECORDED")
    if n_slots == 0:
        print("SLOTS_RECORDED = 0 — nothing to inspect")
        return 1

    print_attrs(attrs)
    print_trust(attrs, gaps, n_slots, samps_per_slot, rate)
    payload_stats(samples, n_slots, samps_per_slot)

    deep_data = None
    if args.deep or args.plot:
        deep_data = deep_inspect(samples, n_slots, samps_per_slot, rate)

    status = 0
    if args.tone is not None:
        if "TX_REPLAY_FREQ_ACTUAL" in attrs:
            # Armed by rx-recorder's tx_replay section (bin-snapped DAC
            # baseband). The RX-observed frequency additionally depends on
            # the mixer/NCO mapping, so this is context, not an assertion.
            armed = float(scalar(attrs["TX_REPLAY_FREQ_ACTUAL"]))
            print(f"file records an armed TX-replay tone: "
                  f"{armed / 1e6:.6f} MHz DAC baseband")
        status = 1 if tone_phase_check(samples, gaps, n_slots, samps_per_slot,
                                       rate, args.tone) > 0 else 0

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        (slots, rms), (freqs, psd_db) = deep_data
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7))
        ax1.plot(slots, rms, lw=0.8)
        for row in gaps[gaps[:, 1] > 0]:
            ax1.axvspan(row[0] / samps_per_slot,
                        (row[0] + row[1]) / samps_per_slot,
                        color="red", alpha=0.25)
        ax1.set_xlabel("slot")
        ax1.set_ylabel("probe RMS (int16)")
        ax1.set_title("per-slot RMS (red: untrusted extents)")
        ax2.plot(freqs / 1e6, psd_db, lw=0.6)
        ax2.set_xlabel("baseband MHz")
        ax2.set_ylabel("dBFS")
        ax2.set_title("averaged spectrum")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"plots -> {args.plot}")

    f.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
