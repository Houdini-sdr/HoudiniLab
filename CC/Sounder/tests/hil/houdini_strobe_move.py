#!/usr/bin/env python3
"""
houdini_strobe_move.py -- nail the TDD replay-strobe offset->position mapping,
separating the LIVE re-placement effect from CFO drift.

.21 runs a TDD framer (armed ONCE); its replay strobe fires a beacon burst. We
walk the strobe offset through a bracketed sequence -- ref, test, ref, test, ...
all offset (ref) between tests -- and free-run capture on .22 for each. Because
the two boards' clocks drift (CFO), the burst phase wanders on its own; the ref
captures on both sides of each test measure that drift so we can INTERPOLATE it
out. The residual = the pure offset->position effect. We also fit the burst
period across frames to report the CFO / drift rate directly.

Verdict: if the drift-corrected measured delta == the commanded offset delta
(1:1, within a burst width), the live re-placement is clean and the UE can chase
drift by nudging the strobe -- method 1 is viable. Also prints drift samples/s so
we know how often the UE must re-place the strobe.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 houdini_strobe_move.py
"""
import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
_EX = os.environ.get(
    "HOUDINI_EXAMPLES",
    os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if _EX not in sys.path:
    sys.path.insert(0, _EX)

import SoapySDR  # noqa: E402
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX  # noqa: E402
import houdini_setup as hs  # noqa: E402
from beacon_tdd import (build_beacon, _arm, _teardown,  # noqa: E402
                        GRID_TICKS, SYM, ARM_MARGIN)


def matched_filter(x, h):
    n = 1 << int(np.ceil(np.log2(len(x) + len(h))))
    C = np.fft.ifft(np.fft.fft(x, n) * np.conj(np.fft.fft(h, n)))
    return np.abs(C[:len(x) - len(h) + 1])


def to_complex(lanes):
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex128)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float64)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return L[:, 0] + 1j * q


def wrap(x, P):
    return ((x + P / 2) % P) - P / 2


def phase_and_period(corr, frame_rx):
    """Precise burst phase + measured frame period, from per-frame peak positions.

    Coarse-fold to center the burst, take the argmax in each centered frame block
    (clean peaks), then line-fit position vs frame index: intercept -> phase (drift
    corrected within the capture), slope -> the actual frame period on .22 (CFO)."""
    P = int(round(frame_rx))
    nf = len(corr) // P
    if nf < 5:
        return None, None, 0.0
    prof = (corr[:nf * P].reshape(nf, P) ** 2).mean(axis=0)
    ph0 = int(np.argmax(prof))
    thr = 0.3 * float(np.sqrt(prof.max()))
    shift = P // 2 - ph0
    c = np.roll(corr[:nf * P], shift)
    idx, pos = [], []
    for k in range(nf):
        seg = c[k * P:(k + 1) * P]
        j = int(np.argmax(seg))
        if seg[j] >= thr:
            idx.append(k)
            pos.append(k * P + j - shift)  # undo the centering shift
    if len(idx) < 5:
        return None, None, 0.0
    idx = np.array(idx, float)
    pos = np.array(pos, float)
    slope, intercept = np.polyfit(idx, pos, 1)
    z = float((prof[ph0] - np.median(prof)) / (prof.std() + 1e-30))
    return intercept % P, slope, z  # phase, period(samples/frame), z


def cap_measure(rsd, ch, native, dtype, match, center_hz, rx_rate, frame_rx, secs, cap_mb):
    buf, _s = hs.capture_rx(rsd, ch, native, dtype, duration_sec=secs,
                            capture_bytes=int(cap_mb * 1024 * 1024))
    iq = to_complex(hs.cs16_lanes(buf))
    n = np.arange(len(iq))
    ddc = iq * np.exp(+2j * np.pi * center_hz * n / rx_rate)
    c0 = matched_filter(ddc, match)
    c1 = matched_filter(ddc, np.conj(match))
    corr = c0 if c0.max() >= c1.max() else c1
    return phase_and_period(corr, frame_rx)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--nco", type=float, default=500.0)
    ap.add_argument("--center-mhz", type=float, default=20.0)
    ap.add_argument("--n-sc", type=int, default=63)
    ap.add_argument("--n-load", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--spf", type=int, default=4)
    ap.add_argument("--secs", type=float, default=0.12)
    ap.add_argument("--cap-mb", type=float, default=24.0)
    a = ap.parse_args()

    print(f"TX {a.tx_ip} ch{a.tx_ch} [TDD framer armed ONCE] -> RX {a.rx_ip} ch{a.rx_ch}  spf={a.spf}")
    tx_ctx = hs.open_device(node=a.tx_ip, ch=a.tx_ch, verbose=False)
    rx_ctx = hs.open_device(node=a.rx_ip, ch=a.rx_ch, verbose=False)
    tsd, rsd = tx_ctx["sdr"], rx_ctx["sdr"]
    rnative, rdtype = rx_ctx["native_fmt"], rx_ctx["dtype"]
    tick_rate = float(dict(tsd.getHardwareInfo()).get("tick_rate_hz", 122.88e6))

    _teardown(tsd)
    ladder = list(tsd.listSampleRates(SOAPY_SDR_TX, a.tx_ch))
    if ladder:
        tsd.setSampleRate(SOAPY_SDR_TX, a.tx_ch, max(ladder))
    dac_rate = float(tsd.getSampleRate(SOAPY_SDR_TX, a.tx_ch))
    tsd.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.nco * 1e6)
    rsd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, 122.88e6)
    rsd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
    rx_rate = float(rsd.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
    frame_rx = a.spf * SYM * rx_rate / tick_rate
    burst_rx = int(round(a.n_load * rx_rate / dac_rate))

    i16, _ = build_beacon("zc", dac_rate, a.n_load, a.center_mhz * 1e6, a.amp, 0.0, a.n_sc)
    m16, _ = build_beacon("zc", rx_rate, burst_rx, 0.0, 1.0, 0.0, a.n_sc)
    match = (m16[0::2].astype(np.float64) + 1j * m16[1::2]).astype(np.complex128)
    match /= np.sqrt(np.sum(np.abs(match) ** 2)) + 1e-30

    REF = GRID_TICKS
    tests = [(SYM // 4) // GRID_TICKS * GRID_TICKS,
             (SYM // 2) // GRID_TICKS * GRID_TICKS,
             (3 * SYM // 4) // GRID_TICKS * GRID_TICKS]
    # bracketed sequence: ref, t1, ref, t2, ref, t3, ref
    seq = [REF]
    for t in tests:
        seq += [t, REF]

    tx = tsd.setupStream(SOAPY_SDR_TX, tx_ctx["native_fmt"], [a.tx_ch], {"tx_mode": "replay"})
    rec = []  # (offset, wall_t, phase, period, z)
    try:
        cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)
        tsd.writeStream(tx, [cs16], a.n_load, 0, 0)
        pat = ["0"] * a.spf
        pat[0] = "6"
        tsd.writeSetting("TDD_SCHED", "".join(pat))
        tsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:len={a.n_load//2},offs={REF}")
        r = _arm(tsd, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        assert r.get("accepted") == 1, f"arm rejected: {r}"
        t_zero = time.monotonic()
        print(f"  armed once: epoch={r['epoch']}  frame_rx={frame_rx:.0f} samp  ref_offs={REF}\n")
        print("   step  offset(tk)  wall(s)  phase(samp)  period(samp)  z")
        for off in seq:
            tsd.writeSetting("TDD_REPLAY_STROBE",
                             f"ch{a.tx_ch}:len={a.n_load//2},offs={off}")  # LIVE, no re-arm
            tw = time.monotonic() - t_zero
            ph, per, z = cap_measure(rsd, a.rx_ch, rnative, rdtype, match,
                                     a.center_mhz * 1e6, rx_rate, frame_rx, a.secs, a.cap_mb)
            rec.append((off, tw, ph, per, z))
            print(f"   {'ref' if off==REF else 'tst':>4}  {off:8d}  {tw:6.2f}  "
                  f"{str(None) if ph is None else f'{ph:8.0f}'}  "
                  f"{str(None) if per is None else f'{per:10.1f}'}  {z:5.1f}")
    finally:
        try:
            tsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:off")
        except Exception:  # noqa: BLE001
            pass
        try:
            tsd.deactivateStream(tx); tsd.closeStream(tx)
        except Exception:  # noqa: BLE001
            pass
        _teardown(tsd)

    P = int(round(frame_rx))
    refs = [(tw, ph) for (o, tw, ph, per, z) in rec if o == REF and ph is not None and z >= 6]
    if len(refs) < 2:
        print("\nRESULT: not enough ref detections")
        return 1
    # drift rate from the ref captures (unwrap phase vs time)
    rt = np.array([r[0] for r in refs])
    rp = np.array([r[1] for r in refs])
    rpu = rp.copy()
    for i in range(1, len(rpu)):
        rpu[i] = rpu[i - 1] + wrap(rp[i] - rpu[i - 1], P)
    drift_slope = np.polyfit(rt, rpu, 1)[0]  # samples/sec
    print(f"\n  measured frame period ~{np.nanmean([r[3] for r in rec if r[3]]):.1f} samp "
          f"(ideal {P}); DRIFT ~{drift_slope:+.0f} samp/s "
          f"({drift_slope/P*1e3:+.2f} frame-offset ppm-ish per s)")

    def ref_baseline(tw):  # unwrapped drift baseline interpolated to time tw
        return np.interp(tw, rt, rpu)

    print("\n  commanded delta vs DRIFT-CORRECTED measured delta (samples):")
    ok = True
    for (o, tw, ph, per, z) in rec:
        if o == REF or ph is None or z < 6:
            continue
        base = ref_baseline(tw)  # where ref would be at this time
        meas = wrap(ph - base, P)          # offset-induced move, drift removed
        cmd = o - REF
        err = meas - cmd
        good = abs(err) < max(burst_rx, P // 40)
        ok = ok and good
        print(f"    offs {o:7d}:  cmd {cmd:+8d}   meas {meas:+8.0f}   err {err:+7.0f}  "
              f"{'ok' if good else 'BAD'}")
    print(f"\nRESULT: offset->position mapping is "
          + ("CLEAN 1:1 (live re-placement is precise; method 1 viable, drift-adjustable)"
             if ok else "NOT clean 1:1 (live re-placement is imprecise -> method 1 risky)"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
