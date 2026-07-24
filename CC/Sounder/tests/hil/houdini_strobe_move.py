#!/usr/bin/env python3
"""
houdini_strobe_move.py -- decisive check for method 1 (client TDD-strobe pilot):
can TDD_REPLAY_STROBE be RE-PLACED on an ALREADY-ARMED, running framer, so the UE
can chase the beacon's drift by nudging the strobe offset every frame?

.21 runs a TDD framer (armed ONCE) whose replay strobe fires a beacon burst on
symbol 0. We then rewrite TDD_REPLAY_STROBE to a sequence of offsets WITHOUT
re-arming, and for each, free-run capture on .22 + matched-filter + fold at the
frame period to measure where the burst actually lands. If the measured phase
TRACKS the commanded offset (live, no re-arm), method 1 is viable AND drift-
adjustable. If the phase stays put (rewrite ignored) or only a re-arm moves it,
method 1 is dead and we pivot.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 houdini_strobe_move.py
"""
import argparse
import os
import sys

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


def measure_phase(rsd, ch, native, dtype, match, center_hz, rx_rate, frame_rx,
                  secs, cap_mb):
    buf, _s = hs.capture_rx(rsd, ch, native, dtype, duration_sec=secs,
                            capture_bytes=int(cap_mb * 1024 * 1024))
    iq = to_complex(hs.cs16_lanes(buf))
    n = np.arange(len(iq))
    ddc = iq * np.exp(+2j * np.pi * center_hz * n / rx_rate)
    c0 = matched_filter(ddc, match)
    c1 = matched_filter(ddc, np.conj(match))
    corr = c0 if c0.max() >= c1.max() else c1
    P = int(round(frame_rx))
    nfold = len(corr) // P
    if nfold < 3:
        return None, 0.0, nfold
    prof = (corr[:nfold * P].reshape(nfold, P) ** 2).mean(axis=0)
    ph = int(np.argmax(prof))
    mask = np.ones(P, dtype=bool)
    g = max(8, P // 64)
    mask[max(0, ph - g):ph + g] = False
    z = (prof[ph] - prof[mask].mean()) / (prof[mask].std() + 1e-30)
    return ph, float(z), nfold


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
    ap.add_argument("--secs", type=float, default=0.25)
    ap.add_argument("--cap-mb", type=float, default=40.0)
    a = ap.parse_args()

    print(f"TX {a.tx_ip} ch{a.tx_ch} [TDD framer, armed ONCE] -> RX {a.rx_ip} ch{a.rx_ch} "
          f"[free-run]  spf={a.spf}")
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

    i16, label = build_beacon("zc", dac_rate, a.n_load, a.center_mhz * 1e6,
                              a.amp, 0.0, a.n_sc)
    m16, _ = build_beacon("zc", rx_rate, burst_rx, 0.0, 1.0, 0.0, a.n_sc)
    match = (m16[0::2].astype(np.float64) + 1j * m16[1::2]).astype(np.complex128)
    match /= np.sqrt(np.sum(np.abs(match) ** 2)) + 1e-30

    tx = tsd.setupStream(SOAPY_SDR_TX, tx_ctx["native_fmt"], [a.tx_ch], {"tx_mode": "replay"})
    rows = []
    try:
        cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)
        tsd.writeStream(tx, [cs16], a.n_load, 0, 0)
        pat = ["0"] * a.spf
        pat[0] = "6"                                      # strobe on symbol 0
        tsd.writeSetting("TDD_SCHED", "".join(pat))
        # ---- ARM ONCE ----
        tsd.writeSetting("TDD_REPLAY_STROBE",
                         f"ch{a.tx_ch}:len={a.n_load//2},offs={GRID_TICKS}")
        r = _arm(tsd, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        assert r.get("accepted") == 1, f"arm rejected: {r}"
        print(f"  armed once: epoch={r['epoch']}  frame_rx={frame_rx:.0f} samp\n")

        # ---- walk the strobe offset LIVE (no re-arm) ----
        offsets = [GRID_TICKS, SYM // 4, SYM // 2, (3 * SYM) // 4]
        offsets = [(o // GRID_TICKS) * GRID_TICKS for o in offsets]
        print("  offset(tk)  ->  measured burst phase(samp)  z(sigma)  [live rewrite]")
        for off in offsets:
            tsd.writeSetting("TDD_REPLAY_STROBE",
                             f"ch{a.tx_ch}:len={a.n_load//2},offs={off}")  # LIVE, no re-arm
            ph, z, nf = measure_phase(rsd, a.rx_ch, rnative, rdtype, match,
                                      a.center_mhz * 1e6, rx_rate, frame_rx,
                                      a.secs, a.cap_mb)
            rows.append((off, ph, z))
            print(f"    {off:7d}     ->  phase {str(ph):>8}   z={z:5.1f}   (folded {nf} frames)")
    finally:
        try:
            tsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:off")
        except Exception:  # noqa: BLE001
            pass
        for s in (tx,):
            try:
                tsd.deactivateStream(s); tsd.closeStream(s)
            except Exception:  # noqa: BLE001
                pass
        _teardown(tsd)

    # ---- verdict: does the measured phase TRACK the commanded offset? ----
    good = [(o, p) for (o, p, z) in rows if p is not None and z >= 6.0]
    print()
    if len(good) < 2:
        print("RESULT: too few detections to judge (raise --secs/--cap-mb / check cable)")
        return 1
    o0, p0 = good[0]
    # phase wraps mod frame; unwrap the deltas vs the first point
    P = int(round(frame_rx))
    dcmd = np.array([(o - o0) for (o, _p) in good])
    dmeas = np.array([((p - p0 + P // 2) % P) - P // 2 for (_o, p) in good])
    err = dmeas - dcmd
    print("  commanded delta vs measured delta (samples):")
    for (dc, dm) in zip(dcmd, dmeas):
        print(f"    cmd {dc:+8d}   meas {dm:+8d}   err {dm-dc:+6d}")
    tracks = np.all(np.abs(err) < max(burst_rx, P // 40))
    print(f"\nRESULT: live strobe re-placement {'WORKS -- burst tracks the offset (method 1 viable, drift-adjustable)' if tracks else 'does NOT track (method 1 needs re-arm / is not viable as-is)'}")
    return 0 if tracks else 1


if __name__ == "__main__":
    sys.exit(main())
