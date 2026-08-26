#!/usr/bin/env python3
"""
houdini_tdd_loopback.py -- v2: beacon-synced UE + tx_advance sweep -> closed,
frame-aligned TDD loop over the native Houdini framer.

Both boards run their OWN TDD framer at the SAME period (spf*SYM ticks). Armed
ONCE each, their frames are phase-locked (only slow CFO drift), so the UE pilot --
fired once per frame by the replay strobe at a fixed offset -- lands at a FIXED,
well-defined position in the BS frame. That fixed position is exactly what a
constant tx_advance buys: the beacon carries the BS frame timing to the UE, and a
single advance keeps the pilot in the BS rx_gate window every frame (no shared
hardware clock).

We LOCATE that fixed landing position by sweeping the BS receive window across the
frame and detecting the pilot -- the dual of a tx_advance sweep, and robust
(re-arming a framer per point would randomize its epoch). .21's ADC_C hears only
.22's DAC_A (the reverse cable; no self-reception), so the peak is unambiguously
the UE pilot. The peak position => the tx_advance that centers the pilot in a
chosen rx_gate slot. The UE also free-run-detects the BS beacon (its timing
reference) as a sanity check.

Fine-grained timed TX on Houdini is ONLY the framer replay strobe (offs on the
3.125 us grid); activateStream(TX,HAS_TIME) is whole-ms, too coarse for a slot.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 houdini_tdd_loopback.py
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
from SoapySDR import (SOAPY_SDR_RX, SOAPY_SDR_TX, SOAPY_SDR_HAS_TIME,  # noqa: E402
                      SOAPY_SDR_END_BURST)
import houdini_setup as hs  # noqa: E402
from houdini_setup import (run_burst, rx_stream_args, rx_framing,  # noqa: E402
                           iq_from_cs16)
from beacon_tdd import (build_beacon, _arm, _cmd, _teardown, _ns_of_tick,  # noqa: E402
                        _hw_tick, _next_window_tick, GRID_TICKS, SYM, ARM_MARGIN)


def matched_filter(x, h):
    n = 1 << int(np.ceil(np.log2(len(x) + len(h))))
    C = np.fft.ifft(np.fft.fft(x, n) * np.conj(np.fft.fft(h, n)))
    return np.abs(C[:len(x) - len(h) + 1])


def detect(iq, match, center_hz, rx_rate):
    """DDC + matched filter (both conj senses): peak dB over median."""
    if len(iq) < len(match) + 4:
        return 0.0
    n = np.arange(len(iq))
    ddc = iq * np.exp(+2j * np.pi * center_hz * n / rx_rate)
    c0 = matched_filter(ddc, match)
    c1 = matched_filter(ddc, np.conj(match))
    corr = c0 if c0.max() >= c1.max() else c1
    return 20 * np.log10(corr.max() / (np.median(corr) + 1e-30))


def arm_retry(sdr, tag, **kw):
    for _ in range(4):
        r = _arm(sdr, **kw)
        if r.get("accepted") == 1:
            return r
        _cmd(sdr, "abort")
    raise RuntimeError(f"{tag} TDD arm rejected: {r}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bs-ip", default="168.6.244.21")
    ap.add_argument("--ue-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--nco", type=float, default=500.0)
    ap.add_argument("--center-mhz", type=float, default=20.0)
    ap.add_argument("--n-sc", type=int, default=63)
    ap.add_argument("--n-load", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--spf", type=int, default=4, help="symbols/frame (both boards)")
    ap.add_argument("--pilot-sym", type=int, default=2, help="target BS rx_gate slot")
    ap.add_argument("--step-us", type=float, default=25.0, help="window-scan step (us)")
    ap.add_argument("--win-per-pt", type=int, default=2)
    ap.add_argument("--det-db", type=float, default=18.0)
    a = ap.parse_args()

    print(f"BS {a.bs_ip} [beacon@0, scan RX window]  UE {a.ue_ip} [pilot strobe @ fixed "
          f"offset]  spf={a.spf}  frame={a.spf*SYM} tk ({a.spf*0.5:.1f} ms)")
    bs = hs.open_device(node=a.bs_ip, ch=a.tx_ch, verbose=False)
    ue = hs.open_device(node=a.ue_ip, ch=a.tx_ch, verbose=False)
    bsd, ued = bs["sdr"], ue["sdr"]
    native, dtype, bps = bs["native_fmt"], bs["dtype"], bs["bytes_per_samp"]
    tick_rate = float(dict(bsd.getHardwareInfo()).get("tick_rate_hz", 122.88e6))
    frame = a.spf * SYM

    for sdr in (bsd, ued):
        _teardown(sdr)
        ladder = list(sdr.listSampleRates(SOAPY_SDR_TX, a.tx_ch))
        if ladder:
            sdr.setSampleRate(SOAPY_SDR_TX, a.tx_ch, max(ladder))
        sdr.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.nco * 1e6)
        sdr.setSampleRate(SOAPY_SDR_RX, a.rx_ch, 122.88e6)
        sdr.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
    dac_rate = float(bsd.getSampleRate(SOAPY_SDR_TX, a.tx_ch))
    rx_rate = float(bsd.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
    spt = rx_rate / tick_rate
    burst_rx = int(round(a.n_load * rx_rate / dac_rate))

    wave, label = build_beacon("zc", dac_rate, a.n_load, a.center_mhz * 1e6,
                               a.amp, 0.0, a.n_sc)
    wave32 = np.ascontiguousarray(wave, np.int16).view(np.int32)
    m16, _ = build_beacon("zc", rx_rate, burst_rx, 0.0, 1.0, 0.0, a.n_sc)
    match = (m16[0::2].astype(np.float64) + 1j * m16[1::2]).astype(np.complex128)
    match /= np.sqrt(np.sum(np.abs(match) ** 2)) + 1e-30
    print(f"  dac {dac_rate/1e6:.2f}  rx {rx_rate/1e6:.2f}  burst {burst_rx} samp  "
          f"{label}  RF {a.nco+a.center_mhz:.1f} MHz")

    bs_tx = bsd.setupStream(SOAPY_SDR_TX, native, [a.tx_ch], {"tx_mode": "replay"})
    bs_rx = bsd.setupStream(SOAPY_SDR_RX, native, [a.rx_ch], rx_stream_args(a.rx_ch))
    ue_tx = ued.setupStream(SOAPY_SDR_TX, ue["native_fmt"], [a.tx_ch], {"tx_mode": "replay"})
    per_packet = rx_framing(bsd, verbose=False)["frame_words"] * (8 // bps)
    cap = max(per_packet, (int(SYM * spt) // 8 // per_packet) * per_packet)  # ~62us window
    step = max(GRID_TICKS,
               int(round(a.step_us * 1e-6 * tick_rate)) // GRID_TICKS * GRID_TICKS)
    ue_sym = a.spf // 2
    ue_off = ue_sym * SYM + GRID_TICKS

    profile = []
    try:
        # --- UE: arm ONCE, pilot strobe at a fixed frame-offset (ue_off) ---
        ued.writeStream(ue_tx, [wave32], a.n_load, 0, 0)
        pu = ["0"] * a.spf
        pu[ue_sym] = "6"
        ued.writeSetting("TDD_SCHED", "".join(pu))
        ued.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:len={a.n_load//2},offs={GRID_TICKS}")
        arm_retry(ued, "UE", symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        print(f"  UE armed ONCE: pilot @ frame-offset {ue_off} tk "
              f"({ue_off/tick_rate*1e6:.1f} us)")

        # --- BS: arm ONCE, beacon@0, rx_gate on ALL symbols so the window scans ---
        bsd.writeStream(bs_tx, [wave32], a.n_load, 0, 0)
        pb = ["2"] * a.spf
        pb[0] = "6"                                   # beacon strobe @ slot0
        bsd.writeSetting("TDD_SCHED", "".join(pb))
        bsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:len={a.n_load//2},offs={GRID_TICKS}")
        rb = arm_retry(bsd, "BS", symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        epoch_bs = rb["epoch"]
        print(f"  BS armed ONCE: epoch={epoch_bs}  scan window {cap} samp "
              f"({cap/rx_rate*1e6:.1f} us)\n")

        # --- UE hears the BS beacon (its timing reference) ---
        buf, _s = hs.capture_rx(ued, a.rx_ch, ue["native_fmt"], dtype,
                                duration_sec=0.05, capture_bytes=8 << 20)
        iqb = np.asarray(hs.iq_from_lanes(hs.cs16_lanes(buf), "interleaved"),
                         dtype=np.complex128)
        print(f"  UE hears BS beacon: {detect(iqb, match, a.center_mhz*1e6, rx_rate):.1f} dB "
              f"(frame-timing reference)\n")

        # --- scan the BS receive window across the frame to LOCATE the UE pilot ---
        print(f"  scan: step {step} tk ({step/tick_rate*1e6:.1f} us), {frame//step} points")
        for w in range(0, frame, step):
            dbs = []
            for _ in range(a.win_per_pt):
                base = _next_window_tick(bsd, tick_rate, epoch_bs, frame, 0)
                ot = base + w
                if bsd.activateStream(bs_rx, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                                      _ns_of_tick(ot), cap) != 0:
                    bsd.deactivateStream(bs_rx)
                    continue
                out = run_burst(bsd, bs_rx, per_packet, cap, dtype, buf_samples=256 * 1024)
                bsd.deactivateStream(bs_rx)
                dbs.append(detect(iq_from_cs16(out["samples"]).astype(np.complex128),
                                  match, a.center_mhz * 1e6, rx_rate))
            md = float(np.median(dbs)) if dbs else 0.0
            profile.append((w, md))
            flag = "  <-- pilot here" if md >= a.det_db else ""
            print(f"    win @ {w:7d} tk ({w/tick_rate*1e6:6.1f} us): det {md:5.1f} dB{flag}")
    finally:
        for sdr in (bsd, ued):
            try:
                sdr.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:off")
            except Exception:  # noqa: BLE001
                pass
        for sdr, s in ((bsd, bs_rx), (bsd, bs_tx), (ued, ue_tx)):
            for fn in (sdr.deactivateStream, sdr.closeStream):
                try:
                    fn(s)
                except Exception:  # noqa: BLE001
                    pass
        for sdr in (bsd, ued):
            _teardown(sdr)

    if not profile:
        print("\nRESULT: no scan data")
        return 1
    w_pk, db_pk = max(profile, key=lambda t: t[1])
    # tx_advance to CENTER the pilot in the pilot-slot rx_gate window:
    advance = (w_pk - a.pilot_sym * SYM) % frame
    print(f"\nRESULT: UE pilot lands at BS-frame position {w_pk} tk "
          f"({w_pk/tick_rate*1e6:.1f} us), peak {db_pk:.1f} dB")
    if db_pk >= a.det_db:
        print(f"  -> FRAME-ALIGNED: the strobe-timed UE pilot lands at a FIXED BS-frame "
              f"position (stable frame-to-frame). To seat it in the rx_gate @ slot "
              f"{a.pilot_sym}, the UE advances its strobe by {advance} tk "
              f"({advance/tick_rate*1e6:.2f} us) -- a CONSTANT tx_advance (beacon-locked).")
        return 0
    print("  -> pilot not clearly located; widen scan / raise amp / check the reverse cable")
    return 1


if __name__ == "__main__":
    sys.exit(main())
