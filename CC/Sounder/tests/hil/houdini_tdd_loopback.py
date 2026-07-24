#!/usr/bin/env python3
"""
houdini_tdd_loopback.py -- v2: beacon-synced UE + tx_advance sweep -> closed,
frame-aligned TDD loop over the native Houdini framer.

Both boards run their OWN TDD framer at the SAME period (spf*SYM ticks), so once
armed their frames are phase-locked (only slow CFO drift). The BS emits the beacon
at slot 0 and gates RX at the pilot slot; the UE fires its pilot once per frame via
the replay strobe at a swept frame-offset. Because the frames are phase-locked, ONE
offset lands the pilot in the BS rx_gate window and stays valid frame-to-frame --
that offset is the tx_advance the beacon-sync makes constant.

The sweep walks the UE pilot's frame-offset and measures the BS gated-window
detection; the peak = the calibrated tx_advance. The UE also free-run-detects the
BS beacon and reports the frame phase the beacon PREDICTS for the alignment (the
beacon carries the BS frame timing -- no shared hardware clock).

Fine-grained timed TX on Houdini is ONLY the framer replay strobe (offs on the
3.125 us grid); activateStream(TX,HAS_TIME) is whole-ms, too coarse for a slot.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 houdini_tdd_loopback.py
    python3 houdini_tdd_loopback.py --spf 4 --step-us 40
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
    """DDC + matched filter (both conj senses): (peak_dB_over_median, peak_index)."""
    n = np.arange(len(iq))
    ddc = iq * np.exp(+2j * np.pi * center_hz * n / rx_rate)
    c0 = matched_filter(ddc, match)
    c1 = matched_filter(ddc, np.conj(match))
    corr = c0 if c0.max() >= c1.max() else c1
    pk = int(np.argmax(corr))
    return 20 * np.log10(corr[pk] / (np.median(corr) + 1e-30)), pk


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
    ap.add_argument("--pilot-sym", type=int, default=2, help="BS rx_gate slot")
    ap.add_argument("--step-us", type=float, default=40.0, help="sweep step (us)")
    ap.add_argument("--win-per-pt", type=int, default=3, help="BS windows per sweep pt")
    ap.add_argument("--det-db", type=float, default=12.0)
    a = ap.parse_args()

    print(f"BS {a.bs_ip} [beacon@0 + rx_gate@{a.pilot_sym}]  UE {a.ue_ip} [pilot strobe, "
          f"swept]  spf={a.spf}  frame={a.spf*SYM} ticks ({a.spf*0.5:.1f} ms)")
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
    bsd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, 122.88e6)
    bsd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
    ued.setSampleRate(SOAPY_SDR_RX, a.rx_ch, 122.88e6)     # UE RX to sync on beacon
    ued.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
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
    ue_rx = ued.setupStream(SOAPY_SDR_RX, ue["native_fmt"], [a.rx_ch], rx_stream_args(a.rx_ch))
    per_packet = rx_framing(bsd, verbose=False)["frame_words"] * (8 // bps)
    cap = (int(SYM * spt) // 2 // per_packet) * per_packet
    step = max(GRID_TICKS, int(round(a.step_us * 1e-6 * tick_rate)) // GRID_TICKS * GRID_TICKS)

    def arm_bs():
        bsd.writeStream(bs_tx, [wave32], a.n_load, 0, 0)
        pat = ["0"] * a.spf
        pat[0] = "6"                                  # beacon strobe @ slot0
        pat[a.pilot_sym] = "2"                        # rx_gate @ pilot slot
        bsd.writeSetting("TDD_SCHED", "".join(pat))
        bsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:len={a.n_load//2},offs={GRID_TICKS}")
        r = _arm(bsd, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        assert r.get("accepted") == 1, f"BS arm rejected: {r}"
        return r["epoch"]

    def arm_ue(tgt):
        """Arm the UE framer so its pilot strobe fires at frame-offset `tgt` ticks."""
        tgt %= frame
        sym = tgt // SYM
        offs = (tgt % SYM) // GRID_TICKS * GRID_TICKS
        offs = max(GRID_TICKS, offs)
        _cmd(ued, "abort")
        ued.writeStream(ue_tx, [wave32], a.n_load, 0, 0)
        pat = ["0"] * a.spf
        pat[sym] = "6"                                # UE pilot strobe
        ued.writeSetting("TDD_SCHED", "".join(pat))
        ued.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:len={a.n_load//2},offs={offs}")
        r = _arm(ued, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        assert r.get("accepted") == 1, f"UE arm rejected: {r}"
        return r["epoch"], sym * SYM + offs

    def bs_window(epoch_bs):
        ot = _next_window_tick(bsd, tick_rate, epoch_bs, frame, a.pilot_sym)
        if bsd.activateStream(bs_rx, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                              _ns_of_tick(ot), cap) != 0:
            bsd.deactivateStream(bs_rx)
            return None
        out = run_burst(bsd, bs_rx, per_packet, cap, dtype, buf_samples=256 * 1024)
        bsd.deactivateStream(bs_rx)
        return iq_from_cs16(out["samples"]).astype(np.complex128)

    profile = []
    peak_frac = None
    try:
        epoch_bs = arm_bs()
        print(f"  BS armed: epoch={epoch_bs}  RX window {cap} samp @ pilot slot\n")

        # --- beacon-phase cross-check: UE free-runs, detects the BS beacon ---
        t0 = _hw_tick(ued, tick_rate)
        buf, _summ = hs.capture_rx(ued, a.rx_ch, ue["native_fmt"], dtype,
                                   duration_sec=0.05, capture_bytes=8 << 20)
        iqb = np.asarray(hs.iq_from_lanes(hs.cs16_lanes(buf), "interleaved"),
                         dtype=np.complex128)
        db_b, pk_b = detect(iqb, match, a.center_mhz * 1e6, rx_rate)
        beacon_tick = t0 + int(round(pk_b / rx_rate * tick_rate))
        beacon_phase = (beacon_tick - epoch_bs) % frame       # note: BS epoch, UE tick (offset absorbed by sweep)
        pred = (beacon_phase + a.pilot_sym * SYM) % frame
        print(f"  UE beacon detect: {db_b:.1f} dB; beacon frame-phase ~{beacon_phase} ticks "
              f"({beacon_phase/tick_rate*1e6:.1f} us); predicted pilot offset ~{pred} ticks\n")

        # --- tx_advance sweep: walk the UE pilot's frame-offset over one frame ---
        print(f"  sweep: step {step} ticks ({step/tick_rate*1e6:.1f} us), "
              f"{frame//step} points")
        for tgt in range(0, frame, step):
            epoch_ue, actual = arm_ue(tgt)
            dbs = []
            for _ in range(a.win_per_pt):
                iq = bs_window(epoch_bs)
                if iq is not None and len(iq) >= burst_rx * 2:
                    d, _pk = detect(iq, match, a.center_mhz * 1e6, rx_rate)
                    dbs.append(d)
            md = float(np.median(dbs)) if dbs else 0.0
            profile.append((actual, md))
            flag = " <-- pilot in BS gate" if md >= a.det_db else ""
            print(f"    off {actual:7d} tk ({actual/tick_rate*1e6:6.1f} us): "
                  f"BS det {md:5.1f} dB{flag}")
    finally:
        for sdr in (bsd, ued):
            try:
                sdr.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:off")
            except Exception:  # noqa: BLE001
                pass
        for sdr, s in ((bsd, bs_rx), (bsd, bs_tx), (ued, ue_rx), (ued, ue_tx)):
            for fn in (sdr.deactivateStream, sdr.closeStream):
                try:
                    fn(s)
                except Exception:  # noqa: BLE001
                    pass
        for sdr in (bsd, ued):
            _teardown(sdr)

    if not profile:
        print("\nRESULT: no sweep data")
        return 1
    off_pk, db_pk = max(profile, key=lambda t: t[1])
    advance = (a.pilot_sym * SYM - off_pk) % frame
    print(f"\nRESULT: peak BS detection {db_pk:.1f} dB at UE pilot offset {off_pk} ticks "
          f"({off_pk/tick_rate*1e6:.1f} us)")
    if db_pk >= a.det_db:
        print(f"  -> FRAME-ALIGNED LOOP CLOSED. Calibrated tx_advance = {advance} ticks "
              f"({advance/tick_rate*1e6:.2f} us) = pilot fires that much before the BS "
              f"pilot-slot start; constant every frame (beacon-locked).")
        return 0
    print("  -> pilot not landed in the BS gate; widen the sweep or check coupling")
    return 1


if __name__ == "__main__":
    sys.exit(main())
