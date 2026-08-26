#!/usr/bin/env python3
"""
houdini_tdd_bs_rx.py -- Prototype the Houdini-native TDD BS-receive path.

Instead of the software-framer "read every slot + dummy-discard non-R" path (where
the host's slot counter is decoupled from the RF timeline), the BS runs its OWN TDD
frame on the RUNNING v1.21 bitstream, which wires rx_enable(tdd_rxen_adc):

  BS (.21):  slot 0 = beacon strobe (TDD_REPLAY_STROBE, so a UE can sync on it)
             pilot slot = rx_gate lane on   -> ADC capture enabled ONLY here
             a TIMED RX window is armed at the pilot slot on the BS's own TDD epoch,
             so the host receives exactly that slot -- gated by the FPGA.

v1 proves the BS-RECEIVE GATING MECHANISM with the UE (.22) transmitting the pilot
CONTINUOUSLY (a stand-in for a beacon-synced, timing-advanced UE -- that alignment is
the v2 step: the UE detects the BS beacon, timestamps it on its own FPGA clock, and
applies a trial-and-error tx_advance so the pilot lands in the BS rx_gate window; no
shared hardware clock needed, the beacon carries the BS frame timing).

CONTROL: with the UE transmitting continuously, capture BOTH the pilot slot (rx_gate)
and a GUARD slot (no rx_gate). If the FPGA rx_gate truly gates the ADC, the pilot is
detected ONLY in the gated slot -- proving native RX gating, not just a timed read.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 houdini_tdd_bs_rx.py
    python3 houdini_tdd_bs_rx.py --ue-off      # sanity: no UE -> nothing in either slot
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
from beacon_tdd import (build_beacon, _arm, _teardown, _ns_of_tick,  # noqa: E402
                        _hw_tick, _next_window_tick, GRID_TICKS, SYM, ARM_MARGIN)


def matched_filter(x, h):
    """|cross-correlation| of x with template h via FFT (h short, x long)."""
    n = 1 << int(np.ceil(np.log2(len(x) + len(h))))
    C = np.fft.ifft(np.fft.fft(x, n) * np.conj(np.fft.fft(h, n)))
    return np.abs(C[:len(x) - len(h) + 1])


def detect_db(iq, match, center_hz, rx_rate):
    """DDC to the pilot band, matched-filter (both conj senses), peak dB over median."""
    n = np.arange(len(iq))
    ddc = iq * np.exp(+2j * np.pi * center_hz * n / rx_rate)
    c0 = matched_filter(ddc, match)
    c1 = matched_filter(ddc, np.conj(match))
    corr = c0 if c0.max() >= c1.max() else c1
    pk = int(np.argmax(corr))
    return 20 * np.log10(corr[pk] / (np.median(corr) + 1e-30)), pk


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bs-ip", default="168.6.244.21", help="BS: beacon TX + gated RX")
    ap.add_argument("--ue-ip", default="168.6.244.22", help="UE: pilot TX")
    ap.add_argument("--tx-ch", type=int, default=1, help="DAC_A = TX ch1 (reversed map)")
    ap.add_argument("--rx-ch", type=int, default=1, help="ADC_C = RX ch1")
    ap.add_argument("--nco", type=float, default=500.0, help="matched NCO (MHz, Zone 1)")
    ap.add_argument("--center-mhz", type=float, default=20.0, help="pilot/beacon baseband")
    ap.add_argument("--n-sc", type=int, default=63, help="ZC subcarriers (sharp peak)")
    ap.add_argument("--n-load", type=int, default=2048, help="replay RAM samples")
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--spf", type=int, default=8, help="symbols per frame")
    ap.add_argument("--beacon-sym", type=int, default=0)
    ap.add_argument("--pilot-sym", type=int, default=4, help="BS rx_gate slot")
    ap.add_argument("--guard-sym", type=int, default=6, help="control: no rx_gate")
    ap.add_argument("--frames", type=int, default=6, help="RX windows per slot")
    ap.add_argument("--det-db", type=float, default=12.0, help="detection threshold")
    ap.add_argument("--ue-off", action="store_true", help="sanity: no UE pilot TX")
    a = ap.parse_args()

    print(f"BS {a.bs_ip} ch{a.tx_ch} [beacon@sym{a.beacon_sym} + rx_gate@sym{a.pilot_sym}]"
          f"  <-  UE {a.ue_ip} ch{a.tx_ch} [pilot{' OFF' if a.ue_off else ' continuous'}]")
    bs = hs.open_device(node=a.bs_ip, ch=a.tx_ch, verbose=False)
    ue = hs.open_device(node=a.ue_ip, ch=a.tx_ch, verbose=False)
    bsd, ued = bs["sdr"], ue["sdr"]
    native, dtype = bs["native_fmt"], bs["dtype"]
    bps = bs["bytes_per_samp"]
    tick_rate = float(dict(bsd.getHardwareInfo()).get("tick_rate_hz", 122.88e6))

    _teardown(bsd)
    for sdr, ch in ((bsd, a.tx_ch), (ued, a.tx_ch)):
        ladder = list(sdr.listSampleRates(SOAPY_SDR_TX, ch))
        if ladder:
            sdr.setSampleRate(SOAPY_SDR_TX, ch, max(ladder))
        sdr.setFrequency(SOAPY_SDR_TX, ch, a.nco * 1e6)
    bsd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, 122.88e6)
    bsd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
    dac_rate = float(bsd.getSampleRate(SOAPY_SDR_TX, a.tx_ch))
    rx_rate = float(bsd.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
    spt = rx_rate / tick_rate
    burst_rx = int(round(a.n_load * rx_rate / dac_rate))

    # UE pilot + BS beacon: same ZC waveform (distinct roles, one template to match)
    pilot16, label = build_beacon("zc", dac_rate, a.n_load, a.center_mhz * 1e6,
                                  a.amp, 0.0, a.n_sc)
    beacon16, _ = build_beacon("zc", dac_rate, a.n_load, a.center_mhz * 1e6,
                               a.amp, 0.0, a.n_sc)
    print(f"  dac {dac_rate/1e6:.2f}  rx {rx_rate/1e6:.2f}  burst {burst_rx} samp  "
          f"pilot={label}  RF {a.nco + a.center_mhz:.1f} MHz")

    # matched-filter template at rx_rate, DDC'd to baseband
    m16, _ = build_beacon("zc", rx_rate, burst_rx, 0.0, 1.0, 0.0, a.n_sc)
    match = (m16[0::2].astype(np.float64) + 1j * m16[1::2]).astype(np.complex128)
    match /= np.sqrt(np.sum(np.abs(match) ** 2)) + 1e-30

    ue_tx = ued.setupStream(SOAPY_SDR_TX, ue["native_fmt"], [a.tx_ch], {"tx_mode": "replay"})
    bs_tx = bsd.setupStream(SOAPY_SDR_TX, native, [a.tx_ch], {"tx_mode": "replay"})
    bs_rx = bsd.setupStream(SOAPY_SDR_RX, native, [a.rx_ch], rx_stream_args(a.rx_ch))
    results = {}
    try:
        # --- UE: continuous pilot replay (v1 stand-in for a synced+advanced UE) ---
        if not a.ue_off:
            ued.writeStream(ue_tx, [np.ascontiguousarray(pilot16, np.int16).view(np.int32)],
                            a.n_load, 0, 0)
            ued.activateStream(ue_tx)
            print("  UE: continuous pilot replay up")

        # --- BS: TDD frame -- beacon strobe @ slot0, rx_gate @ pilot slot ---
        bsd.writeStream(bs_tx, [np.ascontiguousarray(beacon16, np.int16).view(np.int32)],
                        a.n_load, 0, 0)
        pattern = ["0"] * a.spf
        pattern[a.beacon_sym] = "6"          # replay_strobe + rx_gate (beacon)
        pattern[a.pilot_sym] = "2"           # rx_gate only (BS receives the pilot here)
        # guard-sym stays '0' (no rx_gate) -- the control slot
        bsd.writeSetting("TDD_SCHED", "".join(pattern))
        bsd.writeSetting("TDD_REPLAY_STROBE",
                         f"ch{a.tx_ch}:len={a.n_load // 2},offs={GRID_TICKS}")
        r = _arm(bsd, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        assert r.get("accepted") == 1, f"TDD arm rejected: {r}"
        epoch, frame = r["epoch"], a.spf * SYM
        per_packet = rx_framing(bsd, verbose=False)["frame_words"] * (8 // bps)
        cap = (int(SYM * spt) // 2 // per_packet) * per_packet
        print(f"  BS TDD armed: epoch={epoch} frame={frame} ticks  RX window {cap} samp\n")

        def pump(sym_off):
            ot = _next_window_tick(bsd, tick_rate, epoch, frame, sym_off)
            rc = bsd.activateStream(bs_rx, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                                    _ns_of_tick(ot), cap)
            if rc != 0:
                bsd.deactivateStream(bs_rx)
                return None
            out = run_burst(bsd, bs_rx, per_packet, cap, dtype, buf_samples=256 * 1024)
            bsd.deactivateStream(bs_rx)
            return iq_from_cs16(out["samples"]).astype(np.complex128), out["total"]

        for tag, sym in (("pilot", a.pilot_sym), ("guard", a.guard_sym)):
            dbs, totals = [], []
            for _ in range(a.frames):
                got = pump(sym)
                if got is None:
                    continue
                iq, total = got
                totals.append(total)
                if len(iq) >= burst_rx * 2:
                    d, _pk = detect_db(iq, match, a.center_mhz * 1e6, rx_rate)
                    dbs.append(d)
            results[tag] = (np.array(dbs), np.array(totals), sym)
            md = float(np.median(dbs)) if len(dbs) else float("nan")
            mt = int(np.median(totals)) if len(totals) else 0
            gate = "rx_gate ON" if tag == "pilot" else "no rx_gate (control)"
            print(f"  {tag:5s} sym{sym} [{gate:20s}]: "
                  f"windows={len(dbs)}  median_samps={mt}  peak_dB(med)={md:.1f}")
    finally:
        try:
            bsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:off")
        except Exception:  # noqa: BLE001
            pass
        for sdr, s in ((bsd, bs_rx), (bsd, bs_tx), (ued, ue_tx)):
            for fn in (sdr.deactivateStream, sdr.closeStream):
                try:
                    fn(s)
                except Exception:  # noqa: BLE001
                    pass
        _teardown(bsd)

    # --- verdict ---
    pilot_db = results.get("pilot", (np.array([]),))[0]
    guard_db = results.get("guard", (np.array([]),))[0]
    pmed = float(np.median(pilot_db)) if len(pilot_db) else float("nan")
    gmed = float(np.median(guard_db)) if len(guard_db) else float("nan")
    print()
    if a.ue_off:
        ok = not (pmed >= a.det_db)
        print(f"RESULT [ue-off sanity]: pilot-slot peak {pmed:.1f} dB "
              f"({'quiet, good' if ok else 'UNEXPECTED signal'})")
        return 0 if ok else 1
    detected = pmed >= a.det_db
    gated = detected and (np.isnan(gmed) or (pmed - gmed) >= 6.0)
    print(f"RESULT: pilot-slot {pmed:.1f} dB vs guard-slot {gmed:.1f} dB -> "
          + ("BS-RECEIVE OVER NATIVE TDD OK"
             + (" + RX-GATED (pilot >> guard)" if gated else
                " (detected, but guard slot not clearly suppressed -- check gating)")
             if detected else "pilot NOT detected in the gated slot"))
    return 0 if detected else 1


if __name__ == "__main__":
    sys.exit(main())
