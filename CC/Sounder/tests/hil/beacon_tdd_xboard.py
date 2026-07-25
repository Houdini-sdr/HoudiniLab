#!/usr/bin/env python3
"""
beacon_tdd_xboard.py -- CROSS-BOARD beacon over the TDD framer.

The TDD framer + replay strobe run on the TX board (.21 DAC_A), firing one beacon
burst per frame; the RX board (.22 ADC_C) has an INDEPENDENT clock, so it can't do
timed windows aligned to .21's frame. Instead .22 free-runs a long capture over
several .21 frames and we detect the PERIODIC beacon bursts: matched-filter the
capture with the beacon waveform (which rejects the always-on CW LO leakage), then
fold the correlation at the frame period so the bursts integrate up.

Same TDD wiring as beacon_tdd.py (TDD_SCHED '6' beacon symbol + TDD_REPLAY_STROBE +
TDD_ARM on .21) -- only the RX side differs (free-run + periodic detection).

Run on the DGX (after: source houdini_test/bin/activate):
    python3 beacon_tdd_xboard.py --beacon chirp
    python3 beacon_tdd_xboard.py --beacon chirp --no-strobe    # leakage-only control
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
from houdini_setup import tx_lfm_chirp  # noqa: E402
from beacon_tdd import (build_beacon, _arm, _teardown,  # noqa: E402
                        GRID_TICKS, SYM, ARM_MARGIN)


def to_complex(lanes):
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex128)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float64)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return L[:, 0] + 1j * q


def matched_filter(x, h):
    """|cross-correlation| of x with template h via FFT (h short, x long)."""
    n = 1 << int(np.ceil(np.log2(len(x) + len(h))))
    C = np.fft.ifft(np.fft.fft(x, n) * np.conj(np.fft.fft(h, n)))
    return np.abs(C[:len(x) - len(h) + 1])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=1, help="DAC_A = TX ch1 (RFSoC4x2 reversed)")
    ap.add_argument("--rx-ch", type=int, default=1, help="cable lands on .22 RX ch1")
    ap.add_argument("--dac-nco", type=float, default=500.0, help="matched NCO (MHz, Zone 1)")
    ap.add_argument("--adc-nco", type=float, default=500.0, help="matched NCO (MHz, Zone 1)")
    ap.add_argument("--center-mhz", type=float, default=20.0)
    ap.add_argument("--beacon", choices=["chirp", "zc"], default="chirp")
    ap.add_argument("--bw-mhz", type=float, default=30.0)
    ap.add_argument("--n-sc", type=int, default=63)
    ap.add_argument("--n-load", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--rx-rate", type=float, default=122.88)
    ap.add_argument("--spf", type=int, default=8, help="symbols/frame (short -> frequent)")
    ap.add_argument("--secs", type=float, default=0.4)
    ap.add_argument("--cap-mb", type=float, default=48.0)
    ap.add_argument("--no-strobe", action="store_true", help="guards only (control)")
    ap.add_argument("--continuous", action="store_true",
                    help="A/B test: continuous replay (activateStream) instead of "
                         "the TDD strobe -- isolates strobe-datapath from RF coupling")
    a = ap.parse_args()

    print(f"TX {a.tx_ip} ch{a.tx_ch} (DAC_A) [TDD strobe] -> RX {a.rx_ip} ch{a.rx_ch} "
          f"(ADC_C) [free-run]  beacon={a.beacon}  no_strobe={a.no_strobe}")
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
    tsd.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.dac_nco * 1e6)
    rsd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, a.rx_rate * 1e6)
    rsd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.adc_nco * 1e6)
    rx_rate = float(rsd.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
    burst_rx = int(round(a.n_load * rx_rate / dac_rate))
    frame_rx = a.spf * SYM * rx_rate / tick_rate         # frame period in .22 samples
    print(f"  dac_rate {dac_rate/1e6:.2f}  rx_rate {rx_rate/1e6:.2f}  "
          f"burst {burst_rx} samp  frame {frame_rx:.0f} samp "
          f"({frame_rx/rx_rate*1e3:.1f} ms)  RF {a.dac_nco+a.center_mhz:.1f} MHz")

    tx = tsd.setupStream(SOAPY_SDR_TX, tx_ctx["native_fmt"], [a.tx_ch],
                         {"tx_mode": "replay"})
    try:
        i16, label = build_beacon(a.beacon, dac_rate, a.n_load, a.center_mhz * 1e6,
                                  a.amp, a.bw_mhz * 1e6, a.n_sc)
        print(f"  replay payload: {label}")
        cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)
        tsd.writeStream(tx, [cs16], a.n_load, 0, 0)
        if a.continuous:
            tsd.activateStream(tx)                     # continuous replay, no TDD
            print("  CONTINUOUS replay (activateStream) -- TDD strobe bypassed")
        else:
            pattern = ["0"] * a.spf
            if not a.no_strobe:
                pattern[0] = "6"
            tsd.writeSetting("TDD_SCHED", "".join(pattern))
            if not a.no_strobe:
                tsd.writeSetting("TDD_REPLAY_STROBE",
                                 f"ch{a.tx_ch}:len={a.n_load // 2},"
                                 f"loops=forever,offs={GRID_TICKS}")  # fill the beacon symbol
            r = _arm(tsd, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
            assert r.get("accepted") == 1, f"TDD arm rejected: {r}"

        buf, summ = hs.capture_rx(rsd, a.rx_ch, rnative, rdtype, duration_sec=a.secs,
                                  capture_bytes=int(a.cap_mb * 1024 * 1024))
        try:
            bank = tsd.readSetting("TX_BANK_STATUS")
            acked = [c for c in bank.split(";") if c.startswith(f"ch{a.tx_ch}:")]
            print(f"  TX bank ch{a.tx_ch}: "
                  + (acked[0].split(':', 1)[1][:80] if acked else "?"))
        except Exception:  # noqa: BLE001
            pass
    finally:
        try:
            tsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:off")
        except Exception:  # noqa: BLE001
            pass
        try:
            tsd.deactivateStream(tx)
            tsd.closeStream(tx)
        except Exception:  # noqa: BLE001
            pass
        _teardown(tsd)

    iq = to_complex(hs.cs16_lanes(buf))
    print(f"\ncaptured {len(iq)} samples  overflows={summ.get('overflows')}")

    # DDC to the beacon centre, matched-filter with the beacon waveform at rx_rate
    # (rejects the CW LO leakage), then fold at the frame period -> periodic burst.
    n = np.arange(len(iq))
    # matched NCO -> the beacon returns at -center (R2C conjugation); DDC to baseband
    ddc = iq * np.exp(+2j * np.pi * a.center_mhz * 1e6 * n / rx_rate)
    if a.beacon == "chirp":
        mi16, _ = tx_lfm_chirp(a.bw_mhz * 1e6, rx_rate, burst_rx, amp_frac=1.0,
                               center_hz=0.0)
    else:
        mi16, _ = build_beacon("zc", rx_rate, burst_rx, 0.0, 1.0, 0.0, a.n_sc)
    match = (mi16[0::2].astype(np.float64) + 1j * mi16[1::2]).astype(np.complex128)
    match /= np.sqrt(np.sum(np.abs(match) ** 2)) + 1e-30

    # try both conjugation senses (R2C may deliver the beacon conjugated)
    c0 = matched_filter(ddc, match)
    c1 = matched_filter(ddc, np.conj(match))
    corr = c0 if c0.max() >= c1.max() else c1
    pk = int(np.argmax(corr))
    single = 20 * np.log10(corr[pk] / (np.median(corr) + 1e-30))

    P = burst_rx if a.continuous else int(round(frame_rx))   # loop vs frame period
    nfold = len(corr) // P
    if nfold >= 3:
        prof = (corr[:nfold * P].reshape(nfold, P) ** 2).mean(axis=0)
        ph = int(np.argmax(prof))
        mask = np.ones(P, dtype=bool)
        g = min(burst_rx, max(4, P // 8))
        mask[max(0, ph - g):ph + g] = False
        mu, sd = float(prof[mask].mean()), float(prof[mask].std())
        z = (prof[ph] - mu) / (sd + 1e-30)
        fold_db = 10 * np.log10(prof[ph] / (mu + 1e-30))
        print(f"folded over {nfold} frames (period {P}): peak phase {ph}, "
              f"{fold_db:.1f} dB over floor, z = {z:.1f} sigma")
    else:
        z, ph = 0.0, 0
        print(f"only {nfold} frames captured — increase --secs/--cap-mb or --spf")

    print(f"single-shot matched-filter peak: {single:.1f} dB over median")
    detected = (not a.no_strobe) and z >= 8.0
    print("\nRESULT:",
          f"BEACON DETECTED over TDD (folded z={z:.1f} over {nfold} frames, phase "
          f"{ph}) -- .21 DAC_A -> .22 ADC_C beacon+framer OK" if detected else
          f"no periodic beacon (folded z={z:.1f})"
          + (" [control: expected]" if a.no_strobe else
             " -- check the .21->.22 cable couples the modulated burst"))
    return 0 if detected else 1


if __name__ == "__main__":
    sys.exit(main())
