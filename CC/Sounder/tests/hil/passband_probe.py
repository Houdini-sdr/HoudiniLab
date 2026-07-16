#!/usr/bin/env python3
"""
passband_probe.py -- map the .21 DAC_A -> .22 ADC_C front-end response.

A CW tone at RF ~840 MHz (dual-NCO: DAC 820 + ADC 388.8) comes back clean, but
adjacent carriers don't -- suspicious of a narrow/comb coupling (cable standing
waves).  A single CW can't tell a flat passband from a comb peak.  This TXes a
COMB of equal-amplitude baseband tones across the RX band at the working carrier
(houdini_setup.tx_replay_comb, built for RX frequency-response characterization)
and reads each tone's received level -- so we can see whether the whole ~122 MHz
beacon band gets through (flat) or only isolated comb peaks do (a wideband beacon
would be shredded by the nulls).

Run on the DGX (after: source houdini_test/bin/activate):
    python3 passband_probe.py                     # comb @ DAC 820 / ADC 388.8
    python3 passband_probe.py --dac-nco 860        # probe the other peak (RF 880)
"""
import argparse
import os
import sys

import numpy as np

_EX = os.environ.get(
    "HOUDINI_EXAMPLES",
    os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if _EX not in sys.path:
    sys.path.insert(0, _EX)

import SoapySDR  # noqa: E402
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX  # noqa: E402
import houdini_setup as hs  # noqa: E402
from sigqual import tone_spectrum, peak_in  # noqa: E402


def to_complex(lanes):
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex64)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float32)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return (L[:, 0] + 1j * q).astype(np.complex64)


def fold(f, fs):
    return f - round(f / fs) * fs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=0)
    ap.add_argument("--rx-ch", type=int, default=2)
    ap.add_argument("--dac-nco", type=float, default=820.0, help="DAC carrier (MHz)")
    ap.add_argument("--adc-nco", type=float, default=None,
                    help="ADC NCO (MHz); default = |fold(dac_nco, Fs_adc)| tracker")
    ap.add_argument("--rate-mhz", type=float, default=122.88)
    ap.add_argument("--n-addr", type=int, default=4096)
    ap.add_argument("--amp", type=float, default=0.9)
    ap.add_argument("--secs", type=float, default=0.6)
    ap.add_argument("--cap-mb", type=float, default=16.0)
    a = ap.parse_args()

    # Comb of baseband tones spanning the RX band, skipping the DC guard and the
    # internal Fs/4 (30.72) and Fs/2 (61.44) spur bins.
    combf = [f for f in np.arange(4.0, 60.0, 4.0)
             if abs(f - 30.72) > 2.0 and f < 60.0]
    print(f"TX {a.tx_ip} ch{a.tx_ch} (DAC_A) -> RX {a.rx_ip} ch{a.rx_ch} (ADC_C)")
    tx_ctx = hs.open_device(node=a.tx_ip, ch=a.tx_ch, verbose=False)
    rx_ctx = (tx_ctx if a.rx_ip == a.tx_ip           # single-board loopback: 1 handle
              else hs.open_device(node=a.rx_ip, ch=a.rx_ch, verbose=False))
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]
    native, dtype = rx_ctx["native_fmt"], rx_ctx["dtype"]
    dac_rate = float(dict(txd.getChannelInfo(SOAPY_SDR_TX, a.tx_ch)).get(
        "rfdc_effective_rate_hz", 983.04e6))
    fs_adc = float(dict(rxd.getChannelInfo(SOAPY_SDR_RX, a.rx_ch)).get(
        "rfdc_sample_rate_hz", 1228.8e6))
    dac_nco = a.dac_nco * 1e6
    adc_nco = a.adc_nco * 1e6 if a.adc_nco is not None else abs(fold(dac_nco, fs_adc))
    print(f"carrier: DAC NCO {dac_nco/1e6:.1f} MHz -> RF ~{dac_nco/1e6:.0f} MHz, "
          f"ADC NCO {adc_nco/1e6:.1f} MHz   comb tones (MHz): "
          f"{[round(f,1) for f in combf]}")

    tx = None
    try:
        iq_a, actual = hs.tx_replay_comb([f * 1e6 for f in combf], dac_rate,
                                         a.n_addr, amp_frac=a.amp)
        tx = txd.setupStream(SOAPY_SDR_TX, "CS16", [a.tx_ch], {"tx_mode": "replay"})
        txd.setFrequency(SOAPY_SDR_TX, a.tx_ch, dac_nco)
        cs16 = np.ascontiguousarray(iq_a, dtype=np.int16).view(np.int32)
        txd.writeStream(tx, [cs16], cs16.size, 0, 0)
        txd.activateStream(tx)
        rxd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, a.rate_mhz * 1e6)
        rxd.setFrequency(SOAPY_SDR_RX, a.rx_ch, adc_nco)
        fs = float(rxd.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
        buf, _ = hs.capture_rx(rxd, a.rx_ch, native, dtype, duration_sec=a.secs,
                               capture_bytes=int(a.cap_mb * 1024 * 1024))
    finally:
        if tx is not None:
            try:
                txd.deactivateStream(tx)
                txd.closeStream(tx)
            except Exception:  # noqa: BLE001
                pass

    lanes = hs.cs16_lanes(buf)
    freqs, mag = tone_spectrum(to_complex(lanes), fs)
    floor = float(np.median(mag))
    # Each baseband comb tone lands at ~+f (or -f by sign convention); take the
    # stronger sideband as its received level.
    rows = []
    for f in actual:
        pp, _ = peak_in(freqs, mag, +f, span_hz=0.4e6)
        pn, _ = peak_in(freqs, mag, -f, span_hz=0.4e6)
        p = max(pp, pn)
        rows.append((f, 20.0 * np.log10(max(p, 1e-12) / max(floor, 1e-12))))
    peak_db = max(r[1] for r in rows)
    print(f"\n{'tone_MHz':>9} {'RF_MHz':>8} {'level_dB':>9} {'vs_peak':>8}  bar")
    for f, db in rows:
        rel = db - peak_db
        bar = "#" * max(0, int((db) / 2))
        print(f"{f/1e6:9.2f} {(dac_nco+f)/1e6:8.1f} {db:9.1f} {rel:8.1f}  {bar}")
    lo = min(r[1] for r in rows)
    spread = peak_db - lo
    print(f"\nband response spread = {spread:.1f} dB over "
          f"{combf[0]:.0f}-{combf[-1]:.0f} MHz baseband "
          f"(RF {(dac_nco/1e6+combf[0]):.0f}-{(dac_nco/1e6+combf[-1]):.0f} MHz)")
    if spread <= 8.0:
        print("  => FLAT enough: a wideband beacon should pass. Proceed to beacon.")
    else:
        print("  => COMBY/selective: nulls will distort a wideband beacon. Consider a "
              "narrower beacon bandwidth, a different/shorter cable, or picking the "
              "flattest sub-band.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
