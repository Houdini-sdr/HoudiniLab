#!/usr/bin/env python3
"""
tone_link.py -- cross-board RF link check for the Sounder HIL bring-up.

TX-replay a tone from board A (DAC_A / .21 ch0) and RX-capture on board B
(ADC_C / .22 ch2), then report the received spectrum peak. Confirms the
.21 DAC_A -> .22 ADC_C cable path end-to-end, independent of the correlator.
The two boards run independent clocks, so the received tone sits near the TX
baseband offset shifted by the inter-board CFO -- we just look for a strong peak.

Reuses SoapyHoudiniSDR/host/examples/houdini_setup. Run on the DGX:
    source houdini_test/bin/activate
    python3 tone_link.py --tx-ip 168.6.244.21 --rx-ip 168.6.244.22 \
        --tx-ch 0 --rx-ch 2 --tone-mhz 20
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


def to_complex(lanes):
    """cs16 lanes [N,4] -> complex baseband. Prefer houdini_setup's layout-aware
    reconstruction; fall back to lane0 + j*lane1."""
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex64)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float32)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return (L[:, 0] + 1j * q).astype(np.complex64)


def fft_peak(iq, fs_hz):
    """(peak_freq_hz, snr_db) of the strongest FFT bin vs the mean of the rest."""
    n = 1 << int(np.floor(np.log2(len(iq))))
    x = iq[:n] * np.hanning(n)
    p = np.abs(np.fft.fftshift(np.fft.fft(x))) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / fs_hz))
    k = int(np.argmax(p))
    noise = (p.sum() - p[k]) / (len(p) - 1)
    snr = 10.0 * np.log10(p[k] / noise) if noise > 0 else float("inf")
    return freqs[k], snr


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=0, help="DAC_A = TX ch0")
    ap.add_argument("--rx-ch", type=int, default=2, help="ADC_C = RX ch2")
    ap.add_argument("--tone-mhz", type=float, default=20.0)
    ap.add_argument("--nco-mhz", type=float, default=None,
                    help="fine NCO on TX+RX (MHz); default = zone1 middle")
    ap.add_argument("--rate-mhz", type=float, default=122.88, help="RX sample rate")
    ap.add_argument("--n-addr", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.25)
    ap.add_argument("--secs", type=float, default=1.0)
    ap.add_argument("--cap-mb", type=float, default=16.0)
    ap.add_argument("--snr-db", type=float, default=15.0, help="detect threshold")
    args = ap.parse_args()

    print(f"TX {args.tx_ip} ch{args.tx_ch} (DAC_A) -> RX {args.rx_ip} "
          f"ch{args.rx_ch} (ADC_C)")
    tx_ctx = hs.open_device(node=args.tx_ip, ch=args.tx_ch, verbose=False)
    rx_ctx = hs.open_device(node=args.rx_ip, ch=args.rx_ch, verbose=False)
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]

    nco = (args.nco_mhz * 1e6 if args.nco_mhz is not None
           else hs.zone1_nco_hz(rxd, rx_ch=args.rx_ch, tx_ch=args.rx_ch))
    dac_rate = float(dict(txd.getChannelInfo(SOAPY_SDR_TX, args.tx_ch)).get(
        "rfdc_effective_rate_hz", 983.04e6))
    print(f"fine NCO {nco/1e6:.3f} MHz (both)  dac_rate {dac_rate/1e6:.2f} MSPS")

    tx = None
    try:
        iq_a, f_act = hs.tx_iq_tone(args.tone_mhz * 1e6, dac_rate, args.n_addr,
                                    amp_frac=args.amp)
        print(f"TX tone baseband {f_act/1e6:.3f} MHz amp={args.amp} "
              f"{args.n_addr} addrs")
        tx = txd.setupStream(SOAPY_SDR_TX, "CS16", [args.tx_ch],
                             {"tx_mode": "replay"})
        cs16 = np.ascontiguousarray(iq_a, dtype=np.int16).view(np.int32)
        txd.writeStream(tx, [cs16], cs16.size, 0, 0)
        txd.setFrequency(SOAPY_SDR_TX, args.tx_ch, nco)
        txd.activateStream(tx)

        try:
            rxd.setSampleRate(SOAPY_SDR_RX, args.rx_ch, args.rate_mhz * 1e6)
        except Exception as e:  # noqa: BLE001
            print(f"  setSampleRate warn: {e}")
        rxd.setFrequency(SOAPY_SDR_RX, args.rx_ch, nco)
        fs = float(rxd.getSampleRate(SOAPY_SDR_RX, args.rx_ch))
        print(f"RX sample rate {fs/1e6:.3f} MSPS -- capturing {args.secs}s ...")
        buf, summ = hs.capture_rx(rxd, args.rx_ch, rx_ctx["native_fmt"],
                                  rx_ctx["dtype"], duration_sec=args.secs,
                                  capture_bytes=int(args.cap_mb * 1024 * 1024))
    finally:
        if tx is not None:
            try:
                txd.deactivateStream(tx)
                txd.closeStream(tx)
            except Exception:  # noqa: BLE001
                pass

    lanes = hs.cs16_lanes(buf)
    print(f"\ncaptured {len(lanes)} words  overflows={summ.get('overflows')}")
    for k in range(lanes.shape[1]):
        col = lanes[:, k].astype(np.float64)
        clip = "  <-- CLIPPING" if max(abs(col.min()), abs(col.max())) > 32000 else ""
        print(f"  lane{k}: std={col.std():8.1f} min={col.min():7.0f} "
              f"max={col.max():7.0f}{clip}")

    iq = to_complex(lanes)
    fpk, snr = fft_peak(iq, fs)
    print(f"\nRX peak {fpk/1e6:+.3f} MHz  (TX offset {f_act/1e6:.3f} MHz + "
          f"inter-board CFO)  SNR ~ {snr:.1f} dB")
    ok = snr >= args.snr_db
    print("RESULT:",
          "TONE DETECTED -- .21 DAC_A -> .22 ADC_C link OK" if ok
          else "no clear tone -- check channel indices / cable / levels")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
