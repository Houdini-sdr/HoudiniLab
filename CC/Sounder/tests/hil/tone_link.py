#!/usr/bin/env python3
"""
tone_link.py -- cross-board RF link check for the Sounder HIL bring-up.

TX-replay a tone on board A (DAC_A / .21 ch0), RX-capture on board B
(ADC_C / .22 ch2), and report the received spectrum's top peaks (DC excluded).
Confirms the .21 DAC_A -> .22 ADC_C cable path end-to-end, independent of the
correlator. The boards run independent clocks, so the received tone sits near
the TX baseband offset shifted by the inter-board CFO.

Mixing follows the proven single-board loopback example (rx_loopback_report):
coarse Fs/4 by default (--mix fs4); --mix nco uses a matched fine NCO instead.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 tone_link.py --tone-mhz 20            # coarse fs4 (default)
    python3 tone_link.py --mix nco --nco-mhz 307  # fine NCO variant
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
    """cs16 lanes -> complex baseband; prefer houdini_setup's layout-aware form."""
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex64)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float32)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return (L[:, 0] + 1j * q).astype(np.complex64)


def fft_peaks(iq, fs_hz, dc_guard_hz=1.0e6, n_top=6):
    """Top-N spectral peaks EXCLUDING a DC guard band -> [(freq_hz, snr_db)]."""
    n = 1 << int(np.floor(np.log2(len(iq))))
    x = iq[:n] * np.hanning(n)
    p = np.abs(np.fft.fftshift(np.fft.fft(x))) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / fs_hz))
    mask = (np.abs(freqs) > dc_guard_hz).astype(np.float64)
    noise = np.median(p[mask > 0]) or 1.0
    order = np.argsort(p * mask)[::-1][:n_top]
    return [(freqs[k], 10.0 * np.log10(p[k] / noise)) for k in order]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=0, help="DAC_A = TX ch0")
    ap.add_argument("--rx-ch", type=int, default=2, help="ADC_C = RX ch2")
    ap.add_argument("--tone-mhz", type=float, default=20.0)
    ap.add_argument("--mix", choices=["fs4", "nco"], default="fs4")
    ap.add_argument("--nco-mhz", type=float, default=None,
                    help="fine NCO (MHz) for --mix nco; default = zone1 middle")
    ap.add_argument("--rate-mhz", type=float, default=122.88)
    ap.add_argument("--n-addr", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--secs", type=float, default=1.0)
    ap.add_argument("--cap-mb", type=float, default=16.0)
    ap.add_argument("--no-tx", action="store_true", help="RX-only: see noise floor")
    ap.add_argument("--snr-db", type=float, default=15.0)
    args = ap.parse_args()

    print(f"TX {args.tx_ip} ch{args.tx_ch} (DAC_A) -> RX {args.rx_ip} "
          f"ch{args.rx_ch} (ADC_C)   mix={args.mix}  no_tx={args.no_tx}")
    tx_ctx = hs.open_device(node=args.tx_ip, ch=args.tx_ch, verbose=False)
    rx_ctx = hs.open_device(node=args.rx_ip, ch=args.rx_ch, verbose=False)
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]
    dac_rate = float(dict(txd.getChannelInfo(SOAPY_SDR_TX, args.tx_ch)).get(
        "rfdc_effective_rate_hz", 983.04e6))

    tx = None
    try:
        if not args.no_tx:
            iq_a, f_act = hs.tx_iq_tone(args.tone_mhz * 1e6, dac_rate, args.n_addr,
                                        amp_frac=args.amp)
            print(f"TX tone baseband {f_act/1e6:.3f} MHz amp={args.amp}")
            tx = txd.setupStream(SOAPY_SDR_TX, "CS16", [args.tx_ch],
                                 {"tx_mode": "replay"})
            cs16 = np.ascontiguousarray(iq_a, dtype=np.int16).view(np.int32)
            txd.writeStream(tx, [cs16], cs16.size, 0, 0)
            txd.activateStream(tx)
            if args.mix == "fs4":
                txd.writeSetting("RFDC_TX_COARSE_MIX", "fs4")
            else:
                nco = (args.nco_mhz * 1e6 if args.nco_mhz is not None
                       else hs.zone1_nco_hz(rxd, rx_ch=args.rx_ch, tx_ch=args.rx_ch))
                txd.setFrequency(SOAPY_SDR_TX, args.tx_ch, nco)
                print(f"fine NCO {nco/1e6:.3f} MHz (both)")
        else:
            f_act = 0.0

        try:
            rxd.setSampleRate(SOAPY_SDR_RX, args.rx_ch, args.rate_mhz * 1e6)
        except Exception as e:  # noqa: BLE001
            print(f"  setSampleRate warn: {e}")
        if args.mix == "fs4":
            rxd.writeSetting("RFDC_RX_COARSE_MIX", "fs4")
        else:
            nco = (args.nco_mhz * 1e6 if args.nco_mhz is not None
                   else hs.zone1_nco_hz(rxd, rx_ch=args.rx_ch, tx_ch=args.rx_ch))
            rxd.setFrequency(SOAPY_SDR_RX, args.rx_ch, nco)
        fs = float(rxd.getSampleRate(SOAPY_SDR_RX, args.rx_ch))
        print(f"RX rate {fs/1e6:.3f} MSPS -- capturing {args.secs}s ...")
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

    peaks = fft_peaks(to_complex(lanes), fs)
    print("\ntop spectral peaks (DC excluded):")
    for f, s in peaks:
        print(f"  {f/1e6:+9.3f} MHz   SNR {s:6.1f} dB")
    best = peaks[0][1] if peaks else 0.0
    ok = (not args.no_tx) and best >= args.snr_db
    print("\nRESULT:",
          f"TONE DETECTED @ {peaks[0][0]/1e6:+.3f} MHz (SNR {best:.1f} dB) -- "
          ".21 DAC_A -> .22 ADC_C link OK" if ok
          else "no clear tone -- try --mix nco / other channels / levels")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
