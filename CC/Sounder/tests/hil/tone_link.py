#!/usr/bin/env python3
"""
tone_link.py -- cross-board RF link check for the Sounder HIL bring-up.

TX-replay a tone on board A (DAC_A / .21 ch0), RX-capture on board B
(ADC_C / .22 ch2), and detect it with the *canonical* loopback recipe from
SoapyHoudiniSDR's rfdc_characterize.py:

  * a MATCHED fine NCO on TX+RX (houdini_setup.zone1_nco_hz -- middle of Nyquist
    zone 1, ~Fs_conv/4; the old fixed 1 GHz default is out-of-zone and rejected),
  * a baseband I/Q tone at ~0.02035*dac_rate (~20 MHz) via the replay stream,
  * detection with sigqual.tone_metrics at |f_bb|, which picks the stronger of
    the +/-f_bb sidebands as the tone (I/Q sign convention) and the other as its
    image -- so it ignores the DC / Fs/4 / Fs/2 internal spurs a global argmax
    would grab (the "fine_nco failure").

The two boards run independent clocks, so the tone lands at f_bb + a small
inter-board CFO (~kHz); --cfo-tol widens the search band around |f_bb|.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 tone_link.py                 # matched fine NCO (recipe), TX on
    python3 tone_link.py --no-tx         # RX-only control (spur/noise floor)
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
from sigqual import tone_spectrum, tone_metrics, dominant_tone  # noqa: E402


def to_complex(lanes):
    """cs16 lanes -> complex baseband via houdini_setup's layout-aware helper."""
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex64)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float32)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return (L[:, 0] + 1j * q).astype(np.complex64)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=0, help="DAC_A = TX ch0")
    ap.add_argument("--rx-ch", type=int, default=2, help="ADC_C = RX ch2")
    ap.add_argument("--tone-mhz", type=float, default=None,
                    help="TX baseband tone (MHz); default 0.02035*dac_rate (~20)")
    ap.add_argument("--mix", choices=["nco", "fs4"], default="nco",
                    help="nco = matched fine NCO (recipe); fs4 = coarse Fs/4")
    ap.add_argument("--nco-mhz", type=float, default=None,
                    help="override the matched fine NCO (MHz); default = zone1 middle")
    ap.add_argument("--rate-mhz", type=float, default=122.88)
    ap.add_argument("--n-addr", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--secs", type=float, default=1.0)
    ap.add_argument("--cap-mb", type=float, default=16.0)
    ap.add_argument("--cfo-tol-mhz", type=float, default=0.5,
                    help="search half-band around |f_bb| for the CFO-shifted tone")
    ap.add_argument("--no-tx", action="store_true", help="RX-only: spur/noise floor")
    args = ap.parse_args()

    print(f"TX {args.tx_ip} ch{args.tx_ch} (DAC_A) -> RX {args.rx_ip} "
          f"ch{args.rx_ch} (ADC_C)   mix={args.mix}  no_tx={args.no_tx}")
    tx_ctx = hs.open_device(node=args.tx_ip, ch=args.tx_ch, verbose=False)
    rx_ctx = (tx_ctx if args.rx_ip == args.tx_ip      # single-board loopback: 1 handle
              else hs.open_device(node=args.rx_ip, ch=args.rx_ch, verbose=False))
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]
    dac_rate = float(dict(txd.getChannelInfo(SOAPY_SDR_TX, args.tx_ch)).get(
        "rfdc_effective_rate_hz", 983.04e6))
    f_bb = (args.tone_mhz * 1e6 if args.tone_mhz is not None
            else float(os.environ.get("HOUDINI_HIL_TXFREQ", "") or 0.02035) * dac_rate)
    # Matched fine NCO: both boards share the converter rate, so one value is
    # zone-1 for both; set it on the .21 DAC and the .22 ADC.
    nco = (args.nco_mhz * 1e6 if args.nco_mhz is not None
           else hs.zone1_nco_hz(rxd, rx_ch=args.rx_ch, tx_ch=args.rx_ch))

    tx = None
    f_act = f_bb
    try:
        if not args.no_tx:
            iq_a, f_act = hs.tx_iq_tone(f_bb, dac_rate, args.n_addr, amp_frac=args.amp)
            print(f"TX tone baseband {f_act/1e6:.4f} MHz amp={args.amp}  "
                  f"mix={args.mix} "
                  + (f"fine NCO {nco/1e6:.3f} MHz (matched)" if args.mix == "nco"
                     else "coarse Fs/4"))
            tx = txd.setupStream(SOAPY_SDR_TX, "CS16", [args.tx_ch],
                                 {"tx_mode": "replay"})
            if args.mix == "nco":
                txd.setFrequency(SOAPY_SDR_TX, args.tx_ch, nco)
            else:
                txd.writeSetting("RFDC_TX_COARSE_MIX", "fs4")
            cs16 = np.ascontiguousarray(iq_a, dtype=np.int16).view(np.int32)
            txd.writeStream(tx, [cs16], cs16.size, 0, 0)
            txd.activateStream(tx)

        try:
            rxd.setSampleRate(SOAPY_SDR_RX, args.rx_ch, args.rate_mhz * 1e6)
        except Exception as e:  # noqa: BLE001
            print(f"  setSampleRate warn: {e}")
        if args.mix == "nco":
            rxd.setFrequency(SOAPY_SDR_RX, args.rx_ch, nco)
        else:
            rxd.writeSetting("RFDC_RX_COARSE_MIX", "fs4")
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

    iq = to_complex(lanes)
    freqs, mag = tone_spectrum(iq, fs)
    # Canonical detection at the EXPECTED |f_bb|, widened to catch the inter-board
    # CFO -- ignores the DC / Fs/4 / Fs/2 internal spurs by construction.
    m = tone_metrics(freqs, mag, f_act, span_hz=args.cfo_tol_mhz * 1e6)
    dom = dominant_tone(freqs, mag, dc_guard_hz=1e6, edge_guard_hz=1e6)
    print(f"\nexpected tone |f_bb| = {f_act/1e6:.4f} MHz  "
          f"(+/- {args.cfo_tol_mhz:.2f} MHz CFO search):")
    print(f"  present={m['present']}  SNR={m['snr_db']:.1f} dB  "
          f"image_rej={m['img_rej_db']:.1f} dB  sign={m['sign']}  "
          f"f_peak={ (m['f_peak'] or 0.0)/1e6:+.4f} MHz")
    print(f"  (strongest in-band peak overall: {dom.get('f_peak',0.0)/1e6:+.3f} MHz "
          f"SNR {dom['snr_db']:.1f} dB -- expect an internal Fs/4=30.72 or "
          f"Fs/2=61.44 spur here)")

    ok = (not args.no_tx) and m["present"]
    cfo = ((m["f_peak"] or 0.0) - (f_act if m["sign"] == "+f" else -f_act))
    print("\nRESULT:",
          f"TONE DETECTED @ {(m['f_peak'] or 0.0)/1e6:+.4f} MHz "
          f"(SNR {m['snr_db']:.1f} dB, img_rej {m['img_rej_db']:.1f} dB, "
          f"CFO {cfo/1e3:+.2f} kHz) -- .21 DAC_A -> .22 ADC_C link OK" if ok
          else "no tone at |f_bb| -- try --mix fs4 / --tone-mhz / other channels")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
