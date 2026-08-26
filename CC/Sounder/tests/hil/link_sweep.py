#!/usr/bin/env python3
"""
link_sweep.py -- find the carrier where the .21 DAC_A -> .22 ADC_C link couples.

The Houdini front-end cable band is ~1 GHz (per SoapyHoudiniSDR conftest: the
coarse Fs/4 ~983 MHz loopback validated it), but the zone-aware device caps the
fine NCO at Nyquist zone 1 (|f| < Fs_conv/2 ~= 614 MHz).  A zone-1 fine NCO
(e.g. 307 MHz) sits BELOW the cable band, so the tone comes back at noise level.

This sweeps the matched fine NCO up toward the zone-1 edge (and coarse Fs/4) and,
for each carrier, reports whether a REAL tone comes back -- the tell is image
rejection (a true one-sided complex tone shows tens of dB; noise shows ~0-2 dB),
corroborated by a rise in total lane power (std) over the --no-tx floor.  Prints
a table so we can read off the best carrier (or conclude the cable band is out of
zone-1 reach and coarse Fs/4 is required).

Run on the DGX (after: source houdini_test/bin/activate):
    python3 link_sweep.py
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
from sigqual import tone_spectrum, tone_metrics  # noqa: E402

# Internal digital spurs to exclude when hunting the tone (Fs/4, Fs/2 of 122.88).
SPUR_HZ = (30.72e6, 61.44e6)


def to_complex(lanes):
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex64)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float32)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return (L[:, 0] + 1j * q).astype(np.complex64)


def best_nonspur_peak(freqs, mag, dc_guard=1e6, spur_guard=0.5e6):
    """Strongest bin excluding DC + the internal Fs/4, Fs/2 spurs -> its freq."""
    bad = np.abs(freqs) <= dc_guard
    for s in SPUR_HZ:
        bad |= (np.abs(freqs - s) <= spur_guard) | (np.abs(freqs + s) <= spur_guard)
    m = mag.copy()
    m[bad] = 0.0
    return float(freqs[int(np.argmax(m))])


def capture_once(txd, rxd, args, mix, nco_hz):
    """Arm the tone with (mix, nco), capture, return (std, tone_freq, snr, img_rej)."""
    dac_rate = float(dict(txd.getChannelInfo(SOAPY_SDR_TX, args.tx_ch)).get(
        "rfdc_effective_rate_hz", 983.04e6))
    f_bb = args.tone_mhz * 1e6
    tx = None
    try:
        if mix != "off":
            iq_a, _ = hs.tx_iq_tone(f_bb, dac_rate, args.n_addr, amp_frac=args.amp)
            tx = txd.setupStream(SOAPY_SDR_TX, "CS16", [args.tx_ch], {"tx_mode": "replay"})
            if mix == "nco":
                txd.setFrequency(SOAPY_SDR_TX, args.tx_ch, nco_hz)
            elif mix == "fs4":
                txd.writeSetting("RFDC_TX_COARSE_MIX", "fs4")
            cs16 = np.ascontiguousarray(iq_a, dtype=np.int16).view(np.int32)
            txd.writeStream(tx, [cs16], cs16.size, 0, 0)
            txd.activateStream(tx)
        rxd.setSampleRate(SOAPY_SDR_RX, args.rx_ch, args.rate_mhz * 1e6)
        if mix == "nco":
            rxd.setFrequency(SOAPY_SDR_RX, args.rx_ch, nco_hz)
        elif mix == "fs4":
            rxd.writeSetting("RFDC_RX_COARSE_MIX", "fs4")
        fs = float(rxd.getSampleRate(SOAPY_SDR_RX, args.rx_ch))
        buf, _ = hs.capture_rx(rxd, args.rx_ch, rx_ctx_native, rx_ctx_dtype,
                               duration_sec=args.secs,
                               capture_bytes=int(args.cap_mb * 1024 * 1024))
    finally:
        if tx is not None:
            try:
                txd.deactivateStream(tx)
                txd.closeStream(tx)
            except Exception:  # noqa: BLE001
                pass
    lanes = hs.cs16_lanes(buf)
    std = float(np.mean([lanes[:, k].astype(np.float64).std()
                         for k in range(lanes.shape[1])]))
    iq = to_complex(lanes)
    freqs, mag = tone_spectrum(iq, fs)
    f_tone = best_nonspur_peak(freqs, mag)
    m = tone_metrics(freqs, mag, f_tone, span_hz=args.cfo_tol_mhz * 1e6)
    return std, (m["f_peak"] or f_tone), m["snr_db"], m["img_rej_db"], m["present"]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=0)
    ap.add_argument("--rx-ch", type=int, default=2)
    ap.add_argument("--tone-mhz", type=float, default=20.0)
    ap.add_argument("--rate-mhz", type=float, default=122.88)
    ap.add_argument("--n-addr", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--secs", type=float, default=0.3)
    ap.add_argument("--cap-mb", type=float, default=8.0)
    ap.add_argument("--cfo-tol-mhz", type=float, default=0.5)
    ap.add_argument("--nco-mhz", default="150,300,450,560,600",
                    help="comma list of fine-NCO carriers to sweep (MHz)")
    args = ap.parse_args()

    global rx_ctx_native, rx_ctx_dtype
    print(f"TX {args.tx_ip} ch{args.tx_ch} (DAC_A) -> RX {args.rx_ip} ch{args.rx_ch} "
          f"(ADC_C)   tone {args.tone_mhz} MHz  amp {args.amp}")
    tx_ctx = hs.open_device(node=args.tx_ip, ch=args.tx_ch, verbose=False)
    rx_ctx = hs.open_device(node=args.rx_ip, ch=args.rx_ch, verbose=False)
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]
    rx_ctx_native, rx_ctx_dtype = rx_ctx["native_fmt"], rx_ctx["dtype"]

    configs = [("off", 0.0)]
    for v in args.nco_mhz.split(","):
        if v.strip():
            configs.append(("nco", float(v) * 1e6))
    configs.append(("fs4", 0.0))

    print(f"\n{'carrier':>18} {'lane_std':>9} {'tone_MHz':>9} {'SNR_dB':>7} "
          f"{'imgrej_dB':>9} {'verdict':>8}")
    floor = None
    rows = []
    for mix, nco in configs:
        try:
            std, f_t, snr, imgrej, present = capture_once(txd, rxd, args, mix, nco)
        except Exception as e:  # noqa: BLE001
            print(f"{mix:>14}{'':>4} FAILED: {e}")
            continue
        if mix == "off":
            floor = std
            label = "no-tx (floor)"
        elif mix == "nco":
            label = f"fine NCO {nco/1e6:.0f}"
        else:
            label = "coarse Fs/4"
        # A REAL tone: strong image rejection AND lane power up over the floor.
        real = present and imgrej >= 10.0 and (floor is None or std >= 1.15 * floor)
        verdict = "TONE" if real else ("weak" if present else "-")
        rows.append((label, std, f_t, snr, imgrej, real))
        print(f"{label:>18} {std:9.2f} {f_t/1e6:+9.3f} {snr:7.1f} {imgrej:9.1f} "
              f"{verdict:>8}")

    tones = [r for r in rows if r[5]]
    print()
    if tones:
        best = max(tones, key=lambda r: r[4])  # highest image rejection
        print(f"BEST: {best[0]} -- tone @ {best[2]/1e6:+.3f} MHz, SNR {best[3]:.1f} dB, "
              f"image_rej {best[4]:.1f} dB.  Use this carrier for the beacon test.")
    else:
        print("NO carrier produced a real tone (image_rej<10 dB + no power rise).\n"
              "  => the cable band is likely out of zone-1 reach (need coarse Fs/4 "
              "with matched DAC/ADC converter rates), or check TX drive / DAC_A "
              "port / cable.  Escalate the frequency plan for this bitstream.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
