#!/usr/bin/env python3
"""
dual_nco_link.py -- reach the ~1 GHz front-end cable band with UNMATCHED NCOs.

A matched fine NCO is capped at the ADC's Nyquist zone 1 (<614 MHz), below the
Houdini cable band (~1 GHz).  But the DAC's zone 1 goes to Fs_dac/2 = 983 MHz, so
we can put the DAC carrier in the cable band and let the ADC UNDERSAMPLE it:

  * TX (.21 DAC_A): fine NCO = f_dac (its own zone 1) -> real RF at f_dac + f_bb,
    placed in the ~0.7-0.98 GHz cable band.
  * RX (.22 ADC_C): the RF aliases into the ADC's zone 1 at |fold(f_rf, Fs_adc)|;
    set the ADC fine NCO there to fold it down to baseband -> tone at ~|f_bb|.

Both NCOs stay legal zone-1 values on their own converter.  Sweeping the DAC
carrier with the ADC NCO auto-tracking the alias, the total lane power (std)
jumps where the carrier hits the cable band -- that peak is the usable carrier.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 dual_nco_link.py
    python3 dual_nco_link.py --dac-nco 940 --fine   # fine ADC-NCO tune @ one carrier
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

SPUR_HZ = (30.72e6, 61.44e6)  # internal Fs/4, Fs/2 of the 122.88 stream


def to_complex(lanes):
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex64)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float32)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return (L[:, 0] + 1j * q).astype(np.complex64)


def fold(f, fs):
    """Alias a frequency into [-fs/2, +fs/2] (undersampling fold)."""
    return f - round(f / fs) * fs


def best_nonspur_peak(freqs, mag, dc_guard=1e6, spur_guard=0.5e6):
    bad = np.abs(freqs) <= dc_guard
    for s in SPUR_HZ:
        bad |= (np.abs(freqs - s) <= spur_guard) | (np.abs(freqs + s) <= spur_guard)
    m = mag.copy()
    m[bad] = 0.0
    return float(freqs[int(np.argmax(m))])


def capture(txd, rxd, a, dac_nco, adc_nco, native, dtype):
    dac_rate = float(dict(txd.getChannelInfo(SOAPY_SDR_TX, a.tx_ch)).get(
        "rfdc_effective_rate_hz", 983.04e6))
    f_bb = a.tone_mhz * 1e6
    tx = None
    try:
        if dac_nco is not None:
            iq_a, _ = hs.tx_iq_tone(f_bb, dac_rate, a.n_addr, amp_frac=a.amp)
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
    std = float(np.mean([lanes[:, k].astype(np.float64).std()
                         for k in range(lanes.shape[1])]))
    freqs, mag = tone_spectrum(to_complex(lanes), fs)
    m = tone_metrics(freqs, mag, f_bb, span_hz=a.cfo_tol_mhz * 1e6)
    f_peak = best_nonspur_peak(freqs, mag)
    return std, m, f_peak


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=0)
    ap.add_argument("--rx-ch", type=int, default=2)
    ap.add_argument("--tone-mhz", type=float, default=20.0)
    ap.add_argument("--rate-mhz", type=float, default=122.88)
    ap.add_argument("--n-addr", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.7)
    ap.add_argument("--secs", type=float, default=0.3)
    ap.add_argument("--cap-mb", type=float, default=8.0)
    ap.add_argument("--cfo-tol-mhz", type=float, default=0.6)
    ap.add_argument("--dac-nco", default="640,700,760,820,880,920,960",
                    help="DAC carriers to sweep (MHz); each < Fs_dac/2 = 983")
    ap.add_argument("--fine", action="store_true",
                    help="single --dac-nco: fine-sweep the ADC NCO +/-30 MHz")
    a = ap.parse_args()

    print(f"TX {a.tx_ip} ch{a.tx_ch} (DAC_A) -> RX {a.rx_ip} ch{a.rx_ch} (ADC_C)  "
          f"tone {a.tone_mhz} MHz  amp {a.amp}")
    tx_ctx = hs.open_device(node=a.tx_ip, ch=a.tx_ch, verbose=False)
    # Single-board loopback (tx_ip == rx_ip): reuse ONE device handle for both
    # directions -- shared clock (no inter-board CFO), short cable.
    rx_ctx = (tx_ctx if a.rx_ip == a.tx_ip
              else hs.open_device(node=a.rx_ip, ch=a.rx_ch, verbose=False))
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]
    native, dtype = rx_ctx["native_fmt"], rx_ctx["dtype"]
    fs_adc = float(dict(rxd.getChannelInfo(SOAPY_SDR_RX, a.rx_ch)).get(
        "rfdc_sample_rate_hz", 1228.8e6))
    fs_dac = float(dict(txd.getChannelInfo(SOAPY_SDR_TX, a.tx_ch)).get(
        "rfdc_sample_rate_hz", 1966.08e6))
    print(f"Fs_dac={fs_dac/1e6:.1f} (zone1<{fs_dac/2e6:.0f})  "
          f"Fs_adc={fs_adc/1e6:.1f} (zone1<{fs_adc/2e6:.0f})")

    # baseline: TX off
    std0, _, _ = capture(txd, rxd, a, None, 0.0, native, dtype)
    print(f"\nno-tx floor: lane std = {std0:.2f}")

    dacs = [float(v) * 1e6 for v in a.dac_nco.split(",") if v.strip()]
    if a.fine and len(dacs) == 1:
        base = abs(fold(dacs[0] + a.tone_mhz * 1e6, fs_adc))
        plan = [(dacs[0], base + off * 1e6) for off in range(-30, 31, 3)]
        col = "adc_nco"
    else:
        plan = [(d, abs(fold(d + a.tone_mhz * 1e6, fs_adc))) for d in dacs]
        col = "dac_nco"

    print(f"\n{col:>9} {'f_rf':>7} {'adc_nco':>8} {'std':>8} {'x_floor':>7} "
          f"{'tone_MHz':>9} {'SNR':>6} {'imgrej':>7} {'peak_MHz':>9}")
    rows = []
    for dac_nco, adc_nco in plan:
        try:
            std, m, f_peak = capture(txd, rxd, a, dac_nco, adc_nco, native, dtype)
        except Exception as e:  # noqa: BLE001
            print(f"{dac_nco/1e6:9.1f}  FAILED: {e}")
            continue
        f_rf = dac_nco + a.tone_mhz * 1e6
        xf = std / std0 if std0 else 0.0
        key = adc_nco if a.fine else dac_nco
        strong = xf >= 2.0 and m["img_rej_db"] >= 10.0
        rows.append((key, dac_nco, adc_nco, std, xf, m, f_peak, strong))
        print(f"{key/1e6:9.1f} {f_rf/1e6:7.1f} {adc_nco/1e6:8.1f} {std:8.1f} "
              f"{xf:7.2f} {(m['f_peak'] or f_peak)/1e6:+9.3f} {m['snr_db']:6.1f} "
              f"{m['img_rej_db']:7.1f} {f_peak/1e6:+9.3f}"
              + ("  <== STRONG" if strong else ""))

    hits = [r for r in rows if r[7]]
    print()
    if hits:
        best = max(hits, key=lambda r: r[4])  # biggest power rise
        print(f"BEST: DAC NCO {best[1]/1e6:.1f} MHz (RF {(best[1]+a.tone_mhz*1e6)/1e6:.1f}"
              f" MHz) + ADC NCO {best[2]/1e6:.1f} MHz -> {best[4]:.1f}x floor, "
              f"tone SNR {best[5]['snr_db']:.1f} dB, img_rej {best[5]['img_rej_db']:.1f} dB.")
        if not a.fine:
            print(f"  Refine: python3 dual_nco_link.py --dac-nco {best[1]/1e6:.0f} --fine")
    else:
        print("No carrier showed a strong in-band tone (>=2x floor + img_rej>=10 dB).\n"
              "  Widen --dac-nco, raise --amp, or the cable band may differ on these "
              "ports -- report back with this table.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
