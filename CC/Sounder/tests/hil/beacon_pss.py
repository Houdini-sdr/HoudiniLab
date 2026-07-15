#!/usr/bin/env python3
"""
beacon_pss.py -- PSS/SSS-style narrowband beacon over the .21 DAC_A -> .22 ADC_C
comb tooth, detected 5G-UE style (DDC -> decimate -> ZC matched filter).

The cross-board channel couples only in a ~2-3 MHz standing-wave tooth at RF
~840 MHz, so a full-band beacon is hopeless.  Instead -- exactly like a 5G SSB --
we put a length-127 Zadoff-Chu sequence on 127 contiguous centre subcarriers
(15 kHz SCS -> 1.9 MHz occupied), size it to sit inside the tooth, and detect it
by downconverting the wideband 122.88 MSPS capture to the beacon band, decimating
to 3.84 MHz, and matched-filtering against the 127-tap ZC.  Correlating at 3.84
instead of 122.88 is ~32x less work -- the same reason a UE searches PSS at a low
rate.

Placement: TX at 30.72 MSPS, N=2048 IFFT (15 kHz SCS), ZC on subcarriers centred
at baseband +10 MHz; DAC NCO 830 -> RF 840 (on the tooth).  RX at 122.88 MSPS,
ADC NCO 388.8 -> the beacon lands near +20 MHz; the script measures the exact
centre (kills inter-board CFO) before the DDC.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 beacon_pss.py
    python3 beacon_pss.py --no-tx      # control: no beacon -> should NOT detect
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

ZC_U = 25          # Zadoff-Chu root (coprime with 127; 5G PSS uses 25/29/34)
N_SC = 127         # occupied subcarriers (5G PSS length)
SCS_HZ = 15e3      # subcarrier spacing


def zc(u, n):
    """Length-n Zadoff-Chu (n odd)."""
    k = np.arange(n)
    return np.exp(-1j * np.pi * u * k * (k + 1) / n)


def beacon_symbol(n_fft, center_hz, rate_hz, u=ZC_U):
    """127-ZC on the centre subcarriers of an n_fft IFFT, centred at center_hz.
    Returns the n_fft-sample complex time-domain symbol (unit-ish amplitude)."""
    seq = zc(u, N_SC)
    X = np.zeros(n_fft, dtype=complex)
    c = int(round(center_hz / (rate_hz / n_fft)))          # centre bin
    idx = (np.arange(N_SC) - (N_SC // 2) + c) % n_fft       # wrapped SC indices
    X[idx] = seq
    x = np.fft.ifft(X) * n_fft
    return x


def lowpass(ntaps, fc):
    """Windowed-sinc LPF; fc = cutoff as a fraction of Nyquist (fs/2)."""
    n = np.arange(ntaps) - (ntaps - 1) / 2.0
    h = fc * np.sinc(fc * n) * np.hamming(ntaps)
    return (h / h.sum()).astype(np.float64)


def to_complex(lanes):
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
    ap.add_argument("--tx-ch", type=int, default=0)
    ap.add_argument("--rx-ch", type=int, default=2)
    ap.add_argument("--dac-nco", type=float, default=820.0, help="TX carrier (MHz)")
    ap.add_argument("--adc-nco", type=float, default=388.8, help="RX NCO (MHz)")
    ap.add_argument("--tx-rate", type=float, default=122.88, help="TX MSPS (=RX; TX min is 122.88)")
    ap.add_argument("--rx-rate", type=float, default=122.88, help="RX MSPS")
    ap.add_argument("--n-fft", type=int, default=8192, help="TX IFFT size (8192@122.88 -> 15 kHz SCS)")
    ap.add_argument("--tx-center", type=float, default=20.0,
                    help="beacon centre in TX baseband (MHz); DAC_NCO+this = RF")
    ap.add_argument("--rx-search", type=float, default=20.0,
                    help="approx RX baseband centre to search (MHz)")
    ap.add_argument("--decim", type=int, default=32, help="RX decimation (122.88->3.84)")
    ap.add_argument("--amp", type=float, default=0.9)
    ap.add_argument("--secs", type=float, default=0.3)
    ap.add_argument("--cap-mb", type=float, default=16.0)
    ap.add_argument("--no-tx", action="store_true")
    a = ap.parse_args()

    rf = a.dac_nco + a.tx_center
    print(f"TX {a.tx_ip} ch{a.tx_ch} (DAC_A) -> RX {a.rx_ip} ch{a.rx_ch} (ADC_C)")
    print(f"beacon: ZC-{N_SC} u={ZC_U}, {N_SC*SCS_HZ/1e6:.2f} MHz occupied, "
          f"TX {a.tx_rate} MSPS N={a.n_fft} centre +{a.tx_center} MHz, "
          f"DAC NCO {a.dac_nco} -> RF {rf:.1f} MHz (tooth), ADC NCO {a.adc_nco}")

    tx_ctx = hs.open_device(node=a.tx_ip, ch=a.tx_ch, verbose=False)
    rx_ctx = hs.open_device(node=a.rx_ip, ch=a.rx_ch, verbose=False)
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]
    native, dtype = rx_ctx["native_fmt"], rx_ctx["dtype"]

    tx = None
    try:
        if not a.no_tx:
            try:
                txd.setSampleRate(SOAPY_SDR_TX, a.tx_ch, a.tx_rate * 1e6)
            except Exception as e:  # noqa: BLE001
                print(f"  TX setSampleRate warn: {e}")
            sym = beacon_symbol(a.n_fft, a.tx_center * 1e6, a.tx_rate * 1e6)
            sym = sym / np.max(np.abs(sym)) * a.amp
            iq_i16 = np.zeros(2 * a.n_fft, dtype=np.int16)
            iq_i16[0::2] = np.round(np.real(sym) * 32767).astype(np.int16)
            iq_i16[1::2] = np.round(np.imag(sym) * 32767).astype(np.int16)
            tx = txd.setupStream(SOAPY_SDR_TX, "CS16", [a.tx_ch], {"tx_mode": "replay"})
            txd.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.dac_nco * 1e6)
            cs16 = np.ascontiguousarray(iq_i16, dtype=np.int16).view(np.int32)
            txd.writeStream(tx, [cs16], cs16.size, 0, 0)
            txd.activateStream(tx)

        rxd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, a.rx_rate * 1e6)
        rxd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.adc_nco * 1e6)
        fs = float(rxd.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
        print(f"RX {fs/1e6:.3f} MSPS -- capturing {a.secs}s ...")
        buf, summ = hs.capture_rx(rxd, a.rx_ch, native, dtype, duration_sec=a.secs,
                                  capture_bytes=int(a.cap_mb * 1024 * 1024))
    finally:
        if tx is not None:
            try:
                txd.deactivateStream(tx)
                txd.closeStream(tx)
            except Exception:  # noqa: BLE001
                pass

    iq = to_complex(hs.cs16_lanes(buf)).astype(np.complex128)
    print(f"captured {len(iq)} samples  overflows={summ.get('overflows')}")

    # 1) find the beacon centre in RX baseband (kills inter-board CFO/placement error)
    seg = iq[:1 << 20] if len(iq) > (1 << 20) else iq
    P = np.abs(np.fft.fftshift(np.fft.fft(seg * np.hanning(len(seg))))) ** 2
    f = np.fft.fftshift(np.fft.fftfreq(len(seg), 1.0 / fs))
    P = np.maximum(P - np.median(P), 0.0)
    band = (f > (a.rx_search - 3) * 1e6) & (f < (a.rx_search + 3) * 1e6)
    f0 = float(np.sum(f[band] * P[band]) / (np.sum(P[band]) + 1e-30))
    occ = float(np.sum(P[band]))                       # beacon band energy
    ref = float(np.sum(P[(f > 40e6) & (f < 46e6)]) + 1e-30)  # empty-band ref
    print(f"beacon centre f0 = {f0/1e6:+.4f} MHz  (band/empty energy ratio "
          f"{10*np.log10(occ/ref):.1f} dB)")

    # 2) DDC to baseband, LPF + decimate to 3.84 MHz (5G-UE narrowband search)
    n = np.arange(len(iq))
    ddc = iq * np.exp(-2j * np.pi * f0 * n / fs)
    h = lowpass(32 * 8 + 1, 1.0 / a.decim)
    dec = np.convolve(ddc, h, mode="same")[::a.decim]
    fs_d = fs / a.decim
    n_sym = int(round(a.n_fft * fs_d / (a.tx_rate * 1e6)))   # symbol length at fs_d
    print(f"decimated to {fs_d/1e6:.3f} MSPS ({len(dec)} samples), "
          f"symbol = {n_sym} samples")

    # 3) matched filter against the ZC symbol at the decimated rate
    match = beacon_symbol(n_sym, 0.0, fs_d)
    match = match / np.sqrt(np.sum(np.abs(match) ** 2))
    corr = np.abs(np.correlate(dec, match, mode="valid"))
    floor = float(np.median(corr))
    pk = int(np.argmax(corr))
    peak_snr = 20.0 * np.log10(corr[pk] / (floor + 1e-30))
    print(f"\nmatched-filter peak: idx {pk}  |corr| {corr[pk]:.2e}  "
          f"SNR {peak_snr:.1f} dB over median")

    # 4) periodicity: the replay loops every n_sym samples -> peaks should recur
    thr = 0.5 * corr[pk]
    peaks = [i for i in range(1, len(corr) - 1)
             if corr[i] > thr and corr[i] >= corr[i - 1] and corr[i] > corr[i + 1]]
    gaps = np.diff(peaks) if len(peaks) > 1 else np.array([])
    good = int(np.sum(np.abs(gaps - n_sym) <= 2)) if len(gaps) else 0
    if len(peaks) > 1:
        print(f"found {len(peaks)} peaks > 50% of max; median gap "
              f"{int(np.median(gaps))} (expect {n_sym}); {good}/{len(gaps)} "
              f"gaps match the loop period")

    detected = (peak_snr >= 12.0 and 10 * np.log10(occ / ref) >= 6.0
                and (a.no_tx is False))
    print("\nRESULT:",
          f"BEACON DETECTED @ decimated idx {pk} (peak {peak_snr:.1f} dB, "
          f"{len(peaks)} periodic hits) -- .21 DAC_A -> .22 ADC_C PSS link OK"
          if detected else
          "no beacon (peak/band too weak)"
          + ("" if a.no_tx else " -- try --tx-center / --dac-nco to re-centre on the tooth"))
    return 0 if detected else 1


if __name__ == "__main__":
    sys.exit(main())
