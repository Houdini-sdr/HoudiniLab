#!/usr/bin/env python3
"""
beacon_sounder.py -- end-to-end: TX the Sounder beacon over real RF and detect it
with the PRODUCTION correlator (CommsLib::find_beacon_avx / find_beacon_cuda via
the beacon_hil harness), closing the loop from the boards to the real Sounder code.

Flow:
  1. beacon_hil --dump-tx  -> the exact 128-sample beacon match the detector uses.
  2. upsample it x8 (the TX replay clocks at 983.04, RX at 122.88) so each rep is
     128 samples at the RX rate; lay down 2 reps + a guard = the RENEW beacon shape.
  3. replay it on .21 ch1 (DAC_A), matched NCO -> the beacon returns to baseband.
  4. capture on .22 ch1 (the cabled ADC), save raw int16 I/Q.
  5. beacon_hil --detect  -> find_beacon_avx / find_beacon_cuda on the live capture.

Run on the DGX (after: source houdini_test/bin/activate), with beacon_hil built in
~/repos/HoudiniLab/CC/Sounder/ :
    python3 beacon_sounder.py
"""
import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np

_EX = os.environ.get(
    "HOUDINI_EXAMPLES",
    os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if _EX not in sys.path:
    sys.path.insert(0, _EX)

import SoapySDR  # noqa: E402
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX  # noqa: E402
import houdini_setup as hs  # noqa: E402


def upsample(x, f):
    """Band-limited integer upsample by f (frequency-domain zero-pad)."""
    n = len(x)
    X = np.fft.fft(x)
    Xu = np.zeros(n * f, dtype=complex)
    h = n // 2
    Xu[:h] = X[:h]
    Xu[-h:] = X[-h:]
    return np.fft.ifft(Xu) * f


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
    ap.add_argument("--tx-ch", type=int, default=1, help="DAC_A = TX ch1")
    ap.add_argument("--rx-ch", type=int, default=1, help="cable lands on .22 RX ch1")
    ap.add_argument("--nco-mhz", type=float, default=500.0, help="matched NCO (Zone 1)")
    ap.add_argument("--upsample", type=int, default=8, help="dac_rate/rx_rate = 983.04/122.88")
    ap.add_argument("--amp", type=float, default=0.6)
    ap.add_argument("--rx-rate", type=float, default=122.88)
    ap.add_argument("--secs", type=float, default=0.2)
    ap.add_argument("--cap-mb", type=float, default=8.0)
    ap.add_argument("--detector",
                    default=os.path.expanduser("~/repos/HoudiniLab/CC/Sounder/beacon_hil"))
    ap.add_argument("--no-tx", action="store_true", help="control: capture with no beacon")
    a = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="beacon_hil_")
    match_bin, cap_bin = os.path.join(tmp, "match.bin"), os.path.join(tmp, "cap.bin")

    # 1) the exact match the detector will use
    subprocess.run([a.detector, "--dump-tx", match_bin], check=True)
    mf = np.fromfile(match_bin, dtype=np.float32)
    match = (mf[0::2] + 1j * mf[1::2]).astype(np.complex128)
    print(f"match: {len(match)} samples")

    # 2) beacon replay loop = 2 reps of the x8-upsampled match + an equal guard
    rep = upsample(match, a.upsample)
    beacon = np.concatenate([rep, rep])              # 2 reps -> find_beacon auto-corr
    loop = np.concatenate([beacon, np.zeros(len(beacon), dtype=complex)])  # + guard
    loop = loop / (np.max(np.abs(loop)) + 1e-30) * a.amp
    n_load = len(loop)
    i16 = np.zeros(2 * n_load, dtype=np.int16)
    i16[0::2] = np.round(loop.real * 32767).astype(np.int16)
    i16[1::2] = np.round(loop.imag * 32767).astype(np.int16)
    print(f"TX {a.tx_ip} ch{a.tx_ch} -> RX {a.rx_ip} ch{a.rx_ch}  NCO {a.nco_mhz} MHz  "
          f"replay {n_load} samp ({a.upsample}x match, 2 reps + guard)")

    tx_ctx = hs.open_device(node=a.tx_ip, ch=a.tx_ch, verbose=False)
    rx_ctx = hs.open_device(node=a.rx_ip, ch=a.rx_ch, verbose=False)
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]
    tx = None
    try:
        if not a.no_tx:
            ladder = list(txd.listSampleRates(SOAPY_SDR_TX, a.tx_ch))
            if ladder:
                txd.setSampleRate(SOAPY_SDR_TX, a.tx_ch, max(ladder))
            tx = txd.setupStream(SOAPY_SDR_TX, tx_ctx["native_fmt"], [a.tx_ch],
                                 {"tx_mode": "replay"})
            txd.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.nco_mhz * 1e6)
            cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)
            txd.writeStream(tx, [cs16], n_load, 0, 0)
            txd.activateStream(tx)
        rxd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, a.rx_rate * 1e6)
        rxd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco_mhz * 1e6)
        buf, summ = hs.capture_rx(rxd, a.rx_ch, rx_ctx["native_fmt"], rx_ctx["dtype"],
                                  duration_sec=a.secs,
                                  capture_bytes=int(a.cap_mb * 1024 * 1024))
    finally:
        if tx is not None:
            try:
                txd.deactivateStream(tx)
                txd.closeStream(tx)
            except Exception:  # noqa: BLE001
                pass

    # 4) save the raw int16 I/Q the way find_beacon consumes it (cint16)
    iq = to_complex(hs.cs16_lanes(buf))
    out = np.zeros(2 * len(iq), dtype=np.int16)
    out[0::2] = np.clip(np.round(iq.real), -32768, 32767).astype(np.int16)
    out[1::2] = np.clip(np.round(iq.imag), -32768, 32767).astype(np.int16)
    out.tofile(cap_bin)
    print(f"captured {len(iq)} samples (overflows={summ.get('overflows')}) -> {cap_bin}")

    # 5) the real Sounder correlator on the live capture
    print("\n--- find_beacon on the live capture ---")
    rc = subprocess.run([a.detector, "--detect", cap_bin,
                         "--window", str(min(len(iq), 1 << 18))]).returncode
    return rc


if __name__ == "__main__":
    sys.exit(main())
