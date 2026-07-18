#!/usr/bin/env python3
"""
beacon_stream.py -- the receiver.cc clientSyncBeacon loop on the live Houdini RX.

Reproduces the UE sync loop (radioRx one search window -> syncSearch -> repeat)
end-to-end on real RF: the RENEW radio abstraction (client_radio_set_->radioRx)
doesn't drive the Houdini SDR, so this drives the Houdini RX over SoapySDR and
feeds each window to the SAME correlator syncSearch uses -- the persistent
beacon_hil --stream (CommsLib::find_beacon_avx + find_beacon_cuda). Reports the
per-window correlator latency and the sync index, exactly like a UE acquiring.

Run on the DGX (after: source houdini_test/bin/activate), beacon_hil built:
    python3 beacon_stream.py
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time

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
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--nco-mhz", type=float, default=500.0)
    ap.add_argument("--upsample", type=int, default=8)
    ap.add_argument("--amp", type=float, default=0.6)
    ap.add_argument("--rx-rate", type=float, default=122.88)
    ap.add_argument("--window", type=int, default=8192, help="syncSearch window (samples)")
    ap.add_argument("--iters", type=int, default=25, help="sync-loop iterations")
    ap.add_argument("--detector",
                    default=os.path.expanduser("~/repos/HoudiniLab/CC/Sounder/beacon_hil"))
    a = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="beacon_stream_")
    match_bin = os.path.join(tmp, "match.bin")
    subprocess.run([a.detector, "--dump-tx", match_bin], check=True)
    mf = np.fromfile(match_bin, dtype=np.float32)
    match = (mf[0::2] + 1j * mf[1::2]).astype(np.complex128)

    rep = upsample(match, a.upsample)
    beacon = np.concatenate([rep, rep])
    loop = np.concatenate([beacon, np.zeros(len(beacon), dtype=complex)])
    loop = loop / (np.max(np.abs(loop)) + 1e-30) * a.amp
    n_load = len(loop)
    i16 = np.zeros(2 * n_load, dtype=np.int16)
    i16[0::2] = np.round(loop.real * 32767).astype(np.int16)
    i16[1::2] = np.round(loop.imag * 32767).astype(np.int16)

    print(f"TX {a.tx_ip} ch{a.tx_ch} -> RX {a.rx_ip} ch{a.rx_ch}  NCO {a.nco_mhz} MHz  "
          f"window {a.window}  iters {a.iters}")
    tx_ctx = hs.open_device(node=a.tx_ip, ch=a.tx_ch, verbose=False)
    rx_ctx = hs.open_device(node=a.rx_ip, ch=a.rx_ch, verbose=False)
    txd, rxd = tx_ctx["sdr"], rx_ctx["sdr"]
    native, dtype = rx_ctx["native_fmt"], rx_ctx["dtype"]

    tx = None
    det = None
    try:
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

        det = subprocess.Popen([a.detector, "--stream", str(a.window)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        cap_bytes = a.window * 4 + (1 << 16)          # >= one window of CS16
        print("\n--- streaming syncSearch (radioRx window -> find_beacon) ---")
        hits, t0 = 0, time.time()
        for _ in range(a.iters):
            buf, _ = hs.capture_rx(rxd, a.rx_ch, native, dtype, duration_sec=0.05,
                                   capture_bytes=cap_bytes)
            iq = to_complex(hs.cs16_lanes(buf))
            if len(iq) < a.window:
                continue
            iq = iq[:a.window]
            out = np.zeros(2 * a.window, dtype=np.int16)
            out[0::2] = np.clip(np.round(iq.real), -32768, 32767).astype(np.int16)
            out[1::2] = np.clip(np.round(-iq.imag), -32768, 32767).astype(np.int16)  # conj
            det.stdin.write(out.tobytes())
            det.stdin.flush()
            line = det.stdout.readline().decode(errors="replace").strip()
            print(f"  {line}")
            if "SYNC" in line:
                hits += 1
        det.stdin.close()
        rate = a.iters / max(time.time() - t0, 1e-6)
        print(f"\nsync loop: {hits}/{a.iters} windows acquired the beacon "
              f"({rate:.0f} windows/s incl. capture overhead)")
    finally:
        if det is not None:
            try:
                det.wait(timeout=5)
            except Exception:  # noqa: BLE001
                det.kill()
        if tx is not None:
            try:
                txd.deactivateStream(tx)
                txd.closeStream(tx)
            except Exception:  # noqa: BLE001
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
