#!/usr/bin/env python3
"""
houdini_rx_format.py -- is the client's one-sided beacon a FORCED-CS16 RX artifact?

The sounder client opens its RX stream as SOAPY_SDR_CS16 (Radio ctor); the working
free-run capture opens the device NATIVE format. The client's captured beacon is
one-sided (upper SB only) so the wideband Gold code is half-cut and find_beacon fails
(9.5 dB) while the free-run is full-band (40 dB). This arms the SAME strobe beacon on
.21 and captures it on .22 two ways -- native format vs forced CS16 -- and scores each
by gold correlation + sideband split, to pin the format as the cause.

Run on the DGX (after: source houdini_test/bin/activate), needs /tmp/beacon_ram.bin +
/tmp/gold.bin (from a sounder run with HOUDINI_DUMP_BEACON=1 HOUDINI_DUMP_GOLD=1):
    python3 houdini_rx_format.py
"""
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
_EX = os.environ.get("HOUDINI_EXAMPLES",
                     os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if _EX not in sys.path:
    sys.path.insert(0, _EX)

import SoapySDR  # noqa: E402
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX  # noqa: E402
import houdini_setup as hs  # noqa: E402
from houdini_setup import rx_stream_args  # noqa: E402
from beacon_tdd import _arm, _teardown, GRID_TICKS, SYM, ARM_MARGIN  # noqa: E402

TX_IP, RX_IP, TXC, RXC, NCO = "168.6.244.21", "168.6.244.22", 1, 1, 500.0


def mf(x, h):
    n = 1 << int(np.ceil(np.log2(len(x) + len(h))))
    C = np.fft.ifft(np.fft.fft(x, n) * np.conj(np.fft.fft(h, n)))
    return np.abs(C[:len(x) - len(h) + 1])


def score(iq, gold, tag):
    iq = np.asarray(iq, dtype=np.complex128)
    c0, c1 = mf(iq, gold), mf(iq, np.conj(gold))
    c = c0 if c0.max() >= c1.max() else c1
    snr = 20 * np.log10(c.max() / (np.median(c) + 1e-30))
    m = min(len(iq), 1 << 13)
    X = np.abs(np.fft.fftshift(np.fft.fft(iq[:m] * np.hanning(m)))) ** 2
    fr = np.fft.fftshift(np.fft.fftfreq(m, 1.0 / 122.88e6)) / 1e6
    lo, hi = X[fr < -6].sum() / X.sum(), X[fr > 6].sum() / X.sum()
    print(f"  {tag:26s} gold {snr:5.1f} dB   low%{100*lo:5.1f} high%{100*hi:5.1f} "
          f"rms {np.sqrt(np.mean(np.abs(iq)**2)):.0f}")
    return snr


def read_stream(sdr, ch, fmt, dtype, nsamp, drain=True):
    rx = sdr.setupStream(SOAPY_SDR_RX, fmt, [ch], rx_stream_args(ch))
    sdr.activateStream(rx)
    if drain:
        # aggressive FIFO drain (recvHoudini does this: readStream timeout~0 until empty)
        junk = np.zeros(2 * 65536, dtype=dtype)
        for _ in range(30):
            if sdr.readStream(rx, [junk], 65536, timeoutUs=0).ret <= 0:
                break
    buf = np.zeros(nsamp, dtype=dtype)
    got = 0
    t0 = time.monotonic()
    while got < nsamp and time.monotonic() - t0 < 3:
        sr = sdr.readStream(rx, [buf[got:]], nsamp - got, timeoutUs=200000)
        if sr.ret > 0:
            got += sr.ret
        elif sr.ret == SoapySDR.SOAPY_SDR_TIMEOUT:
            continue
        else:
            break
    sdr.deactivateStream(rx)
    sdr.closeStream(rx)
    return buf[:got]


def main():
    ram = np.fromfile("/tmp/beacon_ram.bin", dtype=np.int16)
    n_load = len(ram) // 2
    cs16 = np.ascontiguousarray(ram, dtype=np.int16).view(np.int32)
    gold = np.fromfile("/tmp/gold.bin", dtype=np.complex64).astype(np.complex128)
    gold /= np.sqrt(np.sum(np.abs(gold) ** 2)) + 1e-30

    tx = hs.open_device(node=TX_IP, ch=TXC, verbose=False)
    rx = hs.open_device(node=RX_IP, ch=RXC, verbose=False)
    tsd, rsd = tx["sdr"], rx["sdr"]
    native = tx["native_fmt"]
    rnative, rdtype = rx["native_fmt"], rx["dtype"]
    print(f"RX device native format = {rnative!r} (dtype {np.dtype(rdtype)})")
    print(f"getNativeStreamFormat(RX) = {rsd.getNativeStreamFormat(SOAPY_SDR_RX, RXC)}")
    print(f"getStreamFormats(RX) = {list(rsd.getStreamFormats(SOAPY_SDR_RX, RXC))}")

    _teardown(tsd)
    ladder = list(tsd.listSampleRates(SOAPY_SDR_TX, TXC))
    if ladder:
        tsd.setSampleRate(SOAPY_SDR_TX, TXC, max(ladder))
    tsd.setFrequency(SOAPY_SDR_TX, TXC, NCO * 1e6)
    rsd.setSampleRate(SOAPY_SDR_RX, RXC, 122.88e6)
    rsd.setFrequency(SOAPY_SDR_RX, RXC, NCO * 1e6)

    txs = tsd.setupStream(SOAPY_SDR_TX, native, [TXC], {"tx_mode": "replay"})
    try:
        tsd.writeStream(txs, [cs16], n_load, 0, 0)
        tsd.writeSetting("TDD_SCHED", "62")
        tsd.writeSetting("TDD_REPLAY_STROBE",
                         f"ch{TXC}:len={n_load // 2},loops=forever,offs={GRID_TICKS}")
        r = _arm(tsd, symbol_ticks=SYM, symbols_per_frame=2, margin=ARM_MARGIN)
        print(f"arm accepted={r.get('accepted')}")
        time.sleep(0.2)

        def cs16_to_iq(b):
            v = b.view(np.int32) if b.dtype != np.int32 else b
            return (v & 0xFFFF).astype(np.int16).astype(np.float64) + \
                1j * (v >> 16).astype(np.int16).astype(np.float64)

        W = 118784  # ~0.97 ms > the 0.5 ms silent rx_gate gap -> always spans a beacon
        for trial in range(2):
            # (A) capture_rx: the WORKING path (no aggressive drain)
            buf, _ = hs.capture_rx(rsd, RXC, rnative, rdtype, duration_sec=0.05,
                                   capture_bytes=4 * W, live_print=False)
            score(cs16_to_iq(buf)[:W], gold, f"[{trial}] A capture_rx")
            # (B) recvHoudini-style: aggressive drain + concat
            score(cs16_to_iq(read_stream(rsd, RXC, "CS16", np.int32, W, drain=True)),
                  gold, f"[{trial}] B drain+concat")
            # (C) NO drain + concat
            score(cs16_to_iq(read_stream(rsd, RXC, "CS16", np.int32, W, drain=False)),
                  gold, f"[{trial}] C nodrain+concat")
    finally:
        try:
            tsd.writeSetting("TDD_REPLAY_STROBE", f"ch{TXC}:off")
        except Exception:  # noqa: BLE001
            pass
        try:
            tsd.deactivateStream(txs); tsd.closeStream(txs)
        except Exception:  # noqa: BLE001
            pass
        _teardown(tsd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
