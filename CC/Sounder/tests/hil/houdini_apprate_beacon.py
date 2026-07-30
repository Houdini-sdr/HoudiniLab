#!/usr/bin/env python3
"""
houdini_apprate_beacon.py -- replay the ISOLATED beacon RAM at a chosen TX rate and
measure what actually comes out on-air (period, isolation, gold), to see whether the
app-rate replay produces the intended isolated beacon or smears it.

Loads /tmp/beacon_ram.bin (the sounder's buildHoudiniBeacon dump: 496-sample core at
the head of a 4096 RAM, silence after), arms it on .21 like armHoudiniTdd (TDD_SCHED
'62' + TDD_REPLAY_STROBE loops=forever), captures free-run on .22, and reports the
autocorrelation period, the folded energy profile (isolated?), and the gold peak.

    python3 houdini_apprate_beacon.py --tx-rate 122.88
    python3 houdini_apprate_beacon.py --tx-rate max      # old x8-style
"""
import argparse
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
from beacon_tdd import _arm, _teardown, GRID_TICKS, SYM, ARM_MARGIN  # noqa: E402

TX_IP, RX_IP, TXC, RXC, NCO = "168.6.244.21", "168.6.244.22", 1, 1, 500.0


def to_c(buf):
    w = hs.cs16_lanes(buf) if False else None
    a = np.frombuffer(buf, dtype=np.int16)
    return a[0::2].astype(np.float64) + 1j * a[1::2]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-rate", default="122.88", help="MHz, or 'max'")
    ap.add_argument("--rx-nco", type=float, default=None,
                    help="RX NCO in MHz (default = TX NCO 500); set 512/488 to "
                         "hardware-compensate the +12 MHz app-rate beacon offset")
    ap.add_argument("--tx-nco", type=float, default=NCO, help="TX NCO MHz")
    ap.add_argument("--secs", type=float, default=0.12)
    ap.add_argument("--cap-mb", type=float, default=16.0)
    a = ap.parse_args()

    ram = np.fromfile("/tmp/beacon_ram.bin", dtype=np.int16)
    n_load = len(ram) // 2
    cs16 = np.ascontiguousarray(ram, dtype=np.int16).view(np.int32)
    gold = np.fromfile("/tmp/gold.bin", dtype=np.complex64).astype(np.complex128)
    gold /= np.linalg.norm(gold) + 1e-30
    core = int(np.max(np.where(np.abs(ram.reshape(-1, 2)).sum(1) > 0)) + 1)
    print(f"beacon RAM {n_load} samp, core (nonzero head) = {core} samp")

    tx = hs.open_device(node=TX_IP, ch=TXC, verbose=False)
    rx = hs.open_device(node=RX_IP, ch=RXC, verbose=False)
    tsd, rsd = tx["sdr"], rx["sdr"]
    native = tx["native_fmt"]
    rnative, rdtype = rx["native_fmt"], rx["dtype"]

    _teardown(tsd)
    if a.tx_rate == "max":
        ladder = list(tsd.listSampleRates(SOAPY_SDR_TX, TXC))
        tx_rate = max(ladder)
    else:
        tx_rate = float(a.tx_rate) * 1e6
    tsd.setSampleRate(SOAPY_SDR_TX, TXC, tx_rate)
    tx_rate = float(tsd.getSampleRate(SOAPY_SDR_TX, TXC))
    tsd.setFrequency(SOAPY_SDR_TX, TXC, a.tx_nco * 1e6)
    rx_nco = a.rx_nco if a.rx_nco is not None else NCO
    rsd.setSampleRate(SOAPY_SDR_RX, RXC, 122.88e6)
    rsd.setFrequency(SOAPY_SDR_RX, RXC, rx_nco * 1e6)
    rx_rate = float(rsd.getSampleRate(SOAPY_SDR_RX, RXC))
    print(f"TX NCO {a.tx_nco} MHz  RX NCO {rx_nco} MHz")
    ratio = tx_rate / rx_rate
    print(f"TX rate {tx_rate/1e6:.2f} MSPS  RX {rx_rate/1e6:.2f}  ratio {ratio:.2f} "
          f"=> expected on-air period {n_load/ratio:.0f} rx-samp, "
          f"core {core/ratio:.0f} rx-samp")

    txs = tsd.setupStream(SOAPY_SDR_TX, native, [TXC], {"tx_mode": "replay"})
    try:
        tsd.writeStream(txs, [cs16], n_load, 0, 0)
        tsd.writeSetting("TDD_SCHED", "62")
        tsd.writeSetting("TDD_REPLAY_STROBE",
                         f"ch{TXC}:len={n_load // 2},loops=forever,offs={GRID_TICKS}")
        r = _arm(tsd, symbol_ticks=SYM, symbols_per_frame=2, margin=ARM_MARGIN)
        print(f"arm accepted={r.get('accepted')}")
        time.sleep(0.2)
        buf, _ = hs.capture_rx(rsd, RXC, rnative, rdtype, duration_sec=a.secs,
                               capture_bytes=int(a.cap_mb * 1024 * 1024),
                               live_print=False)
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

    iq = to_c(buf)
    iq.astype(np.complex64).tofile("/tmp/apr_cap.bin")
    print(f"captured {len(iq)} samp rms {np.sqrt(np.mean(np.abs(iq)**2)):.1f}")

    # measure the true period via magnitude autocorrelation (search 200..8192)
    m = min(len(iq), 1 << 17)
    e = np.abs(iq[:m]) ** 2
    e = e - e.mean()
    ac = np.abs(np.fft.ifft(np.abs(np.fft.fft(e)) ** 2))
    ac[0] = 0
    per = int(np.argmax(ac[200:8192]) + 200)
    print(f"measured on-air period = {per} rx-samp  (expected {n_load/ratio:.0f})")

    # fold at the measured period -> is the beacon isolated?
    P = per
    nf = m // P
    prof = (np.abs(iq[:nf * P].reshape(nf, P)) ** 2).mean(0)
    hot = np.where(prof > prof.max() * 0.25)[0]
    print(f"fold-{P}: hot samples [{hot.min()}..{hot.max()}] = {len(hot)}/{P} "
          f"({'ISOLATED' if len(hot) < P * 0.3 else 'SPREAD'})")

    # gold peak (freq-swept)
    nn = np.arange(len(iq))

    def gcpk(x):
        n = 1 << int(np.ceil(np.log2(len(x) + len(gold))))
        c = np.abs(np.fft.ifft(np.fft.fft(x, n) * np.conj(np.fft.fft(gold, n)))[:len(x) - len(gold) + 1])
        return c.max() / (np.median(c) + 1e-30)
    best = max(((gcpk(iq * np.exp(-2j * np.pi * f * 1e6 * nn / rx_rate)), f)
                for f in np.arange(-20, 20.01, 0.5)), key=lambda t: t[0])
    print(f"gold peak/median = {best[0]:.1f} at DDC {best[1]:+.1f} MHz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
