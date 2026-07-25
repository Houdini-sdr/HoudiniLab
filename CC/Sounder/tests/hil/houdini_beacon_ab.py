#!/usr/bin/env python3
"""
houdini_beacon_ab.py -- A/B the EXACT sounder beacon RAM played CONTINUOUSLY vs via
the TDD framer STROBE, scored by the CLIENT's own gold correlation.

The decoupled design is impossible (an armed framer silences a continuous replay), so
the beacon must ride the framer strobe -- but that path gave the client only ~11 dB
gold correlation while continuous replay synced. This isolates the strobe datapath:
load /tmp/beacon_ram.bin (dumped by buildHoudiniBeacon, HOUDINI_DUMP_BEACON=1) into
the replay RAM, play it two ways on .21, capture on .22, and for each correlate the
capture against /tmp/gold.bin (gold_cf32, the 128-tap the client matches) exactly like
find_beacon. If continuous >> strobe on the SAME RAM, the strobe datapath is the fault.

  A  continuous  activateStream(tx)                         -- proven client-syncable
  B  strobe      TDD_SCHED '6..' + TDD_REPLAY_STROBE len=n_load/2 loops=forever + arm

Run on the DGX (after: source houdini_test/bin/activate), having first produced the
dumps by running the sounder once with HOUDINI_DUMP_BEACON=1 HOUDINI_DUMP_GOLD=1:
    python3 houdini_beacon_ab.py
"""
import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
_EX = os.environ.get(
    "HOUDINI_EXAMPLES",
    os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if _EX not in sys.path:
    sys.path.insert(0, _EX)

import SoapySDR  # noqa: E402
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX  # noqa: E402
import houdini_setup as hs  # noqa: E402
from beacon_tdd import (_arm, _teardown, _cmd, GRID_TICKS, SYM,  # noqa: E402
                        ARM_MARGIN)


def to_complex(lanes):
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex128)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float64)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return L[:, 0] + 1j * q


def mf(x, h):
    """|matched filter| of long x with short template h, via FFT."""
    n = 1 << int(np.ceil(np.log2(len(x) + len(h))))
    C = np.fft.ifft(np.fft.fft(x, n) * np.conj(np.fft.fft(h, n)))
    return np.abs(C[:len(x) - len(h) + 1])


def gold_snr(iq, center_mhz, rx_rate, gold):
    """Client-style detection: DDC to baseband, correlate vs gold (both senses)."""
    n = np.arange(len(iq))
    ddc = iq * np.exp(+2j * np.pi * center_mhz * 1e6 * n / rx_rate)
    c0, c1 = mf(ddc, gold), mf(ddc, np.conj(gold))
    corr = c0 if c0.max() >= c1.max() else c1
    snr = 20 * np.log10(corr.max() / (np.median(corr) + 1e-30))
    return snr, corr


def period_hint(iq, center_mhz, rx_rate, gold, pmax=1200):
    """Autocorrelation of the gold-correlation envelope -> beacon repeat period."""
    _, corr = gold_snr(iq, center_mhz, rx_rate, gold)
    e = corr - corr.mean()
    m = min(len(e), 1 << 16)
    ac = np.abs(np.fft.ifft(np.abs(np.fft.fft(e[:m])) ** 2))
    ac[0] = 0
    lag = int(np.argmax(ac[16:pmax])) + 16
    return lag


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--nco", type=float, default=500.0)
    ap.add_argument("--center-mhz", type=float, default=0.0,
                    help="beacon baseband centre (buildHoudiniBeacon core is at 0)")
    ap.add_argument("--beacon-ram", default="/tmp/beacon_ram.bin")
    ap.add_argument("--gold", default="/tmp/gold.bin")
    ap.add_argument("--spf", type=int, default=2, help="framer symbols/frame for B")
    ap.add_argument("--secs", type=float, default=0.25)
    ap.add_argument("--cap-mb", type=float, default=32.0)
    a = ap.parse_args()

    ram = np.fromfile(a.beacon_ram, dtype=np.int16)
    n_load = len(ram) // 2
    cs16 = np.ascontiguousarray(ram, dtype=np.int16).view(np.int32)
    gold = np.fromfile(a.gold, dtype=np.complex64).astype(np.complex128)
    gold /= np.sqrt(np.sum(np.abs(gold) ** 2)) + 1e-30
    print(f"beacon RAM {n_load} samp; gold {len(gold)} taps")

    tx = hs.open_device(node=a.tx_ip, ch=a.tx_ch, verbose=False)
    rx = hs.open_device(node=a.rx_ip, ch=a.rx_ch, verbose=False)
    tsd, rsd = tx["sdr"], rx["sdr"]
    native = tx["native_fmt"]
    rnative, rdtype = rx["native_fmt"], rx["dtype"]

    _teardown(tsd)
    ladder = list(tsd.listSampleRates(SOAPY_SDR_TX, a.tx_ch))
    if ladder:
        tsd.setSampleRate(SOAPY_SDR_TX, a.tx_ch, max(ladder))
    dac_rate = float(tsd.getSampleRate(SOAPY_SDR_TX, a.tx_ch))
    tsd.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.nco * 1e6)
    rsd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, 122.88e6)
    rsd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
    rx_rate = float(rsd.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
    print(f"dac {dac_rate/1e6:.2f}  rx {rx_rate/1e6:.2f}  nco {a.nco}  "
          f"expected continuous period {n_load}/8 = {n_load//8} rx samp")

    txs = tsd.setupStream(SOAPY_SDR_TX, native, [a.tx_ch], {"tx_mode": "replay"})

    def capture():
        buf, _ = hs.capture_rx(rsd, a.rx_ch, rnative, rdtype, duration_sec=a.secs,
                               capture_bytes=int(a.cap_mb * 1024 * 1024))
        return to_complex(hs.cs16_lanes(buf))

    out = {}
    try:
        # ---- A: continuous replay ----
        tsd.writeStream(txs, [cs16], n_load, 0, 0)
        tsd.activateStream(txs)
        time.sleep(0.2)
        iqA = capture()
        snrA, _ = gold_snr(iqA, a.center_mhz, rx_rate, gold)
        perA = period_hint(iqA, a.center_mhz, rx_rate, gold)
        out["A"] = (snrA, perA, float(np.sqrt(np.mean(np.abs(iqA) ** 2))))
        tsd.deactivateStream(txs)
        _teardown(tsd)
        time.sleep(0.3)

        # ---- B: framer strobe ----
        tsd.writeStream(txs, [cs16], n_load, 0, 0)
        sched = "6" + "2" * (a.spf - 1)
        tsd.writeSetting("TDD_SCHED", sched)
        tsd.writeSetting("TDD_REPLAY_STROBE",
                         f"ch{a.tx_ch}:len={n_load // 2},loops=forever,"
                         f"offs={GRID_TICKS}")
        r = _arm(tsd, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        print(f"B arm: accepted={r.get('accepted')} sched={sched}")
        time.sleep(0.2)
        iqB = capture()
        try:
            bank = tsd.readSetting("TX_BANK_STATUS")
            ack = [c for c in bank.split(";") if c.startswith(f"ch{a.tx_ch}:")]
            print("B TX bank:", ack[0].split(":", 1)[1][:60] if ack else "?")
        except Exception:  # noqa: BLE001
            pass
        snrB, _ = gold_snr(iqB, a.center_mhz, rx_rate, gold)
        perB = period_hint(iqB, a.center_mhz, rx_rate, gold)
        out["B"] = (snrB, perB, float(np.sqrt(np.mean(np.abs(iqB) ** 2))))
    finally:
        try:
            tsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:off")
        except Exception:  # noqa: BLE001
            pass
        try:
            tsd.deactivateStream(txs); tsd.closeStream(txs)
        except Exception:  # noqa: BLE001
            pass
        _teardown(tsd)

    print(f"\n{'mode':22s} {'gold-SNR':>9s} {'period':>7s} {'wideRMS':>8s}")
    for k, name in (("A", "continuous"), ("B", "framer strobe")):
        if k in out:
            s, p, w = out[k]
            print(f"{k} {name:20s} {s:8.1f}dB {p:7d} {w:8.2f}")
    if "A" in out and "B" in out:
        print(f"\ncontinuous - strobe = {out['A'][0] - out['B'][0]:+.1f} dB "
              f"(expected continuous period ~{n_load//8})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
