#!/usr/bin/env python3
"""
houdini_cont_vs_framer.py -- does arming the TDD framer suppress a CONTINUOUS
(activateStream) replay beacon, and which schedule symbols keep it on air?

The decoupled BS design wants ONE board to (a) emit a continuous replay beacon the
free-running UE can always sync on AND (b) gate its own RX to the pilot slot via the
framer. But arming the framer may take over the TX datapath so the replay FIFO only
reaches the DAC during tx_gate symbols. This probe loads a chirp into the replay RAM,
activateStream()s it (continuous), then for each --sched arms the framer and measures
the beacon RMS at the RX board. Schedules to compare (each same length):

  (none)   no arm, pure continuous replay              -- control, must be LOUD
  22222    all rx_gate (my current armHoudiniTdd)        -- suspected SILENT
  11111    all tx_gate                                   -- replay->DAC every symbol?
  33333    all tx_gate+rx_gate (full duplex)             -- beacon + capture together
  30000    tx_gate only in symbol 0                      -- beacon 1/frame

Run on the DGX (after: source houdini_test/bin/activate):
    python3 houdini_cont_vs_framer.py
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
from houdini_setup import tx_lfm_chirp  # noqa: E402
from beacon_tdd import _arm, _teardown, SYM, ARM_MARGIN  # noqa: E402


def to_complex(lanes):
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex128)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float64)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return L[:, 0] + 1j * q


def band_rms(rsd, rx_ch, rnative, rdtype, center_mhz, rx_rate, secs, cap_mb):
    buf, _ = hs.capture_rx(rsd, rx_ch, rnative, rdtype, duration_sec=secs,
                           capture_bytes=int(cap_mb * 1024 * 1024))
    iq = to_complex(hs.cs16_lanes(buf))
    n = np.arange(len(iq))
    ddc = iq * np.exp(+2j * np.pi * center_mhz * 1e6 * n / rx_rate)
    return float(np.sqrt(np.mean(np.abs(ddc) ** 2))), float(np.sqrt(np.mean(np.abs(iq) ** 2)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--nco", type=float, default=500.0)
    ap.add_argument("--center-mhz", type=float, default=20.0)
    ap.add_argument("--bw-mhz", type=float, default=30.0)
    ap.add_argument("--n-load", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--secs", type=float, default=0.2)
    ap.add_argument("--cap-mb", type=float, default=24.0)
    ap.add_argument("--scheds", default="none,22222,11111,33333,30000")
    a = ap.parse_args()

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

    i16, _ = tx_lfm_chirp(a.bw_mhz * 1e6, dac_rate, a.n_load, amp_frac=a.amp,
                          center_hz=a.center_mhz * 1e6)
    cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)

    txs = tsd.setupStream(SOAPY_SDR_TX, native, [a.tx_ch], {"tx_mode": "replay"})
    print(f"TX {a.tx_ip} ch{a.tx_ch} continuous replay -> RX {a.rx_ip} ch{a.rx_ch}, "
          f"dac {dac_rate/1e6:.2f} rx {rx_rate/1e6:.2f}")
    results = []
    try:
        tsd.writeStream(txs, [cs16], a.n_load, 0, 0)   # load replay RAM
        tsd.activateStream(txs)                        # CONTINUOUS replay
        time.sleep(0.2)
        for sched in [s.strip() for s in a.scheds.split(",") if s.strip()]:
            if sched != "none":
                spf = len(sched)
                tsd.writeSetting("TDD_SCHED", sched)
                r = _arm(tsd, symbol_ticks=SYM, symbols_per_frame=spf, margin=ARM_MARGIN)
                acc = r.get("accepted")
                time.sleep(0.2)
            else:
                acc = "-"
            brms, wrms = band_rms(rsd, a.rx_ch, rnative, rdtype, a.center_mhz,
                                  rx_rate, a.secs, a.cap_mb)
            try:
                bank = tsd.readSetting("TX_BANK_STATUS")
                ack = [c for c in bank.split(";") if c.startswith(f"ch{a.tx_ch}:")]
                ackstr = ack[0].split(":", 1)[1][:36] if ack else "?"
            except Exception:  # noqa: BLE001
                ackstr = "?"
            results.append((sched, acc, brms, wrms, ackstr))
            print(f"  sched={sched:6s} armed={str(acc):3s}  band-RMS={brms:8.2f}  "
                  f"wide-RMS={wrms:7.2f}  bank[{ackstr}]")
            if sched != "none":
                from beacon_tdd import _cmd
                _cmd(tsd, "abort")
                time.sleep(0.15)
    finally:
        try:
            tsd.deactivateStream(txs); tsd.closeStream(txs)
        except Exception:  # noqa: BLE001
            pass
        _teardown(tsd)

    base = next((r[2] for r in results if r[0] == "none"), 1.0)
    print("\nsummary (band-RMS relative to continuous 'none'):")
    for sched, acc, brms, wrms, _s in results:
        print(f"  {sched:6s} armed={str(acc):3s}  {brms:8.2f}  "
              f"({20*np.log10(brms/max(base,1e-9)):+5.1f} dB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
