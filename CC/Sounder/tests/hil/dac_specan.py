#!/usr/bin/env python3
"""
dac_specan.py -- confirm the DAC actually emits, on an Anritsu MS2690A.

TX a tone via the replay RAM (plain activateStream, NO TDD) on a chosen DAC, then
read the spectrum analyzer: the tone should appear at RF = NCO + f_bb, disappear
when TX is deactivated, and the carrier/image tell us the DAC's I/Q quality. This
is the ground-truth check the earlier loopback work was missing -- it looks at the
DAC output directly instead of through the (non-coupling) DAC->ADC path.

Rig: .21 DAC_B (TX ch1) -> Anritsu MS2690A at 168.6.244.20 (SICL-LAN raw SCPI
:49153). SpecAn class ported from Houdini-Streaming/tools/specan_capture.py
(itself the proven port of host_app/specan.cpp).

Run on the DGX (after: source houdini_test/bin/activate):
    python3 dac_specan.py --nco-mhz 500 --tone-mhz 20      # RF tone at 520 MHz
"""
import argparse
import os
import socket
import sys

import numpy as np

_EX = os.environ.get(
    "HOUDINI_EXAMPLES",
    os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if _EX not in sys.path:
    sys.path.insert(0, _EX)

import SoapySDR  # noqa: E402
from SoapySDR import SOAPY_SDR_TX  # noqa: E402
import houdini_setup as hs  # noqa: E402
from houdini_setup import tx_iq_tone  # noqa: E402


class SpecAn:
    """MS2690A raw-SCPI control (port of Houdini-Streaming specan_capture.py)."""

    def __init__(self, ip, port=49153, rcvtimeo=30.0):
        self.ip, self.port, self.rcvtimeo = ip, port, rcvtimeo
        self.sock = None
        self.carry = b""

    def connect(self):
        self.sock = socket.create_connection((self.ip, self.port), timeout=10.0)
        self.sock.settimeout(self.rcvtimeo)
        idn = self.query("*IDN?")
        print(f"[SA] connected: {idn}")
        return idn

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def send(self, cmd):
        self.sock.sendall((cmd + "\r\n").encode())

    def query(self, cmd):
        self.send(cmd)
        out = bytearray()
        buf, self.carry = self.carry, b""
        while True:
            nl = buf.find(b"\n")
            if nl >= 0:
                out += buf[:nl]
                self.carry = buf[nl + 1:]
                return out.replace(b"\r", b"").decode(errors="replace").strip()
            out += buf
            buf = self.sock.recv(4096)
            if not buf:
                return out.replace(b"\r", b"").decode(errors="replace").strip()

    def wait_opc(self, timeout_sec=60):
        self.sock.settimeout(timeout_sec)
        try:
            return self.query("*OPC?").startswith("1")
        finally:
            self.sock.settimeout(self.rcvtimeo)

    def peak(self, center_hz, span_hz=5e6, rbw_hz=10e3, ref_dbm=10.0):
        """Single sweep at (center, span); return (peak_freq_hz, peak_dbm)."""
        try:
            self.send("*CLS")
            self.send(f":FREQ:CENT {center_hz:.6E}")
            self.send(f":FREQ:SPAN {span_hz:.6E}")
            if rbw_hz > 0:
                self.send(f":BWID:RES {rbw_hz:.6E}")
            else:
                self.send(":BWID:RES:AUTO ON")
            self.send(f":DISP:WIND:TRAC:Y:SCAL:RLEV {ref_dbm:.1f}")
            self.send(":INIT:CONT OFF")
            self.send(":INIT:IMM")
            if not self.wait_opc(60):
                print("[SA] sweep timed out", file=sys.stderr)
                return None, None
            self.send(":CALC:MARK1:STAT ON")
            self.send(":CALC:MARK1:MAX")
            return (float(self.query(":CALC:MARK1:X?")),
                    float(self.query(":CALC:MARK1:Y?")))
        finally:
            self.send(":INIT:CONT ON")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--board", default="168.6.244.21")
    ap.add_argument("--tx-ch", type=int, default=1, help="DAC_B = TX ch1")
    ap.add_argument("--nco-mhz", type=float, default=500.0, help="DAC fine NCO (Zone 1)")
    ap.add_argument("--tone-mhz", type=float, default=20.0, help="baseband tone offset")
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--n-load", type=int, default=2048)
    ap.add_argument("--sa-ip", default="168.6.244.20")
    ap.add_argument("--span-mhz", type=float, default=120.0, help="wide-sweep span")
    ap.add_argument("--rbw-khz", type=float, default=10.0)
    ap.add_argument("--ref-dbm", type=float, default=10.0)
    ap.add_argument("--scan", action="store_true",
                    help="wide sweep (TX on) to find WHERE the DAC emits, if anywhere")
    a = ap.parse_args()

    print(f"TX {a.board} ch{a.tx_ch} (DAC_B), NCO {a.nco_mhz} MHz + tone -> RF, "
          f"read on Anritsu {a.sa_ip}")
    ctx = hs.open_device(node=a.board, ch=a.tx_ch, verbose=False)
    sdr = ctx["sdr"]
    ladder = list(sdr.listSampleRates(SOAPY_SDR_TX, a.tx_ch))
    if ladder:
        sdr.setSampleRate(SOAPY_SDR_TX, a.tx_ch, max(ladder))
    dac_rate = float(dict(sdr.getChannelInfo(SOAPY_SDR_TX, a.tx_ch)).get(
        "rfdc_effective_rate_hz", 983.04e6))
    nco = a.nco_mhz * 1e6

    tx = None
    sa = SpecAn(a.sa_ip)
    try:
        i16, f_act = tx_iq_tone(a.tone_mhz * 1e6, dac_rate, a.n_load, amp_frac=a.amp)
        tx = sdr.setupStream(SOAPY_SDR_TX, ctx["native_fmt"], [a.tx_ch],
                             {"tx_mode": "replay"})
        sdr.setFrequency(SOAPY_SDR_TX, a.tx_ch, nco)
        cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)
        sdr.writeStream(tx, [cs16], a.n_load, 0, 0)
        sdr.activateStream(tx)
        rf_tone, rf_car, rf_img = nco + f_act, nco, nco - f_act
        print(f"  replay tone f_bb={f_act/1e6:.3f} MHz -> expect RF tone {rf_tone/1e6:.2f}, "
              f"carrier {rf_car/1e6:.2f}, image {rf_img/1e6:.2f} MHz  (dac_rate "
              f"{dac_rate/1e6:.2f})")

        sa.connect()
        rbw = a.rbw_khz * 1e3
        if a.scan:
            print("\n  SCAN (TX on) -- strongest peak per 60-MHz window:")
            for c in (20, 80, 160, 260, 360, 460, 500, 520, 600, 720, 850, 940):
                pf, pa = sa.peak(c * 1e6, 60e6, rbw, a.ref_dbm)
                hot = "  <== signal" if pa > -70 else ""
                print(f"    ~{c:4d} MHz: peak {pf/1e6:8.3f} MHz @ {pa:7.2f} dBm{hot}")
            return 0
        wf, wa = sa.peak(nco, a.span_mhz * 1e6, rbw, a.ref_dbm)
        print(f"\n  WIDE sweep @ {nco/1e6:.0f} MHz span {a.span_mhz:.0f} MHz: "
              f"peak {wf/1e6:.3f} MHz @ {wa:.2f} dBm")
        levels = {}
        for name, f in (("carrier", rf_car), ("tone", rf_tone), ("image", rf_img)):
            pf, pa = sa.peak(f, 5e6, rbw, a.ref_dbm)
            levels[name] = (pf, pa)
            print(f"  {name:>7} @ {f/1e6:7.2f} MHz: peak {pf/1e6:8.3f} MHz @ {pa:7.2f} dBm")

        sdr.deactivateStream(tx)                       # TX OFF baseline
        bf, ba = sa.peak(rf_tone, 5e6, rbw, a.ref_dbm)
        print(f"  TX-OFF  @ {rf_tone/1e6:7.2f} MHz: peak {bf/1e6:8.3f} MHz @ {ba:7.2f} dBm")
    finally:
        try:
            if tx is not None:
                sdr.deactivateStream(tx)
                sdr.closeStream(tx)
        except Exception:  # noqa: BLE001
            pass
        sa.close()

    tone_f, tone_a = levels["tone"]
    on_off = tone_a - ba
    freq_ok = abs(tone_f - rf_tone) <= 1e6
    print(f"\n  tone at expected RF? {freq_ok} (|{tone_f/1e6:.3f}-{rf_tone/1e6:.3f}| "
          f"< 1 MHz);  TX-on minus TX-off = {on_off:.1f} dB")
    print(f"  carrier leak {levels['carrier'][1]-tone_a:+.1f} dB, "
          f"image {levels['image'][1]-tone_a:+.1f} dB (rel. to tone)")
    ok = freq_ok and on_off >= 10.0
    print("\nRESULT:",
          f"DAC EMITTING CORRECTLY -- tone at {tone_f/1e6:.3f} MHz @ {tone_a:.1f} dBm, "
          f"{on_off:.1f} dB over the TX-off floor" if ok else
          f"DAC output NOT confirmed (tone {on_off:.1f} dB over floor, freq_ok={freq_ok})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
